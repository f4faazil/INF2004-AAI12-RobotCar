/*
 * AAI12 Buddy 1: opt-in MQTT 3.1.1 smoke-test adapter for the existing
 * micro T-Kernel CYW43/lwIP NO_SYS=1 owner task.
 *
 * This is NOT a general-purpose network API. The only networking operation
 * done outside the owner task is none. All lwIP raw TCP callbacks run in the
 * same owner task. In particular, never call this module from motor control.
 *
 * Qualification: publish one online marker, enqueue a three-byte DIAGNOSTIC
 * sample through the RTOS-protected shared telemetry inbox, then
 * peek/publish/commit. The radio task is the SOLE consumer and the ONLY
 * place that calls lwIP/MQTT. Other TASKS may submit via telemetry_rtos.h.
 * This is NOT sensor telemetry and does not demonstrate motor isolation.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lwip/err.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/tcp.h"

#include "mqtt_lwip.h"
#include "mqtt_session.h"
#include "telemetry_rtos.h"
#include "mqtt_test_config.h"

#define AAI12_MQTT_RX_CHUNK 64U

static const uint8_t qualification_message[] = "AAI12_PICO_W_ONLINE";
static const char robot_telemetry_topic[] = "aai12/robot/telemetry";

typedef struct {
    struct tcp_pcb *pcb;
    mqtt_session_t session;
    ip4_addr_t broker;
    bool initialized;
    bool in_callback;
    bool aborted_in_callback;
    uint32_t published_online_session;
    /* TCP may have accepted publication while the queue mutex is busy.
     * Complete the pending commit before trying another publish. */
    bool pending_commit;
} mqtt_lwip_adapter_t;

static mqtt_lwip_adapter_t adapter;

static void tcp_detach(struct tcp_pcb *pcb)
{
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_sent(pcb, NULL);
    tcp_err(pcb, NULL);
    tcp_poll(pcb, NULL, 0);
}

/* TCP PCB lifetime is owned only by this adapter.
 * tcp_abort() invoked from a raw lwIP callback REQUIRES that callback to
 * return ERR_ABRT, rather than touching the PCB afterward. */
static void close_tcp(void *context)
{
    mqtt_lwip_adapter_t *a = (mqtt_lwip_adapter_t *)context;
    struct tcp_pcb *pcb = a->pcb;

    if (pcb == NULL) return;
    a->pcb = NULL;
    tcp_detach(pcb);
    tcp_abort(pcb);
    if (a->in_callback) a->aborted_in_callback = true;
}

static bool write_tcp(void *context, const uint8_t *bytes, size_t size)
{
    mqtt_lwip_adapter_t *a = (mqtt_lwip_adapter_t *)context;
    err_t result;

    if (a->pcb == NULL || bytes == NULL || size == 0U || size > 65535U ||
        size > (size_t)tcp_sndbuf(a->pcb)) return false;

    result = tcp_write(a->pcb, bytes, (u16_t)size, TCP_WRITE_FLAG_COPY);
    if (result != ERR_OK) return false;

    /* tcp_write(...COPY) has already queued all bytes. Once queued, even if
     * tcp_output is temporarily unable to transmit, returning false would
     * cause MQTT to enqueue a duplicate packet next time. lwIP retries. */
    (void)tcp_output(a->pcb);
    return true;
}

/* On error lwIP ALREADY freed the PCB. Never tcp_close/tcp_abort here. */
static void on_error(void *context, err_t err)
{
    mqtt_lwip_adapter_t *a = (mqtt_lwip_adapter_t *)context;
    (void)err;
    a->pcb = NULL;
    mqtt_session_tcp_lost(&a->session, sys_now());
}

static err_t on_sent(void *context, struct tcp_pcb *pcb, u16_t len)
{
    (void)context;
    (void)pcb;
    (void)len;
    return ERR_OK;
}

static err_t on_connected(void *context, struct tcp_pcb *pcb, err_t err)
{
    mqtt_lwip_adapter_t *a = (mqtt_lwip_adapter_t *)context;

    if (pcb != a->pcb) return ERR_OK;
    if (err != ERR_OK) {
        mqtt_session_tcp_lost(&a->session, sys_now());
        close_tcp(a);
        return ERR_ABRT;
    }

    a->aborted_in_callback = false;
    a->in_callback = true;
    mqtt_session_tcp_connected(&a->session, sys_now());
    a->in_callback = false;
    if (a->aborted_in_callback) {
        a->aborted_in_callback = false;
        return ERR_ABRT;
    }
    return ERR_OK;
}

static err_t on_received(void *context, struct tcp_pcb *pcb,
                         struct pbuf *p, err_t err)
{
    mqtt_lwip_adapter_t *a = (mqtt_lwip_adapter_t *)context;
    u16_t offset;
    u16_t total;
    uint8_t scratch[AAI12_MQTT_RX_CHUNK];

    if (pcb != a->pcb) {
        if (p != NULL) pbuf_free(p);
        return ERR_OK;
    }
    if (p == NULL) {             /* Peer sent FIN; lwIP still owns pcb. */
        err_t close_result;
        a->pcb = NULL;
        tcp_detach(pcb);
        close_result = tcp_close(pcb);
        if (close_result != ERR_OK) tcp_abort(pcb);
        mqtt_session_tcp_lost(&a->session, sys_now());
        return (close_result == ERR_OK) ? ERR_OK : ERR_ABRT;
    }
    if (err != ERR_OK) {
        pbuf_free(p);
        close_tcp(a);
        mqtt_session_tcp_lost(&a->session, sys_now());
        return ERR_ABRT;
    }

    total = p->tot_len;
    /* We accepted the full TCP payload into the bounded parser path. */
    tcp_recved(pcb, total);
    offset = 0U;
    a->aborted_in_callback = false;
    a->in_callback = true;
    while (offset < total) {
        u16_t count = (u16_t)(total - offset);
        if (count > AAI12_MQTT_RX_CHUNK) count = AAI12_MQTT_RX_CHUNK;
        if (pbuf_copy_partial(p, scratch, count, offset) != count) {
            close_tcp(a);
            mqtt_session_tcp_lost(&a->session, sys_now());
            break;
        }
        mqtt_session_tcp_received(&a->session, scratch, count, sys_now());
        offset = (u16_t)(offset + count);
        if (a->aborted_in_callback) break;
    }
    a->in_callback = false;
    pbuf_free(p);
    if (a->aborted_in_callback) {
        a->aborted_in_callback = false;
        return ERR_ABRT;
    }
    return ERR_OK;
}

static bool start_tcp(void *context)
{
    mqtt_lwip_adapter_t *a = (mqtt_lwip_adapter_t *)context;
    struct tcp_pcb *pcb;
    err_t result;

    if (a->pcb != NULL) return false;
    pcb = tcp_new();
    if (pcb == NULL) return false;
    a->pcb = pcb;
    tcp_arg(pcb, a);
    tcp_recv(pcb, on_received);
    tcp_sent(pcb, on_sent);
    tcp_err(pcb, on_error);
    result = tcp_connect(pcb, (const ip_addr_t *)&a->broker,
                         (u16_t)AAI12_MQTT_BROKER_PORT, on_connected);
    if (result != ERR_OK) {
        close_tcp(a);
        return false;
    }
    return true;
}

void mqtt_lwip_poll(struct netif *netif)
{
    static const mqtt_session_transport_t transport = {
        start_tcp, write_tcp, close_tcp
    };
    const uint32_t now = sys_now();
    bool ready;

    if (!adapter.initialized) {
        if (netif == NULL ||
            !ip4addr_aton(AAI12_MQTT_BROKER_IP, &adapter.broker) ||
            ip4_addr_isany_val(adapter.broker) ||
            AAI12_MQTT_BROKER_PORT == 0U ||
            !mqtt_session_init(&adapter.session, &transport, &adapter,
                               AAI12_MQTT_CLIENT_ID)) {
            return;            /* Invalid local configuration; no network */
        }
        /* usermain() must initialize mutex and inbox before tasks start.
         * Do not reset a live queue or lazily create cross-core resources. */
        if (!telemetry_rtos_ready()) return;
        adapter.initialized = true;
    }

    ready = netif != NULL && netif_is_up(netif) &&
            netif_is_link_up(netif) &&
            !ip4_addr_isany_val(*netif_ip4_addr(netif));
    if (ready != adapter.session.network_up) {
        mqtt_session_set_network(&adapter.session, ready, now);
    }
    mqtt_session_poll(&adapter.session, now);

    /* Smoke test ONLY: one QoS 0 message after each CONNACK (not real car
     * telemetry, no delivery acknowledgement). Retried if TX is full. */
    if (adapter.session.state == MQTT_SESSION_ONLINE &&
        adapter.session.sessions_online != adapter.published_online_session) {
        if (mqtt_session_publish(&adapter.session, AAI12_MQTT_TEST_TOPIC,
                                 qualification_message,
                                 sizeof(qualification_message) - 1U, now)) {
            adapter.published_online_session = adapter.session.sessions_online;
        }
        /* Send the binary frame in a later poll, not the same tick. */
        return;
    }

    /* A prior QoS-0 packet may be locally accepted, but removal could have
     * failed due to momentary mutex contention. Commit exactly once before
     * attempting any further peek/publish; never retransmit that head. */
    if (adapter.pending_commit) {
        if (telemetry_rtos_commit() == TELEMETRY_INBOX_OK) {
            adapter.pending_commit = false;
        }
        return;
    }

    if (adapter.session.state == MQTT_SESSION_ONLINE &&
        adapter.published_online_session == adapter.session.sessions_online) {
        uint8_t frame[TELEMETRY_QUEUE_MAX_BYTES];
        size_t length = 0U;
        if (telemetry_rtos_peek(now, frame, sizeof frame, &length) ==
            TELEMETRY_INBOX_OK &&
            mqtt_session_publish(&adapter.session, robot_telemetry_topic,
                                 frame, length, now)) {
            /* TCP accepted the QoS 0 packet locally, not broker-ACKed.
             * Contended commit is retried without republishing. */
            if (telemetry_rtos_commit() != TELEMETRY_INBOX_OK) {
                adapter.pending_commit = true;
            }
        }
    }
}
