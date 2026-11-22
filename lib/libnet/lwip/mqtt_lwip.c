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
#include "mqtt_heartbeat.h"
#include "mqtt_mailbox.h"
#include "telemetry_rtos.h"
#include "mqtt_test_config.h"

#if PICO_RP2040
#include "hardware/structs/rosc.h"
#include "hardware/structs/watchdog.h"
#endif

#define AAI12_MQTT_RX_CHUNK 64U
#define AAI12_MQTT_OUTPUT_TIMEOUT_MS 5000U
#define AAI12_MQTT_DIAG_PAYLOAD_BYTES 64U
#define AAI12_DIAG_PART1_TYPE 0x23U
#define AAI12_DIAG_PART2_TYPE 0x24U
#define AAI12_ONLINE_MARKER_TYPE 0x22U

/* The online marker is the single byte 0x22 (it was the 19 characters AAI12_PICO_W_ONLINE). */
static const uint8_t qualification_message[] = { AAI12_ONLINE_MARKER_TYPE };
static const char robot_telemetry_topic[] = "aai12/robot/telemetry";
static const char diagnostic_topic[] = "aai12/diagnostic";
static const char heartbeat_topic[] = "aai12/robot/heartbeat";
/* PC -> car: the barcode letter map and the operator's START / STOP, delivered to the robot through mqtt_mailbox.
 * The robot applies its own policy to a command (refused while picked up, START refused while running).  Commands are
 * published by the dashboard WITHOUT the retain flag, so a command is never replayed when the car reconnects. */
#ifndef AAI12_MQTT_SUBSCRIBE
#define AAI12_MQTT_SUBSCRIBE 1   /* the adapter packet-count host test builds with 0 */
#endif
#if AAI12_MQTT_SUBSCRIBE
static const char subscribe_map_topic[] = "aai12/robot/map";
static const char subscribe_cmd_topic[] = "aai12/robot/cmd";
#endif

/* An incoming message, on the owner task: copy it into the mailbox and return. */
static void on_session_message(void *context, const uint8_t *topic, size_t topic_length,
                               const uint8_t *payload, size_t payload_length)
{
    (void)context;
    (void)mqtt_mailbox_rx_push(topic, (unsigned int)topic_length, payload, (unsigned int)payload_length);
}

enum {
    MQTT_DIAG_NONE = 0,
    MQTT_DIAG_NETWORK_DOWN = 1,
    MQTT_DIAG_REMOTE_FIN = 2,
    MQTT_DIAG_TCP_ERROR = 3,
    MQTT_DIAG_RX_ERROR = 4,
    MQTT_DIAG_OUTPUT_TIMEOUT = 5,
    MQTT_DIAG_SESSION_TIMEOUT = 6,
    MQTT_DIAG_TRANSPORT_FAILURE = 7
};

typedef struct {
    uint32_t valid;
    uint32_t reason;
    uint32_t at_ms;
    uint32_t state;
    uint32_t connects_started;
    uint32_t sessions_online;
    uint32_t owner_calls;
    uint32_t owner_max_gap_ms;
    uint32_t mqtt_polls;
    uint32_t mqtt_max_gap_ms;
    uint32_t last_poll_ms;
    uint32_t last_tx_ms;
    uint32_t ping_attempts;
    uint32_t pings;
    uint32_t ping_responses;
    uint32_t tcp_write_failures;
    uint32_t tcp_output_failures;
    uint32_t tcp_errors;
    int32_t last_tcp_error;
    uint32_t usb_irq_count;
    uint32_t usb_service_count;
    uint32_t usb_event_count;
    uint32_t usb_link_state;
    uint32_t wifi_link_changes;
    uint32_t dhcp_changes;
    uint32_t last_wifi_change_ms;
    uint32_t last_dhcp_change_ms;
    uint32_t join_attempts;
    uint32_t join_recoveries;
} mqtt_failure_snapshot_t;

typedef struct {
    struct tcp_pcb *pcb;
    mqtt_session_t session;
    ip4_addr_t broker;
    bool initialized;
    bool in_callback;
    bool aborted_in_callback;
    bool output_pending;
    uint32_t output_pending_since_ms;
    uint32_t published_online_session;
    /* TCP may have accepted publication while the queue mutex is busy.
     * Complete the pending commit before trying another publish. */
    bool pending_commit;
    uint32_t observed_failures;
    uint32_t diagnostic_session;
    uint32_t diagnostic_part;
    uint32_t poll_last_ms;
    uint32_t owner_last_ms;
    bool poll_seen;
    bool owner_seen;
    uint32_t wifi_link_up;
    uint32_t dhcp_complete;
    uint32_t join_attempts;
    uint32_t join_recoveries;
    mqtt_failure_snapshot_t previous;
    mqtt_heartbeat_t heartbeat;
} mqtt_lwip_adapter_t;

static mqtt_lwip_adapter_t adapter;
static T_MQTT_LWIP_STATUS diagnostics;

/* The diagnostic records are binary, all numbers big-endian.  Part 1 is 50 bytes and part 2 is 58 bytes, where the text form was
 * about 150 and 140.  The dashboard (server/parsers.js, parseDiagnostic) turns them back into the same named fields.
 *   0x23  part 1: boot nonce u32, reset reason u8, timer tick config u16, failure reason u8, failure time u32, mqtt state u8,
 *         then u32 each: owner calls, owner max gap, mqtt polls, mqtt max gap, last poll, last tx, ping attempts, pings, ping responses
 *   0x24  part 2: boot nonce u32, then u32 each: tcp write failures, tcp output failures, tcp errors, last tcp error (signed),
 *         usb irq, usb service, usb event; usb link state u8; then u32 each: wifi link changes, last wifi change,
 *         dhcp changes, last dhcp change, join attempts, join recoveries */
typedef struct {
    uint8_t *bytes;
    size_t capacity;
    size_t length;
    bool valid;
} mqtt_diag_builder_t;

static void diag_u8(mqtt_diag_builder_t *d, uint32_t value)
{
    if (d->length >= d->capacity) {
        d->valid = false;
        return;
    }
    d->bytes[d->length++] = (uint8_t)value;
}

static void diag_u16(mqtt_diag_builder_t *d, uint32_t value)
{
    diag_u8(d, value >> 8);
    diag_u8(d, value);
}

static void diag_u32(mqtt_diag_builder_t *d, uint32_t value)
{
    diag_u16(d, value >> 16);
    diag_u16(d, value);
}

static uint32_t make_boot_nonce(uint32_t now)
{
    uint32_t nonce = now ^ diagnostics.owner_calls ^ 0x4d515454U;
#if PICO_RP2040
    for (unsigned int i = 0; i < 32U; i++) {
        nonce = (nonce << 1) ^ (rosc_hw->randombit & 1U);
    }
    diagnostics.reset_reason = watchdog_hw->reason;
    /* Report only ENABLE and CYCLES; RUNNING/COUNT change while sampled. */
    diagnostics.timer_tick_config = watchdog_hw->tick & 0x3ffU;
#endif
    return nonce;
}

static void capture_failure(mqtt_lwip_adapter_t *a, uint32_t reason,
                            uint32_t now)
{
    mqtt_failure_snapshot_t *s = &a->previous;

    s->valid = 1U;
    s->reason = reason;
    s->at_ms = now;
    s->state = (uint32_t)a->session.state;
    s->connects_started = a->session.connects_started;
    s->sessions_online = a->session.sessions_online;
    s->owner_calls = diagnostics.owner_calls;
    s->owner_max_gap_ms = diagnostics.owner_max_gap_ms;
    s->mqtt_polls = a->session.poll_calls;
    s->mqtt_max_gap_ms = diagnostics.max_poll_gap_ms;
    s->last_poll_ms = a->session.last_poll_ms;
    s->last_tx_ms = a->session.last_tx_ms;
    s->ping_attempts = a->session.ping_attempts;
    s->pings = a->session.pings;
    s->ping_responses = a->session.ping_responses;
    s->tcp_write_failures = diagnostics.tcp_write_failures;
    s->tcp_output_failures = diagnostics.tcp_output_failures;
    s->tcp_errors = diagnostics.tcp_errors;
    s->last_tcp_error = diagnostics.last_tcp_error;
    s->usb_irq_count = diagnostics.usb_irq_count;
    s->usb_service_count = diagnostics.usb_service_count;
    s->usb_event_count = diagnostics.usb_event_count;
    s->usb_link_state = diagnostics.usb_link_state;
    s->wifi_link_changes = diagnostics.wifi_link_changes;
    s->dhcp_changes = diagnostics.dhcp_changes;
    s->last_wifi_change_ms = diagnostics.last_wifi_change_ms;
    s->last_dhcp_change_ms = diagnostics.last_dhcp_change_ms;
    s->join_attempts = a->join_attempts;
    s->join_recoveries = a->join_recoveries;
}

static void capture_if_new_failure(mqtt_lwip_adapter_t *a, uint32_t now,
                                   uint32_t reason)
{
    if (a->session.failures == a->observed_failures) return;
    a->observed_failures = a->session.failures;
    capture_failure(a, reason, now);
}

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

    a->output_pending = false;
    if (pcb == NULL) return;
    a->pcb = NULL;
    tcp_detach(pcb);
    tcp_abort(pcb);
    if (a->in_callback) a->aborted_in_callback = true;
}

static bool write_retryable(err_t result)
{
    return result == ERR_MEM || result == ERR_BUF ||
           result == ERR_WOULDBLOCK;
}

static mqtt_session_write_result_t write_tcp(void *context,
                                             const uint8_t *bytes,
                                             size_t size)
{
    mqtt_lwip_adapter_t *a = (mqtt_lwip_adapter_t *)context;
    err_t result;

    diagnostics.tcp_write_calls++;
    if (a->pcb == NULL) {
        diagnostics.last_tcp_write_result = ERR_CONN;
        diagnostics.tcp_write_failures++;
        diagnostics.tcp_write_fatal_errors++;
        return MQTT_SESSION_WRITE_FATAL;
    }
    if (bytes == NULL || size == 0U || size > 65535U) {
        diagnostics.last_tcp_write_result = ERR_ARG;
        diagnostics.tcp_write_failures++;
        diagnostics.tcp_write_fatal_errors++;
        return MQTT_SESSION_WRITE_FATAL;
    }
    if (a->output_pending) {
        diagnostics.last_tcp_write_result = ERR_WOULDBLOCK;
        diagnostics.tcp_write_failures++;
        return MQTT_SESSION_WRITE_RETRY;
    }
    if (size > (size_t)tcp_sndbuf(a->pcb)) {
        diagnostics.last_tcp_write_result = ERR_MEM;
        diagnostics.tcp_write_failures++;
        return MQTT_SESSION_WRITE_RETRY;
    }

    result = tcp_write(a->pcb, bytes, (u16_t)size, TCP_WRITE_FLAG_COPY);
    diagnostics.last_tcp_write_result = result;
    if (result != ERR_OK) {
        diagnostics.tcp_write_failures++;
        if (write_retryable(result)) return MQTT_SESSION_WRITE_RETRY;
        diagnostics.tcp_write_fatal_errors++;
        return MQTT_SESSION_WRITE_FATAL;
    }

    /* tcp_write(...COPY) has already queued all bytes. Once queued, even if
     * tcp_output is temporarily unable to transmit, returning false would
     * cause MQTT to enqueue a duplicate packet next time. lwIP retries. */
    result = tcp_output(a->pcb);
    diagnostics.last_tcp_output_result = result;
    if (result != ERR_OK) {
        diagnostics.tcp_output_failures++;
        a->output_pending = true;
        a->output_pending_since_ms = sys_now();
    }
    return MQTT_SESSION_WRITE_ACCEPTED;
}

static bool service_pending_output(mqtt_lwip_adapter_t *a, uint32_t now)
{
    err_t result;

    if (!a->output_pending || a->pcb == NULL) return true;
    diagnostics.tcp_output_retries++;
    result = tcp_output(a->pcb);
    diagnostics.last_tcp_output_result = result;
    if (result == ERR_OK) {
        a->output_pending = false;
        return true;
    }
    diagnostics.tcp_output_failures++;
    if ((uint32_t)(now - a->output_pending_since_ms) <
        AAI12_MQTT_OUTPUT_TIMEOUT_MS) {
        return true;
    }

    diagnostics.tcp_output_timeouts++;
    capture_failure(a, MQTT_DIAG_OUTPUT_TIMEOUT, now);
    close_tcp(a);
    mqtt_session_tcp_lost(&a->session, now);
    a->observed_failures = a->session.failures;
    return false;
}

/* On error lwIP ALREADY freed the PCB. Never tcp_close/tcp_abort here. */
static void on_error(void *context, err_t err)
{
    mqtt_lwip_adapter_t *a = (mqtt_lwip_adapter_t *)context;
    diagnostics.tcp_errors++;
    diagnostics.last_tcp_error = err;
    a->pcb = NULL;
    a->output_pending = false;
    diagnostics.disconnect_callbacks++;
    capture_failure(a, MQTT_DIAG_TCP_ERROR, sys_now());
    mqtt_session_tcp_lost(&a->session, sys_now());
    a->observed_failures = a->session.failures;
}

static err_t on_sent(void *context, struct tcp_pcb *pcb, u16_t len)
{
    mqtt_lwip_adapter_t *a = (mqtt_lwip_adapter_t *)context;

    if (pcb != a->pcb) return ERR_OK;
    diagnostics.tcp_sent_callbacks++;
    diagnostics.tcp_acked_bytes += len;
    diagnostics.last_tcp_ack_ms = sys_now();
    return ERR_OK;
}

static err_t on_connected(void *context, struct tcp_pcb *pcb, err_t err)
{
    mqtt_lwip_adapter_t *a = (mqtt_lwip_adapter_t *)context;

    if (pcb != a->pcb) return ERR_OK;
    if (err != ERR_OK) {
        diagnostics.disconnect_callbacks++;
        diagnostics.last_tcp_error = err;
        capture_failure(a, MQTT_DIAG_TCP_ERROR, sys_now());
        mqtt_session_tcp_lost(&a->session, sys_now());
        a->observed_failures = a->session.failures;
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
        a->output_pending = false;
        tcp_detach(pcb);
        close_result = tcp_close(pcb);
        if (close_result != ERR_OK) tcp_abort(pcb);
        diagnostics.disconnect_callbacks++;
        capture_failure(a, MQTT_DIAG_REMOTE_FIN, sys_now());
        mqtt_session_tcp_lost(&a->session, sys_now());
        a->observed_failures = a->session.failures;
        return (close_result == ERR_OK) ? ERR_OK : ERR_ABRT;
    }
    if (err != ERR_OK) {
        pbuf_free(p);
        diagnostics.disconnect_callbacks++;
        capture_failure(a, MQTT_DIAG_RX_ERROR, sys_now());
        close_tcp(a);
        mqtt_session_tcp_lost(&a->session, sys_now());
        a->observed_failures = a->session.failures;
        return ERR_ABRT;
    }

    total = p->tot_len;
    /* We accepted the full TCP payload into the bounded parser path. */
    tcp_recved(pcb, total);
    offset = 0U;
    a->aborted_in_callback = false;
    a->in_callback = true;
    while (offset < total) {
        uint32_t failures_before;
        u16_t count = (u16_t)(total - offset);
        if (count > AAI12_MQTT_RX_CHUNK) count = AAI12_MQTT_RX_CHUNK;
        if (pbuf_copy_partial(p, scratch, count, offset) != count) {
            diagnostics.disconnect_callbacks++;
            capture_failure(a, MQTT_DIAG_RX_ERROR, sys_now());
            close_tcp(a);
            mqtt_session_tcp_lost(&a->session, sys_now());
            a->observed_failures = a->session.failures;
            break;
        }
        failures_before = a->session.failures;
        mqtt_session_tcp_received(&a->session, scratch, count, sys_now());
        if (a->session.failures != failures_before) {
            capture_if_new_failure(a, sys_now(), MQTT_DIAG_RX_ERROR);
        }
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

static bool publish_with_diagnostics(mqtt_lwip_adapter_t *a,
                                     const char *topic,
                                     const uint8_t *payload, size_t length,
                                     uint32_t now)
{
    uint32_t failures_before = a->session.failures;
    bool accepted = mqtt_session_publish(&a->session, topic, payload,
                                         length, now);

    if (a->session.failures != failures_before) {
        capture_if_new_failure(a, now, MQTT_DIAG_TRANSPORT_FAILURE);
    }
    return accepted;
}

void mqtt_lwip_observe_owner(uint32_t wifi_link_up, uint32_t dhcp_complete,
                             uint32_t join_attempts,
                             uint32_t join_recoveries,
                             uint32_t usb_irq_count,
                             uint32_t usb_service_count,
                             uint32_t usb_event_count,
                             uint32_t usb_link_state)
{
    const uint32_t now = sys_now();
    uint32_t gap;

    diagnostics.owner_calls++;
    if (adapter.owner_seen) {
        gap = now - adapter.owner_last_ms;
        if (gap > diagnostics.owner_max_gap_ms) {
            diagnostics.owner_max_gap_ms = gap;
        }
    }
    adapter.owner_last_ms = now;
    adapter.owner_seen = true;
    diagnostics.owner_last_ms = now;
    diagnostics.usb_irq_count = usb_irq_count;
    diagnostics.usb_service_count = usb_service_count;
    diagnostics.usb_event_count = usb_event_count;
    diagnostics.usb_link_state = usb_link_state;
    if (wifi_link_up != adapter.wifi_link_up) {
        adapter.wifi_link_up = wifi_link_up;
        diagnostics.wifi_link_changes++;
        diagnostics.last_wifi_change_ms = now;
    }
    if (dhcp_complete != adapter.dhcp_complete) {
        adapter.dhcp_complete = dhcp_complete;
        diagnostics.dhcp_changes++;
        diagnostics.last_dhcp_change_ms = now;
    }
    adapter.join_attempts = join_attempts;
    adapter.join_recoveries = join_recoveries;
}

static bool publish_diagnostic(mqtt_lwip_adapter_t *a, uint32_t now)
{
    uint8_t payload[AAI12_MQTT_DIAG_PAYLOAD_BYTES];
    const mqtt_failure_snapshot_t *s = &a->previous;
    mqtt_diag_builder_t d = { payload, sizeof payload, 0U, true };

    if (!s->valid) {
        capture_failure(a, MQTT_DIAG_NONE, now);
    }
    if (a->diagnostic_part == 0U) {
        diag_u8(&d, AAI12_DIAG_PART1_TYPE);
        diag_u32(&d, diagnostics.boot_nonce);
        diag_u8(&d, diagnostics.reset_reason);
        diag_u16(&d, diagnostics.timer_tick_config);
        diag_u8(&d, s->reason);
        diag_u32(&d, s->at_ms);
        diag_u8(&d, s->state);
        diag_u32(&d, s->owner_calls);
        diag_u32(&d, s->owner_max_gap_ms);
        diag_u32(&d, s->mqtt_polls);
        diag_u32(&d, s->mqtt_max_gap_ms);
        diag_u32(&d, s->last_poll_ms);
        diag_u32(&d, s->last_tx_ms);
        diag_u32(&d, s->ping_attempts);
        diag_u32(&d, s->pings);
        diag_u32(&d, s->ping_responses);
    } else {
        diag_u8(&d, AAI12_DIAG_PART2_TYPE);
        diag_u32(&d, diagnostics.boot_nonce);
        diag_u32(&d, s->tcp_write_failures);
        diag_u32(&d, s->tcp_output_failures);
        diag_u32(&d, s->tcp_errors);
        diag_u32(&d, (uint32_t)s->last_tcp_error);
        diag_u32(&d, s->usb_irq_count);
        diag_u32(&d, s->usb_service_count);
        diag_u32(&d, s->usb_event_count);
        diag_u8(&d, s->usb_link_state);
        diag_u32(&d, s->wifi_link_changes);
        diag_u32(&d, s->last_wifi_change_ms);
        diag_u32(&d, s->dhcp_changes);
        diag_u32(&d, s->last_dhcp_change_ms);
        diag_u32(&d, s->join_attempts);
        diag_u32(&d, s->join_recoveries);
    }
    if (!d.valid ||
        !publish_with_diagnostics(a, diagnostic_topic,
                                  payload, d.length, now)) {
        diagnostics.diagnostic_publish_failures++;
        return false;
    }
    diagnostics.diagnostic_publishes++;
    a->diagnostic_part++;
    if (a->diagnostic_part == 2U) {
        a->diagnostic_session = a->session.sessions_online;
        a->diagnostic_part = 0U;
    }
    return true;
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
    uint32_t failures_before;
    uint32_t write_failures_before;
    uint32_t ping_attempts_before;
    uint32_t subscribes_before;
    uint32_t poll_gap;
    mqtt_session_state_t state_before;
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
#if AAI12_MQTT_SUBSCRIBE
        (void)mqtt_session_add_subscription(&adapter.session, subscribe_map_topic);
        (void)mqtt_session_add_subscription(&adapter.session, subscribe_cmd_topic);
#endif
        mqtt_session_set_message_handler(&adapter.session, on_session_message, &adapter);
        /* usermain() must initialize mutex and inbox before tasks start.
         * Do not reset a live queue or lazily create cross-core resources. */
        if (!telemetry_rtos_ready()) return;
        adapter.initialized = true;
        diagnostics.boot_nonce = make_boot_nonce(now);
        mqtt_heartbeat_init(&adapter.heartbeat, diagnostics.boot_nonce);
    }

    if (adapter.poll_seen) {
        poll_gap = now - adapter.poll_last_ms;
        if (poll_gap > diagnostics.max_poll_gap_ms) {
            diagnostics.max_poll_gap_ms = poll_gap;
        }
    }
    adapter.poll_last_ms = now;
    adapter.poll_seen = true;

    ready = netif != NULL && netif_is_up(netif) &&
            netif_is_link_up(netif) &&
            !ip4_addr_isany_val(*netif_ip4_addr(netif));
    if (ready != adapter.session.network_up) {
        if (!ready && adapter.session.network_up) {
            capture_failure(&adapter, MQTT_DIAG_NETWORK_DOWN, now);
        }
        mqtt_session_set_network(&adapter.session, ready, now);
    }
    if (!service_pending_output(&adapter, now)) return;
    failures_before = adapter.session.failures;
    write_failures_before = diagnostics.tcp_write_failures;
    ping_attempts_before = adapter.session.ping_attempts;
    subscribes_before = adapter.session.subscribes;
    state_before = adapter.session.state;
    mqtt_session_poll(&adapter.session, now);
    if (adapter.session.failures != failures_before) {
        capture_if_new_failure(&adapter, now,
            diagnostics.tcp_write_failures != write_failures_before
                ? MQTT_DIAG_TRANSPORT_FAILURE : MQTT_DIAG_SESSION_TIMEOUT);
        adapter.previous.state = (uint32_t)state_before;
    }
    mqtt_heartbeat_set_session(&adapter.heartbeat,
        adapter.session.state == MQTT_SESSION_ONLINE,
        adapter.session.sessions_online, now);
    /* Keep one MQTT control-packet attempt per owner iteration. A heartbeat
     * remains due and will be retried on the next pass after keepalive work. */
    if (adapter.session.ping_attempts != ping_attempts_before) return;
    if (adapter.session.subscribes != subscribes_before) return;      /* a SUBSCRIBE went out this pass */

    /* Smoke test ONLY: one QoS 0 message after each CONNACK (not real car
     * telemetry, no delivery acknowledgement). Retried if TX is full. */
    if (adapter.session.state == MQTT_SESSION_ONLINE &&
        adapter.session.sessions_online != adapter.published_online_session) {
        if (publish_with_diagnostics(&adapter, AAI12_MQTT_TEST_TOPIC,
                                     qualification_message,
                                     sizeof qualification_message,
                                     now)) {
            adapter.published_online_session = adapter.session.sessions_online;
        }
        /* Send the binary frame in a later poll, not the same tick. */
        return;
    }

    /* Preserve the established first online marker, then send two compact
     * failure records before ordinary telemetry.  At most one packet is
     * attempted per owner iteration. */
    if (adapter.session.state == MQTT_SESSION_ONLINE &&
        adapter.diagnostic_session != adapter.session.sessions_online) {
        (void)publish_diagnostic(&adapter, now);
        return;
    }

    /* Independent, fixed-size liveness publication. It runs on the existing
     * owner and before the robot queue, so queued sensor traffic cannot starve
     * it. A failed local TX is retried with the same sequence next iteration. */
    if (adapter.session.state == MQTT_SESSION_ONLINE &&
        mqtt_heartbeat_due(&adapter.heartbeat, now)) {
        uint8_t payload[MQTT_HEARTBEAT_PAYLOAD_BYTES];
        size_t length = mqtt_heartbeat_prepare(&adapter.heartbeat, now,
                                                payload, sizeof payload);
        if (length != 0U && publish_with_diagnostics(&adapter,
                heartbeat_topic, payload, length, now)) {
            mqtt_heartbeat_published(&adapter.heartbeat, now);
        }
        return;
    }

    /* A reply from the robot (the map acknowledgement): one message, sent ahead of ordinary telemetry. */
    if (adapter.session.state == MQTT_SESSION_ONLINE) {
        char reply_topic[MQTT_MAILBOX_TOPIC_BYTES + 1U];
        uint8_t reply[MQTT_MAILBOX_PAYLOAD_BYTES];
        unsigned int reply_length = 0U;

        if (mqtt_mailbox_tx_peek(reply_topic, reply, &reply_length) != 0U) {
            if (publish_with_diagnostics(&adapter, reply_topic, reply, reply_length, now)) {
                mqtt_mailbox_tx_done();
            }
            return;
        }
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
            publish_with_diagnostics(&adapter, robot_telemetry_topic,
                                     frame, length, now)) {
            /* TCP accepted the QoS 0 packet locally, not broker-ACKed.
             * Contended commit is retried without republishing. */
            if (telemetry_rtos_commit() != TELEMETRY_INBOX_OK) {
                adapter.pending_commit = true;
            }
        }
    }
}

void mqtt_lwip_get_status(T_MQTT_LWIP_STATUS *status)
{
    if (status == NULL) return;

    diagnostics.initialized = adapter.initialized;
    diagnostics.state = (uint32_t)adapter.session.state;
    diagnostics.tcp_active = adapter.session.tcp_active;
    diagnostics.connects_started = adapter.session.connects_started;
    diagnostics.sessions_online = adapter.session.sessions_online;
    diagnostics.session_failures = adapter.session.failures;
    diagnostics.poll_calls = adapter.session.poll_calls;
    diagnostics.last_poll_ms = adapter.session.last_poll_ms;
    diagnostics.last_tx_ms = adapter.session.last_tx_ms;
    diagnostics.ping_attempts = adapter.session.ping_attempts;
    diagnostics.pings_queued = adapter.session.pings;
    diagnostics.ping_responses = adapter.session.ping_responses;
    diagnostics.waiting_pingqueue = adapter.session.waiting_pingqueue;
    diagnostics.ping_queue_timeouts = adapter.session.ping_queue_timeouts;
    diagnostics.waiting_pingresp = adapter.session.waiting_pingresp;
    diagnostics.tcp_output_pending = adapter.output_pending;
    *status = diagnostics;
}
