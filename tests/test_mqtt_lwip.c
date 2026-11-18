/* Mock lwIP callback test: MQTT broker is simulated, not a hardware test. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "mqtt_lwip.h"
#include "mqtt_session.h"
#include "telemetry_rtos.h"
#include "telemetry_tkernel_port.h"
#include "mock_mutex_port.h"

static uint32_t clock_ms;
uint64_t time_us_64(void)
{
    return (uint64_t)clock_ms * 1000ULL;
}
static struct tcp_pcb pcb;
static void *callback_arg;
static err_t (*connected_handler)(void *, struct tcp_pcb *, err_t);
static err_t (*receive_handler)(void *, struct tcp_pcb *, struct pbuf *, err_t);
static err_t (*sent_handler)(void *, struct tcp_pcb *, u16_t);
static void (*error_handler)(void *, err_t);
static int connection_attempts;
static int abort_count;
static int send_count;
static int output_count;
static err_t mock_write_result = ERR_OK;
static err_t mock_output_result = ERR_OK;
static u16_t mock_tx_room = 1024U;
static uint8_t last_send[512];
static size_t last_send_len;

uint32_t sys_now(void) { return clock_ms; }
int ip4addr_aton(const char *text, ip4_addr_t *out)
{
    (void)text;
    out->addr = 0x01020304U;
    return 1;
}
struct tcp_pcb *tcp_new(void) { memset(&pcb, 0, sizeof pcb); return &pcb; }
void tcp_arg(struct tcp_pcb *p, void *arg)
{
    (void)p;
    callback_arg = arg;
}
void tcp_recv(struct tcp_pcb *p, err_t (*cb)(void *, struct tcp_pcb *, struct pbuf *, err_t))
{
    (void)p;
    receive_handler = cb;
}
void tcp_sent(struct tcp_pcb *p, err_t (*cb)(void *, struct tcp_pcb *, u16_t))
{
    (void)p;
    sent_handler = cb;
}
void tcp_err(struct tcp_pcb *p, void (*cb)(void *, err_t))
{
    (void)p;
    error_handler = cb;
}
void tcp_poll(struct tcp_pcb *p, err_t (*cb)(void *, struct tcp_pcb *), unsigned char interval)
{
    (void)p;
    (void)cb;
    (void)interval;
}
void tcp_abort(struct tcp_pcb *p)
{
    assert(p == &pcb);
    abort_count++;
}
err_t tcp_close(struct tcp_pcb *p)
{
    assert(p == &pcb);
    return ERR_OK;
}
err_t tcp_connect(struct tcp_pcb *p, const ip_addr_t *target, u16_t port,
                  err_t (*cb)(void *, struct tcp_pcb *, err_t))
{
    (void)target;
    assert(p == &pcb && port == 1883U);
    connected_handler = cb;
    connection_attempts++;
    return ERR_OK;
}
err_t tcp_write(struct tcp_pcb *p, const void *bytes, u16_t n, unsigned char options)
{
    assert(p == &pcb && options == TCP_WRITE_FLAG_COPY);
    if (mock_write_result != ERR_OK) return mock_write_result;
    assert(n <= sizeof last_send);
    memcpy(last_send, bytes, n);
    last_send_len = n;
    send_count++;
    return ERR_OK;
}
u16_t tcp_sndbuf(const struct tcp_pcb *p)
{
    assert(p == &pcb);
    return mock_tx_room;
}
err_t tcp_output(struct tcp_pcb *p)
{
    assert(p == &pcb);
    output_count++;
    return mock_output_result;
}
void tcp_recved(struct tcp_pcb *p, u16_t n)
{
    assert(p == &pcb && n > 0);
}
u16_t pbuf_copy_partial(const struct pbuf *p, void *out, u16_t n, u16_t off)
{
    if ((uint32_t)n + off > p->tot_len) return 0;
    memcpy(out, p->data + off, n);
    return n;
}
void pbuf_free(struct pbuf *p) { (void)p; }

static int contains_bytes(const uint8_t *haystack, size_t size,
                          const char *needle, size_t n)
{
    size_t i;
    if (n > size) return 0;
    for (i = 0; i <= size - n; i++) {
        if (memcmp(haystack + i, needle, n) == 0) return 1;
    }
    return 0;
}

static void simulate_recv(const uint8_t *bytes, size_t size)
{
    struct pbuf buffer;
    assert(receive_handler != NULL);
    buffer.data = bytes;
    buffer.tot_len = (u16_t)size;
    assert(receive_handler(callback_arg, &pcb, &buffer, ERR_OK) == ERR_OK);
}

int main(void)
{
    static const uint8_t connack_part1[] = { 0x20U, 0x02U };
    static const uint8_t connack_part2[] = { 0x00U, 0x00U };
    static const uint8_t pingresp[] = { 0xd0U, 0x00U };
    struct netif interface = {0};
    T_MQTT_LWIP_STATUS diagnostics;
    uint32_t write_failures_before;
    int sends_before_output_retry;
    void (*saved_error_handler)(void *, err_t);
    void *saved_callback_arg;

    interface.ip_addr.addr = 1U;
    interface.up = 1;
    interface.link = 1;
    mqtt_lwip_observe_owner(0U, 0U, 0U, 0U, 0U, 1U, 1U, 1U);
    /* Fail closed before usermain creates the shared inbox. */
    mqtt_lwip_poll(&interface);
    assert(connection_attempts == 0);
    /* Model usermain boot-time mutex creation. */
    assert(telemetry_rtos_boot_init() == 1);
    assert(telemetry_rtos_ready());
    mqtt_lwip_poll(&interface);
    assert(connection_attempts == 1);
    assert(connected_handler != NULL);
    assert(connected_handler(callback_arg, &pcb, ERR_OK) == ERR_OK);
    assert(send_count == 1 && last_send_len > 14U);
    assert(last_send[0] == 0x10U);    /* MQTT CONNECT */

    simulate_recv(connack_part1, sizeof connack_part1);
    assert(send_count == 1);         /* Fragmented CONNACK, still waiting */
    simulate_recv(connack_part2, sizeof connack_part2);
    mqtt_lwip_poll(&interface);
    assert(send_count == 2 && last_send[0] == 0x30U); /* MQTT PUBLISH */
    assert(contains_bytes(last_send, last_send_len, "AAI12_PICO_W_ONLINE", 18U));
    /* Simulate another robot component submitting telemetry. */
{
    static const uint8_t frame[] = {0x04U, 0x25U, 0x03U};
    assert(telemetry_rtos_submit(frame, sizeof frame, clock_ms)
           == TELEMETRY_INBOX_OK);
}
    /* The one-owner queue retains its message when the TCP send buffer
     * cannot accommodate the full MQTT PUBLISH packet. */
    mock_tx_room = 0U;
    mqtt_lwip_poll(&interface);
    assert(send_count == 2);            /* Peeked, not committed */
    clock_ms = 250U;                   /* Still fresh (< 1000ms) */
    mqtt_lwip_observe_owner(1U, 1U, 1U, 0U, 3U, 4U, 5U, 1U);
    mock_tx_room = 1024U;
    mqtt_lwip_poll(&interface);
    assert(send_count == 3 && last_send[0] == 0x30U); /* Diagnostic part 1 */
    assert(contains_bytes(last_send, last_send_len, "aai12/diagnostic",
                          sizeof("aai12/diagnostic") - 1U));
    mqtt_lwip_poll(&interface);
    assert(send_count == 4 && last_send[0] == 0x30U); /* Diagnostic part 2 */
    mqtt_lwip_poll(&interface);
    assert(send_count == 5 && last_send[0] == 0x30U); /* Queued PUBLISH */
    assert(contains_bytes(last_send, last_send_len,
                      "aai12/robot/telemetry",
                      sizeof("aai12/robot/telemetry") - 1U));
    assert(last_send_len >= 3U);
    assert(last_send[last_send_len - 3U] == 0x04U);
    assert(last_send[last_send_len - 2U] == 0x25U);
    assert(last_send[last_send_len - 1U] == 0x03U);
    mqtt_lwip_poll(&interface);
    assert(send_count == 5);         /* No duplicate binary frame */

    /* Model another RTOS task submitting a short payload, without lwIP. */
    {
        static const uint8_t external_frame[] = { 0xa5U, 0x5aU };
        assert(telemetry_rtos_submit(external_frame, sizeof external_frame,
                                     clock_ms) == TELEMETRY_INBOX_OK);
        /* First lock = peek, second lock = post-publish commit. Force
         * commit to report BUSY after the packet was queued to TCP. */
        mock_mutex_fail_after(2);
        mqtt_lwip_poll(&interface);
        assert(send_count == 6 && last_send[0] == 0x30U);
        assert(last_send[last_send_len - 2U] == 0xa5U);
        assert(last_send[last_send_len - 1U] == 0x5aU);
        mqtt_lwip_poll(&interface); /* pending commit only, NOT resend */
        assert(send_count == 6);
        mqtt_lwip_poll(&interface);
        assert(send_count == 6); /* No duplicate */
    }

    clock_ms = 20250U;
    mqtt_lwip_poll(&interface);
    assert(send_count == 7 && last_send[0] == 0xc0U);
    mqtt_lwip_get_status(&diagnostics);
    assert(diagnostics.initialized == 1U);
    assert(diagnostics.poll_calls > 0U);
    assert(diagnostics.last_poll_ms == clock_ms);
    assert(diagnostics.owner_calls == 2U);
    assert(diagnostics.owner_max_gap_ms == 250U);
    assert(diagnostics.usb_irq_count == 3U);
    assert(diagnostics.usb_service_count == 4U);
    assert(diagnostics.usb_event_count == 5U);
    assert(diagnostics.diagnostic_publishes == 2U);
    assert(diagnostics.state == MQTT_SESSION_ONLINE);
    assert(diagnostics.ping_attempts == 1U);
    assert(diagnostics.pings_queued == 1U);
    assert(diagnostics.waiting_pingresp == 1U);
    simulate_recv(pingresp, sizeof pingresp);
    mqtt_lwip_poll(&interface);
    assert(send_count == 7);         /* Ping response accepted */
    mqtt_lwip_get_status(&diagnostics);
    assert(diagnostics.ping_responses == 1U);
    assert(diagnostics.waiting_pingresp == 0U);
    assert(sent_handler != NULL);
    assert(sent_handler(callback_arg, &pcb, 2U) == ERR_OK);
    mqtt_lwip_get_status(&diagnostics);
    assert(diagnostics.tcp_sent_callbacks == 1U);
    assert(diagnostics.tcp_acked_bytes == 2U);
    assert(diagnostics.last_tcp_ack_ms == clock_ms);

    write_failures_before = diagnostics.tcp_write_failures;
    mock_write_result = ERR_MEM;
    clock_ms = 40251U;
    mqtt_lwip_poll(&interface);
    mqtt_lwip_get_status(&diagnostics);
    assert(diagnostics.ping_attempts == 2U);
    assert(diagnostics.pings_queued == 1U);
    assert(diagnostics.tcp_write_failures == write_failures_before + 1U);
    assert(diagnostics.last_tcp_write_result == ERR_MEM);
    mock_write_result = ERR_OK;

    mock_output_result = ERR_MEM;
    mqtt_lwip_poll(&interface);
    mqtt_lwip_get_status(&diagnostics);
    assert(diagnostics.pings_queued == 2U);
    assert(diagnostics.tcp_output_failures == 1U);
    assert(diagnostics.last_tcp_output_result == ERR_MEM);
    assert(diagnostics.tcp_output_pending == 1U);
    sends_before_output_retry = send_count;
    mock_output_result = ERR_OK;
    clock_ms = 40252U;
    mqtt_lwip_poll(&interface);
    assert(send_count == sends_before_output_retry);
    mqtt_lwip_get_status(&diagnostics);
    assert(diagnostics.tcp_output_pending == 0U);
    assert(diagnostics.tcp_output_retries == 1U);
    simulate_recv(pingresp, sizeof pingresp);

    interface.link = 0;
    mqtt_lwip_poll(&interface);
    assert(abort_count == 1);        /* Link loss releases PCB */
    interface.ip_addr.addr = 0U;     /* Owner invalidates the old DHCP lease. */
    interface.link = 1;
    clock_ms = 40300U;
    mqtt_lwip_poll(&interface);
    assert(connection_attempts == 1); /* No stale-address reconnect. */
    interface.ip_addr.addr = 1U;      /* Model a fresh DHCP bind. */
    mqtt_lwip_poll(&interface);
    assert(connection_attempts == 2); /* Can reconnect */
    assert(connected_handler(callback_arg, &pcb, ERR_OK) == ERR_OK);
    assert(send_count == 9 && last_send[0] == 0x10U); /* CONNECT */
    simulate_recv(connack_part1, sizeof connack_part1);
    simulate_recv(connack_part2, sizeof connack_part2);
    mqtt_lwip_poll(&interface);
    assert(send_count == 10);           /* Online marker on session 2 */
    mock_tx_room = 0U;
    mqtt_lwip_poll(&interface);
    assert(send_count == 10);           /* Diagnostic blocked by sndbuf */
    clock_ms = 41301U;                 /* Head now 1001ms old: stale */
    mock_tx_room = 1024U;
    mqtt_lwip_poll(&interface);
    assert(send_count == 11);           /* Previous-session diagnostic part 1 */
    assert(contains_bytes(last_send, last_send_len, "r=1", 3U));
    assert(contains_bytes(last_send, last_send_len, "og=250", 6U));
    mqtt_lwip_poll(&interface);
    assert(send_count == 12);           /* Previous-session diagnostic part 2 */
    assert(contains_bytes(last_send, last_send_len, "ui=3", 4U));
    mqtt_lwip_poll(&interface);
    assert(send_count == 12);           /* Stale item NOT published */
    mqtt_lwip_poll(&interface);
    assert(send_count == 12);           /* No accidental requeue */

    /* The recovered session must independently schedule its keepalive. */
    clock_ms = 61301U;
    mqtt_lwip_poll(&interface);
    assert(send_count == 13 && last_send[0] == 0xc0U);
    simulate_recv(pingresp, sizeof pingresp);

    /* A fatal tcp_write result closes the unusable PCB immediately. */
    mock_write_result = ERR_CONN;
    clock_ms = 81301U;
    mqtt_lwip_poll(&interface);
    assert(send_count == 13 && abort_count == 2);
    mqtt_lwip_get_status(&diagnostics);
    assert(diagnostics.state == MQTT_SESSION_BACKOFF);
    assert(diagnostics.last_tcp_write_result == ERR_CONN);
    assert(diagnostics.tcp_write_fatal_errors == 1U);
    mock_write_result = ERR_OK;

    /* Backoff reconnects, and a raw lwIP fatal callback also reconnects. */
    clock_ms = 83301U;
    mqtt_lwip_poll(&interface);
    assert(connection_attempts == 3);
    assert(connected_handler(callback_arg, &pcb, ERR_OK) == ERR_OK);
    simulate_recv(connack_part1, sizeof connack_part1);
    simulate_recv(connack_part2, sizeof connack_part2);
    mqtt_lwip_poll(&interface);
    assert(send_count == 15); /* CONNECT plus session-3 online marker. */
    saved_error_handler = error_handler;
    saved_callback_arg = callback_arg;
    assert(saved_error_handler != NULL);
    saved_error_handler(saved_callback_arg, ERR_RST);
    mqtt_lwip_get_status(&diagnostics);
    assert(diagnostics.state == MQTT_SESSION_BACKOFF);
    assert(diagnostics.tcp_errors == 1U);
    assert(diagnostics.last_tcp_error == ERR_RST);

    clock_ms = 85301U;
    mqtt_lwip_poll(&interface);
    assert(connection_attempts == 4);
    assert(connected_handler(callback_arg, &pcb, ERR_OK) == ERR_OK);
    simulate_recv(connack_part1, sizeof connack_part1);
    simulate_recv(connack_part2, sizeof connack_part2);
    mqtt_lwip_poll(&interface);
    assert(send_count == 17); /* CONNECT plus session-4 online marker. */
    mqtt_lwip_poll(&interface);
    assert(send_count == 18); /* Failed-session diagnostic part 1. */
    assert(contains_bytes(last_send, last_send_len, "r=3", 3U));
    mqtt_lwip_poll(&interface);
    assert(send_count == 19); /* Failed-session diagnostic part 2. */
    assert(contains_bytes(last_send, last_send_len, "te=1", 4U));

    /* A persistently failed tcp_output retries the queued bytes, never calls
     * tcp_write again, then abandons the connection after a bounded timeout. */
    mock_output_result = ERR_MEM;
    clock_ms = 105301U;
    mqtt_lwip_poll(&interface);
    assert(send_count == 20 && last_send[0] == 0xc0U);
    sends_before_output_retry = send_count;
    clock_ms = 110300U;
    mqtt_lwip_poll(&interface);
    assert(send_count == sends_before_output_retry);
    clock_ms = 110301U;
    mqtt_lwip_poll(&interface);
    assert(send_count == sends_before_output_retry && abort_count == 3);
    mqtt_lwip_get_status(&diagnostics);
    assert(diagnostics.state == MQTT_SESSION_BACKOFF);
    assert(diagnostics.tcp_output_timeouts == 1U);
    assert(diagnostics.tcp_output_pending == 0U);
    assert(diagnostics.connects_started == 4U);
    assert(diagnostics.sessions_online == 4U);
    assert(diagnostics.session_failures == 3U);
    assert(output_count > send_count);

    puts("PASS: MQTT lwIP backpressure/output/error/reconnect mock tests");
    return 0;
}
