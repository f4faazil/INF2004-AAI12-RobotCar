/*
 * test_mqtt_client.c
 * Host tests for the MQTT client state machine against a scripted fake
 * network: connect, subscribe, publish, keep-alive, drops and the bounded
 * reconnect backoff.
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "mqtt_client.h"
#include "test_support.h"

/* ---- the fake network ------------------------------------------------- */

typedef struct {
    int      connect_result;        /* what tcp_connect returns */
    int      connect_calls;
    uint8_t  sent[1024];
    uint16_t sent_len;
    int      send_mode;             /* 0 ok, 1 error, 2 would block, 3 partial */
    uint8_t  rx[1024];
    uint16_t rx_len;
    uint16_t rx_pos;
    uint16_t rx_chunk;              /* bytes per recv (0 = as many as asked) */
    bool     rx_closed;
    int      closes;
    uint32_t last_connect_ms;
} fake_t;

static fake_t *g_fake;

static int f_connect(void *ctx)
{
    fake_t *f = (fake_t *)ctx;

    f->connect_calls++;
    return f->connect_result;
}

static int f_send(void *ctx, const uint8_t *d, uint16_t n)
{
    fake_t *f = (fake_t *)ctx;

    if (f->send_mode == 1) { return -1; }
    if (f->send_mode == 2) { return 0; }
    if (f->send_mode == 3) { return (int)(n / 2U); }
    memcpy(&f->sent[f->sent_len], d, n);
    f->sent_len = (uint16_t)(f->sent_len + n);
    return (int)n;
}

static int f_recv(void *ctx, uint8_t *buf, uint16_t cap)
{
    fake_t *f = (fake_t *)ctx;
    uint16_t avail = (uint16_t)(f->rx_len - f->rx_pos);
    uint16_t n = avail;

    if (f->rx_closed) { return -1; }
    if (f->rx_chunk != 0U && n > f->rx_chunk) { n = f->rx_chunk; }
    if (n > cap) { n = cap; }
    memcpy(buf, &f->rx[f->rx_pos], n);
    f->rx_pos = (uint16_t)(f->rx_pos + n);
    return (int)n;
}

static void f_close(void *ctx)
{
    ((fake_t *)ctx)->closes++;
}

static void feed(fake_t *f, const uint8_t *d, uint16_t n)
{
    memcpy(&f->rx[f->rx_len], d, n);
    f->rx_len = (uint16_t)(f->rx_len + n);
}

/* ---- the client under test -------------------------------------------- */

static char     g_topic[64];
static uint8_t  g_payload[64];
static uint16_t g_payload_len;
static int      g_messages;

static void on_msg(void *ctx, const char *topic, const uint8_t *payload, uint16_t len)
{
    (void)ctx;
    strncpy(g_topic, topic, sizeof(g_topic) - 1U);
    memcpy(g_payload, payload, len);
    g_payload_len = len;
    g_messages++;
}

typedef struct {
    mqtt_client_t c;
    fake_t        f;
    uint32_t      t;
} rig_t;

static const uint8_t CONNACK_OK[] = { 0x20, 0x02, 0x00, 0x00 };
static const uint8_t CONNACK_REFUSED[] = { 0x20, 0x02, 0x00, 0x05 };
static const uint8_t PINGRESP[] = { 0xD0, 0x00 };

static void rig_init(rig_t *r)
{
    mqtt_config_t cfg;
    mqtt_transport_t net;

    memset(r, 0, sizeof(*r));
    cfg.client_id = "robot";
    cfg.username = NULL;
    cfg.password = NULL;
    cfg.status_topic = "robot/status";
    cfg.command_topic = "robot/cmd";
    cfg.keepalive_s = 30U;
    cfg.connect_timeout_ms = 5000U;
    cfg.backoff_initial_ms = 1000U;
    cfg.backoff_max_ms = 8000U;
    cfg.on_message = on_msg;
    cfg.cb_ctx = NULL;
    net.tcp_connect = f_connect;
    net.tcp_send = f_send;
    net.tcp_recv = f_recv;
    net.tcp_close = f_close;
    net.ctx = &r->f;
    r->f.connect_result = 1;
    r->t = 1000U;
    g_messages = 0;
    g_fake = &r->f;
    mqtt_client_init(&r->c, &cfg, &net);
    mqtt_client_set_link(&r->c, true);
}

/* Run for 'ms', stepping every 100 ms. */
static void run(rig_t *r, uint32_t ms)
{
    uint32_t end = r->t + ms;

    while (r->t < end) {
        mqtt_client_step(&r->c, r->t);
        r->t += 100U;
    }
}

static void connect_ok(rig_t *r)
{
    mqtt_client_step(&r->c, r->t);
    feed(&r->f, CONNACK_OK, sizeof(CONNACK_OK));
    r->t += 100U;
    mqtt_client_step(&r->c, r->t);
    r->t += 100U;
}

/* ---- tests ------------------------------------------------------------ */

/* Connecting sends a CONNECT with the will, then on CONNACK subscribes to the
 * command topic and announces "online".  Catches: a missing will (the
 * laptop would never learn the robot died), no subscription (no commands). */
static void test_connect_subscribe_and_birth_message(void)
{
    rig_t r;
    uint8_t expect[128];
    uint16_t n;
    mqtt_connect_t cc = { "robot", 30U, "robot/status", (const uint8_t *)"offline", 7U, true,
                          NULL, NULL };

    rig_init(&r);
    mqtt_client_step(&r.c, r.t);
    CHECK_INT(MQTT_AWAIT_CONNACK, mqtt_client_state(&r.c));
    n = mqtt_encode_connect(&cc, expect, sizeof(expect));
    CHECK(n > 0U);
    CHECK(r.f.sent_len >= n);
    CHECK(memcmp(r.f.sent, expect, n) == 0);

    r.f.sent_len = 0U;
    feed(&r.f, CONNACK_OK, sizeof(CONNACK_OK));
    r.t += 100U;
    mqtt_client_step(&r.c, r.t);
    CHECK_INT(MQTT_CONNECTED, mqtt_client_state(&r.c));

    n = mqtt_encode_subscribe(1U, "robot/cmd", expect, sizeof(expect));
    CHECK(r.f.sent_len >= n);
    CHECK(memcmp(r.f.sent, expect, n) == 0);
    {
        uint8_t birth[64];
        uint16_t bn = mqtt_encode_publish("robot/status", (const uint8_t *)"online", 6U, true,
                                          birth, sizeof(birth));

        CHECK_INT((long)(n + bn), r.f.sent_len);
        CHECK(memcmp(&r.f.sent[n], birth, bn) == 0);
    }
}

/* Telemetry can be published once connected. */
static void test_publish_when_connected(void)
{
    rig_t r;
    uint8_t expect[64];
    uint16_t n;
    uint16_t before;

    rig_init(&r);
    connect_ok(&r);
    before = r.f.sent_len;
    CHECK(mqtt_client_publish(&r.c, "robot/t", (const uint8_t *)"hi", 2U, false));
    n = mqtt_encode_publish("robot/t", (const uint8_t *)"hi", 2U, false, expect, sizeof(expect));
    CHECK_INT((long)(before + n), r.f.sent_len);
    CHECK(memcmp(&r.f.sent[before], expect, n) == 0);
}

/* While disconnected, publishing is refused and counted - it never blocks and
 * nothing is queued inside the client.  Catches: a network outage backing
 * pressure into the control tasks (Tribunal rule). */
static void test_publish_while_offline_is_dropped_and_counted(void)
{
    rig_t r;

    rig_init(&r);
    r.f.connect_result = -1;
    mqtt_client_step(&r.c, r.t);
    CHECK(!mqtt_client_publish(&r.c, "robot/t", (const uint8_t *)"hi", 2U, false));
    CHECK(!mqtt_client_publish(&r.c, "robot/t", (const uint8_t *)"hi", 2U, false));
    CHECK_INT(2, r.c.dropped_offline);
    CHECK_INT(0, r.f.sent_len);
}

/* Failed connection attempts back off 1, 2, 4, 8, 8 s: bounded, not tight
 * loops against a dead broker.  Catches: hammering the network (and the CPU)
 * during an outage, and an unbounded backoff that would miss the 30 s
 * reconnect target. */
static void test_reconnect_backoff_is_exponential_and_bounded(void)
{
    rig_t r;
    uint32_t attempt_times[8];
    int n = 0;
    int last_calls = 0;
    uint32_t start;

    rig_init(&r);
    r.f.connect_result = -1;
    start = r.t;
    while ((r.t - start) < 30000U && n < 8) {
        mqtt_client_step(&r.c, r.t);
        if (r.f.connect_calls != last_calls) {
            attempt_times[n++] = r.t - start;
            last_calls = r.f.connect_calls;
        }
        r.t += 100U;
    }
    CHECK(n >= 6);
    CHECK_INT(1000, attempt_times[1] - attempt_times[0]);
    CHECK_INT(2000, attempt_times[2] - attempt_times[1]);
    CHECK_INT(4000, attempt_times[3] - attempt_times[2]);
    CHECK_INT(8000, attempt_times[4] - attempt_times[3]);
    CHECK_INT(8000, attempt_times[5] - attempt_times[4]);       /* capped */
}

/* A refused CONNACK (return code 5) drops the connection and backs off. */
static void test_refused_connack_disconnects_and_backs_off(void)
{
    rig_t r;

    rig_init(&r);
    mqtt_client_step(&r.c, r.t);
    feed(&r.f, CONNACK_REFUSED, sizeof(CONNACK_REFUSED));
    r.t += 100U;
    mqtt_client_step(&r.c, r.t);
    CHECK_INT(MQTT_DISCONNECTED, mqtt_client_state(&r.c));
    CHECK_INT(1, r.f.closes);
    CHECK_INT(1, r.c.failures);
}

/* No CONNACK within the timeout: give up the attempt. */
static void test_connack_timeout(void)
{
    rig_t r;

    rig_init(&r);
    mqtt_client_step(&r.c, r.t);
    run(&r, 4900U);
    CHECK_INT(MQTT_AWAIT_CONNACK, mqtt_client_state(&r.c));
    run(&r, 300U);
    CHECK_INT(MQTT_DISCONNECTED, mqtt_client_state(&r.c));
}

/* The broker/network disappears: the client notices, backs off and, when the
 * service returns, reconnects; backoff restarts at 1 s after success.  The
 * whole recovery completes within the Tribunal's 30 s target.  Catches: a
 * client that stays dead after one drop. */
static void test_recovers_after_an_outage_within_30_seconds(void)
{
    rig_t r;
    uint32_t outage_end;
    uint32_t back;

    rig_init(&r);
    connect_ok(&r);
    CHECK_INT(MQTT_CONNECTED, mqtt_client_state(&r.c));

    r.f.rx_closed = true;                       /* connection lost */
    r.f.connect_result = -1;                    /* and the network stays down */
    run(&r, 20000U);
    CHECK_INT(MQTT_DISCONNECTED, mqtt_client_state(&r.c));
    CHECK(r.c.failures >= 3U);

    outage_end = r.t;
    r.f.rx_closed = false;
    r.f.rx_len = 0U;
    r.f.rx_pos = 0U;
    r.f.connect_result = 1;
    feed(&r.f, CONNACK_OK, sizeof(CONNACK_OK));
    back = 0U;
    while ((back < 30000U) && (mqtt_client_state(&r.c) != MQTT_CONNECTED)) {
        mqtt_client_step(&r.c, r.t);
        r.t += 100U;
        back = r.t - outage_end;
    }
    CHECK_INT(MQTT_CONNECTED, mqtt_client_state(&r.c));
    CHECK(back <= 9000U);                       /* one capped backoff, not 30 s */
    CHECK_INT(1, r.c.reconnects);
}

/* After a successful connection the backoff starts from the beginning again. */
static void test_backoff_resets_after_success(void)
{
    rig_t r;
    uint32_t t_drop;

    rig_init(&r);
    r.f.connect_result = -1;
    run(&r, 12000U);                            /* several failures: backoff grown */
    r.f.connect_result = 1;
    feed(&r.f, CONNACK_OK, sizeof(CONNACK_OK));
    run(&r, 9000U);
    CHECK_INT(MQTT_CONNECTED, mqtt_client_state(&r.c));

    r.f.rx_closed = true;                       /* drop */
    t_drop = r.t;
    run(&r, 200U);
    CHECK_INT(MQTT_DISCONNECTED, mqtt_client_state(&r.c));
    r.f.rx_closed = false;
    r.f.rx_len = 0U;
    r.f.rx_pos = 0U;
    feed(&r.f, CONNACK_OK, sizeof(CONNACK_OK));
    run(&r, 1500U);                             /* only the initial 1 s backoff */
    CHECK_INT(MQTT_CONNECTED, mqtt_client_state(&r.c));
    CHECK(r.t - t_drop < 3000U);
}

/* An incoming PUBLISH on the command topic reaches the callback, even when it
 * arrives one byte at a time. */
static void test_incoming_command_is_delivered_even_if_fragmented(void)
{
    /* remaining length 12 = 2 (topic length) + 9 (topic) + 1 payload byte */
    static const uint8_t PUB[] = { 0x30, 0x0C, 0x00, 0x09, 'r', 'o', 'b', 'o', 't', '/', 'c', 'm',
                                   'd', 0x10 };
    rig_t r;
    int i;

    rig_init(&r);
    connect_ok(&r);
    r.f.rx_chunk = 1U;
    feed(&r.f, PUB, sizeof(PUB));
    for (i = 0; i < 30; i++) {
        mqtt_client_step(&r.c, r.t);
        r.t += 100U;
    }
    CHECK_INT(1, g_messages);
    CHECK(strcmp(g_topic, "robot/cmd") == 0);
    CHECK_INT(1, g_payload_len);
    CHECK_INT(0x10, g_payload[0]);
    CHECK_INT(1, r.c.messages_received);
}

/* Idle for half the keep-alive: a PINGREQ goes out; PINGRESP keeps the
 * connection.  No PINGRESP within the keep-alive: the connection is dropped.
 * Catches: a half-open connection the client believes is alive forever. */
static void test_keepalive_ping_and_timeout(void)
{
    rig_t r;
    uint16_t before;
    uint32_t t0;

    rig_init(&r);
    connect_ok(&r);
    before = r.f.sent_len;
    run(&r, 14000U);
    CHECK_INT((long)before, r.f.sent_len);       /* still within keepalive/2 */
    run(&r, 2000U);
    CHECK_INT((long)(before + 2), r.f.sent_len); /* PINGREQ C0 00 */
    CHECK_INT(0xC0, r.f.sent[before]);

    feed(&r.f, PINGRESP, sizeof(PINGRESP));
    run(&r, 200U);
    CHECK_INT(MQTT_CONNECTED, mqtt_client_state(&r.c));

    t0 = r.t;
    run(&r, 40000U);                             /* the broker goes silent */
    CHECK(r.c.failures >= 1U);                   /* the dead connection was dropped */
    CHECK(r.f.closes >= 1);
    CHECK(r.t > t0);
}

/* A socket error while publishing drops the connection (so it will be
 * rebuilt) rather than leaving a broken stream. */
static void test_send_error_drops_the_connection(void)
{
    rig_t r;

    rig_init(&r);
    connect_ok(&r);
    r.f.send_mode = 1;
    CHECK(!mqtt_client_publish(&r.c, "robot/t", (const uint8_t *)"hi", 2U, false));
    CHECK_INT(MQTT_DISCONNECTED, mqtt_client_state(&r.c));
}

/* A socket that cannot take the packet right now (would block) drops THAT
 * packet and keeps the connection - the stream is still in sync. */
static void test_would_block_drops_the_packet_not_the_connection(void)
{
    rig_t r;

    rig_init(&r);
    connect_ok(&r);
    r.f.send_mode = 2;
    CHECK(!mqtt_client_publish(&r.c, "robot/t", (const uint8_t *)"hi", 2U, false));
    CHECK_INT(MQTT_CONNECTED, mqtt_client_state(&r.c));
    CHECK_INT(1, r.c.dropped_busy);
}

/* A partial write would desynchronise the stream: reconnect. */
static void test_partial_write_drops_the_connection(void)
{
    rig_t r;

    rig_init(&r);
    connect_ok(&r);
    r.f.send_mode = 3;
    CHECK(!mqtt_client_publish(&r.c, "robot/t", (const uint8_t *)"hello", 5U, false));
    CHECK_INT(MQTT_DISCONNECTED, mqtt_client_state(&r.c));
}

/* With the WiFi link down no connection is attempted and no failures are
 * counted; when the link returns the client connects at once. */
static void test_link_down_pauses_attempts_without_failures(void)
{
    rig_t r;

    rig_init(&r);
    mqtt_client_set_link(&r.c, false);
    run(&r, 10000U);
    CHECK_INT(0, r.f.connect_calls);
    CHECK_INT(0, r.c.failures);
    mqtt_client_set_link(&r.c, true);
    mqtt_client_step(&r.c, r.t);
    CHECK_INT(1, r.f.connect_calls);
}

/* Losing the link while connected tears the connection down cleanly. */
static void test_link_loss_while_connected_disconnects(void)
{
    rig_t r;

    rig_init(&r);
    connect_ok(&r);
    mqtt_client_set_link(&r.c, false);
    mqtt_client_step(&r.c, r.t);
    CHECK_INT(MQTT_DISCONNECTED, mqtt_client_state(&r.c));
    CHECK_INT(1, r.f.closes);
    CHECK_INT(0, r.c.failures);
}

/* A pending TCP connect (returns 0) is polled until it completes or times
 * out; the step never blocks. */
static void test_pending_tcp_connect_is_polled_then_times_out(void)
{
    rig_t r;

    rig_init(&r);
    r.f.connect_result = 0;
    run(&r, 3000U);
    CHECK_INT(MQTT_TCP_CONNECTING, mqtt_client_state(&r.c));
    CHECK(r.f.connect_calls > 5);
    run(&r, 3000U);                             /* past connect_timeout_ms */
    CHECK_INT(MQTT_DISCONNECTED, mqtt_client_state(&r.c));
}

int main(void)
{
    RUN(test_connect_subscribe_and_birth_message);
    RUN(test_publish_when_connected);
    RUN(test_publish_while_offline_is_dropped_and_counted);
    RUN(test_reconnect_backoff_is_exponential_and_bounded);
    RUN(test_refused_connack_disconnects_and_backs_off);
    RUN(test_connack_timeout);
    RUN(test_recovers_after_an_outage_within_30_seconds);
    RUN(test_backoff_resets_after_success);
    RUN(test_incoming_command_is_delivered_even_if_fragmented);
    RUN(test_keepalive_ping_and_timeout);
    RUN(test_send_error_drops_the_connection);
    RUN(test_would_block_drops_the_packet_not_the_connection);
    RUN(test_partial_write_drops_the_connection);
    RUN(test_link_down_pauses_attempts_without_failures);
    RUN(test_link_loss_while_connected_disconnects);
    RUN(test_pending_tcp_connect_is_polled_then_times_out);
    return TEST_REPORT();
}
