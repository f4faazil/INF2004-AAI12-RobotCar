#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../app_program/mqtt_session.h"

typedef struct {
    unsigned starts;
    unsigned sends;
    unsigned closes;
    bool allow_connect;
    mqtt_session_write_result_t write_result;
    uint8_t last[MQTT_SESSION_TX_BYTES];
    size_t last_len;
} mock_t;

static bool mock_connect(void *ptr)
{
    mock_t *m = ptr;
    m->starts++;
    return m->allow_connect;
}
static mqtt_session_write_result_t mock_write(void *ptr,
                                              const uint8_t *packet,
                                              size_t length)
{
    mock_t *m = ptr;
    m->sends++;
    if (m->write_result != MQTT_SESSION_WRITE_ACCEPTED ||
        length > sizeof m->last) return m->write_result;
    memcpy(m->last, packet, length);
    m->last_len = length;
    return MQTT_SESSION_WRITE_ACCEPTED;
}
static void mock_close(void *ptr)
{
    mock_t *m = ptr;
    m->closes++;
}
static mqtt_session_transport_t ops(void)
{
    mqtt_session_transport_t t = {mock_connect, mock_write, mock_close};
    return t;
}
static void setup(mqtt_session_t *s, mock_t *m)
{
    const mqtt_session_transport_t t = ops();
    memset(m, 0, sizeof *m);
    m->allow_connect = true;
    m->write_result = MQTT_SESSION_WRITE_ACCEPTED;
    assert(mqtt_session_init(s, &t, m, "aai12-picow"));
}
static void connected(mqtt_session_t *s)
{
    mqtt_session_set_network(s, true, 100U);
    mqtt_session_poll(s, 100U);
    assert(s->state == MQTT_SESSION_CONNECTING);
    mqtt_session_tcp_connected(s, 120U);
    assert(s->state == MQTT_SESSION_WAIT_CONNACK);
}
static void online(mqtt_session_t *s)
{
    const uint8_t ack[] = {0x20U, 0x02U, 0x00U, 0x00U};
    connected(s);
    mqtt_session_tcp_received(s, ack, sizeof ack, 130U);
    assert(s->state == MQTT_SESSION_ONLINE);
}
static void test_validation(void)
{
    mqtt_session_t s;
    mock_t m;
    mqtt_session_transport_t t = ops();
    char long_id[MQTT_SESSION_CLIENT_ID_BYTES + 2U];
    setup(&s, &m);
    assert(!mqtt_session_init(&s, NULL, &m, "hi"));
    assert(!mqtt_session_init(&s, &t, &m, ""));
    memset(long_id, 'a', sizeof long_id);
    long_id[sizeof long_id - 1U] = '\0';
    assert(!mqtt_session_init(&s, &t, &m, long_id));
    t.write_tcp = NULL;
    assert(!mqtt_session_init(&s, &t, &m, "ok"));
}
static void test_connect_and_fragment(void)
{
    mqtt_session_t s;
    mock_t m;
    const uint8_t ack[] = {0x20U, 0x02U, 0x00U, 0x00U};
    setup(&s, &m);
    assert(s.state == MQTT_SESSION_WAIT_NETWORK);
    mqtt_session_poll(&s, 0U);
    assert(m.starts == 0U);
    connected(&s);
    assert(m.starts == 1U && m.last[0] == 0x10U);
    mqtt_session_tcp_received(&s, ack, 2U, 125U);
    assert(s.state == MQTT_SESSION_WAIT_CONNACK);
    mqtt_session_tcp_received(&s, ack + 2U, 2U, 130U);
    assert(s.state == MQTT_SESSION_ONLINE && s.sessions_online == 1U);
}
static void test_rejected_ack(void)
{
    mqtt_session_t s;
    mock_t m;
    const uint8_t ack[] = {0x20U, 0x02U, 0x00U, 0x05U};
    setup(&s, &m);
    connected(&s);
    mqtt_session_tcp_received(&s, ack, sizeof ack, 130U);
    assert(s.state == MQTT_SESSION_BACKOFF && m.closes == 1U);
    mqtt_session_poll(&s, 2129U);
    assert(m.starts == 1U);
    mqtt_session_poll(&s, 2130U);
    assert(m.starts == 2U && s.state == MQTT_SESSION_CONNECTING);
}
static void test_bad_ack(void)
{
    mqtt_session_t s;
    mock_t m;
    const uint8_t bad[] = {0x20U, 0x02U, 0x01U, 0x00U};
    setup(&s, &m);
    connected(&s);
    mqtt_session_tcp_received(&s, bad, sizeof bad, 130U);
    assert(s.state == MQTT_SESSION_BACKOFF && m.closes == 1U);
}
static void test_connect_timeout(void)
{
    mqtt_session_t s;
    mock_t m;
    setup(&s, &m);
    mqtt_session_set_network(&s, true, 10U);
    mqtt_session_poll(&s, 10U);
    mqtt_session_poll(&s, 10009U);
    assert(s.state == MQTT_SESSION_CONNECTING);
    mqtt_session_poll(&s, 10010U);
    assert(s.state == MQTT_SESSION_BACKOFF && m.closes == 1U);
}
static void test_connack_timeout(void)
{
    mqtt_session_t s;
    mock_t m;
    setup(&s, &m);
    connected(&s);
    mqtt_session_poll(&s, 5119U);
    assert(s.state == MQTT_SESSION_WAIT_CONNACK);
    mqtt_session_poll(&s, 5120U);
    assert(s.state == MQTT_SESSION_BACKOFF && m.closes == 1U);
}
static void test_publish_binary_and_failure(void)
{
    mqtt_session_t s;
    mock_t m;
    const uint8_t payload[] = {0x00U, 0xffU, 0x17U};
    setup(&s, &m);
    online(&s);
    m.write_result = MQTT_SESSION_WRITE_RETRY;
    assert(!mqtt_session_publish(&s, "aai12/car/status", payload,
                                 sizeof payload, 250U));
    assert(s.publishes == 0U);
    m.write_result = MQTT_SESSION_WRITE_ACCEPTED;
    assert(mqtt_session_publish(&s, "aai12/car/status", payload,
                                sizeof payload, 300U));
    assert(s.publishes == 1U && m.last[0] == 0x30U);
    assert(memcmp(m.last + m.last_len - sizeof payload,
                  payload, sizeof payload) == 0);
    assert(!mqtt_session_publish(&s, "bad/#", payload, 3U, 310U));
}
static void test_ping_and_reply(void)
{
    mqtt_session_t s;
    mock_t m;
    const uint8_t pingresp[] = {0xd0U, 0x00U};
    setup(&s, &m);
    online(&s);
    mqtt_session_poll(&s, 20119U);
    assert(!s.waiting_pingresp);
    mqtt_session_poll(&s, 20120U);
    assert(s.waiting_pingresp && m.last_len == 2U && m.last[0] == 0xc0U);
    assert(s.ping_attempts == 1U && s.pings == 1U);
    mqtt_session_tcp_received(&s, pingresp, 1U, 20121U);
    assert(s.waiting_pingresp);
    mqtt_session_tcp_received(&s, pingresp + 1U, 1U, 20122U);
    assert(!s.waiting_pingresp && s.state == MQTT_SESSION_ONLINE);
    assert(s.ping_responses == 1U);
}
static void test_ping_timeout(void)
{
    mqtt_session_t s;
    mock_t m;
    setup(&s, &m);
    online(&s);
    mqtt_session_poll(&s, 20120U);
    mqtt_session_poll(&s, 25119U);
    assert(s.state == MQTT_SESSION_ONLINE);
    mqtt_session_poll(&s, 25120U);
    assert(s.state == MQTT_SESSION_BACKOFF && m.closes == 1U);
}

static void test_ping_backpressure_recovers(void)
{
    mqtt_session_t s;
    mock_t m;

    setup(&s, &m);
    online(&s);
    m.write_result = MQTT_SESSION_WRITE_RETRY;
    mqtt_session_poll(&s, 20120U);
    assert(s.state == MQTT_SESSION_ONLINE);
    assert(s.waiting_pingqueue && !s.waiting_pingresp);
    assert(s.ping_attempts == 1U && s.pings == 0U);

    m.write_result = MQTT_SESSION_WRITE_ACCEPTED;
    mqtt_session_poll(&s, 21120U);
    assert(s.state == MQTT_SESSION_ONLINE);
    assert(!s.waiting_pingqueue && s.waiting_pingresp);
    assert(s.ping_attempts == 2U && s.pings == 1U);
    assert(s.ping_queue_timeouts == 0U && m.closes == 0U);
}

static void test_ping_backpressure_timeout(void)
{
    mqtt_session_t s;
    mock_t m;

    setup(&s, &m);
    online(&s);
    m.write_result = MQTT_SESSION_WRITE_RETRY;
    mqtt_session_poll(&s, 20120U);
    mqtt_session_poll(&s, 25119U);
    assert(s.state == MQTT_SESSION_ONLINE && s.waiting_pingqueue);
    mqtt_session_poll(&s, 25120U);
    assert(s.state == MQTT_SESSION_BACKOFF);
    assert(!s.waiting_pingqueue && !s.waiting_pingresp);
    assert(s.ping_queue_timeouts == 1U && m.closes == 1U);
}

static void test_fatal_write_reconnect_keepalive(void)
{
    mqtt_session_t s;
    mock_t m;
    const uint8_t ack[] = {0x20U, 0x02U, 0x00U, 0x00U};

    setup(&s, &m);
    online(&s);
    m.write_result = MQTT_SESSION_WRITE_FATAL;
    mqtt_session_poll(&s, 20120U);
    assert(s.state == MQTT_SESSION_BACKOFF && m.closes == 1U);
    assert(s.pings == 0U && !s.waiting_pingqueue);

    m.write_result = MQTT_SESSION_WRITE_ACCEPTED;
    mqtt_session_poll(&s, 22119U);
    assert(s.state == MQTT_SESSION_BACKOFF);
    mqtt_session_poll(&s, 22120U);
    assert(s.state == MQTT_SESSION_CONNECTING && s.connects_started == 2U);
    mqtt_session_tcp_connected(&s, 22130U);
    mqtt_session_tcp_received(&s, ack, sizeof ack, 22140U);
    assert(s.state == MQTT_SESSION_ONLINE && s.sessions_online == 2U);

    mqtt_session_poll(&s, 42129U);
    assert(s.pings == 0U);
    mqtt_session_poll(&s, 42130U);
    assert(s.state == MQTT_SESSION_ONLINE && s.waiting_pingresp);
    assert(s.pings == 1U && m.last[0] == 0xc0U);
}
static void test_lost_error_no_double_close(void)
{
    mqtt_session_t s;
    mock_t m;
    setup(&s, &m);
    online(&s);
    mqtt_session_tcp_lost(&s, 1000U);
    assert(m.closes == 0U && s.state == MQTT_SESSION_BACKOFF);
    mqtt_session_tcp_lost(&s, 1001U);
    assert(m.closes == 0U);
    mqtt_session_set_network(&s, false, 1100U);
    assert(s.state == MQTT_SESSION_WAIT_NETWORK);
    mqtt_session_set_network(&s, true, 1200U);
    mqtt_session_poll(&s, 1200U);
    assert(s.state == MQTT_SESSION_CONNECTING && m.starts == 2U);
}
static void test_network_drop_closes(void)
{
    mqtt_session_t s;
    mock_t m;
    setup(&s, &m);
    online(&s);
    mqtt_session_set_network(&s, false, 500U);
    assert(m.closes == 1U && s.state == MQTT_SESSION_WAIT_NETWORK);
    mqtt_session_poll(&s, 1000U);
    assert(m.starts == 1U);
}
static void test_connect_unavailable(void)
{
    mqtt_session_t s;
    mock_t m;
    setup(&s, &m);
    m.allow_connect = false;
    mqtt_session_set_network(&s, true, 0U);
    mqtt_session_poll(&s, 0U);
    assert(s.state == MQTT_SESSION_BACKOFF && m.closes == 0U);
}
static void test_wraparound(void)
{
    mqtt_session_t s;
    mock_t m;
    setup(&s, &m);
    mqtt_session_set_network(&s, true, UINT32_MAX - 4U);
    mqtt_session_poll(&s, UINT32_MAX - 4U);
    mqtt_session_poll(&s, 9994U); /* elapsed 9999 ms */
    assert(s.state == MQTT_SESSION_CONNECTING);
    mqtt_session_poll(&s, 9995U); /* elapsed 10000 ms */
    assert(s.state == MQTT_SESSION_BACKOFF);
}

/* The observed headless reconnect cadence was 131.072 seconds.  That value
 * resembles a 17-bit millisecond wrap, so prove it has no special meaning to
 * the 32-bit, subtraction-based keepalive state machine. */
static void test_131_second_timestamp_still_pings(void)
{
    mqtt_session_t s;
    mock_t m;

    setup(&s, &m);
    online(&s);
    mqtt_session_poll(&s, 131072U);
    assert(s.state == MQTT_SESSION_ONLINE);
    assert(s.waiting_pingresp && s.pings == 1U);
    assert(m.last_len == 2U && m.last[0] == 0xc0U);
}

static void test_keepalive_across_clock_wrap(void)
{
    mqtt_session_t s;
    mock_t m;
    const uint8_t payload[] = {0x01U};
    const uint32_t sent = UINT32_MAX - 10000U;

    setup(&s, &m);
    online(&s);
    assert(mqtt_session_publish(&s, "aai12/test", payload,
                                sizeof payload, sent));
    mqtt_session_poll(&s, 9998U); /* 19,999 ms after the publish */
    assert(!s.waiting_pingresp);
    mqtt_session_poll(&s, 9999U); /* 20,000 ms after the publish */
    assert(s.waiting_pingresp && s.pings == 1U);
    assert(m.last_len == 2U && m.last[0] == 0xc0U);
}

int main(void)
{
    test_validation();
    test_connect_and_fragment();
    test_rejected_ack();
    test_bad_ack();
    test_connect_timeout();
    test_connack_timeout();
    test_publish_binary_and_failure();
    test_ping_and_reply();
    test_ping_timeout();
    test_ping_backpressure_recovers();
    test_ping_backpressure_timeout();
    test_fatal_write_reconnect_keepalive();
    test_lost_error_no_double_close();
    test_network_drop_closes();
    test_connect_unavailable();
    test_wraparound();
    test_131_second_timestamp_still_pings();
    test_keepalive_across_clock_wrap();
    puts("PASS: 18 MQTT session state-machine tests");
    return 0;
}
