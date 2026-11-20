#include "../app_program/mqtt_wire.h"

#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void test_connect_known_bytes(void)
{
    uint8_t buf[64] = {0U};
    static const uint8_t want[] = {
        0x10U, 0x11U, 0x00U, 0x04U, 'M', 'Q', 'T', 'T',
        0x04U, 0x02U, 0x00U, 0x1eU, 0x00U, 0x05U,
        'A', 'A', 'I', '1', '2'
    };
    size_t n = mqtt_wire_connect(buf, sizeof buf, "AAI12", 30U);
    assert(n == sizeof want);
    assert(memcmp(buf, want, n) == 0);
}

static void test_binary_publish(void)
{
    uint8_t buf[64] = {0U};
    static const uint8_t payload[] = {0x00U, 0xffU, 0x7fU};
    size_t n = mqtt_wire_publish_qos0(buf, sizeof buf, "aai12/telemetry",
                                      payload, sizeof payload);
    assert(n == 22U);
    assert(buf[0] == 0x30U && buf[1] == 20U);
    assert(buf[2] == 0U && buf[3] == 15U);
    assert(memcmp(buf + 4, "aai12/telemetry", 15U) == 0);
    assert(memcmp(buf + 19, payload, sizeof payload) == 0);
}

static void test_remaining_length_two_bytes(void)
{
    uint8_t buf[180] = {0U};
    uint8_t payload[116] = {0U};
    size_t n = mqtt_wire_publish_qos0(buf, sizeof buf, "aai12/telemetry",
                                      payload, sizeof payload);
    assert(n == 136U); /* 1 byte type + 2-byte Remaining Length + 133 */
    assert(buf[0] == 0x30U && buf[1] == 0x85U && buf[2] == 0x01U);
}

static void test_small_buffers(void)
{
    uint8_t buf[64];
    memset(buf, 0xa5, sizeof buf);
    assert(mqtt_wire_connect(buf, 18U, "AAI12", 30U) == 0U);
    assert(buf[0] == 0xa5U);
    assert(mqtt_wire_publish_qos0(buf, 21U, "aai12/telemetry", (const uint8_t *)"abc", 3U) == 0U);
    assert(buf[0] == 0xa5U);
}

static void test_invalid_args(void)
{
    uint8_t buf[32] = {0U};
    assert(mqtt_wire_connect(NULL, 20U, "a", 30U) == 0U);
    assert(mqtt_wire_connect(buf, 20U, "", 30U) == 0U);
    assert(mqtt_wire_publish_qos0(buf, sizeof buf, "a/#", (const uint8_t *)"", 0U) == 0U);
    assert(mqtt_wire_publish_qos0(buf, sizeof buf, "a/+", (const uint8_t *)"", 0U) == 0U);
    assert(mqtt_wire_publish_qos0(buf, sizeof buf, "a", NULL, 1U) == 0U);
    assert(mqtt_wire_publish_qos0(buf, sizeof buf, "a", NULL, SIZE_MAX) == 0U);
}

static void test_connack_success_and_refusal(void)
{
    uint8_t code = 0xffU;
    static const uint8_t yes[4] = {0x20U, 0x02U, 0x00U, 0x00U};
    static const uint8_t refused[4] = {0x20U, 0x02U, 0x00U, 0x05U};
    assert(mqtt_wire_parse_connack(yes, 4U, &code) && code == 0U);
    assert(mqtt_wire_parse_connack(refused, 4U, &code) && code == 5U);
}

static void test_malformed_connack(void)
{
    uint8_t code = 0x7aU;
    static const uint8_t bad_header[] = {0x21U, 0x02U, 0x00U, 0x00U};
    static const uint8_t bad_flags[] = {0x20U, 0x02U, 0x02U, 0x00U};
    static const uint8_t bad_refusal[] = {0x20U, 0x02U, 0x01U, 0x05U};
    static const uint8_t bad_code[] = {0x20U, 0x02U, 0x00U, 0x06U};
    assert(!mqtt_wire_parse_connack(bad_header, 4U, &code));
    assert(!mqtt_wire_parse_connack(bad_flags, 4U, &code));
    assert(!mqtt_wire_parse_connack(bad_refusal, 4U, &code));
    assert(!mqtt_wire_parse_connack(bad_code, 4U, &code));
    assert(!mqtt_wire_parse_connack(bad_code, 3U, &code));
    assert(code == 0x7aU);
}

static void test_ping_and_disconnect(void)
{
    uint8_t buf[2] = {0U};
    static const uint8_t ping_response[] = {0xd0U, 0x00U};
    assert(mqtt_wire_pingreq(buf, 2U) == 2U);
    assert(buf[0] == 0xc0U && buf[1] == 0U);
    assert(mqtt_wire_is_pingresp(ping_response, 2U));
    assert(!mqtt_wire_is_pingresp(buf, 2U));
    assert(mqtt_wire_disconnect(buf, 2U) == 2U);
    assert(buf[0] == 0xe0U && buf[1] == 0U);
    assert(mqtt_wire_disconnect(buf, 1U) == 0U);
}

static void test_subscribe_known_bytes(void)
{
    uint8_t buf[64];
    size_t n = mqtt_wire_subscribe(buf, sizeof buf, 1U, "aai12/robot/map");

    assert(n == 1U + 1U + 2U + 2U + 15U + 1U);
    assert(buf[0] == 0x82U && buf[1] == 2U + 2U + 15U + 1U);
    assert(buf[2] == 0U && buf[3] == 1U && buf[4] == 0U && buf[5] == 15U);
    assert(memcmp(&buf[6], "aai12/robot/map", 15U) == 0 && buf[21] == 0x00U);
    assert(mqtt_wire_subscribe(buf, 21U, 1U, "aai12/robot/map") == 0U);   /* one byte short */
    assert(mqtt_wire_subscribe(buf, sizeof buf, 0U, "x") == 0U);          /* packet id 0 is illegal */
    assert(mqtt_wire_subscribe(buf, sizeof buf, 1U, "") == 0U);
    assert(mqtt_wire_subscribe(buf, sizeof buf, 1U, NULL) == 0U);
}

static void test_suback_parse(void)
{
    const uint8_t ok[] = {0x90U, 0x03U, 0x00U, 0x02U, 0x00U};
    const uint8_t refused[] = {0x90U, 0x03U, 0x00U, 0x02U, 0x80U};
    const uint8_t bad_code[] = {0x90U, 0x03U, 0x00U, 0x02U, 0x03U};
    const uint8_t two_filters[] = {0x90U, 0x04U, 0x00U, 0x02U, 0x00U, 0x00U};
    uint16_t id = 0U;
    uint8_t code = 9U;

    assert(mqtt_wire_parse_suback(ok, sizeof ok, &id, &code) && id == 2U && code == 0U);
    assert(mqtt_wire_parse_suback(refused, sizeof refused, &id, &code) && code == 0x80U);
    assert(!mqtt_wire_parse_suback(bad_code, sizeof bad_code, &id, &code));
    assert(!mqtt_wire_parse_suback(two_filters, sizeof two_filters, &id, &code));
    assert(!mqtt_wire_parse_suback(ok, sizeof ok - 1U, &id, &code));
    assert(!mqtt_wire_parse_suback(NULL, 5U, &id, &code));
}

static void test_publish_qos0_parse(void)
{
    /* PUBLISH, retain set, topic "a/b", payload "A=L,F=U" */
    const uint8_t pkt[] = {0x31U, 0x0cU, 0x00U, 0x03U, 'a', '/', 'b',
                           'A', '=', 'L', ',', 'F', '=', 'U'};
    const uint8_t empty[] = {0x30U, 0x05U, 0x00U, 0x03U, 'a', '/', 'b'};
    const uint8_t qos1[] = {0x32U, 0x07U, 0x00U, 0x03U, 'a', '/', 'b', 0x00U, 0x01U};
    const uint8_t dup[] = {0x38U, 0x05U, 0x00U, 0x03U, 'a', '/', 'b'};
    const uint8_t short_topic[] = {0x30U, 0x04U, 0x00U, 0x09U, 'a', '/'};
    const uint8_t trailing[] = {0x30U, 0x05U, 0x00U, 0x03U, 'a', '/', 'b', 'X'};
    const uint8_t *topic = NULL;
    const uint8_t *payload = NULL;
    size_t tl = 0U;
    size_t pl = 0U;

    assert(mqtt_wire_parse_publish_qos0(pkt, sizeof pkt, &topic, &tl, &payload, &pl));
    assert(tl == 3U && memcmp(topic, "a/b", 3U) == 0 && pl == 7U && memcmp(payload, "A=L,F=U", 7U) == 0);
    assert(mqtt_wire_parse_publish_qos0(empty, sizeof empty, &topic, &tl, &payload, &pl) && pl == 0U);
    assert(!mqtt_wire_parse_publish_qos0(qos1, sizeof qos1, &topic, &tl, &payload, &pl));
    assert(!mqtt_wire_parse_publish_qos0(dup, sizeof dup, &topic, &tl, &payload, &pl));
    assert(!mqtt_wire_parse_publish_qos0(short_topic, sizeof short_topic, &topic, &tl, &payload, &pl));
    assert(!mqtt_wire_parse_publish_qos0(trailing, sizeof trailing, &topic, &tl, &payload, &pl));
    assert(!mqtt_wire_parse_publish_qos0(pkt, 3U, &topic, &tl, &payload, &pl));
}

int main(void)
{
    test_subscribe_known_bytes();
    test_suback_parse();
    test_publish_qos0_parse();
    test_connect_known_bytes();
    test_binary_publish();
    test_remaining_length_two_bytes();
    test_small_buffers();
    test_invalid_args();
    test_connack_success_and_refusal();
    test_malformed_connack();
    test_ping_and_disconnect();
    puts("PASS: 11 host-side MQTT 3.1.1 wire-format tests");
    return 0;
}
