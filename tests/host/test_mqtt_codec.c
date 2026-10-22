/*
 * test_mqtt_codec.c
 * Host tests for the MQTT 3.1.1 codec.  Expected bytes are worked out from
 * the OASIS MQTT 3.1.1 specification (fixed header, remaining-length
 * encoding, variable headers), not copied from the implementation.
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "mqtt_codec.h"
#include "test_support.h"

static void check_bytes(const uint8_t *expected, size_t n, const uint8_t *actual, uint16_t got)
{
    CHECK_INT((long)n, got);
    CHECK(memcmp(expected, actual, n) == 0);
}

/* CONNECT: protocol "MQTT" level 4, clean session, keepalive 60, client id
 * "abc".  Remaining length = 6 + 1 + 1 + 2 + (2 + 3) = 15. */
static void test_connect_packet(void)
{
    static const uint8_t EXPECTED[] = { 0x10, 0x0F, 0x00, 0x04, 'M', 'Q', 'T', 'T', 0x04,
                                        0x02, 0x00, 0x3C, 0x00, 0x03, 'a', 'b', 'c' };
    uint8_t out[64];
    mqtt_connect_t c = { "abc", 60U, NULL, NULL, 0U, false, NULL, NULL };

    check_bytes(EXPECTED, sizeof(EXPECTED), out, mqtt_encode_connect(&c, out, sizeof(out)));
}

/* CONNECT with a will: flags = clean(0x02) | will(0x04) | will-retain(0x20);
 * the will topic and message follow the client id.  Remaining length =
 * 10 (variable header) + 3 (client id) + 5 (will topic) + 4 (will message). */
static void test_connect_with_will(void)
{
    static const uint8_t EXPECTED[] = {
        0x10, 0x16, 0x00, 0x04, 'M', 'Q', 'T', 'T', 0x04, 0x26, 0x00, 0x1E,
        0x00, 0x01, 'r',
        0x00, 0x03, 's', '/', 'x',
        0x00, 0x02, 'o', 'f' };
    uint8_t out[64];
    static const uint8_t will[] = { 'o', 'f' };
    mqtt_connect_t c = { "r", 30U, "s/x", will, 2U, true, NULL, NULL };

    check_bytes(EXPECTED, sizeof(EXPECTED), out, mqtt_encode_connect(&c, out, sizeof(out)));
}

/* Username and password set flags 0x80 and 0x40. */
static void test_connect_with_credentials(void)
{
    uint8_t out[64];
    mqtt_connect_t c = { "id", 10U, NULL, NULL, 0U, false, "u", "p" };
    uint16_t n = mqtt_encode_connect(&c, out, sizeof(out));

    CHECK(n > 0U);
    CHECK_INT(0xC2, out[9]);                    /* user | pass | clean */
    CHECK_INT('p', out[n - 1U]);
}

/* PUBLISH QoS 0: topic "a/b", payload "hi". */
static void test_publish_packet(void)
{
    static const uint8_t EXPECTED[] = { 0x30, 0x07, 0x00, 0x03, 'a', '/', 'b', 'h', 'i' };
    static const uint8_t payload[] = { 'h', 'i' };
    uint8_t out[32];

    check_bytes(EXPECTED, sizeof(EXPECTED), out,
                mqtt_encode_publish("a/b", payload, 2U, false, out, sizeof(out)));
    CHECK_INT(0x31, (mqtt_encode_publish("a/b", payload, 2U, true, out, sizeof(out)), out[0]));
}

/* The remaining length uses the continuation-bit encoding: a 130-byte payload
 * makes the remaining length 135 = 0x87 0x01.  Catches: a one-byte length,
 * which would corrupt any larger telemetry packet. */
static void test_remaining_length_uses_two_bytes_above_127(void)
{
    uint8_t payload[130];
    uint8_t out[160];
    uint16_t n;

    memset(payload, 0x55, sizeof(payload));
    n = mqtt_encode_publish("a/b", payload, 130U, false, out, sizeof(out));
    CHECK_INT(1 + 2 + 2 + 3 + 130, n);
    CHECK_INT(0x30, out[0]);
    CHECK_INT(0x87, out[1]);
    CHECK_INT(0x01, out[2]);
}

/* SUBSCRIBE: packet id 1, topic "a/b", QoS 0. Fixed header flags are 0x2. */
static void test_subscribe_packet(void)
{
    static const uint8_t EXPECTED[] = { 0x82, 0x08, 0x00, 0x01, 0x00, 0x03, 'a', '/', 'b', 0x00 };
    uint8_t out[32];

    check_bytes(EXPECTED, sizeof(EXPECTED), out, mqtt_encode_subscribe(1U, "a/b", out, sizeof(out)));
}

static void test_pingreq_and_disconnect(void)
{
    static const uint8_t PING[] = { 0xC0, 0x00 };
    static const uint8_t DISC[] = { 0xE0, 0x00 };
    uint8_t out[8];

    check_bytes(PING, 2U, out, mqtt_encode_pingreq(out, sizeof(out)));
    check_bytes(DISC, 2U, out, mqtt_encode_disconnect(out, sizeof(out)));
}

/* A packet that does not fit yields 0 rather than a truncated packet. */
static void test_too_small_buffer_encodes_nothing(void)
{
    uint8_t out[4];
    mqtt_connect_t c = { "abc", 60U, NULL, NULL, 0U, false, NULL, NULL };

    CHECK_INT(0, mqtt_encode_connect(&c, out, sizeof(out)));
    CHECK_INT(0, mqtt_encode_publish("a/b", (const uint8_t *)"hi", 2U, false, out, sizeof(out)));
}

/* ---- decoder ---------------------------------------------------------- */

static mqtt_packet_kind_t feed_all(mqtt_parser_t *p, const uint8_t *d, uint16_t n, mqtt_packet_t *pkt)
{
    uint16_t used = 0U;
    mqtt_packet_kind_t k = MQTT_PKT_NONE;

    while ((used < n) && (k == MQTT_PKT_NONE)) {
        uint16_t c = 0U;

        k = mqtt_parser_feed(p, &d[used], (uint16_t)(n - used), &c, pkt);
        used = (uint16_t)(used + c);
    }
    return k;
}

static void test_decode_connack(void)
{
    static const uint8_t ACCEPTED[] = { 0x20, 0x02, 0x00, 0x00 };
    static const uint8_t REFUSED[] = { 0x20, 0x02, 0x00, 0x05 };
    mqtt_parser_t p;
    mqtt_packet_t pkt;

    mqtt_parser_init(&p);
    CHECK_INT(MQTT_PKT_CONNACK, feed_all(&p, ACCEPTED, 4U, &pkt));
    CHECK_INT(0, pkt.connack_code);
    CHECK_INT(MQTT_PKT_CONNACK, feed_all(&p, REFUSED, 4U, &pkt));
    CHECK_INT(5, pkt.connack_code);
}

static void test_decode_suback_and_pingresp(void)
{
    static const uint8_t SUBACK[] = { 0x90, 0x03, 0x00, 0x01, 0x00 };
    static const uint8_t PINGRESP[] = { 0xD0, 0x00 };
    mqtt_parser_t p;
    mqtt_packet_t pkt;

    mqtt_parser_init(&p);
    CHECK_INT(MQTT_PKT_SUBACK, feed_all(&p, SUBACK, 5U, &pkt));
    CHECK_INT(1, pkt.packet_id);
    CHECK_INT(0, pkt.suback_code);
    CHECK_INT(MQTT_PKT_PINGRESP, feed_all(&p, PINGRESP, 2U, &pkt));
}

/* An incoming PUBLISH gives topic and payload. */
static void test_decode_publish(void)
{
    static const uint8_t PUB[] = { 0x30, 0x0A, 0x00, 0x05, 'r', '/', 'c', 'm', 'd', 0x10, 0x09, 0x21 };
    mqtt_parser_t p;
    mqtt_packet_t pkt;

    mqtt_parser_init(&p);
    CHECK_INT(MQTT_PKT_PUBLISH, feed_all(&p, PUB, sizeof(PUB), &pkt));
    CHECK(strcmp(pkt.topic, "r/cmd") == 0);
    CHECK_INT(3, pkt.payload_len);
    CHECK_INT(0x10, pkt.payload[0]);
    CHECK_INT(0x21, pkt.payload[2]);
    CHECK_INT(0, pkt.qos);
}

/* A packet delivered one byte at a time parses the same.  Catches: a parser
 * that assumes a whole packet per TCP read (TCP is a byte stream). */
static void test_fragmented_stream(void)
{
    static const uint8_t PUB[] = { 0x30, 0x0A, 0x00, 0x05, 'r', '/', 'c', 'm', 'd', 0x10, 0x09, 0x21 };
    mqtt_parser_t p;
    mqtt_packet_t pkt;
    mqtt_packet_kind_t k = MQTT_PKT_NONE;
    size_t i;

    mqtt_parser_init(&p);
    for (i = 0U; i < sizeof(PUB); i++) {
        uint16_t used = 0U;

        k = mqtt_parser_feed(&p, &PUB[i], 1U, &used, &pkt);
        CHECK_INT(1, used);
        if (i + 1U < sizeof(PUB)) {
            CHECK_INT(MQTT_PKT_NONE, k);
        }
    }
    CHECK_INT(MQTT_PKT_PUBLISH, k);
    CHECK(strcmp(pkt.topic, "r/cmd") == 0);
}

/* Two packets in one read: feeding returns the first and reports how much it
 * used so the second is not lost. */
static void test_two_packets_in_one_read(void)
{
    static const uint8_t TWO[] = { 0xD0, 0x00, 0x20, 0x02, 0x00, 0x00 };
    mqtt_parser_t p;
    mqtt_packet_t pkt;
    uint16_t used = 0U;

    mqtt_parser_init(&p);
    CHECK_INT(MQTT_PKT_PINGRESP, mqtt_parser_feed(&p, TWO, sizeof(TWO), &used, &pkt));
    CHECK_INT(2, used);
    CHECK_INT(MQTT_PKT_CONNACK, mqtt_parser_feed(&p, &TWO[used], (uint16_t)(sizeof(TWO) - used),
                                                 &used, &pkt));
}

/* A packet larger than the buffer is skipped without overflow, and the stream
 * stays in sync for the next one.  Catches: a long packet corrupting memory or
 * desynchronising the stream. */
static void test_oversize_packet_is_skipped_and_stream_stays_in_sync(void)
{
    uint8_t big[3 + 300 + 2];
    mqtt_parser_t p;
    mqtt_packet_t pkt;
    mqtt_packet_kind_t k;
    static const uint8_t PING[] = { 0xD0, 0x00 };

    memset(big, 0x41, sizeof(big));
    big[0] = 0x30;                              /* PUBLISH */
    big[1] = (uint8_t)(300 & 0x7F) | 0x80U;     /* remaining length 300 */
    big[2] = (uint8_t)(300 >> 7);
    mqtt_parser_init(&p);
    k = feed_all(&p, big, (uint16_t)(3 + 300), &pkt);
    CHECK_INT(MQTT_PKT_OTHER, k);
    CHECK_INT(1, p.skipped_packets);
    CHECK_INT(MQTT_PKT_PINGRESP, feed_all(&p, PING, 2U, &pkt));
}

/* A packet whose length exceeds 16 bits is still skipped correctly: the skip
 * counter must be wide enough.  Catches: a 16-bit counter wrapping and the
 * parser never resynchronising. */
static void test_a_very_large_packet_is_skipped_to_its_end(void)
{
    static const uint8_t HEAD[] = { 0x30, 0xF0, 0xA2, 0x04 };      /* 70000 bytes */
    uint8_t chunk[1000];
    static const uint8_t PING[] = { 0xD0, 0x00 };
    mqtt_parser_t p;
    mqtt_packet_t pkt;
    mqtt_packet_kind_t k;
    unsigned sent;

    memset(chunk, 0x41, sizeof(chunk));
    mqtt_parser_init(&p);
    CHECK_INT(MQTT_PKT_NONE, feed_all(&p, HEAD, sizeof(HEAD), &pkt));
    k = MQTT_PKT_NONE;
    for (sent = 0U; sent < 70000U; sent += 1000U) {
        k = feed_all(&p, chunk, 1000U, &pkt);
        if (sent + 1000U < 70000U) {
            CHECK_INT(MQTT_PKT_NONE, k);
        }
    }
    CHECK_INT(MQTT_PKT_OTHER, k);
    CHECK_INT(MQTT_PKT_PINGRESP, feed_all(&p, PING, 2U, &pkt));
}

/* A malformed remaining length (5 continuation bytes) is an error. */
static void test_malformed_remaining_length_is_an_error(void)
{
    static const uint8_t BAD[] = { 0x30, 0x80, 0x80, 0x80, 0x80, 0x01 };
    mqtt_parser_t p;
    mqtt_packet_t pkt;

    mqtt_parser_init(&p);
    CHECK_INT(MQTT_PKT_ERROR, feed_all(&p, BAD, sizeof(BAD), &pkt));
}

/* A PUBLISH whose topic length runs past the packet is malformed. */
static void test_publish_with_a_bad_topic_length_is_an_error(void)
{
    static const uint8_t BAD[] = { 0x30, 0x04, 0x00, 0x09, 'a', 'b' };
    mqtt_parser_t p;
    mqtt_packet_t pkt;

    mqtt_parser_init(&p);
    CHECK_INT(MQTT_PKT_ERROR, feed_all(&p, BAD, sizeof(BAD), &pkt));
}

/* Packet types the robot does not use are reported as OTHER, not as errors. */
static void test_unused_packet_types_are_other(void)
{
    static const uint8_t PUBACK[] = { 0x40, 0x02, 0x00, 0x07 };
    mqtt_parser_t p;
    mqtt_packet_t pkt;

    mqtt_parser_init(&p);
    CHECK_INT(MQTT_PKT_OTHER, feed_all(&p, PUBACK, sizeof(PUBACK), &pkt));
}

int main(void)
{
    RUN(test_connect_packet);
    RUN(test_connect_with_will);
    RUN(test_connect_with_credentials);
    RUN(test_publish_packet);
    RUN(test_remaining_length_uses_two_bytes_above_127);
    RUN(test_subscribe_packet);
    RUN(test_pingreq_and_disconnect);
    RUN(test_too_small_buffer_encodes_nothing);
    RUN(test_decode_connack);
    RUN(test_decode_suback_and_pingresp);
    RUN(test_decode_publish);
    RUN(test_fragmented_stream);
    RUN(test_two_packets_in_one_read);
    RUN(test_oversize_packet_is_skipped_and_stream_stays_in_sync);
    RUN(test_a_very_large_packet_is_skipped_to_its_end);
    RUN(test_malformed_remaining_length_is_an_error);
    RUN(test_publish_with_a_bad_topic_length_is_an_error);
    RUN(test_unused_packet_types_are_other);
    return TEST_REPORT();
}
