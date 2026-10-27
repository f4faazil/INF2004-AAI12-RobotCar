#include "../app_program/mqtt_stream.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void fragmented_connack(void)
{
    mqtt_stream_t s;
    size_t n;
    const uint8_t response[] = {0x20U, 0x02U, 0x00U, 0x00U};
    size_t i;
    mqtt_stream_init(&s);
    for (i = 0U; i < 3U; ++i) {
        assert(mqtt_stream_feed(&s, &response[i], 1U, &n) == MQTT_STREAM_MORE);
        assert(n == 1U);
    }
    assert(mqtt_stream_feed(&s, &response[3], 1U, &n) == MQTT_STREAM_PACKET);
    assert(mqtt_stream_packet_length(&s) == sizeof response);
    assert(memcmp(mqtt_stream_packet(&s), response, sizeof response) == 0);
}

static void coalesced_packets(void)
{
    mqtt_stream_t s;
    const uint8_t joined[] = {0xd0U, 0x00U, 0x20U, 0x02U, 0x00U, 0x00U};
    size_t n;
    mqtt_stream_init(&s);
    assert(mqtt_stream_feed(&s, joined, sizeof joined, &n) == MQTT_STREAM_PACKET);
    assert(n == 2U);
    assert(mqtt_stream_packet_length(&s) == 2U);
    mqtt_stream_reset(&s);
    assert(mqtt_stream_feed(&s, joined + n, sizeof joined - n, &n) == MQTT_STREAM_PACKET);
    assert(n == 4U);
    assert(mqtt_stream_packet(&s)[0] == 0x20U);
}

static void split_multibyte_length_and_binary_payload(void)
{
    mqtt_stream_t s;
    uint8_t packet[136];
    size_t n;
    size_t i;
    /* MQTT PUBLISH: Remaining Length = 2 + 1 topic + 130 payload = 133 */
    packet[0] = 0x30U;
    packet[1] = 0x85U; /* 133 encoded as base-128 varint */
    packet[2] = 0x01U;
    packet[3] = 0x00U;
    packet[4] = 0x01U;
    packet[5] = 's';
    for (i = 6U; i < sizeof packet; ++i) {
        packet[i] = (uint8_t)(i & 0xffU);
    }
    packet[10] = 0U; /* Binary MQTT payload can contain NUL */
    mqtt_stream_init(&s);
    assert(mqtt_stream_feed(&s, packet, 2U, &n) == MQTT_STREAM_MORE);
    assert(mqtt_stream_feed(&s, packet + 2U, 1U, &n) == MQTT_STREAM_MORE);
    assert(mqtt_stream_feed(&s, packet + 3U, sizeof packet - 3U, &n) == MQTT_STREAM_PACKET);
    assert(n == sizeof packet - 3U);
    assert(mqtt_stream_packet_length(&s) == sizeof packet);
    assert(memcmp(mqtt_stream_packet(&s), packet, sizeof packet) == 0);
}

static void reject_large_length(void)
{
    mqtt_stream_t s;
    size_t n;
    const uint8_t hdr[] = {0x30U, 0xffU, 0x02U}; /* Remaining Length 383 */
    mqtt_stream_init(&s);
    assert(mqtt_stream_feed(&s, hdr, sizeof hdr, &n) == MQTT_STREAM_ERROR);
    assert(n == sizeof hdr);
    assert(mqtt_stream_packet(&s) == NULL);
}

static void reject_five_byte_remaining_length(void)
{
    mqtt_stream_t s;
    size_t n;
    const uint8_t hdr[] = {0x30U, 0x80U, 0x80U, 0x80U, 0x80U};
    mqtt_stream_init(&s);
    assert(mqtt_stream_feed(&s, hdr, sizeof hdr, &n) == MQTT_STREAM_ERROR);
    assert(n == sizeof hdr);
}

static void reject_reserved_type(void)
{
    mqtt_stream_t s;
    size_t n;
    const uint8_t hdr[] = {0x00U, 0x00U};
    mqtt_stream_init(&s);
    assert(mqtt_stream_feed(&s, hdr, sizeof hdr, &n) == MQTT_STREAM_ERROR);
}

static void reset_after_error(void)
{
    mqtt_stream_t s;
    size_t n;
    const uint8_t invalid[] = {0x30U, 0xffU, 0x02U};
    const uint8_t ping[] = {0xd0U, 0x00U};
    mqtt_stream_init(&s);
    assert(mqtt_stream_feed(&s, invalid, sizeof invalid, &n) == MQTT_STREAM_ERROR);
    assert(mqtt_stream_feed(&s, ping, sizeof ping, &n) == MQTT_STREAM_ERROR);
    mqtt_stream_reset(&s);
    assert(mqtt_stream_feed(&s, ping, sizeof ping, &n) == MQTT_STREAM_PACKET);
}

static void packet_must_be_consumed(void)
{
    mqtt_stream_t s;
    size_t n;
    const uint8_t ping[] = {0xd0U, 0x00U};
    mqtt_stream_init(&s);
    assert(mqtt_stream_feed(&s, ping, sizeof ping, &n) == MQTT_STREAM_PACKET);
    assert(mqtt_stream_feed(&s, ping, sizeof ping, &n) == MQTT_STREAM_ERROR);
    assert(n == 0U);
    mqtt_stream_reset(&s);
    assert(mqtt_stream_packet(&s) == NULL);
}

static void null_arguments(void)
{
    mqtt_stream_t s;
    size_t n = 99U;
    mqtt_stream_init(&s);
    assert(mqtt_stream_feed(NULL, NULL, 0U, &n) == MQTT_STREAM_ERROR);
    assert(n == 0U);
    assert(mqtt_stream_feed(&s, NULL, 1U, &n) == MQTT_STREAM_ERROR);
    assert(n == 0U);
    assert(mqtt_stream_feed(&s, NULL, 0U, &n) == MQTT_STREAM_MORE);
    assert(mqtt_stream_feed(&s, NULL, 0U, NULL) == MQTT_STREAM_ERROR);
}

static void header_only_split_then_complete(void)
{
    mqtt_stream_t s;
    size_t n;
    const uint8_t ping[] = {0xd0U, 0x00U};
    mqtt_stream_init(&s);
    assert(mqtt_stream_feed(&s, ping, 1U, &n) == MQTT_STREAM_MORE);
    assert(mqtt_stream_feed(&s, ping + 1U, 1U, &n) == MQTT_STREAM_PACKET);
    assert(mqtt_stream_packet_length(&s) == 2U);
}

int main(void)
{
    fragmented_connack();
    coalesced_packets();
    split_multibyte_length_and_binary_payload();
    reject_large_length();
    reject_five_byte_remaining_length();
    reject_reserved_type();
    reset_after_error();
    packet_must_be_consumed();
    null_arguments();
    header_only_split_then_complete();
    puts("PASS: 10 MQTT TCP-stream framing tests");
    return 0;
}
