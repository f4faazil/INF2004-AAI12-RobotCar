#include "mqtt_wire.h"

#include <string.h>

/* MQTT Remaining Length is a base-128 integer, maximum 268,435,455. */
#define MQTT_MAX_REMAINING_LENGTH UINT32_C(268435455)

static size_t mqtt_remaining_length_size(size_t value)
{
    size_t bytes = 1U;
    while (value >= 128U) {
        value /= 128U;
        bytes++;
    }
    return bytes;
}

static size_t mqtt_encode_remaining_length(uint8_t *out, size_t value)
{
    size_t offset = 0U;
    do {
        uint8_t digit = (uint8_t)(value % 128U);
        value /= 128U;
        if (value != 0U) {
            digit = (uint8_t)(digit | 0x80U);
        }
        out[offset++] = digit;
    } while (value != 0U);
    return offset;
}

static void mqtt_write_u16_be(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value >> 8);
    out[1] = (uint8_t)value;
}

static bool mqtt_topic_valid(const char *topic, size_t *length)
{
    const size_t n = (topic == NULL) ? 0U : strlen(topic);
    size_t i;
    if (n == 0U || n > UINT16_MAX) {
        return false;
    }
    for (i = 0U; i < n; i++) {
        if (topic[i] == '+' || topic[i] == '#') {
            return false;
        }
    }
    *length = n;
    return true;
}

size_t mqtt_wire_connect(uint8_t *out, size_t capacity,
                         const char *client_id, uint16_t keep_alive_seconds)
{
    size_t id_len;
    size_t remaining;
    size_t header_len;
    size_t offset;
    static const uint8_t variable_header[8] = {
        0x00U, 0x04U, 'M', 'Q', 'T', 'T', 0x04U, 0x02U
    };

    if (out == NULL || client_id == NULL) {
        return 0U;
    }
    id_len = strlen(client_id);
    if (id_len == 0U || id_len > UINT16_MAX) {
        return 0U;
    }
    remaining = 12U + id_len; /* 10-byte header + 2-byte ID length + ID */
    if (remaining > MQTT_MAX_REMAINING_LENGTH) {
        return 0U;
    }
    header_len = 1U + mqtt_remaining_length_size(remaining);
    if (capacity < header_len + remaining) {
        return 0U;
    }

    out[0] = 0x10U; /* CONNECT */
    offset = 1U + mqtt_encode_remaining_length(&out[1], remaining);
    memcpy(&out[offset], variable_header, sizeof variable_header);
    offset += sizeof variable_header;
    mqtt_write_u16_be(&out[offset], keep_alive_seconds);
    offset += 2U;
    mqtt_write_u16_be(&out[offset], (uint16_t)id_len);
    offset += 2U;
    memcpy(&out[offset], client_id, id_len);
    return offset + id_len;
}

size_t mqtt_wire_publish_qos0(uint8_t *out, size_t capacity,
                              const char *topic,
                              const uint8_t *payload, size_t payload_length)
{
    size_t topic_len;
    size_t remaining;
    size_t header_len;
    size_t offset;

    if (out == NULL || !mqtt_topic_valid(topic, &topic_len) ||
        (payload_length != 0U && payload == NULL)) {
        return 0U;
    }
    /* Check without size_t overflow, even with hostile lengths. */
    if (payload_length > (size_t)MQTT_MAX_REMAINING_LENGTH - 2U - topic_len) {
        return 0U;
    }
    remaining = 2U + topic_len + payload_length;
    header_len = 1U + mqtt_remaining_length_size(remaining);
    if (capacity < header_len + remaining) {
        return 0U;
    }

    out[0] = 0x30U; /* PUBLISH, QoS 0, retain 0 */
    offset = 1U + mqtt_encode_remaining_length(&out[1], remaining);
    mqtt_write_u16_be(&out[offset], (uint16_t)topic_len);
    offset += 2U;
    memcpy(&out[offset], topic, topic_len);
    offset += topic_len;
    if (payload_length != 0U) {
        memcpy(&out[offset], payload, payload_length);
    }
    return offset + payload_length;
}

size_t mqtt_wire_pingreq(uint8_t *out, size_t capacity)
{
    if (out == NULL || capacity < 2U) {
        return 0U;
    }
    out[0] = 0xc0U;
    out[1] = 0x00U;
    return 2U;
}

size_t mqtt_wire_disconnect(uint8_t *out, size_t capacity)
{
    if (out == NULL || capacity < 2U) {
        return 0U;
    }
    out[0] = 0xe0U;
    out[1] = 0x00U;
    return 2U;
}

bool mqtt_wire_parse_connack(const uint8_t *packet, size_t length,
                             uint8_t *return_code)
{
    if (packet == NULL || return_code == NULL || length != 4U ||
        packet[0] != 0x20U || packet[1] != 0x02U ||
        (packet[2] & 0xfeU) != 0U || packet[3] > 5U ||
        (packet[3] != 0U && packet[2] != 0U)) {
        return false;
    }
    *return_code = packet[3];
    return true;
}

bool mqtt_wire_is_pingresp(const uint8_t *packet, size_t length)
{
    return packet != NULL && length == 2U &&
           packet[0] == 0xd0U && packet[1] == 0x00U;
}

size_t mqtt_wire_subscribe(uint8_t *out, size_t capacity, uint16_t packet_id, const char *topic_filter)
{
    const size_t n = (topic_filter == NULL) ? 0U : strlen(topic_filter);
    size_t remaining;
    size_t header_len;
    size_t offset;

    if (out == NULL || n == 0U || n > UINT16_MAX || packet_id == 0U) {
        return 0U;
    }
    remaining = 2U + 2U + n + 1U;                       /* packet id, filter length, filter, requested QoS */
    header_len = 1U + mqtt_remaining_length_size(remaining);
    if (capacity < header_len + remaining) {
        return 0U;
    }
    out[0] = 0x82U;                                     /* SUBSCRIBE: the reserved flag bits must be 0010 */
    offset = 1U + mqtt_encode_remaining_length(&out[1], remaining);
    mqtt_write_u16_be(&out[offset], packet_id);
    offset += 2U;
    mqtt_write_u16_be(&out[offset], (uint16_t)n);
    offset += 2U;
    memcpy(&out[offset], topic_filter, n);
    offset += n;
    out[offset] = 0x00U;                                /* QoS 0 */
    return offset + 1U;
}

bool mqtt_wire_parse_suback(const uint8_t *packet, size_t length, uint16_t *packet_id, uint8_t *code)
{
    if (packet == NULL || packet_id == NULL || code == NULL || length != 5U ||
        packet[0] != 0x90U || packet[1] != 0x03U) {
        return false;
    }
    if (!(packet[4] <= 2U || packet[4] == 0x80U)) {
        return false;
    }
    *packet_id = (uint16_t)(((uint16_t)packet[2] << 8) | packet[3]);
    *code = packet[4];
    return true;
}

bool mqtt_wire_parse_publish_qos0(const uint8_t *packet, size_t length,
                                  const uint8_t **topic, size_t *topic_length,
                                  const uint8_t **payload, size_t *payload_length)
{
    size_t remaining = 0U;
    size_t multiplier = 1U;
    size_t i = 1U;
    size_t topic_len;

    if (packet == NULL || topic == NULL || topic_length == NULL || payload == NULL ||
        payload_length == NULL || length < 4U) {
        return false;
    }
    if ((packet[0] & 0xf0U) != 0x30U) {
        return false;
    }
    if ((packet[0] & 0x0eU) != 0U) {                    /* DUP or QoS 1/2: we only accept QoS 0 */
        return false;
    }
    for (;;) {
        uint8_t digit;

        if (i >= length || i > 4U) {
            return false;
        }
        digit = packet[i++];
        remaining += (size_t)(digit & 0x7fU) * multiplier;
        multiplier *= 128U;
        if ((digit & 0x80U) == 0U) {
            break;
        }
    }
    if (remaining != (length - i) || remaining < 2U) {
        return false;
    }
    topic_len = ((size_t)packet[i] << 8) | packet[i + 1U];
    if (topic_len == 0U || topic_len > (remaining - 2U)) {
        return false;
    }
    *topic = &packet[i + 2U];
    *topic_length = topic_len;
    *payload = &packet[i + 2U + topic_len];
    *payload_length = remaining - 2U - topic_len;
    return true;
}
