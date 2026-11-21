#include "mqtt_stream.h"

#include <string.h>

void mqtt_stream_init(mqtt_stream_t *stream)
{
    if (stream != NULL) {
        memset(stream, 0, sizeof *stream);
    }
}

void mqtt_stream_reset(mqtt_stream_t *stream)
{
    if (stream != NULL) {
        stream->used = 0U;
        stream->expected = 0U;
        stream->state = 0U;
    }
}

/* Decode the 1-to-4 byte MQTT Remaining Length once enough bytes arrive. */
static int check_header(mqtt_stream_t *stream)
{
    uint32_t remaining = 0U;
    uint32_t multiplier = 1U;
    size_t i;

    if (stream->used < 2U) {
        return 0;
    }
    if ((stream->bytes[0] >> 4) == 0U ||
        (stream->bytes[0] >> 4) == 15U) {
        return -1; /* Reserved MQTT control packet types */
    }

    for (i = 1U; i < stream->used && i <= 4U; i++) {
        const uint8_t encoded = stream->bytes[i];
        remaining += (uint32_t)(encoded & 0x7fU) * multiplier;
        if ((encoded & 0x80U) == 0U) {
            const size_t header_bytes = 1U + i;
            if (remaining > MQTT_STREAM_BUFFER_BYTES - header_bytes) {
                return -1; /* Limit buffer and refuse unbounded packets */
            }
            stream->expected = header_bytes + (size_t)remaining;
            return 1;
        }
        if (i == 4U) {
            return -1; /* MQTT Remaining Length cannot use five bytes */
        }
        multiplier *= 128U;
    }
    return 0;
}

mqtt_stream_result_t mqtt_stream_feed(mqtt_stream_t *stream,
                                      const uint8_t *data, size_t length,
                                      size_t *consumed)
{
    size_t i;

    if (consumed != NULL) {
        *consumed = 0U;
    }
    if (stream == NULL || consumed == NULL ||
        (length != 0U && data == NULL)) {
        return MQTT_STREAM_ERROR;
    }
    if (stream->state != 0U) {
        return MQTT_STREAM_ERROR;
    }

    for (i = 0U; i < length; i++) {
        int check;
        if (stream->used >= MQTT_STREAM_BUFFER_BYTES) {
            stream->state = 2U;
            return MQTT_STREAM_ERROR;
        }
        stream->bytes[stream->used++] = data[i];
        *consumed = i + 1U;

        if (stream->expected == 0U) {
            check = check_header(stream);
            if (check < 0) {
                stream->state = 2U;
                return MQTT_STREAM_ERROR;
            }
        }
        if (stream->expected != 0U && stream->used == stream->expected) {
            stream->state = 1U;
            return MQTT_STREAM_PACKET;
        }
    }
    return MQTT_STREAM_MORE;
}

const uint8_t *mqtt_stream_packet(const mqtt_stream_t *stream)
{
    return (stream != NULL && stream->state == 1U) ? stream->bytes : NULL;
}

size_t mqtt_stream_packet_length(const mqtt_stream_t *stream)
{
    return (stream != NULL && stream->state == 1U) ? stream->used : 0U;
}
