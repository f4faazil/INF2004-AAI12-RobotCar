#ifndef AAI12_MQTT_STREAM_H
#define AAI12_MQTT_STREAM_H

#include <stddef.h>
#include <stdint.h>

/*
 * Small, allocation-free MQTT 3.1.1 packet framer for a TCP byte stream.
 * The receiver must be used by one network-owner task, not by multiple cores
 * or ISRs simultaneously.
 *
 * Handles TCP fragmentation and coalescing. Framing does not imply that the
 * received MQTT control packet is semantically valid. Use mqtt_wire helpers
 * or specific packet validators on each completed packet.
 *
 * A complete packet stays in the buffer until mqtt_stream_reset().
 * A malformed/oversize packet puts the receiver in an error state; reset it
 * only when abandoning that stream (normally close/reconnect the TCP socket).
 */
#define MQTT_STREAM_BUFFER_BYTES 256U

typedef enum {
    MQTT_STREAM_MORE = 0,
    MQTT_STREAM_PACKET = 1,
    MQTT_STREAM_ERROR = -1
} mqtt_stream_result_t;

typedef struct {
    uint8_t bytes[MQTT_STREAM_BUFFER_BYTES];
    size_t used;
    size_t expected;  /* Zero until MQTT Remaining Length is fully parsed */
    uint8_t state;    /* 0 = receiving, 1 = complete, 2 = error */
} mqtt_stream_t;

void mqtt_stream_init(mqtt_stream_t *stream);

/*
 * Feed up to length bytes. Returns on the FIRST complete packet, possibly
 * before consuming all input. *consumed is always the number of input bytes
 * consumed by this call. Caller should process packet, reset stream, then
 * call again with input + consumed and length - consumed.
 *
 * A zero-byte input is allowed with data == NULL. Every other argument must
 * be valid. A stream with an unconsumed packet must be reset before feeding.
 */
mqtt_stream_result_t mqtt_stream_feed(mqtt_stream_t *stream,
                                      const uint8_t *data, size_t length,
                                      size_t *consumed);

/* Only valid after feed returned MQTT_STREAM_PACKET. */
const uint8_t *mqtt_stream_packet(const mqtt_stream_t *stream);
size_t mqtt_stream_packet_length(const mqtt_stream_t *stream);

void mqtt_stream_reset(mqtt_stream_t *stream);

#endif /* AAI12_MQTT_STREAM_H */
