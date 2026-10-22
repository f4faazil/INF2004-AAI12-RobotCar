#ifndef AAI12_MQTT_WIRE_H
#define AAI12_MQTT_WIRE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* MQTT v3.1.1 packet encoder for a future single-owner lwIP TCP service.
 * This module performs NO networking and uses NO dynamic allocation.
 * 0 means encoding failed; otherwise returns the complete packet length.
 * The caller owns the destination buffer and must keep it alive through send.
 */
size_t mqtt_wire_connect(uint8_t *out, size_t capacity,
                         const char *client_id, uint16_t keep_alive_seconds);

/* QoS 0 telemetry publish. Topic must be nonempty and contain no +/# wildcards.
 * The payload may contain zero bytes and has no terminator.
 */
size_t mqtt_wire_publish_qos0(uint8_t *out, size_t capacity,
                              const char *topic,
                              const uint8_t *payload, size_t payload_length);

size_t mqtt_wire_pingreq(uint8_t *out, size_t capacity);
size_t mqtt_wire_disconnect(uint8_t *out, size_t capacity);

/* Structural decoder of a COMPLETE, four-byte MQTT 3.1.1 CONNACK packet.
 * It is not a TCP stream parser. Result codes 0..5 follow MQTT v3.1.1.
 * Returns true for a well-formed CONNACK even when the broker rejects us.
 */
bool mqtt_wire_parse_connack(const uint8_t *packet, size_t length,
                             uint8_t *return_code);

bool mqtt_wire_is_pingresp(const uint8_t *packet, size_t length);

#endif
