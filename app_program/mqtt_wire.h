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

/* SUBSCRIBE for ONE topic filter at requested QoS 0 (so the broker delivers at QoS 0 at most).  The filter may contain
 * the + and # wildcards.  0 means encoding failed. */
size_t mqtt_wire_subscribe(uint8_t *out, size_t capacity, uint16_t packet_id, const char *topic_filter);

/* A complete SUBACK for one filter.  *code is the granted QoS (0..2) or 0x80 for a refusal. */
bool mqtt_wire_parse_suback(const uint8_t *packet, size_t length, uint16_t *packet_id, uint8_t *code);

/* A complete PUBLISH at QoS 0 (the retain flag is allowed).  The topic and payload point into the packet.
 * Returns false for QoS 1 or 2, a malformed length or a duplicate flag. */
bool mqtt_wire_parse_publish_qos0(const uint8_t *packet, size_t length,
                                  const uint8_t **topic, size_t *topic_length,
                                  const uint8_t **payload, size_t *payload_length);

#endif
