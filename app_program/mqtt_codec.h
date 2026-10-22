/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * mqtt_codec.h
 * MQTT 3.1.1 packet encoding and an incremental decoder (Buddy 1).  Only what
 * the robot needs: CONNECT (with an optional will), PUBLISH and SUBSCRIBE at
 * QoS 0, PINGREQ and DISCONNECT out; CONNACK, SUBACK, PUBLISH and PINGRESP
 * in.  The lwIP port in the radio profile has no MQTT application layer, so
 * this is a self-contained implementation over a plain TCP byte stream.
 *
 * Pure C, no allocation: callers pass buffers.
 */
#ifndef MQTT_CODEC_H
#define MQTT_CODEC_H

#include <stdbool.h>
#include <stdint.h>

#define MQTT_MAX_PACKET     (256U)      /* larger packets are skipped */
#define MQTT_MAX_TOPIC      (64U)
#define MQTT_MAX_PAYLOAD    (128U)

typedef struct {
    const char    *client_id;
    uint16_t       keepalive_s;
    const char    *will_topic;          /* NULL: no will */
    const uint8_t *will_payload;
    uint16_t       will_len;
    bool           will_retain;
    const char    *username;            /* NULL: none */
    const char    *password;
} mqtt_connect_t;

/* Encoders return the packet length, or 0 if it does not fit in 'cap'. */
uint16_t mqtt_encode_connect(const mqtt_connect_t *c, uint8_t *out, uint16_t cap);
uint16_t mqtt_encode_publish(const char *topic, const uint8_t *payload, uint16_t len,
                             bool retain, uint8_t *out, uint16_t cap);
uint16_t mqtt_encode_subscribe(uint16_t packet_id, const char *topic, uint8_t *out,
                               uint16_t cap);
uint16_t mqtt_encode_pingreq(uint8_t *out, uint16_t cap);
uint16_t mqtt_encode_disconnect(uint8_t *out, uint16_t cap);

typedef enum {
    MQTT_PKT_NONE = 0,      /* need more bytes */
    MQTT_PKT_CONNACK,
    MQTT_PKT_PUBLISH,
    MQTT_PKT_SUBACK,
    MQTT_PKT_PINGRESP,
    MQTT_PKT_OTHER,         /* well-formed but not used by the robot */
    MQTT_PKT_ERROR          /* malformed: drop the connection */
} mqtt_packet_kind_t;

typedef struct {
    mqtt_packet_kind_t kind;
    uint8_t            connack_code;    /* 0 = accepted */
    uint16_t           packet_id;
    uint8_t            suback_code;
    uint8_t            qos;
    char               topic[MQTT_MAX_TOPIC + 1U];
    uint16_t           payload_len;
    uint8_t            payload[MQTT_MAX_PAYLOAD];
} mqtt_packet_t;

typedef struct {
    uint8_t  stage;                     /* 0 header, 1 length, 2 body, 3 skipping */
    uint8_t  header;
    uint32_t remaining;
    uint32_t multiplier;
    uint8_t  length_bytes;
    uint16_t have;
    uint32_t skip_count;                /* bytes of an oversize packet skipped so far */
    uint8_t  body[MQTT_MAX_PACKET];
    uint32_t skipped_packets;           /* oversize packets discarded */
} mqtt_parser_t;

void mqtt_parser_init(mqtt_parser_t *p);

/*
 * Feed bytes from the stream.  Consumes input up to and including the end of
 * one packet; *consumed says how many bytes were used so the caller can feed
 * the rest again.  Returns MQTT_PKT_NONE until a packet completes.
 */
mqtt_packet_kind_t mqtt_parser_feed(mqtt_parser_t *p, const uint8_t *data, uint16_t len,
                                    uint16_t *consumed, mqtt_packet_t *pkt);

#endif /* MQTT_CODEC_H */
