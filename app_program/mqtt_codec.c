/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * mqtt_codec.c
 * MQTT 3.1.1 encoding and incremental decoding.  See mqtt_codec.h.
 */
#include <stddef.h>

#include "mqtt_codec.h"

#define TYPE_CONNECT        (0x10U)
#define TYPE_PUBLISH        (0x30U)
#define TYPE_SUBSCRIBE      (0x82U)         /* type 8 with the required flags 0x2 */
#define TYPE_PINGREQ        (0xC0U)
#define TYPE_DISCONNECT     (0xE0U)

#define PKT_CONNACK         (2U)
#define PKT_PUBLISH         (3U)
#define PKT_SUBACK          (9U)
#define PKT_PINGRESP        (13U)

#define FLAG_CLEAN          (0x02U)
#define FLAG_WILL           (0x04U)
#define FLAG_WILL_RETAIN    (0x20U)
#define FLAG_PASSWORD       (0x40U)
#define FLAG_USERNAME       (0x80U)
#define PROTOCOL_LEVEL      (4U)            /* MQTT 3.1.1 */

#define LEN_CONTINUE        (0x80U)
#define LEN_MASK            (0x7FU)
#define LEN_MAX_BYTES       (4U)
#define LEN_MULTIPLIER      (128U)

#define STAGE_HEADER        (0U)
#define STAGE_LENGTH        (1U)
#define STAGE_BODY          (2U)
#define STAGE_SKIP          (3U)

/* ---- encoder helpers --------------------------------------------------- */

typedef struct {
    uint8_t *buf;
    uint16_t cap;
    uint16_t len;
    bool     ok;
} wr_t;

static void wr_byte(wr_t *w, uint8_t b)
{
    if (w->len < w->cap) {
        w->buf[w->len] = b;
        w->len++;
    } else {
        w->ok = false;
    }
}

static uint16_t str_len(const char *s)
{
    uint16_t n = 0U;

    while (s[n] != '\0') {
        n++;
    }
    return n;
}

static void wr_u16(wr_t *w, uint16_t v)
{
    wr_byte(w, (uint8_t)(v >> 8));
    wr_byte(w, (uint8_t)(v & 0xFFU));
}

static void wr_bytes(wr_t *w, const uint8_t *d, uint16_t n)
{
    uint16_t i;

    for (i = 0U; i < n; i++) {
        wr_byte(w, d[i]);
    }
}

static void wr_string(wr_t *w, const char *s)
{
    uint16_t n = str_len(s);

    wr_u16(w, n);
    wr_bytes(w, (const uint8_t *)s, n);
}

static void wr_remaining_length(wr_t *w, uint32_t value)
{
    do {
        uint8_t digit = (uint8_t)(value % LEN_MULTIPLIER);

        value /= LEN_MULTIPLIER;
        if (value > 0U) {
            digit |= LEN_CONTINUE;
        }
        wr_byte(w, digit);
    } while (value > 0U);
}

/* Assemble [first byte][remaining length][body] into out. */
static uint16_t finish(uint8_t first, const uint8_t *body, uint16_t body_len, uint8_t *out,
                       uint16_t cap)
{
    wr_t w = { out, cap, 0U, true };

    wr_byte(&w, first);
    wr_remaining_length(&w, body_len);
    wr_bytes(&w, body, body_len);
    return w.ok ? w.len : 0U;
}

/* ---- encoders ---------------------------------------------------------- */

uint16_t mqtt_encode_connect(const mqtt_connect_t *c, uint8_t *out, uint16_t cap)
{
    uint8_t body[MQTT_MAX_PACKET];
    wr_t w = { body, (uint16_t)sizeof(body), 0U, true };
    uint8_t flags = FLAG_CLEAN;

    if (c->will_topic != NULL) {
        flags |= FLAG_WILL;
        if (c->will_retain) {
            flags |= FLAG_WILL_RETAIN;
        }
    }
    if (c->username != NULL) {
        flags |= FLAG_USERNAME;
    }
    if (c->password != NULL) {
        flags |= FLAG_PASSWORD;
    }

    wr_string(&w, "MQTT");
    wr_byte(&w, PROTOCOL_LEVEL);
    wr_byte(&w, flags);
    wr_u16(&w, c->keepalive_s);
    wr_string(&w, c->client_id);
    if (c->will_topic != NULL) {
        wr_string(&w, c->will_topic);
        wr_u16(&w, c->will_len);
        wr_bytes(&w, c->will_payload, c->will_len);
    }
    if (c->username != NULL) {
        wr_string(&w, c->username);
    }
    if (c->password != NULL) {
        wr_string(&w, c->password);
    }
    return w.ok ? finish(TYPE_CONNECT, body, w.len, out, cap) : 0U;
}

uint16_t mqtt_encode_publish(const char *topic, const uint8_t *payload, uint16_t len,
                             bool retain, uint8_t *out, uint16_t cap)
{
    uint8_t body[MQTT_MAX_PACKET];
    wr_t w = { body, (uint16_t)sizeof(body), 0U, true };

    wr_string(&w, topic);
    wr_bytes(&w, payload, len);
    return w.ok ? finish((uint8_t)(TYPE_PUBLISH | (retain ? 1U : 0U)), body, w.len, out, cap)
                : 0U;
}

uint16_t mqtt_encode_subscribe(uint16_t packet_id, const char *topic, uint8_t *out,
                               uint16_t cap)
{
    uint8_t body[MQTT_MAX_PACKET];
    wr_t w = { body, (uint16_t)sizeof(body), 0U, true };

    wr_u16(&w, packet_id);
    wr_string(&w, topic);
    wr_byte(&w, 0U);                            /* requested QoS 0 */
    return w.ok ? finish(TYPE_SUBSCRIBE, body, w.len, out, cap) : 0U;
}

uint16_t mqtt_encode_pingreq(uint8_t *out, uint16_t cap)
{
    return finish(TYPE_PINGREQ, NULL, 0U, out, cap);
}

uint16_t mqtt_encode_disconnect(uint8_t *out, uint16_t cap)
{
    return finish(TYPE_DISCONNECT, NULL, 0U, out, cap);
}

/* ---- decoder ----------------------------------------------------------- */

void mqtt_parser_init(mqtt_parser_t *p)
{
    p->stage = STAGE_HEADER;
    p->header = 0U;
    p->remaining = 0U;
    p->multiplier = 1U;
    p->length_bytes = 0U;
    p->have = 0U;
    p->skip_count = 0U;
    p->skipped_packets = 0U;
}

static void reset_for_next(mqtt_parser_t *p)
{
    p->stage = STAGE_HEADER;
    p->remaining = 0U;
    p->multiplier = 1U;
    p->length_bytes = 0U;
    p->have = 0U;
    p->skip_count = 0U;
}

/* Interpret a complete packet body. */
static mqtt_packet_kind_t interpret(const mqtt_parser_t *p, mqtt_packet_t *pkt)
{
    const uint8_t type = (uint8_t)(p->header >> 4);
    const uint8_t *b = p->body;
    const uint16_t n = p->have;

    pkt->kind = MQTT_PKT_OTHER;
    pkt->payload_len = 0U;
    pkt->topic[0] = '\0';
    pkt->qos = 0U;

    if (type == PKT_CONNACK) {
        if (n != 2U) {
            return MQTT_PKT_ERROR;
        }
        pkt->connack_code = b[1];
        pkt->kind = MQTT_PKT_CONNACK;
    } else if (type == PKT_SUBACK) {
        if (n < 3U) {
            return MQTT_PKT_ERROR;
        }
        pkt->packet_id = (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
        pkt->suback_code = b[2];
        pkt->kind = MQTT_PKT_SUBACK;
    } else if (type == PKT_PINGRESP) {
        pkt->kind = MQTT_PKT_PINGRESP;
    } else if (type == PKT_PUBLISH) {
        uint16_t topic_len;
        uint16_t pos;
        uint16_t i;

        pkt->qos = (uint8_t)((p->header >> 1) & 0x03U);
        if (n < 2U) {
            return MQTT_PKT_ERROR;
        }
        topic_len = (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
        pos = (uint16_t)(2U + topic_len);
        if ((pos > n) || (topic_len > MQTT_MAX_TOPIC)) {
            return MQTT_PKT_ERROR;
        }
        for (i = 0U; i < topic_len; i++) {
            pkt->topic[i] = (char)b[2U + i];
        }
        pkt->topic[topic_len] = '\0';
        if (pkt->qos > 0U) {
            if ((uint16_t)(pos + 2U) > n) {
                return MQTT_PKT_ERROR;
            }
            pkt->packet_id = (uint16_t)(((uint16_t)b[pos] << 8) | b[pos + 1U]);
            pos = (uint16_t)(pos + 2U);
        }
        if ((uint16_t)(n - pos) > MQTT_MAX_PAYLOAD) {
            return MQTT_PKT_OTHER;              /* payload too large to hold */
        }
        pkt->payload_len = (uint16_t)(n - pos);
        for (i = 0U; i < pkt->payload_len; i++) {
            pkt->payload[i] = b[pos + i];
        }
        pkt->kind = MQTT_PKT_PUBLISH;
    } else {
        /* a packet type the robot does not use */
    }
    return pkt->kind;
}

mqtt_packet_kind_t mqtt_parser_feed(mqtt_parser_t *p, const uint8_t *data, uint16_t len,
                                    uint16_t *consumed, mqtt_packet_t *pkt)
{
    uint16_t i = 0U;

    pkt->kind = MQTT_PKT_NONE;
    while (i < len) {
        const uint8_t b = data[i];

        if (p->stage == STAGE_HEADER) {
            p->header = b;
            p->stage = STAGE_LENGTH;
            i++;
        } else if (p->stage == STAGE_LENGTH) {
            p->remaining += (uint32_t)(b & LEN_MASK) * p->multiplier;
            p->multiplier *= LEN_MULTIPLIER;
            p->length_bytes++;
            i++;
            if ((b & LEN_CONTINUE) != 0U) {
                if (p->length_bytes >= LEN_MAX_BYTES) {
                    *consumed = i;
                    reset_for_next(p);
                    pkt->kind = MQTT_PKT_ERROR;
                    return MQTT_PKT_ERROR;
                }
            } else {
                p->have = 0U;
                if (p->remaining == 0U) {
                    *consumed = i;
                    {
                        mqtt_packet_kind_t k = interpret(p, pkt);

                        reset_for_next(p);
                        return k;
                    }
                }
                p->stage = (p->remaining > MQTT_MAX_PACKET) ? STAGE_SKIP : STAGE_BODY;
            }
        } else if (p->stage == STAGE_BODY) {
            p->body[p->have] = b;
            p->have++;
            i++;
            if (p->have >= p->remaining) {
                mqtt_packet_kind_t k = interpret(p, pkt);

                *consumed = i;
                reset_for_next(p);
                return k;
            }
        } else {                                /* STAGE_SKIP */
            p->skip_count++;
            i++;
            if (p->skip_count >= p->remaining) {
                p->skipped_packets++;
                pkt->kind = MQTT_PKT_OTHER;
                *consumed = i;
                reset_for_next(p);
                return MQTT_PKT_OTHER;
            }
        }
    }
    *consumed = i;
    return MQTT_PKT_NONE;
}
