/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * mqtt_client.c
 * MQTT client state machine.  See mqtt_client.h.
 */
#include <stddef.h>

#include "mqtt_client.h"

#define SEND_BUF            (192U)
#define RECV_BUF            (96U)
#define READS_PER_STEP      (4U)            /* bounded work per call */
#define MS_PER_S            (1000U)
#define PING_RESPONSE_MS    (10000U)        /* PINGRESP overdue after this */
#define SUBSCRIBE_ID        (1U)
#define STATUS_ONLINE       "online"
#define STATUS_OFFLINE      "offline"
#define STATUS_LEN          (7U)            /* strlen("offline") */
#define ONLINE_LEN          (6U)

typedef enum {
    SEND_OK = 0,
    SEND_BLOCKED,
    SEND_FAILED
} send_result_t;

static void parser_reset(mqtt_client_t *c)
{
    mqtt_parser_init(&c->parser);
}

static void fail(mqtt_client_t *c, uint32_t now_ms)
{
    c->net.tcp_close(c->net.ctx);
    c->state = MQTT_DISCONNECTED;
    parser_reset(c);
    c->ping_outstanding = false;
    c->failures++;
    c->next_attempt_ms = now_ms + c->backoff_ms;
    c->backoff_ms *= 2U;
    if (c->backoff_ms > c->cfg.backoff_max_ms) {
        c->backoff_ms = c->cfg.backoff_max_ms;
    }
}

static send_result_t send_packet(mqtt_client_t *c, const uint8_t *data, uint16_t n)
{
    int sent;

    if (n == 0U) {
        return SEND_FAILED;                     /* the packet did not fit: a bug */
    }
    sent = c->net.tcp_send(c->net.ctx, data, n);
    if (sent == 0) {
        return SEND_BLOCKED;
    }
    if (sent != (int)n) {
        return SEND_FAILED;                     /* error or a partial write */
    }
    c->last_tx_ms = c->now_ms;
    return SEND_OK;
}

void mqtt_client_init(mqtt_client_t *c, const mqtt_config_t *cfg, const mqtt_transport_t *net)
{
    c->cfg = *cfg;
    c->net = *net;
    parser_reset(c);
    c->state = MQTT_DISCONNECTED;
    c->link_up = false;
    c->now_ms = 0U;
    c->next_attempt_ms = 0U;
    c->backoff_ms = cfg->backoff_initial_ms;
    c->attempt_start_ms = 0U;
    c->last_tx_ms = 0U;
    c->ping_sent_ms = 0U;
    c->ping_outstanding = false;
    c->ever_connected = false;
    c->connect_attempts = 0U;
    c->failures = 0U;
    c->reconnects = 0U;
    c->dropped_offline = 0U;
    c->dropped_busy = 0U;
    c->messages_received = 0U;
}

void mqtt_client_set_link(mqtt_client_t *c, bool link_up)
{
    c->link_up = link_up;
}

mqtt_state_t mqtt_client_state(const mqtt_client_t *c)
{
    return c->state;
}

static bool send_connect(mqtt_client_t *c)
{
    uint8_t buf[SEND_BUF];
    mqtt_connect_t cc;
    uint16_t n;

    cc.client_id = c->cfg.client_id;
    cc.keepalive_s = c->cfg.keepalive_s;
    cc.will_topic = c->cfg.status_topic;
    cc.will_payload = (const uint8_t *)STATUS_OFFLINE;
    cc.will_len = STATUS_LEN;
    cc.will_retain = true;
    cc.username = c->cfg.username;
    cc.password = c->cfg.password;
    n = mqtt_encode_connect(&cc, buf, sizeof(buf));
    return send_packet(c, buf, n) == SEND_OK;
}

/* After CONNACK: subscribe to commands and announce we are online. */
static bool send_session_start(mqtt_client_t *c)
{
    uint8_t buf[SEND_BUF];
    uint16_t n = mqtt_encode_subscribe(SUBSCRIBE_ID, c->cfg.command_topic, buf, sizeof(buf));

    if (send_packet(c, buf, n) != SEND_OK) {
        return false;
    }
    n = mqtt_encode_publish(c->cfg.status_topic, (const uint8_t *)STATUS_ONLINE, ONLINE_LEN,
                            true, buf, sizeof(buf));
    return send_packet(c, buf, n) == SEND_OK;
}

/* Read and decode what the broker sent; returns false on a dead connection. */
static bool pump_receive(mqtt_client_t *c, uint32_t now_ms)
{
    uint8_t buf[RECV_BUF];
    uint8_t reads;

    for (reads = 0U; reads < READS_PER_STEP; reads++) {
        int got = c->net.tcp_recv(c->net.ctx, buf, sizeof(buf));
        uint16_t used = 0U;

        if (got < 0) {
            return false;
        }
        if (got == 0) {
            break;
        }
        while (used < (uint16_t)got) {
            uint16_t step = 0U;
            mqtt_packet_kind_t k = mqtt_parser_feed(&c->parser, &buf[used],
                                                    (uint16_t)((uint16_t)got - used), &step,
                                                    &c->pkt);

            used = (uint16_t)(used + step);
            if (k == MQTT_PKT_ERROR) {
                return false;
            }
            if (k == MQTT_PKT_CONNACK) {
                if ((c->state != MQTT_AWAIT_CONNACK) || (c->pkt.connack_code != 0U)) {
                    return false;
                }
                c->state = MQTT_CONNECTED;
                if (c->ever_connected) {
                    c->reconnects++;
                }
                c->ever_connected = true;
                c->backoff_ms = c->cfg.backoff_initial_ms;
                if (!send_session_start(c)) {
                    return false;
                }
            } else if (k == MQTT_PKT_PUBLISH) {
                if ((c->state == MQTT_CONNECTED) && (c->cfg.on_message != NULL)) {
                    c->messages_received++;
                    c->cfg.on_message(c->cfg.cb_ctx, c->pkt.topic, c->pkt.payload,
                                      c->pkt.payload_len);
                }
            } else if (k == MQTT_PKT_PINGRESP) {
                c->ping_outstanding = false;
            } else {
                /* SUBACK / OTHER / NONE: nothing to do */
            }
        }
    }
    (void)now_ms;
    return true;
}

void mqtt_client_step(mqtt_client_t *c, uint32_t now_ms)
{
    c->now_ms = now_ms;

    if (!c->link_up) {
        if (c->state != MQTT_DISCONNECTED) {
            c->net.tcp_close(c->net.ctx);
            c->state = MQTT_DISCONNECTED;
            parser_reset(c);
            c->ping_outstanding = false;
        }
        c->next_attempt_ms = now_ms;            /* try at once when the link returns */
        return;
    }

    switch (c->state) {
    case MQTT_DISCONNECTED:
        if ((int32_t)(now_ms - c->next_attempt_ms) >= 0) {
            c->state = MQTT_TCP_CONNECTING;
            c->attempt_start_ms = now_ms;
            c->connect_attempts++;
        } else {
            break;
        }
        /* fall through: try the connect straight away */

    case MQTT_TCP_CONNECTING: {
        int r = c->net.tcp_connect(c->net.ctx);

        if (r < 0) {
            fail(c, now_ms);
        } else if (r > 0) {
            if (send_connect(c)) {
                c->state = MQTT_AWAIT_CONNACK;
            } else {
                fail(c, now_ms);
            }
        } else if ((now_ms - c->attempt_start_ms) >= c->cfg.connect_timeout_ms) {
            fail(c, now_ms);
        } else {
            /* still connecting */
        }
        break;
    }

    case MQTT_AWAIT_CONNACK:
        if (!pump_receive(c, now_ms)) {
            fail(c, now_ms);
        } else if ((c->state == MQTT_AWAIT_CONNACK) &&
                   ((now_ms - c->attempt_start_ms) >= c->cfg.connect_timeout_ms)) {
            fail(c, now_ms);
        } else {
            /* waiting */
        }
        break;

    case MQTT_CONNECTED:
        if (!pump_receive(c, now_ms)) {
            fail(c, now_ms);
            break;
        }
        if (c->ping_outstanding) {
            if ((now_ms - c->ping_sent_ms) >= PING_RESPONSE_MS) {
                fail(c, now_ms);                /* a half-open connection */
            }
        } else if ((now_ms - c->last_tx_ms) >= ((uint32_t)c->cfg.keepalive_s * MS_PER_S / 2U)) {
            uint8_t buf[4];
            uint16_t n = mqtt_encode_pingreq(buf, sizeof(buf));

            if (send_packet(c, buf, n) == SEND_OK) {
                c->ping_outstanding = true;
                c->ping_sent_ms = now_ms;
            } else {
                fail(c, now_ms);
            }
        } else {
            /* idle but within the keep-alive */
        }
        break;

    default:
        break;
    }
}

bool mqtt_client_publish(mqtt_client_t *c, const char *topic, const uint8_t *payload,
                         uint16_t len, bool retain)
{
    uint8_t buf[SEND_BUF];
    uint16_t n;
    send_result_t r;

    if (c->state != MQTT_CONNECTED) {
        c->dropped_offline++;
        return false;
    }
    n = mqtt_encode_publish(topic, payload, len, retain, buf, sizeof(buf));
    r = send_packet(c, buf, n);
    if (r == SEND_BLOCKED) {
        c->dropped_busy++;
        return false;
    }
    if (r == SEND_FAILED) {
        fail(c, c->now_ms);
        return false;
    }
    return true;
}
