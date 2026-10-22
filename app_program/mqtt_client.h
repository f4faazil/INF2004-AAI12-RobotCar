/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * mqtt_client.h
 * MQTT client state machine (Buddy 1): connect, subscribe to the command
 * topic, publish, keep alive, and reconnect with bounded exponential backoff.
 *
 * It is written to run inside the lowest-priority WiFi task and never to
 * block: every call does a bounded amount of work and talks to the network
 * through a small transport interface (tcp_connect/send/recv/close) that the
 * lwIP raw-TCP adapter implements on the car and a fake implements in the host
 * tests.  Losing the network never reaches the control tasks: publishes while
 * disconnected are dropped and counted.
 */
#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <stdbool.h>
#include <stdint.h>

#include "mqtt_codec.h"

typedef struct {
    int  (*tcp_connect)(void *ctx);                                 /* 1 up, 0 pending, <0 failed */
    int  (*tcp_send)(void *ctx, const uint8_t *data, uint16_t n);   /* bytes sent, 0 would block, <0 error */
    int  (*tcp_recv)(void *ctx, uint8_t *buf, uint16_t cap);        /* bytes, 0 none, <0 closed */
    void (*tcp_close)(void *ctx);
    void *ctx;
} mqtt_transport_t;

typedef void (*mqtt_message_cb)(void *ctx, const char *topic, const uint8_t *payload,
                                uint16_t len);

typedef struct {
    const char     *client_id;
    const char     *username;           /* NULL: none */
    const char     *password;
    const char     *status_topic;       /* will "offline" / birth "online", retained */
    const char     *command_topic;      /* subscribed */
    uint16_t        keepalive_s;
    uint32_t        connect_timeout_ms;
    uint32_t        backoff_initial_ms;
    uint32_t        backoff_max_ms;
    mqtt_message_cb on_message;
    void           *cb_ctx;
} mqtt_config_t;

/* net_state in the heartbeat message uses these values (3 bits). */
typedef enum {
    MQTT_DISCONNECTED = 0,
    MQTT_TCP_CONNECTING,
    MQTT_AWAIT_CONNACK,
    MQTT_CONNECTED
} mqtt_state_t;

typedef struct {
    mqtt_config_t    cfg;
    mqtt_transport_t net;
    mqtt_parser_t    parser;
    mqtt_packet_t    pkt;
    mqtt_state_t     state;
    bool             link_up;
    uint32_t         now_ms;
    uint32_t         next_attempt_ms;
    uint32_t         backoff_ms;
    uint32_t         attempt_start_ms;
    uint32_t         last_tx_ms;
    uint32_t         ping_sent_ms;
    bool             ping_outstanding;
    bool             ever_connected;

    uint32_t         connect_attempts;
    uint32_t         failures;
    uint32_t         reconnects;        /* successful connections after the first */
    uint32_t         dropped_offline;   /* publishes refused while not connected */
    uint32_t         dropped_busy;      /* publishes the socket could not take */
    uint32_t         messages_received;
} mqtt_client_t;

void mqtt_client_init(mqtt_client_t *c, const mqtt_config_t *cfg, const mqtt_transport_t *net);

/* The WiFi link state.  With the link down no connection is attempted and
 * nothing is counted as a failure. */
void mqtt_client_set_link(mqtt_client_t *c, bool link_up);

/* Advance the state machine; call every ~50-100 ms from the WiFi task. */
void mqtt_client_step(mqtt_client_t *c, uint32_t now_ms);

/* Publish at QoS 0.  Returns false (and counts a drop) if not connected. */
bool mqtt_client_publish(mqtt_client_t *c, const char *topic, const uint8_t *payload,
                         uint16_t len, bool retain);

mqtt_state_t mqtt_client_state(const mqtt_client_t *c);

#endif /* MQTT_CLIENT_H */
