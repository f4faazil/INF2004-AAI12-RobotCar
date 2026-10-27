#ifndef AAI12_MQTT_SESSION_H
#define AAI12_MQTT_SESSION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mqtt_stream.h"

/*
 * MQTT 3.1.1 client lifecycle, independent of lwIP and micro T-Kernel.
 *
 * All functions must run on ONE owner task. No ISR or other core may call
 * this directly. The transport callbacks must NOT block:
 *
 *   start_tcp: 1 if async TCP connect was started; 0 if it could not start.
 *   write_tcp: 1 only if the entire byte buffer was COPIED/queued; 0 on
 *              temporary backpressure or failure. No partial writes.
 *   close_tcp: close/abort the active connection and release its ownership.
 *
 * The future lwIP adapter must call these callbacks on the radio-owner task.
 * The adapter must NOT call close_tcp for a PCB already freed by lwIP err_cb.
 */
#define MQTT_SESSION_TX_BYTES          256U
#define MQTT_SESSION_CLIENT_ID_BYTES    32U
#define MQTT_SESSION_BACKOFF_MS       2000U
#define MQTT_SESSION_TCP_TIMEOUT_MS  10000U
#define MQTT_SESSION_ACK_TIMEOUT_MS   5000U
#define MQTT_SESSION_KEEPALIVE_SEC      30U
#define MQTT_SESSION_PING_IDLE_MS    20000U
#define MQTT_SESSION_PING_TIMEOUT_MS  5000U

typedef enum {
    MQTT_SESSION_WAIT_NETWORK = 0,
    MQTT_SESSION_CONNECTING,
    MQTT_SESSION_WAIT_CONNACK,
    MQTT_SESSION_ONLINE,
    MQTT_SESSION_BACKOFF
} mqtt_session_state_t;

typedef struct {
    bool (*start_tcp)(void *context);
    bool (*write_tcp)(void *context, const uint8_t *packet, size_t bytes);
    void (*close_tcp)(void *context);
} mqtt_session_transport_t;

typedef struct {
    mqtt_session_transport_t transport;
    void *context;
    mqtt_stream_t rx;
    char client_id[MQTT_SESSION_CLIENT_ID_BYTES];
    uint8_t tx[MQTT_SESSION_TX_BYTES];
    mqtt_session_state_t state;
    uint32_t state_since_ms;
    uint32_t last_tx_ms;
    uint32_t ping_since_ms;
    uint32_t connects_started;
    uint32_t sessions_online;
    uint32_t failures;
    uint32_t publishes;
    uint32_t pings;
    bool network_up;
    bool tcp_active;
    bool waiting_pingresp;
} mqtt_session_t;

bool mqtt_session_init(mqtt_session_t *session,
                       const mqtt_session_transport_t *transport,
                       void *context, const char *client_id);

/* Call on the owner task after receiving a synchronized WiFi/DHCP status. */
void mqtt_session_set_network(mqtt_session_t *session, bool up,
                              uint32_t now_ms);

/* Periodic non-blocking tick, for example every 100 ms on the owner task. */
void mqtt_session_poll(mqtt_session_t *session, uint32_t now_ms);

/* Adapter forwards the TCP connected callback (success only). */
void mqtt_session_tcp_connected(mqtt_session_t *session, uint32_t now_ms);

/* Adapter forwards remote FIN / lwIP error. It has ALREADY dropped the PCB. */
void mqtt_session_tcp_lost(mqtt_session_t *session, uint32_t now_ms);

/* Adapter forwards TCP payload bytes in order; handles fragmented/coalesced packets. */
void mqtt_session_tcp_received(mqtt_session_t *session,
                               const uint8_t *data, size_t length,
                               uint32_t now_ms);

/* QoS 0; true means packet bytes accepted by the transport, NOT broker receipt.
 * Call only from the owner task. Caller's telemetry buffer remains theirs.
 * Nothing pops the telemetry queue in this module.
 */
bool mqtt_session_publish(mqtt_session_t *session, const char *topic,
                          const uint8_t *payload, size_t bytes,
                          uint32_t now_ms);

#endif /* AAI12_MQTT_SESSION_H */
