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
 *   write_tcp: ACCEPTED only if the entire byte buffer was COPIED/queued;
 *              RETRY for temporary backpressure; FATAL if this TCP session
 *              can no longer accept data. No partial writes.
 *   close_tcp: close/abort the active connection and release its ownership.
 *
 * The lwIP adapter must call these callbacks on the radio-owner task.
 * The adapter must NOT call close_tcp for a PCB already freed by lwIP err_cb.
 */
#define MQTT_SESSION_TX_BYTES          256U
#define MQTT_SESSION_CLIENT_ID_BYTES    32U
#define MQTT_SESSION_BACKOFF_MS       2000U
#define MQTT_SESSION_TCP_TIMEOUT_MS  10000U
#define MQTT_SESSION_ACK_TIMEOUT_MS   5000U
#define MQTT_SESSION_KEEPALIVE_SEC      30U
#define MQTT_SESSION_PING_IDLE_MS    20000U
#define MQTT_SESSION_PING_QUEUE_TIMEOUT_MS 5000U
#define MQTT_SESSION_PING_TIMEOUT_MS  5000U

typedef enum {
    MQTT_SESSION_WAIT_NETWORK = 0,
    MQTT_SESSION_CONNECTING,
    MQTT_SESSION_WAIT_CONNACK,
    MQTT_SESSION_ONLINE,
    MQTT_SESSION_BACKOFF
} mqtt_session_state_t;

typedef enum {
    MQTT_SESSION_WRITE_RETRY = 0,
    MQTT_SESSION_WRITE_ACCEPTED,
    MQTT_SESSION_WRITE_FATAL
} mqtt_session_write_result_t;

typedef struct {
    bool (*start_tcp)(void *context);
    mqtt_session_write_result_t (*write_tcp)(void *context,
                                             const uint8_t *packet,
                                             size_t bytes);
    void (*close_tcp)(void *context);
} mqtt_session_transport_t;

/* An incoming QoS 0 PUBLISH, delivered on the owner task.  topic is NOT terminated.  The data is only valid during the
 * call: copy it.  The handler must not block and must not call back into the session. */
typedef void (*mqtt_session_message_fn)(void *context, const uint8_t *topic, size_t topic_length,
                                        const uint8_t *payload, size_t payload_length);

#define MQTT_SESSION_MAX_SUBSCRIPTIONS  2U

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
    uint32_t poll_calls;
    uint32_t last_poll_ms;
    uint32_t ping_attempts;
    uint32_t pings;
    uint32_t ping_responses;
    uint32_t ping_queue_since_ms;
    uint32_t ping_queue_timeouts;
    bool network_up;
    bool tcp_active;
    bool waiting_pingqueue;
    bool waiting_pingresp;

    /* Subscriptions, sent one at a time after each CONNACK.  Packet id n + 1 is subscription n. */
    const char *subscriptions[MQTT_SESSION_MAX_SUBSCRIPTIONS];
    uint8_t subscription_count;
    uint8_t subscriptions_sent;
    uint8_t subscriptions_acked;
    uint32_t subscribes;
    uint32_t messages_received;
    mqtt_session_message_fn on_message;
    void *message_context;
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

/* Add a topic filter to subscribe to after every CONNACK (call before the first poll; the string must stay valid).
 * False when the table is full. */
bool mqtt_session_add_subscription(mqtt_session_t *session, const char *topic_filter);

/* Where incoming PUBLISH messages go (owner task, QoS 0 only). */
void mqtt_session_set_message_handler(mqtt_session_t *session, mqtt_session_message_fn handler, void *context);

/* QoS 0; true means packet bytes accepted by the transport, NOT broker receipt.
 * Call only from the owner task. Caller's telemetry buffer remains theirs.
 * Nothing pops the telemetry queue in this module.
 */
bool mqtt_session_publish(mqtt_session_t *session, const char *topic,
                          const uint8_t *payload, size_t bytes,
                          uint32_t now_ms);

#endif /* AAI12_MQTT_SESSION_H */
