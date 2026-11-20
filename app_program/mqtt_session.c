#include "mqtt_session.h"

#include "mqtt_wire.h"
#include <string.h>

static void reset_stream(mqtt_session_t *session)
{
    mqtt_stream_reset(&session->rx);
    session->waiting_pingqueue = false;
    session->waiting_pingresp = false;
}

static void enter_backoff(mqtt_session_t *session, uint32_t now_ms,
                          bool transport_has_pcb)
{
    if (transport_has_pcb && session->tcp_active) {
        session->transport.close_tcp(session->context);
    }
    session->tcp_active = false;
    session->state = session->network_up ? MQTT_SESSION_BACKOFF
                                         : MQTT_SESSION_WAIT_NETWORK;
    session->state_since_ms = now_ms;
    session->failures++;
    reset_stream(session);
}

bool mqtt_session_init(mqtt_session_t *session,
                       const mqtt_session_transport_t *transport,
                       void *context, const char *client_id)
{
    size_t id_length;

    if (session == NULL || transport == NULL || client_id == NULL ||
        transport->start_tcp == NULL || transport->write_tcp == NULL ||
        transport->close_tcp == NULL) {
        return false;
    }
    id_length = strlen(client_id);
    if (id_length == 0U || id_length >= MQTT_SESSION_CLIENT_ID_BYTES) {
        return false;
    }

    memset(session, 0, sizeof(*session));
    session->transport = *transport;
    session->context = context;
    memcpy(session->client_id, client_id, id_length + 1U);
    session->state = MQTT_SESSION_WAIT_NETWORK;
    mqtt_stream_init(&session->rx);
    return true;
}

void mqtt_session_set_network(mqtt_session_t *session, bool up,
                              uint32_t now_ms)
{
    if (session == NULL) return;
    if (!up) {
        if (session->tcp_active) {
            session->transport.close_tcp(session->context);
        }
        session->tcp_active = false;
        session->network_up = false;
        session->state = MQTT_SESSION_WAIT_NETWORK;
        session->state_since_ms = now_ms;
        reset_stream(session);
    } else {
        session->network_up = true;
    }
}

void mqtt_session_poll(mqtt_session_t *session, uint32_t now_ms)
{
    size_t packet_length;
    mqtt_session_write_result_t write_result;

    if (session == NULL) return;
    session->poll_calls++;
    session->last_poll_ms = now_ms;
    if (!session->network_up) return;

    if (session->state == MQTT_SESSION_BACKOFF) {
        if ((uint32_t)(now_ms - session->state_since_ms) <
            MQTT_SESSION_BACKOFF_MS) return;
        session->state = MQTT_SESSION_WAIT_NETWORK;
    }

    if (session->state == MQTT_SESSION_WAIT_NETWORK) {
        if (session->transport.start_tcp(session->context)) {
            session->tcp_active = true;
            session->connects_started++;
            session->state = MQTT_SESSION_CONNECTING;
            session->state_since_ms = now_ms;
        } else {
            enter_backoff(session, now_ms, false);
        }
        return;
    }

    if (session->state == MQTT_SESSION_CONNECTING &&
        (uint32_t)(now_ms - session->state_since_ms) >=
        MQTT_SESSION_TCP_TIMEOUT_MS) {
        enter_backoff(session, now_ms, true);
        return;
    }
    if (session->state == MQTT_SESSION_WAIT_CONNACK &&
        (uint32_t)(now_ms - session->state_since_ms) >=
        MQTT_SESSION_ACK_TIMEOUT_MS) {
        enter_backoff(session, now_ms, true);
        return;
    }
    if (session->state != MQTT_SESSION_ONLINE) return;

    if (session->waiting_pingresp) {
        if ((uint32_t)(now_ms - session->ping_since_ms) >=
            MQTT_SESSION_PING_TIMEOUT_MS) {
            enter_backoff(session, now_ms, true);
        }
        return;
    }
    if ((uint32_t)(now_ms - session->last_tx_ms) <
        MQTT_SESSION_PING_IDLE_MS) return;

    if (!session->waiting_pingqueue) {
        session->waiting_pingqueue = true;
        session->ping_queue_since_ms = now_ms;
    }
    session->ping_attempts++;
    packet_length = mqtt_wire_pingreq(session->tx, sizeof(session->tx));
    write_result = packet_length == 0U ? MQTT_SESSION_WRITE_FATAL :
            session->transport.write_tcp(session->context, session->tx,
                                         packet_length);
    if (write_result == MQTT_SESSION_WRITE_ACCEPTED) {
        session->last_tx_ms = now_ms;
        session->ping_since_ms = now_ms;
        session->waiting_pingqueue = false;
        session->waiting_pingresp = true;
        session->pings++;
    } else if (write_result == MQTT_SESSION_WRITE_FATAL) {
        enter_backoff(session, now_ms, true);
    } else if ((uint32_t)(now_ms - session->ping_queue_since_ms) >=
               MQTT_SESSION_PING_QUEUE_TIMEOUT_MS) {
        session->ping_queue_timeouts++;
        enter_backoff(session, now_ms, true);
    }
    /* Temporary backpressure is retried, but never beyond keepalive safety. */
}

void mqtt_session_tcp_connected(mqtt_session_t *session, uint32_t now_ms)
{
    size_t length;
    mqtt_session_write_result_t write_result;

    if (session == NULL || session->state != MQTT_SESSION_CONNECTING) return;
    length = mqtt_wire_connect(session->tx, sizeof(session->tx),
                               session->client_id,
                               MQTT_SESSION_KEEPALIVE_SEC);
    write_result = length == 0U ? MQTT_SESSION_WRITE_FATAL :
            session->transport.write_tcp(session->context, session->tx,
                                         length);
    if (write_result != MQTT_SESSION_WRITE_ACCEPTED) {
        enter_backoff(session, now_ms, true);
        return;
    }
    session->state = MQTT_SESSION_WAIT_CONNACK;
    session->state_since_ms = now_ms;
    session->last_tx_ms = now_ms;
    reset_stream(session);
}

void mqtt_session_tcp_lost(mqtt_session_t *session, uint32_t now_ms)
{
    if (session == NULL) return;
    /* Error callback's TCP PCB is already gone. NEVER close it here. */
    if (session->tcp_active) {
        enter_backoff(session, now_ms, false);
    }
}

static bool received_packet(mqtt_session_t *session,
                            const uint8_t *packet, size_t length,
                            uint32_t now_ms)
{
    uint8_t return_code;

    if (session->state == MQTT_SESSION_WAIT_CONNACK) {
        if (!mqtt_wire_parse_connack(packet, length, &return_code) ||
            return_code != 0U || packet[2] != 0U) return false;
        /* CONNECT used Clean Session=1, so broker cannot resume a session. */
        session->state = MQTT_SESSION_ONLINE;
        session->state_since_ms = now_ms;
        session->sessions_online++;
        return true;
    }
    if (session->state == MQTT_SESSION_ONLINE) {
        if (!session->waiting_pingresp ||
            !mqtt_wire_is_pingresp(packet, length)) return false;
        session->waiting_pingresp = false;
        session->ping_responses++;
        return true;
    }
    return false;
}

void mqtt_session_tcp_received(mqtt_session_t *session,
                               const uint8_t *data, size_t length,
                               uint32_t now_ms)
{
    size_t offset = 0U;
    size_t consumed;
    mqtt_stream_result_t result;

    if (session == NULL || (data == NULL && length != 0U)) return;
    if (!session->tcp_active ||
        (session->state != MQTT_SESSION_WAIT_CONNACK &&
         session->state != MQTT_SESSION_ONLINE)) return;

    while (offset < length) {
        result = mqtt_stream_feed(&session->rx, data + offset,
                                  length - offset, &consumed);
        offset += consumed;
        if (result == MQTT_STREAM_ERROR) {
            enter_backoff(session, now_ms, true);
            return;
        }
        if (result == MQTT_STREAM_PACKET) {
            if (!received_packet(session, mqtt_stream_packet(&session->rx),
                                 mqtt_stream_packet_length(&session->rx),
                                 now_ms)) {
                enter_backoff(session, now_ms, true);
                return;
            }
            mqtt_stream_reset(&session->rx);
        } else {
            break;
        }
    }
}

bool mqtt_session_publish(mqtt_session_t *session, const char *topic,
                          const uint8_t *payload, size_t bytes,
                          uint32_t now_ms)
{
    size_t length;
    mqtt_session_write_result_t write_result;

    if (session == NULL || session->state != MQTT_SESSION_ONLINE ||
        session->waiting_pingqueue || session->waiting_pingresp) return false;
    length = mqtt_wire_publish_qos0(session->tx, sizeof(session->tx),
                                    topic, payload, bytes);
    if (length == 0U) {
        return false;
    }
    write_result = session->transport.write_tcp(session->context, session->tx,
                                                length);
    if (write_result == MQTT_SESSION_WRITE_FATAL) {
        enter_backoff(session, now_ms, true);
        return false;
    }
    if (write_result != MQTT_SESSION_WRITE_ACCEPTED) return false;
    session->last_tx_ms = now_ms;
    session->publishes++;
    return true;
}
