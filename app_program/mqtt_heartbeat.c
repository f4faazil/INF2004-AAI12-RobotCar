#include "mqtt_heartbeat.h"

static void put_u32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value >> 24);
    out[1] = (uint8_t)(value >> 16);
    out[2] = (uint8_t)(value >> 8);
    out[3] = (uint8_t)value;
}

void mqtt_heartbeat_init(mqtt_heartbeat_t *heartbeat, uint32_t boot_id)
{
    if (heartbeat == NULL) return;
    heartbeat->boot_id = boot_id;
    heartbeat->sequence = 0U;
    heartbeat->online_session = 0U;
    heartbeat->last_publish_ms = 0U;
    heartbeat->online = false;
}

void mqtt_heartbeat_set_session(mqtt_heartbeat_t *heartbeat, bool online,
                                uint32_t online_session, uint32_t now_ms)
{
    if (heartbeat == NULL) return;
    if (!online) {
        heartbeat->online = false;
        return;
    }
    if (!heartbeat->online || heartbeat->online_session != online_session) {
        heartbeat->online = true;
        heartbeat->online_session = online_session;
        heartbeat->last_publish_ms = now_ms;
    }
}

bool mqtt_heartbeat_due(const mqtt_heartbeat_t *heartbeat, uint32_t now_ms)
{
    return heartbeat != NULL && heartbeat->online &&
           (uint32_t)(now_ms - heartbeat->last_publish_ms) >=
               MQTT_HEARTBEAT_INTERVAL_MS;
}

size_t mqtt_heartbeat_prepare(const mqtt_heartbeat_t *heartbeat,
                              uint32_t now_ms, uint8_t *out, size_t capacity)
{
    if (heartbeat == NULL || out == NULL ||
        capacity < MQTT_HEARTBEAT_PAYLOAD_BYTES ||
        !mqtt_heartbeat_due(heartbeat, now_ms)) return 0U;
    out[0] = (uint8_t)MQTT_HEARTBEAT_TYPE;
    put_u32(&out[1], heartbeat->boot_id);
    put_u32(&out[5], heartbeat->sequence);
    put_u32(&out[9], now_ms);
    return MQTT_HEARTBEAT_PAYLOAD_BYTES;
}

void mqtt_heartbeat_published(mqtt_heartbeat_t *heartbeat, uint32_t now_ms)
{
    if (heartbeat == NULL || !heartbeat->online) return;
    heartbeat->sequence++;
    heartbeat->last_publish_ms = now_ms;
}
