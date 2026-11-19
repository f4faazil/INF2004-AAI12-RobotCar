#ifndef AAI12_MQTT_HEARTBEAT_H
#define AAI12_MQTT_HEARTBEAT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MQTT_HEARTBEAT_INTERVAL_MS 10000U
#define MQTT_HEARTBEAT_PAYLOAD_BYTES 64U

typedef struct {
    uint32_t boot_id;
    uint32_t sequence;
    uint32_t online_session;
    uint32_t last_publish_ms;
    bool online;
} mqtt_heartbeat_t;

void mqtt_heartbeat_init(mqtt_heartbeat_t *heartbeat, uint32_t boot_id);

/* Call from the MQTT owner only. A new session waits one full interval;
 * sequence remains boot-monotonic across reconnects. */
void mqtt_heartbeat_set_session(mqtt_heartbeat_t *heartbeat, bool online,
                                uint32_t online_session, uint32_t now_ms);

bool mqtt_heartbeat_due(const mqtt_heartbeat_t *heartbeat, uint32_t now_ms);

/* Produces: v=1,b=hhhhhhhh,seq=N,up=N. Returns zero unless due or on error.
 * Preparing does not advance state, so temporary TX backpressure is safe. */
size_t mqtt_heartbeat_prepare(const mqtt_heartbeat_t *heartbeat,
                              uint32_t now_ms, char *out, size_t capacity);

/* Call only after MQTT accepted the complete QoS-0 PUBLISH locally. */
void mqtt_heartbeat_published(mqtt_heartbeat_t *heartbeat, uint32_t now_ms);

#endif /* AAI12_MQTT_HEARTBEAT_H */
