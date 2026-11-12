#ifndef AAI12_MQTT_LWIP_H
#define AAI12_MQTT_LWIP_H

#include <stdint.h>

struct netif;

typedef struct {
    uint32_t initialized;
    uint32_t state;
    uint32_t tcp_active;
    uint32_t connects_started;
    uint32_t sessions_online;
    uint32_t session_failures;
    uint32_t poll_calls;
    uint32_t last_poll_ms;
    uint32_t last_tx_ms;
    uint32_t ping_attempts;
    uint32_t pings_queued;
    uint32_t ping_responses;
    uint32_t waiting_pingqueue;
    uint32_t ping_queue_timeouts;
    uint32_t waiting_pingresp;
    uint32_t tcp_write_calls;
    uint32_t tcp_write_failures;
    uint32_t tcp_write_fatal_errors;
    uint32_t tcp_output_failures;
    uint32_t tcp_output_pending;
    uint32_t tcp_output_retries;
    uint32_t tcp_output_timeouts;
    uint32_t tcp_sent_callbacks;
    uint32_t tcp_acked_bytes;
    uint32_t last_tcp_ack_ms;
    int32_t last_tcp_write_result;
    int32_t last_tcp_output_result;
    uint32_t tcp_errors;
    int32_t last_tcp_error;
} T_MQTT_LWIP_STATUS;

/* Called ONLY by the existing CYW43/lwIP owner task, never from another
 * T-Kernel task, a timer interrupt, or the second core. */
void mqtt_lwip_poll(struct netif *netif);

/* Owner-task-only snapshot. Cross-core readers use cyw43_utk_get_status(),
 * which mirrors these fields through its existing seqlock. */
void mqtt_lwip_get_status(T_MQTT_LWIP_STATUS *status);

#endif /* AAI12_MQTT_LWIP_H */
