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
    uint32_t max_poll_gap_ms;
    uint32_t owner_calls;
    uint32_t owner_last_ms;
    uint32_t owner_max_gap_ms;
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
    uint32_t disconnect_callbacks;
    uint32_t diagnostic_publishes;
    uint32_t diagnostic_publish_failures;
    uint32_t boot_nonce;
    uint32_t reset_reason;
    uint32_t timer_tick_config;
    uint32_t usb_irq_count;
    uint32_t usb_service_count;
    uint32_t usb_event_count;
    uint32_t usb_link_state;
    uint32_t wifi_link_changes;
    uint32_t dhcp_changes;
    uint32_t last_wifi_change_ms;
    uint32_t last_dhcp_change_ms;
} T_MQTT_LWIP_STATUS;

/* Called ONLY by the existing CYW43/lwIP owner task, never from another
 * T-Kernel task, a timer interrupt, or the second core. */
void mqtt_lwip_poll(struct netif *netif);

/* Cheap owner-loop observation.  Called once per CYW43 iteration before the
 * MQTT poll; it performs no allocation, network operation, or formatting. */
void mqtt_lwip_observe_owner(uint32_t wifi_link_up, uint32_t dhcp_complete,
                             uint32_t join_attempts,
                             uint32_t join_recoveries,
                             uint32_t usb_irq_count,
                             uint32_t usb_service_count,
                             uint32_t usb_event_count,
                             uint32_t usb_link_state);

/* Owner-task-only snapshot. Cross-core readers use cyw43_utk_get_status(),
 * which mirrors these fields through its existing seqlock. */
void mqtt_lwip_get_status(T_MQTT_LWIP_STATUS *status);

#endif /* AAI12_MQTT_LWIP_H */
