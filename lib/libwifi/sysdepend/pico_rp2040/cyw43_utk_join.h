/* Driver-independent WiFi reassociation state policy. */

#ifndef CYW43_UTK_JOIN_H
#define CYW43_UTK_JOIN_H

#include <stdint.h>

#define CYW43_UTK_JOIN_WAIT_SCAN   0U
#define CYW43_UTK_JOIN_JOINING     1U
#define CYW43_UTK_JOIN_CONNECTED   2U
#define CYW43_UTK_JOIN_LOSS_GRACE  3U
#define CYW43_UTK_JOIN_BACKOFF     4U
#define CYW43_UTK_JOIN_DISCONNECTING 5U

#define CYW43_UTK_JOIN_RETRY_MIN_MS 1000U
#define CYW43_UTK_JOIN_RETRY_MAX_MS 30000U
#define CYW43_UTK_JOIN_TIMEOUT_US   30000000ULL
#define CYW43_UTK_LINK_LOSS_GRACE_US 1000000ULL
#define CYW43_UTK_LEAVE_TIMEOUT_US   5000000ULL
/* Joined but still no IP address after this long: leave and join again.  The
 * link can stay up while DHCP never completes (seen on a phone hotspot after
 * quick reboots: minutes with no address), and nothing else would restart it. */
#define CYW43_UTK_NO_IP_TIMEOUT_US  30000000ULL

typedef enum {
    CYW43_UTK_JOIN_ACTION_NONE = 0,
    CYW43_UTK_JOIN_ACTION_START,
    CYW43_UTK_JOIN_ACTION_CONNECTED,
    CYW43_UTK_JOIN_ACTION_LEAVE,        /* leave the network, then join again */
} T_CYW43_UTK_JOIN_ACTION;

typedef struct {
    uint32_t state;
    uint32_t started;
    uint32_t complete;
    int32_t result;
    uint32_t elapsed_ms;
    uint32_t attempts;
    uint32_t failures;
    uint32_t successes;
    uint32_t recoveries;
    uint32_t link_losses;
    uint32_t retry_delay_ms;
    uint32_t retry_remaining_ms;
    uint32_t consecutive_failures;
    uint32_t recovery_pending;
    uint64_t attempt_started_us;
    uint64_t retry_at_us;
    uint64_t link_down_started_us;
    uint64_t ip_wait_started_us;        /* when the wait for an IP address began */
    uint64_t leave_started_us;
    uint32_t no_ip_rejoins;             /* times the no-IP timeout forced a rejoin */
} T_CYW43_UTK_JOIN_POLICY;

uint32_t cyw43_utk_join_retry_delay_ms(uint32_t failures);
void cyw43_utk_join_policy_init(T_CYW43_UTK_JOIN_POLICY *policy,
                                int32_t initial_result);
void cyw43_utk_join_policy_begin(T_CYW43_UTK_JOIN_POLICY *policy,
                                 uint64_t now_us);
void cyw43_utk_join_policy_request_result(
        T_CYW43_UTK_JOIN_POLICY *policy, uint64_t now_us,
        int32_t request_result);
T_CYW43_UTK_JOIN_ACTION cyw43_utk_join_policy_update(
        T_CYW43_UTK_JOIN_POLICY *policy, uint64_t now_us,
        uint32_t link_up, int32_t link_status, int32_t timeout_result);

/* The same, and also told whether an IP address has been obtained.  Returns
 * CYW43_UTK_JOIN_ACTION_LEAVE when the link has been up without an address for
 * CYW43_UTK_NO_IP_TIMEOUT_US. */
T_CYW43_UTK_JOIN_ACTION cyw43_utk_join_policy_update_ip(
        T_CYW43_UTK_JOIN_POLICY *policy, uint64_t now_us,
        uint32_t link_up, uint32_t have_ip, int32_t link_status,
        int32_t timeout_result);

#endif
