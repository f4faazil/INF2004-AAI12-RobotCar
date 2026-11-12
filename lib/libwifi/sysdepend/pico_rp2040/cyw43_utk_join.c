/* Driver-independent WiFi reassociation state policy. */

#include <stddef.h>

#include "cyw43_utk_join.h"

uint32_t cyw43_utk_join_retry_delay_ms(uint32_t failures)
{
    uint32_t delay = CYW43_UTK_JOIN_RETRY_MIN_MS;

    while(failures > 1U && delay < CYW43_UTK_JOIN_RETRY_MAX_MS) {
        if(delay > CYW43_UTK_JOIN_RETRY_MAX_MS / 2U) {
            delay = CYW43_UTK_JOIN_RETRY_MAX_MS;
        } else {
            delay *= 2U;
        }
        failures--;
    }
    return delay;
}

void cyw43_utk_join_policy_init(T_CYW43_UTK_JOIN_POLICY *policy,
                                int32_t initial_result)
{
    if(policy == NULL) return;
    *policy = (T_CYW43_UTK_JOIN_POLICY) {
        .state = CYW43_UTK_JOIN_WAIT_SCAN,
        .result = initial_result,
    };
}

void cyw43_utk_join_policy_begin(T_CYW43_UTK_JOIN_POLICY *policy,
                                 uint64_t now_us)
{
    if(policy == NULL) return;
    policy->attempt_started_us = now_us;
    policy->state = CYW43_UTK_JOIN_JOINING;
    policy->started = 1;
    policy->complete = 0;
    policy->elapsed_ms = 0;
    policy->retry_delay_ms = 0;
    policy->retry_remaining_ms = 0;
    policy->attempts++;
}

static void schedule_retry(T_CYW43_UTK_JOIN_POLICY *policy,
                           uint64_t now_us, int32_t failure)
{
    policy->consecutive_failures++;
    policy->retry_delay_ms = cyw43_utk_join_retry_delay_ms(
            policy->consecutive_failures);
    policy->retry_at_us = now_us
                        + (uint64_t)policy->retry_delay_ms * 1000ULL;
    policy->retry_remaining_ms = policy->retry_delay_ms;
    policy->state = CYW43_UTK_JOIN_BACKOFF;
    policy->complete = 1;
    policy->result = failure;
    policy->failures++;
}

void cyw43_utk_join_policy_request_result(
        T_CYW43_UTK_JOIN_POLICY *policy, uint64_t now_us,
        int32_t request_result)
{
    if(policy == NULL || policy->state != CYW43_UTK_JOIN_JOINING
       || request_result == 0) return;
    schedule_retry(policy, now_us, request_result);
}

static T_CYW43_UTK_JOIN_ACTION complete_join(
        T_CYW43_UTK_JOIN_POLICY *policy, uint64_t now_us)
{
    policy->elapsed_ms = (uint32_t)
            ((now_us - policy->attempt_started_us) / 1000ULL);
    policy->state = CYW43_UTK_JOIN_CONNECTED;
    policy->complete = 1;
    policy->result = 0;
    policy->successes++;
    policy->retry_delay_ms = 0;
    policy->retry_remaining_ms = 0;
    policy->consecutive_failures = 0;
    if(policy->recovery_pending) {
        policy->recoveries++;
        policy->recovery_pending = 0;
    }
    return CYW43_UTK_JOIN_ACTION_CONNECTED;
}

T_CYW43_UTK_JOIN_ACTION cyw43_utk_join_policy_update(
        T_CYW43_UTK_JOIN_POLICY *policy, uint64_t now_us,
        uint32_t link_up, int32_t link_status, int32_t timeout_result)
{
    if(policy == NULL) return CYW43_UTK_JOIN_ACTION_NONE;

    switch(policy->state) {
    case CYW43_UTK_JOIN_JOINING:
        policy->elapsed_ms = (uint32_t)
                ((now_us - policy->attempt_started_us) / 1000ULL);
        if(link_up) {
            return complete_join(policy, now_us);
        }
        if(link_status < 0) {
            schedule_retry(policy, now_us, link_status);
        } else if(now_us - policy->attempt_started_us
                  >= CYW43_UTK_JOIN_TIMEOUT_US) {
            schedule_retry(policy, now_us, timeout_result);
        }
        break;

    case CYW43_UTK_JOIN_CONNECTED:
        if(!link_up) {
            policy->link_down_started_us = now_us;
            policy->state = CYW43_UTK_JOIN_LOSS_GRACE;
        }
        break;

    case CYW43_UTK_JOIN_LOSS_GRACE:
        if(link_up) {
            policy->state = CYW43_UTK_JOIN_CONNECTED;
        } else if(now_us - policy->link_down_started_us
                  >= CYW43_UTK_LINK_LOSS_GRACE_US) {
            policy->link_losses++;
            policy->recovery_pending = 1;
            cyw43_utk_join_policy_begin(policy, now_us);
            return CYW43_UTK_JOIN_ACTION_START;
        }
        break;

    case CYW43_UTK_JOIN_BACKOFF:
        /* The driver can report success just after an attempt timeout. */
        if(link_up) {
            return complete_join(policy, now_us);
        }
        if(now_us >= policy->retry_at_us) {
            cyw43_utk_join_policy_begin(policy, now_us);
            return CYW43_UTK_JOIN_ACTION_START;
        }
        policy->retry_remaining_ms = (uint32_t)
                ((policy->retry_at_us - now_us + 999ULL) / 1000ULL);
        break;

    default:
        break;
    }
    return CYW43_UTK_JOIN_ACTION_NONE;
}
