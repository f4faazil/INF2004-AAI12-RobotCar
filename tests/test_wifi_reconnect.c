#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "cyw43_utk_join.h"

#define LINK_DOWN       0
#define LINK_JOIN       1
#define LINK_NONET     -2
#define REQUEST_ERROR  -7
#define TIMEOUT_ERROR  -110

static void begin_accepted(T_CYW43_UTK_JOIN_POLICY *policy, uint64_t now)
{
    cyw43_utk_join_policy_begin(policy, now);
    cyw43_utk_join_policy_request_result(policy, now, 0);
    assert(policy->state == CYW43_UTK_JOIN_JOINING);
    assert(policy->started == 1U);
    assert(policy->complete == 0U);
}

static void establish_first_link(T_CYW43_UTK_JOIN_POLICY *policy)
{
    T_CYW43_UTK_JOIN_ACTION action;

    cyw43_utk_join_policy_init(policy, -5);
    begin_accepted(policy, 0);
    action = cyw43_utk_join_policy_update(
            policy, 5000, 1, LINK_JOIN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_CONNECTED);
    assert(policy->state == CYW43_UTK_JOIN_CONNECTED);
    assert(policy->attempts == 1U);
    assert(policy->successes == 1U);
    assert(policy->failures == 0U);
    assert(policy->recoveries == 0U);
    assert(policy->complete == 1U);
    assert(policy->result == 0);
}

static void test_failed_join_retry_success(void)
{
    T_CYW43_UTK_JOIN_POLICY policy;
    T_CYW43_UTK_JOIN_ACTION action;
    const uint64_t start = 100000ULL;

    cyw43_utk_join_policy_init(&policy, -5);
    assert(policy.state == CYW43_UTK_JOIN_WAIT_SCAN);
    begin_accepted(&policy, start);
    assert(policy.attempts == 1U);

    cyw43_utk_join_policy_request_result(
            &policy, start, REQUEST_ERROR);
    assert(policy.state == CYW43_UTK_JOIN_BACKOFF);
    assert(policy.complete == 1U);
    assert(policy.result == REQUEST_ERROR);
    assert(policy.failures == 1U);
    assert(policy.retry_delay_ms == 1000U);
    assert(policy.retry_remaining_ms == 1000U);

    action = cyw43_utk_join_policy_update(
            &policy, start + 999999ULL, 0, LINK_DOWN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_NONE);
    assert(policy.state == CYW43_UTK_JOIN_BACKOFF);
    assert(policy.retry_remaining_ms == 1U);

    action = cyw43_utk_join_policy_update(
            &policy, start + 1000000ULL, 0, LINK_DOWN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_START);
    assert(policy.state == CYW43_UTK_JOIN_JOINING);
    assert(policy.attempts == 2U);
    assert(policy.complete == 0U);
    cyw43_utk_join_policy_request_result(
            &policy, start + 1000000ULL, 0);

    action = cyw43_utk_join_policy_update(
            &policy, start + 1005000ULL, 1, LINK_JOIN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_CONNECTED);
    assert(policy.state == CYW43_UTK_JOIN_CONNECTED);
    assert(policy.attempts == 2U);
    assert(policy.failures == 1U);
    assert(policy.successes == 1U);
    assert(policy.recoveries == 0U);
    assert(policy.consecutive_failures == 0U);
    assert(policy.retry_delay_ms == 0U);
    assert(policy.retry_remaining_ms == 0U);
    assert(policy.result == 0);
}

static void test_temporary_link_loss(void)
{
    T_CYW43_UTK_JOIN_POLICY policy;
    T_CYW43_UTK_JOIN_ACTION action;

    establish_first_link(&policy);
    action = cyw43_utk_join_policy_update(
            &policy, 2000000ULL, 0, LINK_DOWN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_NONE);
    assert(policy.state == CYW43_UTK_JOIN_LOSS_GRACE);

    action = cyw43_utk_join_policy_update(
            &policy, 2999999ULL, 1, LINK_JOIN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_NONE);
    assert(policy.state == CYW43_UTK_JOIN_CONNECTED);
    assert(policy.attempts == 1U);
    assert(policy.successes == 1U);
    assert(policy.link_losses == 0U);
    assert(policy.recoveries == 0U);
}

static void test_sustained_link_loss_recovery(void)
{
    T_CYW43_UTK_JOIN_POLICY policy;
    T_CYW43_UTK_JOIN_ACTION action;

    establish_first_link(&policy);
    (void)cyw43_utk_join_policy_update(
            &policy, 2000000ULL, 0, LINK_DOWN, TIMEOUT_ERROR);
    action = cyw43_utk_join_policy_update(
            &policy, 3000000ULL, 0, LINK_DOWN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_START);
    assert(policy.state == CYW43_UTK_JOIN_JOINING);
    assert(policy.attempts == 2U);
    assert(policy.link_losses == 1U);
    assert(policy.recovery_pending == 1U);
    cyw43_utk_join_policy_request_result(&policy, 3000000ULL, 0);

    action = cyw43_utk_join_policy_update(
            &policy, 3200000ULL, 1, LINK_JOIN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_CONNECTED);
    assert(policy.state == CYW43_UTK_JOIN_CONNECTED);
    assert(policy.successes == 2U);
    assert(policy.recoveries == 1U);
    assert(policy.link_losses == 1U);
    assert(policy.recovery_pending == 0U);
    assert(policy.failures == 0U);
}

static void test_late_success_during_backoff(void)
{
    T_CYW43_UTK_JOIN_POLICY policy;
    T_CYW43_UTK_JOIN_ACTION action;

    cyw43_utk_join_policy_init(&policy, -5);
    begin_accepted(&policy, 0);
    action = cyw43_utk_join_policy_update(
            &policy, 10000ULL, 0, LINK_NONET, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_NONE);
    assert(policy.state == CYW43_UTK_JOIN_BACKOFF);
    assert(policy.failures == 1U);

    action = cyw43_utk_join_policy_update(
            &policy, 500000ULL, 1, LINK_JOIN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_CONNECTED);
    assert(policy.state == CYW43_UTK_JOIN_CONNECTED);
    assert(policy.attempts == 1U);
    assert(policy.failures == 1U);
    assert(policy.successes == 1U);
    assert(policy.consecutive_failures == 0U);
    assert(policy.retry_delay_ms == 0U);
}

static void test_timeout_and_retry_backoff(void)
{
    T_CYW43_UTK_JOIN_POLICY policy;
    T_CYW43_UTK_JOIN_ACTION action;
    uint64_t retry_at;

    cyw43_utk_join_policy_init(&policy, -5);
    begin_accepted(&policy, 0);
    action = cyw43_utk_join_policy_update(
            &policy, CYW43_UTK_JOIN_TIMEOUT_US - 1ULL,
            0, LINK_DOWN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_NONE);
    assert(policy.state == CYW43_UTK_JOIN_JOINING);

    action = cyw43_utk_join_policy_update(
            &policy, CYW43_UTK_JOIN_TIMEOUT_US,
            0, LINK_DOWN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_NONE);
    assert(policy.state == CYW43_UTK_JOIN_BACKOFF);
    assert(policy.result == TIMEOUT_ERROR);
    assert(policy.failures == 1U);
    assert(policy.retry_delay_ms == 1000U);

    retry_at = policy.retry_at_us;
    action = cyw43_utk_join_policy_update(
            &policy, retry_at, 0, LINK_DOWN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_START);
    assert(policy.attempts == 2U);
    cyw43_utk_join_policy_request_result(
            &policy, retry_at, REQUEST_ERROR);
    assert(policy.state == CYW43_UTK_JOIN_BACKOFF);
    assert(policy.failures == 2U);
    assert(policy.consecutive_failures == 2U);
    assert(policy.retry_delay_ms == 2000U);

    action = cyw43_utk_join_policy_update(
            &policy, policy.retry_at_us, 0, LINK_DOWN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_START);
    assert(policy.attempts == 3U);
    assert(policy.complete == 0U);
}

static void test_bounded_backoff(void)
{
    assert(cyw43_utk_join_retry_delay_ms(0U) == 1000U);
    assert(cyw43_utk_join_retry_delay_ms(1U) == 1000U);
    assert(cyw43_utk_join_retry_delay_ms(2U) == 2000U);
    assert(cyw43_utk_join_retry_delay_ms(3U) == 4000U);
    assert(cyw43_utk_join_retry_delay_ms(4U) == 8000U);
    assert(cyw43_utk_join_retry_delay_ms(5U) == 16000U);
    assert(cyw43_utk_join_retry_delay_ms(6U) == 30000U);
    assert(cyw43_utk_join_retry_delay_ms(UINT32_MAX) == 30000U);
}

int main(void)
{
    test_failed_join_retry_success();
    test_temporary_link_loss();
    test_sustained_link_loss_recovery();
    test_late_success_during_backoff();
    test_timeout_and_retry_backoff();
    test_bounded_backoff();
    puts("PASS: 6 WiFi reassociation policy tests");
    return 0;
}
