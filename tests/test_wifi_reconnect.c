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

#define SEC 1000000ULL

/* Joined but never given an address: after 30 s the policy asks the driver to
 * leave and join again.  Catches: a DHCP stall that is never recovered. */
static void test_no_ip_for_too_long_forces_a_rejoin(void)
{
    T_CYW43_UTK_JOIN_POLICY policy;
    T_CYW43_UTK_JOIN_ACTION action;
    uint64_t t;

    establish_first_link(&policy);              /* connected at 5 ms, no address yet */
    action = cyw43_utk_join_policy_update_ip(
            &policy, 5000ULL + 29ULL * SEC, 1, 0, LINK_JOIN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_NONE);
    assert(policy.no_ip_rejoins == 0U);

    t = 5000ULL + 30ULL * SEC;
    action = cyw43_utk_join_policy_update_ip(
            &policy, t, 1, 0, LINK_JOIN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_LEAVE);
    assert(policy.no_ip_rejoins == 1U);

    /* A delayed disconnection must complete before the new attempt starts. */
    action = cyw43_utk_join_policy_update_ip(
            &policy, t + SEC, 1, 0, LINK_JOIN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_NONE);
    assert(policy.state == CYW43_UTK_JOIN_DISCONNECTING);
    action = cyw43_utk_join_policy_update_ip(
            &policy, t + 2ULL * SEC, 0, 0, LINK_DOWN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_START);
    cyw43_utk_join_policy_request_result(&policy, t + 2ULL * SEC, 0);
    action = cyw43_utk_join_policy_update_ip(
            &policy, t + 4ULL * SEC, 1, 1, LINK_JOIN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_CONNECTED);
    action = cyw43_utk_join_policy_update_ip(
            &policy, t + 200ULL * SEC, 1, 1, LINK_JOIN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_NONE);
    assert(policy.no_ip_rejoins == 1U);
}

/* The driver's join state may clear before lwIP processes the link callback.
 * Do not rejoin while that stale link-up flag remains set. */
static void test_stale_link_up_waits_for_callback(void)
{
    T_CYW43_UTK_JOIN_POLICY policy;
    uint64_t t = 5000ULL + 30ULL * SEC;

    establish_first_link(&policy);
    assert(cyw43_utk_join_policy_update_ip(
            &policy, t, 1, 0, LINK_JOIN, TIMEOUT_ERROR)
           == CYW43_UTK_JOIN_ACTION_LEAVE);
    assert(cyw43_utk_join_policy_update_ip(
            &policy, t + SEC, 1, 0, LINK_DOWN, TIMEOUT_ERROR)
           == CYW43_UTK_JOIN_ACTION_NONE);
    assert(policy.state == CYW43_UTK_JOIN_DISCONNECTING);
    assert(cyw43_utk_join_policy_update_ip(
            &policy, t + 2ULL * SEC, 0, 0, LINK_DOWN, TIMEOUT_ERROR)
           == CYW43_UTK_JOIN_ACTION_START);
}

/* A missing disconnect event cannot leave recovery stuck forever. */
static void test_missing_link_down_uses_bounded_fallback(void)
{
    T_CYW43_UTK_JOIN_POLICY policy;
    uint64_t t = 5000ULL + 30ULL * SEC;

    establish_first_link(&policy);
    assert(cyw43_utk_join_policy_update_ip(
            &policy, t, 1, 0, LINK_JOIN, TIMEOUT_ERROR)
           == CYW43_UTK_JOIN_ACTION_LEAVE);
    assert(cyw43_utk_join_policy_update_ip(
            &policy, t + CYW43_UTK_LEAVE_TIMEOUT_US - 1ULL,
            1, 0, LINK_JOIN, TIMEOUT_ERROR) == CYW43_UTK_JOIN_ACTION_NONE);
    assert(cyw43_utk_join_policy_update_ip(
            &policy, t + CYW43_UTK_LEAVE_TIMEOUT_US,
            1, 0, LINK_JOIN, TIMEOUT_ERROR) == CYW43_UTK_JOIN_ACTION_START);
}

/* An address that arrives in time never triggers anything.  Catches: a rejoin
 * on a healthy link. */
static void test_an_address_in_time_never_rejoins(void)
{
    T_CYW43_UTK_JOIN_POLICY policy;
    uint64_t t;

    establish_first_link(&policy);
    for(t = 1ULL; t <= 300ULL; t++) {
        assert(cyw43_utk_join_policy_update_ip(
                &policy, 5000ULL + t * SEC, 1, 1, LINK_JOIN, TIMEOUT_ERROR)
               == CYW43_UTK_JOIN_ACTION_NONE);
    }
    assert(policy.no_ip_rejoins == 0U);
}

/* The 30 s are counted from the last time an address was held, so a lease that
 * is lost later gets the same wait.  Catches: a timer measured from boot. */
static void test_losing_the_address_starts_a_new_wait(void)
{
    T_CYW43_UTK_JOIN_POLICY policy;
    const uint64_t last_ok = 5000ULL + 100ULL * SEC;

    establish_first_link(&policy);
    assert(cyw43_utk_join_policy_update_ip(
            &policy, last_ok, 1, 1, LINK_JOIN, TIMEOUT_ERROR)
           == CYW43_UTK_JOIN_ACTION_NONE);
    assert(cyw43_utk_join_policy_update_ip(
            &policy, last_ok + 10ULL * SEC, 1, 0, LINK_JOIN, TIMEOUT_ERROR)
           == CYW43_UTK_JOIN_ACTION_NONE);
    assert(cyw43_utk_join_policy_update_ip(
            &policy, last_ok + 29ULL * SEC, 1, 0, LINK_JOIN, TIMEOUT_ERROR)
           == CYW43_UTK_JOIN_ACTION_NONE);
    assert(cyw43_utk_join_policy_update_ip(
            &policy, last_ok + 30ULL * SEC, 1, 0, LINK_JOIN, TIMEOUT_ERROR)
           == CYW43_UTK_JOIN_ACTION_LEAVE);
}

/* If it still gets no address after the rejoin, it tries again every 30 s.
 * Catches: giving up after one rejoin. */
static void test_a_repeated_stall_keeps_rejoining(void)
{
    T_CYW43_UTK_JOIN_POLICY policy;
    uint64_t t = 5000ULL;
    unsigned int round;

    establish_first_link(&policy);
    for(round = 1U; round <= 3U; round++) {
        t += 31ULL * SEC;     /* the previous attempt reconnected 1 s after it began */
        assert(cyw43_utk_join_policy_update_ip(
                &policy, t, 1, 0, LINK_JOIN, TIMEOUT_ERROR)
               == CYW43_UTK_JOIN_ACTION_LEAVE);
        assert(policy.no_ip_rejoins == round);
        assert(cyw43_utk_join_policy_update_ip(
                &policy, t + SEC / 2ULL, 0, 0, LINK_DOWN, TIMEOUT_ERROR)
               == CYW43_UTK_JOIN_ACTION_START);
        cyw43_utk_join_policy_request_result(&policy, t + SEC / 2ULL, 0);
        assert(cyw43_utk_join_policy_update_ip(
                &policy, t + SEC, 1, 0, LINK_JOIN, TIMEOUT_ERROR)
               == CYW43_UTK_JOIN_ACTION_CONNECTED);
    }
}

/* A link that is down is the existing recovery path's job, not the no-IP
 * check's.  Catches: LEAVE during a link loss. */
static void test_no_ip_check_leaves_link_loss_alone(void)
{
    T_CYW43_UTK_JOIN_POLICY policy;
    T_CYW43_UTK_JOIN_ACTION action;

    establish_first_link(&policy);
    action = cyw43_utk_join_policy_update_ip(
            &policy, 5000ULL + 40ULL * SEC, 0, 0, LINK_DOWN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_NONE);
    assert(policy.state == CYW43_UTK_JOIN_LOSS_GRACE);
    assert(policy.no_ip_rejoins == 0U);
    action = cyw43_utk_join_policy_update_ip(
            &policy, 5000ULL + 42ULL * SEC, 0, 0, LINK_DOWN, TIMEOUT_ERROR);
    assert(action == CYW43_UTK_JOIN_ACTION_START);
}

/* The old entry point assumes an address, so existing callers never rejoin.
 * Catches: changing behaviour for callers that do not track DHCP. */
static void test_old_update_never_leaves(void)
{
    T_CYW43_UTK_JOIN_POLICY policy;

    establish_first_link(&policy);
    assert(cyw43_utk_join_policy_update(
            &policy, 5000ULL + 500ULL * SEC, 1, LINK_JOIN, TIMEOUT_ERROR)
           == CYW43_UTK_JOIN_ACTION_NONE);
    assert(policy.no_ip_rejoins == 0U);
}

int main(void)
{
    test_failed_join_retry_success();
    test_temporary_link_loss();
    test_sustained_link_loss_recovery();
    test_late_success_during_backoff();
    test_timeout_and_retry_backoff();
    test_bounded_backoff();
    test_no_ip_for_too_long_forces_a_rejoin();
    test_stale_link_up_waits_for_callback();
    test_missing_link_down_uses_bounded_fallback();
    test_an_address_in_time_never_rejoins();
    test_losing_the_address_starts_a_new_wait();
    test_a_repeated_stall_keeps_rejoining();
    test_no_ip_check_leaves_link_loss_alone();
    test_old_update_never_leaves();
    puts("PASS: 14 WiFi reassociation policy tests");
    return 0;
}
