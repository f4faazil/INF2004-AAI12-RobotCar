#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <tk/tkernel.h>
#include "telemetry_rtos.h"
#include "telemetry_tkernel_port.h"

/* telemetry_robot_submit() stamps each record with the Pico clock; the host test supplies a fake one. */
static uint64_t fake_clock_us;
uint64_t time_us_64(void) { return fake_clock_us; }

static int mock_held;
static int mock_busy;
static int mock_create_error;
static unsigned int creates, deletes, locks, unlocks, polls;

ID tk_cre_mtx(T_CMTX *cfg)
{
    assert(cfg != NULL);
    assert((cfg->mtxatr & TA_INHERIT) != 0U);
    creates++;
    if (mock_create_error) return E_TMOUT;
    return 7;
}
ER tk_del_mtx(ID id)
{
    assert(id == 7);
    deletes++;
    return E_OK;
}
ER tk_loc_mtx(ID id, TMO timeout)
{
    assert(id == 7);
    assert(timeout == TMO_POL); /* Never block. */
    polls++;
    if (mock_busy || mock_held) return E_TMOUT;
    mock_held = 1;
    locks++;
    return E_OK;
}
ER tk_unl_mtx(ID id)
{
    assert(id == 7);
    assert(mock_held);
    mock_held = 0;
    unlocks++;
    return E_OK;
}

static void test_initialize(void)
{
    uint8_t b = 1U;
    assert(telemetry_rtos_submit(&b, 1U, 0U) == TELEMETRY_INBOX_INVALID);
    mock_create_error = 1;
    assert(!telemetry_rtos_init());
    assert(creates == 1U);
    mock_create_error = 0;
    assert(telemetry_rtos_init());
    assert(!telemetry_rtos_init()); /* Never reinitialize a live queue. */
    assert(creates == 2U && deletes == 0U);
}

static void test_basic_copy_and_commit(void)
{
    uint8_t src[] = {0x04U, 0x25U, 0x03U};
    uint8_t out[TELEMETRY_QUEUE_MAX_BYTES] = {0};
    size_t len = 0U;
    assert(telemetry_rtos_submit(src, sizeof src, 100U) == TELEMETRY_INBOX_OK);
    src[0] = 0xffU;
    assert(telemetry_rtos_peek(200U, out, sizeof out, &len) == TELEMETRY_INBOX_OK);
    assert(len == 3U && out[0] == 4U && out[1] == 0x25U && out[2] == 3U);
    /* A peek does not remove the head. */
    assert(telemetry_rtos_peek(201U, out, sizeof out, &len) == TELEMETRY_INBOX_OK);
    assert(telemetry_rtos_commit() == TELEMETRY_INBOX_OK);
    assert(telemetry_rtos_peek(201U, out, sizeof out, &len) == TELEMETRY_INBOX_EMPTY);
}

static void test_nonblocking_busy(void)
{
    uint8_t b = 0x55U;
    uint8_t out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t len = 0U;
    telemetry_inbox_stats_t stats;
    unsigned int polls_before = polls;
    mock_busy = 1;
    assert(telemetry_rtos_submit(&b, 1U, 250U) == TELEMETRY_INBOX_BUSY);
    assert(telemetry_rtos_peek(251U, out, sizeof out, &len) == TELEMETRY_INBOX_BUSY);
    assert(telemetry_rtos_commit() == TELEMETRY_INBOX_BUSY);
    assert(telemetry_rtos_snapshot(&stats) == TELEMETRY_INBOX_BUSY);
    assert(polls - polls_before == 4U); /* Exactly one attempt each. */
    mock_busy = 0;
    assert(telemetry_rtos_snapshot(&stats) == TELEMETRY_INBOX_OK);
    assert(stats.queued == 0U);
}

static void test_full_and_stale(void)
{
    uint8_t b = 9U;
    uint8_t out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t len = 0U;
    telemetry_inbox_stats_t stats;
    unsigned int i;
    for (i = 0U; i < TELEMETRY_QUEUE_CAPACITY; i++) {
        assert(telemetry_rtos_submit(&b, 1U, 500U) == TELEMETRY_INBOX_OK);
    }
    assert(telemetry_rtos_submit(&b, 1U, 500U) == TELEMETRY_INBOX_FULL);
    assert(telemetry_rtos_snapshot(&stats) == TELEMETRY_INBOX_OK);
    assert(stats.queued == 16U && stats.dropped_full == 1U);
    assert(telemetry_rtos_peek(1501U, out, sizeof out, &len) == TELEMETRY_INBOX_EMPTY);
    assert(telemetry_rtos_snapshot(&stats) == TELEMETRY_INBOX_OK);
    assert(stats.queued == 0U && stats.dropped_stale == 16U);
}

static void test_invalid_and_balanced(void)
{
    telemetry_inbox_stats_t stats;
    uint8_t b = 0U;
    assert(telemetry_rtos_submit(NULL, 1U, 0U) == TELEMETRY_INBOX_INVALID);
    assert(telemetry_rtos_submit(&b, 0U, 0U) == TELEMETRY_INBOX_INVALID);
    assert(telemetry_rtos_snapshot(NULL) == TELEMETRY_INBOX_INVALID);
    assert(telemetry_rtos_snapshot(&stats) == TELEMETRY_INBOX_OK);
    assert(mock_held == 0);
    assert(locks == unlocks);
}

/* The robot-side entry point stamps the record with the clock in milliseconds and hands it to the inbox. */
static void test_robot_submit_uses_the_clock(void)
{
    static const uint8_t frame[3] = { 1U, 2U, 3U };
    uint8_t out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t len = 0U;
    fake_clock_us = 5000000ULL;                 /* 5000 ms */
    assert(telemetry_robot_submit(frame, sizeof frame) == (int)TELEMETRY_INBOX_OK);
    assert(telemetry_rtos_peek(5500U, out, sizeof out, &len) == TELEMETRY_INBOX_OK);   /* 500 ms old: fresh */
    assert(len == sizeof frame && memcmp(out, frame, sizeof frame) == 0);
    assert(telemetry_rtos_commit() == TELEMETRY_INBOX_OK);
    fake_clock_us = 6000000ULL;
    assert(telemetry_robot_submit(frame, sizeof frame) == (int)TELEMETRY_INBOX_OK);
    assert(telemetry_rtos_peek(7001U, out, sizeof out, &len) == TELEMETRY_INBOX_EMPTY);  /* 1001 ms old: stale, so proof the stamp was 6000 */
}

int main(void)
{
    test_initialize();
    test_basic_copy_and_commit();
    test_nonblocking_busy();
    test_full_and_stale();
    test_invalid_and_balanced();
    test_robot_submit_uses_the_clock();
    puts("PASS: 6 RTOS mutex-port mock tests (non-waiting lock)");
    return 0;
}
