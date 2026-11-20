#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <tk/tkernel.h>
#include "telemetry_rtos.h"

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

int main(void)
{
    test_initialize();
    test_basic_copy_and_commit();
    test_nonblocking_busy();
    test_full_and_stale();
    test_invalid_and_balanced();
    puts("PASS: 5 RTOS mutex-port mock tests (non-waiting lock)");
    return 0;
}
