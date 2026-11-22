#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "telemetry_inbox.h"

typedef struct {
    int held;
    int force_busy;
    unsigned int locks;
    unsigned int unlocks;
} mock_guard_t;

static bool try_lock(void *context)
{
    mock_guard_t *g = (mock_guard_t *)context;
    if (g->held || g->force_busy) return false;
    g->held = 1;
    ++g->locks;
    return true;
}
static void unlock(void *context)
{
    mock_guard_t *g = (mock_guard_t *)context;
    assert(g->held);
    g->held = 0;
    ++g->unlocks;
}

static void setup(telemetry_inbox_t *inbox, mock_guard_t *g)
{
    memset(inbox, 0, sizeof(*inbox));
    memset(g, 0, sizeof(*g));
    assert(telemetry_inbox_init(inbox, try_lock, unlock, g));
}

static void test_initialization(void)
{
    telemetry_inbox_t box = {0};
    mock_guard_t guard = {0};
    uint8_t sample = 5U;
    assert(!telemetry_inbox_init(NULL, try_lock, unlock, &guard));
    assert(!telemetry_inbox_init(&box, NULL, unlock, &guard));
    assert(!telemetry_inbox_init(&box, try_lock, NULL, &guard));
    assert(telemetry_inbox_submit(&box, &sample, 1U, 0U) == TELEMETRY_INBOX_INVALID);
    setup(&box, &guard);
}

static void test_copy_and_commit(void)
{
    telemetry_inbox_t box;
    mock_guard_t guard;
    uint8_t sample[] = {0x04U, 0x25U, 0x03U};
    uint8_t output[TELEMETRY_QUEUE_MAX_BYTES] = {0};
    size_t n = 0U;
    setup(&box, &guard);
    assert(telemetry_inbox_submit(&box, sample, sizeof sample, 100U) == TELEMETRY_INBOX_OK);
    sample[0] = 0xffU;
    assert(telemetry_inbox_peek(&box, 200U, output, sizeof output, &n) == TELEMETRY_INBOX_OK);
    assert(n == 3U && output[0] == 0x04U && output[1] == 0x25U && output[2] == 0x03U);
    assert(telemetry_inbox_commit(&box) == TELEMETRY_INBOX_OK);
    assert(telemetry_inbox_peek(&box, 201U, output, sizeof output, &n) == TELEMETRY_INBOX_EMPTY);
    assert(guard.locks == guard.unlocks);
}

static void test_busy_submit(void)
{
    telemetry_inbox_t box;
    mock_guard_t guard;
    telemetry_inbox_stats_t stats;
    uint8_t sample = 1U;
    setup(&box, &guard);
    guard.force_busy = 1;
    assert(telemetry_inbox_submit(&box, &sample, 1U, 0U) == TELEMETRY_INBOX_BUSY);
    guard.force_busy = 0;
    assert(telemetry_inbox_snapshot(&box, &stats) == TELEMETRY_INBOX_OK);
    assert(stats.queued == 0U && stats.dropped_full == 0U);
}

static void test_busy_consumer(void)
{
    telemetry_inbox_t box;
    mock_guard_t guard;
    uint8_t sample = 9U, out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t n = 0U;
    setup(&box, &guard);
    assert(telemetry_inbox_submit(&box, &sample, 1U, 0U) == TELEMETRY_INBOX_OK);
    guard.force_busy = 1;
    assert(telemetry_inbox_peek(&box, 0U, out, sizeof out, &n) == TELEMETRY_INBOX_BUSY);
    assert(telemetry_inbox_commit(&box) == TELEMETRY_INBOX_BUSY);
    guard.force_busy = 0;
    assert(telemetry_inbox_peek(&box, 0U, out, sizeof out, &n) == TELEMETRY_INBOX_OK);
    assert(out[0] == 9U);
}

static void test_full(void)
{
    telemetry_inbox_t box;
    mock_guard_t guard;
    telemetry_inbox_stats_t stats;
    uint8_t sample = 1U;
    unsigned int i;
    setup(&box, &guard);
    for (i = 0U; i < TELEMETRY_QUEUE_CAPACITY; i++) {
        assert(telemetry_inbox_submit(&box, &sample, 1U, 0U) == TELEMETRY_INBOX_OK);
    }
    assert(telemetry_inbox_submit(&box, &sample, 1U, 0U) == TELEMETRY_INBOX_FULL);
    assert(telemetry_inbox_snapshot(&box, &stats) == TELEMETRY_INBOX_OK);
    assert(stats.queued == 16U && stats.dropped_full == 1U);
}

static void test_stale_1001(void)
{
    telemetry_inbox_t box;
    mock_guard_t guard;
    telemetry_inbox_stats_t stats;
    uint8_t sample = 1U, out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t n = 0U;
    setup(&box, &guard);
    assert(telemetry_inbox_submit(&box, &sample, 1U, 100U) == TELEMETRY_INBOX_OK);
    assert(telemetry_inbox_peek(&box, 1101U, out, sizeof out, &n) == TELEMETRY_INBOX_EMPTY);
    assert(telemetry_inbox_snapshot(&box, &stats) == TELEMETRY_INBOX_OK);
    assert(stats.dropped_stale == 1U && stats.queued == 0U);
}

static void test_exact_age_1000(void)
{
    telemetry_inbox_t box;
    mock_guard_t guard;
    uint8_t sample = 2U, out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t n = 0U;
    setup(&box, &guard);
    assert(telemetry_inbox_submit(&box, &sample, 1U, 100U) == TELEMETRY_INBOX_OK);
    assert(telemetry_inbox_peek(&box, 1100U, out, sizeof out, &n) == TELEMETRY_INBOX_OK);
}

static void test_retry_and_fifo(void)
{
    telemetry_inbox_t box;
    mock_guard_t guard;
    uint8_t a = 0x11U, b = 0x22U, out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t n = 0U;
    setup(&box, &guard);
    assert(telemetry_inbox_submit(&box, &a, 1U, 0U) == TELEMETRY_INBOX_OK);
    assert(telemetry_inbox_submit(&box, &b, 1U, 1U) == TELEMETRY_INBOX_OK);
    assert(telemetry_inbox_peek(&box, 2U, out, sizeof out, &n) == TELEMETRY_INBOX_OK);
    assert(out[0] == a);
    assert(telemetry_inbox_peek(&box, 3U, out, sizeof out, &n) == TELEMETRY_INBOX_OK);
    assert(out[0] == a); /* Still head when publish cannot accept */
    assert(telemetry_inbox_commit(&box) == TELEMETRY_INBOX_OK);
    assert(telemetry_inbox_peek(&box, 3U, out, sizeof out, &n) == TELEMETRY_INBOX_OK);
    assert(out[0] == b);
}

static void test_wraparound(void)
{
    telemetry_inbox_t box;
    mock_guard_t guard;
    uint8_t sample = 1U, out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t n = 0U;
    setup(&box, &guard);
    assert(telemetry_inbox_submit(&box, &sample, 1U, UINT32_MAX - 400U) == TELEMETRY_INBOX_OK);
    assert(telemetry_inbox_peek(&box, 100U, out, sizeof out, &n) == TELEMETRY_INBOX_OK);
}

static void test_invalid_args(void)
{
    telemetry_inbox_t box;
    mock_guard_t guard;
    uint8_t sample = 1U, out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t n = 0U;
    setup(&box, &guard);
    assert(telemetry_inbox_submit(NULL, &sample, 1U, 0U) == TELEMETRY_INBOX_INVALID);
    assert(telemetry_inbox_submit(&box, NULL, 1U, 0U) == TELEMETRY_INBOX_INVALID);
    assert(telemetry_inbox_submit(&box, &sample, 0U, 0U) == TELEMETRY_INBOX_INVALID);
    assert(telemetry_inbox_submit(&box, &sample, TELEMETRY_QUEUE_MAX_BYTES + 1U, 0U) == TELEMETRY_INBOX_INVALID);
    assert(telemetry_inbox_peek(&box, 0U, out, 1U, &n) == TELEMETRY_INBOX_INVALID);
    assert(telemetry_inbox_snapshot(&box, NULL) == TELEMETRY_INBOX_INVALID);
}

static void test_full_max_bytes(void)
{
    telemetry_inbox_t box;
    mock_guard_t guard;
    uint8_t sample[TELEMETRY_QUEUE_MAX_BYTES], out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t n = 0U;
    setup(&box, &guard);
    memset(sample, 0xa5, sizeof sample);
    assert(telemetry_inbox_submit(&box, sample, sizeof sample, 5U) == TELEMETRY_INBOX_OK);
    assert(telemetry_inbox_peek(&box, 5U, out, sizeof out, &n) == TELEMETRY_INBOX_OK);
    assert(n == sizeof sample && memcmp(out, sample, n) == 0);
}

int main(void)
{
    test_initialization();
    test_copy_and_commit();
    test_busy_submit();
    test_busy_consumer();
    test_full();
    test_stale_1001();
    test_exact_age_1000();
    test_retry_and_fifo();
    test_wraparound();
    test_invalid_args();
    test_full_max_bytes();
    puts("PASS: 11 guarded telemetry inbox host tests");
    return 0;
}
