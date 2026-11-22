#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../app_program/telemetry_queue.h"

static void test_empty(void)
{
    telemetry_queue_t q;
    uint8_t out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t length = 123U;
    telemetry_queue_init(&q);
    assert(telemetry_queue_pop_fresh(&q, 0U, out, sizeof(out), &length) == TELEMETRY_QUEUE_EMPTY);
    assert(length == 123U);
    assert(telemetry_queue_count(&q) == 0U);
}

static void test_fifo_and_copy(void)
{
    telemetry_queue_t q;
    uint8_t input[] = {0x12U, 0x34U};
    uint8_t out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t length = 0U;
    telemetry_queue_init(&q);
    assert(telemetry_queue_push(&q, input, 2U, 100U) == TELEMETRY_QUEUE_OK);
    input[0] = 0xFFU;
    assert(telemetry_queue_push(&q, input, 1U, 101U) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_pop_fresh(&q, 120U, out, sizeof(out), &length) == TELEMETRY_QUEUE_OK);
    assert(length == 2U && out[0] == 0x12U && out[1] == 0x34U);
    assert(telemetry_queue_pop_fresh(&q, 120U, out, sizeof(out), &length) == TELEMETRY_QUEUE_OK);
    assert(length == 1U && out[0] == 0xFFU);
}

static void test_full_drops_new(void)
{
    telemetry_queue_t q;
    uint8_t p[2] = {0U, 0U};
    uint8_t out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t length;
    unsigned i;
    telemetry_queue_init(&q);
    for (i = 0; i < TELEMETRY_QUEUE_CAPACITY; ++i) {
        p[0] = (uint8_t)i;
        assert(telemetry_queue_push(&q, p, 2U, 0U) == TELEMETRY_QUEUE_OK);
    }
    p[0] = 99U;
    assert(telemetry_queue_push(&q, p, 2U, 0U) == TELEMETRY_QUEUE_FULL);
    assert(q.dropped_full == 1U && telemetry_queue_count(&q) == TELEMETRY_QUEUE_CAPACITY);
    assert(telemetry_queue_pop_fresh(&q, 100U, out, sizeof(out), &length) == TELEMETRY_QUEUE_OK);
    assert(out[0] == 0U);  /* Oldest was preserved; new was dropped. */
    assert(telemetry_queue_push(&q, p, 2U, 100U) == TELEMETRY_QUEUE_OK);
    for (i = 1; i < TELEMETRY_QUEUE_CAPACITY; ++i) {
        assert(telemetry_queue_pop_fresh(&q, 100U, out, sizeof(out), &length) == TELEMETRY_QUEUE_OK);
        assert(out[0] == (uint8_t)i);
    }
    assert(telemetry_queue_pop_fresh(&q, 100U, out, sizeof(out), &length) == TELEMETRY_QUEUE_OK);
    assert(out[0] == 99U);
}

static void test_stale_and_exact_boundary(void)
{
    telemetry_queue_t q;
    uint8_t p = 10U, out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t length = 0U;
    telemetry_queue_init(&q);
    assert(telemetry_queue_push(&q, &p, 1U, 0U) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_pop_fresh(&q, 1000U, out, sizeof(out), &length) == TELEMETRY_QUEUE_OK);
    assert(out[0] == 10U);
    assert(telemetry_queue_push(&q, &p, 1U, 0U) == TELEMETRY_QUEUE_OK);
    p = 11U;
    assert(telemetry_queue_push(&q, &p, 1U, 1001U) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_pop_fresh(&q, 1001U, out, sizeof(out), &length) == TELEMETRY_QUEUE_OK);
    assert(out[0] == 11U && q.dropped_stale == 1U);
}

static void test_wraparound_age(void)
{
    telemetry_queue_t q;
    uint8_t p = 0xA5U, out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t length;
    telemetry_queue_init(&q);
    assert(telemetry_queue_push(&q, &p, 1U, UINT32_MAX - 100U) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_pop_fresh(&q, 899U, out, sizeof(out), &length) == TELEMETRY_QUEUE_OK);
    assert(out[0] == 0xA5U);
    assert(telemetry_queue_push(&q, &p, 1U, UINT32_MAX - 100U) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_pop_fresh(&q, 900U, out, sizeof(out), &length) == TELEMETRY_QUEUE_EMPTY);
    assert(q.dropped_stale == 1U);
}

static void test_invalid_and_max_boundary(void)
{
    telemetry_queue_t q;
    uint8_t p[TELEMETRY_QUEUE_MAX_BYTES], out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t length = 0U;
    telemetry_queue_init(&q);
    memset(p, 0x3CU, sizeof(p));
    assert(telemetry_queue_push(&q, p, 0U, 0U) == TELEMETRY_QUEUE_INVALID);
    assert(telemetry_queue_push(&q, p, TELEMETRY_QUEUE_MAX_BYTES + 1U, 0U) == TELEMETRY_QUEUE_INVALID);
    assert(telemetry_queue_push(&q, NULL, 1U, 0U) == TELEMETRY_QUEUE_INVALID);
    assert(telemetry_queue_push(NULL, p, 1U, 0U) == TELEMETRY_QUEUE_INVALID);
    assert(telemetry_queue_count(&q) == 0U);
    assert(telemetry_queue_push(&q, p, sizeof(p), 0U) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_pop_fresh(&q, 0U, out, TELEMETRY_QUEUE_MAX_BYTES - 1U, &length) == TELEMETRY_QUEUE_INVALID);
    assert(telemetry_queue_count(&q) == 1U);
    assert(telemetry_queue_pop_fresh(&q, 0U, out, sizeof(out), &length) == TELEMETRY_QUEUE_OK);
    assert(length == TELEMETRY_QUEUE_MAX_BYTES && memcmp(p, out, sizeof(p)) == 0);
}

static void test_drop_counter_saturates(void)
{
    telemetry_queue_t q;
    uint8_t p = 0x11U;
    unsigned i;
    telemetry_queue_init(&q);
    for (i = 0; i < TELEMETRY_QUEUE_CAPACITY; ++i) {
        assert(telemetry_queue_push(&q, &p, 1U, 0U) == TELEMETRY_QUEUE_OK);
    }
    q.dropped_full = UINT32_MAX;
    assert(telemetry_queue_push(&q, &p, 1U, 0U) == TELEMETRY_QUEUE_FULL);
    assert(q.dropped_full == UINT32_MAX);
}


static void test_peek_retries_without_removal(void)
{
    telemetry_queue_t q;
    uint8_t p[] = {0x04U, 0x25U, 0x03U};
    uint8_t out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t length = 0U;

    telemetry_queue_init(&q);
    assert(telemetry_queue_push(&q, p, sizeof p, 200U) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_peek_fresh(&q, 201U, out, sizeof out, &length) == TELEMETRY_QUEUE_OK);
    assert(length == 3U && memcmp(out, p, 3U) == 0);
    assert(telemetry_queue_count(&q) == 1U);

    /* Simulate a failed/blocked MQTT attempt: NO commit, same frame retried. */
    memset(out, 0, sizeof out);
    assert(telemetry_queue_peek_fresh(&q, 350U, out, sizeof out, &length) == TELEMETRY_QUEUE_OK);
    assert(length == 3U && memcmp(out, p, 3U) == 0);
    assert(telemetry_queue_count(&q) == 1U);

    /* Simulate MQTT accepting local TX: NOW commit. */
    assert(telemetry_queue_commit_head(&q) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_count(&q) == 0U);
    assert(telemetry_queue_commit_head(&q) == TELEMETRY_QUEUE_EMPTY);
    assert(telemetry_queue_peek_fresh(&q, 351U, out, sizeof out, &length) == TELEMETRY_QUEUE_EMPTY);
}

static void test_peek_stale_boundary_and_fifo(void)
{
    telemetry_queue_t q;
    uint8_t old = 10U, fresh = 20U;
    uint8_t out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t length = 0U;

    telemetry_queue_init(&q);
    assert(telemetry_queue_push(&q, &old, 1U, 0U) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_push(&q, &fresh, 1U, 1U) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_peek_fresh(&q, 1000U, out, sizeof out, &length) == TELEMETRY_QUEUE_OK);
    assert(out[0] == old && telemetry_queue_count(&q) == 2U);
    /* At 1001 the old frame expires. */
    assert(telemetry_queue_peek_fresh(&q, 1001U, out, sizeof out, &length) == TELEMETRY_QUEUE_OK);
    assert(out[0] == fresh && q.dropped_stale == 1U);
    assert(telemetry_queue_count(&q) == 1U);
    assert(telemetry_queue_commit_head(&q) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_count(&q) == 0U);
}

static void test_peek_stale_when_transport_blocked(void)
{
    telemetry_queue_t q;
    uint8_t p = 0x5aU, out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t length = 123U;

    telemetry_queue_init(&q);
    assert(telemetry_queue_push(&q, &p, 1U, 20U) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_peek_fresh(&q, 1020U, out, sizeof out, &length) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_count(&q) == 1U);
    assert(telemetry_queue_peek_fresh(&q, 1021U, out, sizeof out, &length) == TELEMETRY_QUEUE_EMPTY);
    assert(q.dropped_stale == 1U && telemetry_queue_count(&q) == 0U);
    assert(telemetry_queue_peek_fresh(&q, 2000U, out, sizeof out, &length) == TELEMETRY_QUEUE_EMPTY);
    assert(q.dropped_stale == 1U);
}

static void test_peek_wraparound_and_invalid(void)
{
    telemetry_queue_t q;
    uint8_t p = 0xA5U, out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t length = 51U;

    telemetry_queue_init(&q);
    assert(telemetry_queue_commit_head(NULL) == TELEMETRY_QUEUE_INVALID);
    assert(telemetry_queue_peek_fresh(NULL, 0U, out, sizeof out, &length) == TELEMETRY_QUEUE_INVALID);
    assert(telemetry_queue_peek_fresh(&q, 0U, out, TELEMETRY_QUEUE_MAX_BYTES - 1U, &length) == TELEMETRY_QUEUE_INVALID);
    assert(telemetry_queue_peek_fresh(&q, 0U, NULL, sizeof out, &length) == TELEMETRY_QUEUE_INVALID);
    assert(telemetry_queue_peek_fresh(&q, 0U, out, sizeof out, NULL) == TELEMETRY_QUEUE_INVALID);
    assert(length == 51U);

    assert(telemetry_queue_push(&q, &p, 1U, UINT32_MAX - 100U) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_peek_fresh(&q, 899U, out, sizeof out, &length) == TELEMETRY_QUEUE_OK);
    assert(out[0] == p && telemetry_queue_count(&q) == 1U);
    assert(telemetry_queue_peek_fresh(&q, 900U, out, sizeof out, &length) == TELEMETRY_QUEUE_EMPTY);
    assert(q.dropped_stale == 1U);
}

int main(void)
{
    test_empty();
    test_fifo_and_copy();
    test_full_drops_new();
    test_stale_and_exact_boundary();
    test_wraparound_age();
    test_invalid_and_max_boundary();
    test_drop_counter_saturates();
    test_peek_retries_without_removal();
    test_peek_stale_boundary_and_fifo();
    test_peek_stale_when_transport_blocked();
    test_peek_wraparound_and_invalid();
    puts("PASS: 11 host-side telemetry queue tests (including peek/commit)");
    return 0;
}
