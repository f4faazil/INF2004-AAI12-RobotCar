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

static void test_invalid_and_128_boundary(void)
{
    telemetry_queue_t q;
    uint8_t p[TELEMETRY_QUEUE_MAX_BYTES], out[TELEMETRY_QUEUE_MAX_BYTES];
    size_t length = 0U;
    telemetry_queue_init(&q);
    memset(p, 0x3CU, sizeof(p));
    assert(telemetry_queue_push(&q, p, 0U, 0U) == TELEMETRY_QUEUE_INVALID);
    assert(telemetry_queue_push(&q, p, 129U, 0U) == TELEMETRY_QUEUE_INVALID);
    assert(telemetry_queue_push(&q, NULL, 1U, 0U) == TELEMETRY_QUEUE_INVALID);
    assert(telemetry_queue_push(NULL, p, 1U, 0U) == TELEMETRY_QUEUE_INVALID);
    assert(telemetry_queue_count(&q) == 0U);
    assert(telemetry_queue_push(&q, p, sizeof(p), 0U) == TELEMETRY_QUEUE_OK);
    assert(telemetry_queue_pop_fresh(&q, 0U, out, 127U, &length) == TELEMETRY_QUEUE_INVALID);
    assert(telemetry_queue_count(&q) == 1U);
    assert(telemetry_queue_pop_fresh(&q, 0U, out, sizeof(out), &length) == TELEMETRY_QUEUE_OK);
    assert(length == 128U && memcmp(p, out, sizeof(p)) == 0);
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

int main(void)
{
    test_empty();
    test_fifo_and_copy();
    test_full_drops_new();
    test_stale_and_exact_boundary();
    test_wraparound_age();
    test_invalid_and_128_boundary();
    test_drop_counter_saturates();
    puts("PASS: 7 host-side telemetry queue tests");
    return 0;
}
