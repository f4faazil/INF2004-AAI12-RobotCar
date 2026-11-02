#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "mock_mutex_port.h"
#include "telemetry_crosscore_demo.h"
#include "telemetry_hump_demo.h"
#include "telemetry_rtos.h"

static uint64_t clock_us = 123456000ULL;
uint64_t time_us_64(void) { return clock_us; }

static void assert_packet(uint8_t expected_height)
{
    uint8_t data[TELEMETRY_QUEUE_MAX_BYTES];
    size_t len = 0U;
    uint8_t status = 0, height = 0;
    bool out_of_range = true, fault = true;
    assert(telemetry_rtos_peek((uint32_t)(clock_us / 1000ULL), data,
                               sizeof data, &len) == TELEMETRY_INBOX_OK);
    assert(len == AAI12_HUMP_DEMO_BYTES);
    assert(telemetry_hump_demo_decode(data, len, &status, &height,
                                       &out_of_range, &fault));
    assert(data[0] == 0x04U);
    assert(status == 6U && height == expected_height);
    assert(!out_of_range && !fault);
}

int main(void)
{
    telemetry_inbox_stats_t stats;
    uint8_t data[TELEMETRY_QUEUE_MAX_BYTES];
    size_t len = 0U;
    unsigned int i;
    assert(telemetry_rtos_init());
    assert(telemetry_rtos_ready());

    /* Producer writes a valid 3-byte diagnostic sample. */
    assert(telemetry_crosscore_demo_emit(1U) == TELEMETRY_INBOX_OK);
    assert_packet(1U);
    assert(telemetry_rtos_commit() == TELEMETRY_INBOX_OK);
    assert(telemetry_rtos_peek((uint32_t)(clock_us / 1000ULL), data,
                               sizeof data, &len) == TELEMETRY_INBOX_EMPTY);

    /* Non-waiting try-lock failure never writes to the queue. */
    mock_mutex_fail_after(1);
    assert(telemetry_crosscore_demo_emit(2U) == TELEMETRY_INBOX_BUSY);
    assert(telemetry_rtos_snapshot(&stats) == TELEMETRY_INBOX_OK);
    assert(stats.queued == 0U);

    /* Multiple records retain FIFO order, including modulo-height wrap. */
    assert(telemetry_crosscore_demo_emit(20U) == TELEMETRY_INBOX_OK);
    assert(telemetry_crosscore_demo_emit(140U) == TELEMETRY_INBOX_OK);
    assert_packet(20U);
    assert(telemetry_rtos_commit() == TELEMETRY_INBOX_OK);
    assert_packet(20U);
    assert(telemetry_rtos_commit() == TELEMETRY_INBOX_OK);

    /* Queue capacity is bounded; 17th producer submission drops newest. */
    for (i = 0U; i < TELEMETRY_QUEUE_CAPACITY; ++i) {
        assert(telemetry_crosscore_demo_emit(i) == TELEMETRY_INBOX_OK);
    }
    assert(telemetry_crosscore_demo_emit(200U) == TELEMETRY_INBOX_FULL);
    assert(telemetry_rtos_snapshot(&stats) == TELEMETRY_INBOX_OK);
    assert(stats.queued == TELEMETRY_QUEUE_CAPACITY);
    assert(stats.dropped_full == 1U);

    /* Radio task's monotonic now discards all 1001-ms-old samples. */
    clock_us += 1001000ULL;
    assert(telemetry_rtos_peek((uint32_t)(clock_us / 1000ULL), data,
                               sizeof data, &len) == TELEMETRY_INBOX_EMPTY);
    assert(telemetry_rtos_snapshot(&stats) == TELEMETRY_INBOX_OK);
    assert(stats.queued == 0U);
    assert(stats.dropped_stale == TELEMETRY_QUEUE_CAPACITY);

    puts("PASS: cross-core demo producer mock (encode, non-waiting, FIFO, full, stale)");
    return 0;
}
