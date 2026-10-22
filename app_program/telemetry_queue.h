#ifndef AAI12_TELEMETRY_QUEUE_H
#define AAI12_TELEMETRY_QUEUE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * PROVISIONAL Week-8 Tribunal queue limits, to be revisited once the
 * final binary payload schemas and RAM measurements are available.
 * Queue uses fixed storage; nothing here allocates memory dynamically.
 *
 * IMPORTANT: This is the portable queue *core*, NOT the RTOS adapter.
 * It is not safe for simultaneous calls on multiple cores/tasks/ISRs.
 * An RTOS integration layer must serialize push/pop and make its producer
 * submission non-blocking. Never call it from an ISR without that design.
 */
#define TELEMETRY_QUEUE_CAPACITY      16U
#define TELEMETRY_QUEUE_MAX_BYTES    128U
#define TELEMETRY_QUEUE_MAX_AGE_MS  1000U

typedef enum {
    TELEMETRY_QUEUE_OK = 0,
    TELEMETRY_QUEUE_EMPTY,
    TELEMETRY_QUEUE_FULL,
    TELEMETRY_QUEUE_INVALID
} telemetry_queue_result_t;

typedef struct {
    uint32_t created_ms;
    uint8_t length;
    uint8_t payload[TELEMETRY_QUEUE_MAX_BYTES];
} telemetry_queue_slot_t;

typedef struct {
    telemetry_queue_slot_t slots[TELEMETRY_QUEUE_CAPACITY];
    uint32_t dropped_full;
    uint32_t dropped_stale;
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} telemetry_queue_t;

/* Call once before use; never reset during simultaneous access. */
void telemetry_queue_init(telemetry_queue_t *queue);

/*
 * Non-waiting queue-core operation: when full, DROP THE NEW message.
 * timestamp is milliseconds from one monotonic clock (not wall time).
 * Copy input bytes; callers can reuse their input buffer after return.
 * Actual RTOS non-blocking guarantee additionally requires a non-waiting
 * synchronization strategy in the integration layer.
 */
telemetry_queue_result_t telemetry_queue_push(telemetry_queue_t *queue,
                                              const uint8_t *payload,
                                              size_t length,
                                              uint32_t created_ms);

/*
 * Discard old records before returning a fresh one. Exactly 1000 ms old
 * is still fresh, and 1001 ms old is stale. Uses unsigned subtraction,
 * valid across a 32-bit timer wrap for realistic (<2^31 ms) delays.
 * Output buffer must have room for MAX_BYTES so caller does not need
 * a racy size-peek. The output is copied, not borrowed from the ring.
 */
telemetry_queue_result_t telemetry_queue_pop_fresh(telemetry_queue_t *queue,
                                                   uint32_t now_ms,
                                                   uint8_t *out_payload,
                                                   size_t out_capacity,
                                                   size_t *out_length);

/* Snapshot helpers also require the RTOS wrapper's lock if shared. */
uint8_t telemetry_queue_count(const telemetry_queue_t *queue);

#endif /* AAI12_TELEMETRY_QUEUE_H */
