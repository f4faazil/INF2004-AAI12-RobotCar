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
/* Largest telemetry record: 18 bytes (the robot's per-sensor record); the demo frames are 3. 32 leaves headroom and replaces the old
 * 128-byte slot, which made the inbox and the queue 2.2 KB each. A longer record is refused (TELEMETRY_QUEUE_INVALID). */
#define TELEMETRY_QUEUE_MAX_BYTES     32U
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


/*
 * Non-destructive consumer read for the MQTT owner task.
 * Drops any head records older than 1000 ms, then COPIES the first fresh
 * payload without removing it. If MQTT has no TX room, retry later; it
 * remains queued and can expire normally.
 *
 * The consumer must call commit_head ONLY after publish succeeds.
 * Both calls need serialization once an RTOS producer is attached.
 * Only one consumer may perform peek/commit; producers only append.
 */
telemetry_queue_result_t telemetry_queue_peek_fresh(telemetry_queue_t *queue,
                                                    uint32_t now_ms,
                                                    uint8_t *out_payload,
                                                    size_t out_capacity,
                                                    size_t *out_length);

/* Remove the current head AFTER MQTT accepted the publish request.
 * This is only a local enqueue success (QoS 0 has no broker ACK).
 * Returns EMPTY when no entry remains; does not increment drop counters.
 */
telemetry_queue_result_t telemetry_queue_commit_head(telemetry_queue_t *queue);

/* Snapshot helpers also require the RTOS wrapper's lock if shared. */
uint8_t telemetry_queue_count(const telemetry_queue_t *queue);

#endif /* AAI12_TELEMETRY_QUEUE_H */
