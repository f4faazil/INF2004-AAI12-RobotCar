#include "telemetry_queue.h"

#include <limits.h>
#include <string.h>

static void increment_saturated(uint32_t *counter)
{
    if (*counter != UINT32_MAX) {
        ++(*counter);
    }
}

void telemetry_queue_init(telemetry_queue_t *queue)
{
    if (queue != NULL) {
        memset(queue, 0, sizeof(*queue));
    }
}

telemetry_queue_result_t telemetry_queue_push(telemetry_queue_t *queue,
                                              const uint8_t *payload,
                                              size_t length,
                                              uint32_t created_ms)
{
    telemetry_queue_slot_t *slot;

    if ((queue == NULL) || (payload == NULL) || (length == 0U) ||
        (length > TELEMETRY_QUEUE_MAX_BYTES)) {
        return TELEMETRY_QUEUE_INVALID;
    }
    if (queue->count == TELEMETRY_QUEUE_CAPACITY) {
        increment_saturated(&queue->dropped_full);
        return TELEMETRY_QUEUE_FULL;
    }

    slot = &queue->slots[queue->tail];
    memcpy(slot->payload, payload, length);
    slot->length = (uint8_t)length;
    slot->created_ms = created_ms;
    queue->tail = (uint8_t)((queue->tail + 1U) % TELEMETRY_QUEUE_CAPACITY);
    ++queue->count;
    return TELEMETRY_QUEUE_OK;
}

telemetry_queue_result_t telemetry_queue_pop_fresh(telemetry_queue_t *queue,
                                                   uint32_t now_ms,
                                                   uint8_t *out_payload,
                                                   size_t out_capacity,
                                                   size_t *out_length)
{
    const telemetry_queue_slot_t *slot;

    if ((queue == NULL) || (out_payload == NULL) || (out_length == NULL) ||
        (out_capacity < TELEMETRY_QUEUE_MAX_BYTES)) {
        return TELEMETRY_QUEUE_INVALID;
    }

    while (queue->count != 0U) {
        slot = &queue->slots[queue->head];
        if ((uint32_t)(now_ms - slot->created_ms) > TELEMETRY_QUEUE_MAX_AGE_MS) {
            queue->head = (uint8_t)((queue->head + 1U) % TELEMETRY_QUEUE_CAPACITY);
            --queue->count;
            increment_saturated(&queue->dropped_stale);
            continue;
        }

        memcpy(out_payload, slot->payload, slot->length);
        *out_length = slot->length;
        queue->head = (uint8_t)((queue->head + 1U) % TELEMETRY_QUEUE_CAPACITY);
        --queue->count;
        return TELEMETRY_QUEUE_OK;
    }
    return TELEMETRY_QUEUE_EMPTY;
}

uint8_t telemetry_queue_count(const telemetry_queue_t *queue)
{
    return (queue == NULL) ? 0U : queue->count;
}
