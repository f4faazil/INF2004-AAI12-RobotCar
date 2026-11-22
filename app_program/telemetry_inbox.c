#include "telemetry_inbox.h"

static telemetry_inbox_result_t map_queue_result(telemetry_queue_result_t result)
{
    switch (result) {
    case TELEMETRY_QUEUE_OK:      return TELEMETRY_INBOX_OK;
    case TELEMETRY_QUEUE_EMPTY:   return TELEMETRY_INBOX_EMPTY;
    case TELEMETRY_QUEUE_FULL:    return TELEMETRY_INBOX_FULL;
    default:                      return TELEMETRY_INBOX_INVALID;
    }
}

bool telemetry_inbox_init(telemetry_inbox_t *inbox,
                          telemetry_inbox_try_lock_fn try_lock,
                          telemetry_inbox_unlock_fn unlock,
                          void *lock_context)
{
    if (inbox == NULL || try_lock == NULL || unlock == NULL) return false;
    /* This is deliberately not an idempotent reset of a live queue. */
    telemetry_queue_init(&inbox->queue);
    inbox->try_lock = try_lock;
    inbox->unlock = unlock;
    inbox->lock_context = lock_context;
    inbox->initialized = true;
    return true;
}

telemetry_inbox_result_t telemetry_inbox_submit(telemetry_inbox_t *inbox,
                                                const uint8_t *bytes,
                                                size_t length,
                                                uint32_t created_ms)
{
    telemetry_queue_result_t result;
    if (inbox == NULL || !inbox->initialized || bytes == NULL || length == 0U ||
        length > TELEMETRY_QUEUE_MAX_BYTES) return TELEMETRY_INBOX_INVALID;
    if (!inbox->try_lock(inbox->lock_context)) return TELEMETRY_INBOX_BUSY;
    result = telemetry_queue_push(&inbox->queue, bytes, length, created_ms);
    inbox->unlock(inbox->lock_context);
    return map_queue_result(result);
}

telemetry_inbox_result_t telemetry_inbox_peek(telemetry_inbox_t *inbox,
                                              uint32_t now_ms,
                                              uint8_t *out,
                                              size_t out_capacity,
                                              size_t *out_length)
{
    telemetry_queue_result_t result;
    if (inbox == NULL || !inbox->initialized || out == NULL || out_length == NULL ||
        out_capacity < TELEMETRY_QUEUE_MAX_BYTES) return TELEMETRY_INBOX_INVALID;
    if (!inbox->try_lock(inbox->lock_context)) return TELEMETRY_INBOX_BUSY;
    result = telemetry_queue_peek_fresh(&inbox->queue, now_ms, out,
                                         out_capacity, out_length);
    inbox->unlock(inbox->lock_context);
    return map_queue_result(result);
}

telemetry_inbox_result_t telemetry_inbox_commit(telemetry_inbox_t *inbox)
{
    telemetry_queue_result_t result;
    if (inbox == NULL || !inbox->initialized) return TELEMETRY_INBOX_INVALID;
    if (!inbox->try_lock(inbox->lock_context)) return TELEMETRY_INBOX_BUSY;
    result = telemetry_queue_commit_head(&inbox->queue);
    inbox->unlock(inbox->lock_context);
    return map_queue_result(result);
}

telemetry_inbox_result_t telemetry_inbox_snapshot(telemetry_inbox_t *inbox,
                                                  telemetry_inbox_stats_t *out)
{
    if (inbox == NULL || !inbox->initialized || out == NULL)
        return TELEMETRY_INBOX_INVALID;
    if (!inbox->try_lock(inbox->lock_context)) return TELEMETRY_INBOX_BUSY;
    out->queued = telemetry_queue_count(&inbox->queue);
    out->dropped_full = inbox->queue.dropped_full;
    out->dropped_stale = inbox->queue.dropped_stale;
    inbox->unlock(inbox->lock_context);
    return TELEMETRY_INBOX_OK;
}
