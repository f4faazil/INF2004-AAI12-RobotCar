#ifndef AAI12_TELEMETRY_INBOX_H
#define AAI12_TELEMETRY_INBOX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "telemetry_queue.h"

/*
 * Stage 5C1: portable, lock-injected wrapper around the existing queue core.
 *
 * Functions below are task-context ONLY, not interrupt-safe. Call init once
 * BEFORE starting any producer or WiFi-owner task. The RTOS integration must
 * supply a truly non-waiting try_lock (e.g. tk_loc_mtx(..., TMO_POL)) and
 * unlock callbacks for one mutex shared across both cores.
 *
 * No network operations or waits are made while holding this mutex.
 * Exactly ONE consumer is allowed to peek/commit; any number of producers
 * may submit. Producers append only; they must not pop/clear the queue.
 *
 * The passed monotonic created_ms/now_ms must use the SAME clock as sys_now()
 * in the MQTT owner task. No ISR callers. QoS 0 publish acceptance is NOT
 * broker confirmation.
 */
typedef bool (*telemetry_inbox_try_lock_fn)(void *context);
typedef void (*telemetry_inbox_unlock_fn)(void *context);

typedef enum {
    TELEMETRY_INBOX_OK = 0,
    TELEMETRY_INBOX_EMPTY,
    TELEMETRY_INBOX_FULL,
    TELEMETRY_INBOX_BUSY,
    TELEMETRY_INBOX_INVALID
} telemetry_inbox_result_t;

typedef struct {
    telemetry_queue_t queue;
    telemetry_inbox_try_lock_fn try_lock;
    telemetry_inbox_unlock_fn unlock;
    void *lock_context;
    bool initialized;
} telemetry_inbox_t;

typedef struct {
    uint8_t queued;
    uint32_t dropped_full;
    uint32_t dropped_stale;
} telemetry_inbox_stats_t;

/* Must not race with any inbox calls, including snapshots. */
bool telemetry_inbox_init(telemetry_inbox_t *inbox,
                          telemetry_inbox_try_lock_fn try_lock,
                          telemetry_inbox_unlock_fn unlock,
                          void *lock_context);

/* Producer: never waits for a busy lock, never calls MQTT/lwIP. */
telemetry_inbox_result_t telemetry_inbox_submit(telemetry_inbox_t *inbox,
                                                const uint8_t *bytes,
                                                size_t length,
                                                uint32_t created_ms);

/* WiFi owner ONLY: copies current fresh head, leaving it queued. */
telemetry_inbox_result_t telemetry_inbox_peek(telemetry_inbox_t *inbox,
                                              uint32_t now_ms,
                                              uint8_t *out,
                                              size_t out_capacity,
                                              size_t *out_length);

/* WiFi owner ONLY: call after a successful local MQTT publish. */
telemetry_inbox_result_t telemetry_inbox_commit(telemetry_inbox_t *inbox);

/* A bounded non-waiting snapshot for debugging and qualification. */
telemetry_inbox_result_t telemetry_inbox_snapshot(telemetry_inbox_t *inbox,
                                                  telemetry_inbox_stats_t *out);

#endif
