#ifndef AAI12_TELEMETRY_RTOS_H
#define AAI12_TELEMETRY_RTOS_H

#include "telemetry_inbox.h"

/*
 * Stage 5C2: micro T-Kernel mutex port for the portable telemetry inbox.
 *
 * - Call telemetry_rtos_init() once from usermain(), before starting ANY
 *   telemetry producer or the CYW43 radio service task.
 * - All other functions are RTOS TASK CONTEXT ONLY (never call from an ISR).
 * - Every operation uses a non-waiting tk_loc_mtx(..., TMO_POL).
 * - A busy mutex causes TELEMETRY_INBOX_BUSY; producers must not wait.
 * - No network function is called by this module.
 * - Timestamps must come from the SAME monotonic millisecond epoch as
 *   the radio task's sys_now(). The producer must not call lwIP from
 *   another core just to obtain the time.
 *
 * The MQTT adapter may consume the inbox; producer tasks use telemetry_rtos_submit().
 */

/* Returns false if already initialized, or tk_cre_mtx() fails. */
bool telemetry_rtos_init(void);
/* Readiness is set before telemetry tasks start; no concurrent reset. */
bool telemetry_rtos_ready(void);

telemetry_inbox_result_t telemetry_rtos_submit(const uint8_t *bytes,
                                                size_t length,
                                                uint32_t created_ms);
telemetry_inbox_result_t telemetry_rtos_peek(uint32_t now_ms,
                                              uint8_t *out,
                                              size_t out_capacity,
                                              size_t *out_length);
telemetry_inbox_result_t telemetry_rtos_commit(void);
telemetry_inbox_result_t telemetry_rtos_snapshot(telemetry_inbox_stats_t *out);

#endif /* AAI12_TELEMETRY_RTOS_H */
