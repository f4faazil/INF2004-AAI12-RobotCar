#ifndef AAI12_TELEMETRY_TKERNEL_PORT_H
#define AAI12_TELEMETRY_TKERNEL_PORT_H

/*
 * Narrow platform ABI used by the portable RTOS telemetry wrapper.
 * Intentionally includes NO C standard headers or T-Kernel headers.
 *
 * Do not include tk/tkernel.h in translation units that also include
 * telemetry_inbox.h/telemetry_queue.h, because the supplied micro T-Kernel
 * defines size_t incompatibly with the ARM C library.
 */
/* Called from usermain() before starting network/telemetry tasks. */
int telemetry_rtos_boot_init(void);
/* Non-waiting queue statistics snapshot. Returns 1 on success. */
int telemetry_rtos_stats_copy(unsigned int *queued,
                              unsigned int *dropped_full,
                              unsigned int *dropped_stale);

int telemetry_tkernel_mutex_create(void);
int telemetry_tkernel_mutex_try_lock(int mutex_id);
void telemetry_tkernel_mutex_unlock(int mutex_id);
void telemetry_tkernel_mutex_delete(int mutex_id);

#endif /* AAI12_TELEMETRY_TKERNEL_PORT_H */
