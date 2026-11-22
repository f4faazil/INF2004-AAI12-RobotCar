#ifndef AAI12_TELEMETRY_CROSSCORE_DEMO_H
#define AAI12_TELEMETRY_CROSSCORE_DEMO_H

/* Narrow test-only ABI for app_main.c, which includes micro T-Kernel headers.
 * Do not include standard C headers here: the provided RTOS defines a
 * conflicting size_t. Invoked only from processor-2 task context.
 * Returns TELEMETRY_INBOX_* integer code; -1 indicates encode failure.
 */
int telemetry_crosscore_demo_emit(unsigned int sequence);

#endif /* AAI12_TELEMETRY_CROSSCORE_DEMO_H */
