/* TEMPORARY Stage 5C5 qualification producer, NOT real robot telemetry.
 * This translation unit has NO micro T-Kernel headers. It executes in the
 * caller's task context and does NOT call lwIP, MQTT, or WiFi APIs.
 */
#include "telemetry_crosscore_demo.h"
#include "telemetry_hump_demo.h"
#include "telemetry_rtos.h"
#include "hardware/timer.h"

int telemetry_crosscore_demo_emit(unsigned int sequence)
{
    uint8_t payload[AAI12_HUMP_DEMO_BYTES];
    const uint8_t simulated_height = (uint8_t)(sequence % 120U);
    const uint32_t timestamp_ms = (uint32_t)(time_us_64() / 1000ULL);

    /* status=6 identifies processor-2-originated diagnostic samples;
     * status=5 was used by the radio-task-only single sample.
     * This remains the provisional 3-byte hump test format. */
    if (!telemetry_hump_demo_encode(payload, 6U, simulated_height,
                                    false, false)) {
        return -1;
    }
    return (int)telemetry_rtos_submit(payload, sizeof payload, timestamp_ms);
}
