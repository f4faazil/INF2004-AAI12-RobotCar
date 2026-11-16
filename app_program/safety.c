/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * safety.c
 * Safety monitor.  See safety.h.
 */
#include "safety.h"

fault_bits_t safety_evaluate(const safety_in_t *in, const safety_limits_t *limits)
{
    fault_bits_t bits = 0U;

    if (in->line_required && in->line_stale) {
        bits |= FAULT_LINE_STALE;
    }
    if (in->driving && in->encoder_fault) {
        bits |= FAULT_ENCODER;
    }
    if (in->driving && (in->motion_age_ms > limits->motion_stall_ms)) {
        bits |= FAULT_MOTION;
    }
    /* The IMU is informational: a failure means "hump N/A", never a stop. */
    if (in->imu_read_failed || (in->imu_age_ms > limits->imu_stale_ms)) {
        bits |= FAULT_IMU;
    }
    if (in->obstacle_watch_required &&
        (in->ultrasonic_fault || (in->ultrasonic_age_ms > limits->ultrasonic_stale_ms))) {
        bits |= FAULT_ULTRASONIC;
    }
    if (in->adc_overflow) {
        bits |= FAULT_LINE_OVERFLOW;
    }
    if (in->barcode_rejected) {
        bits |= FAULT_BARCODE_REJECT;
    }
    return bits;
}
