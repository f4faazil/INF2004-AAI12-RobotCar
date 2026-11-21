/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * safety.h
 * Safety monitor: turns "is each producer alive and healthy?" into fault
 * bits.  It only RAISES faults (through fault_set); clearing is the Vehicle
 * Controller's decision, so a fault stays recorded until authorised clearing.
 *
 * Which faults matter depends on what the car is doing: stale line data only
 * matters while it is following a line, a silent encoder only while it is
 * driving, and so on.  Pure C.
 */
#ifndef SAFETY_H
#define SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#include "fault_mask.h"

typedef struct {
    uint32_t motion_stall_ms;       /* the motion task has not run for this long */
    uint32_t imu_stale_ms;
    uint32_t ultrasonic_stale_ms;
} safety_limits_t;

typedef struct {
    /* what the car is doing */
    bool line_required;             /* following / reacquiring a line */
    bool driving;                   /* a drive/move/turn command is active */
    bool obstacle_watch_required;   /* moving forward: must be able to see obstacles */

    /* producer health */
    bool     line_stale;            /* from line_nav */
    bool     encoder_fault;         /* from motion flags */
    uint32_t motion_age_ms;         /* since the last motion step */
    bool     imu_read_failed;
    uint32_t imu_age_ms;
    bool     ultrasonic_fault;      /* from the monitor: persistent invalid readings */
    uint32_t ultrasonic_age_ms;

    /* one-shot events */
    bool     adc_overflow;
    bool     barcode_rejected;
} safety_in_t;

/* Returns the fault bits that should be raised now. */
fault_bits_t safety_evaluate(const safety_in_t *in, const safety_limits_t *limits);

#endif /* SAFETY_H */
