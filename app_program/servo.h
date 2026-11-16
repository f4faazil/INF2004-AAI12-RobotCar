/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * servo.h
 * Scanning-servo angle to pulse-width mapping and settling time (Buddy 5).
 * Pure C; the HAL turns the pulse width into a 50 Hz PWM duty on GP12.
 *
 * Convention: 90 degrees is straight ahead.  Whether larger angles point
 * left or right depends on how the horn was fitted (the bench found the rest
 * position pointing right and the horn not re-aligned), so servo_cfg has a
 * trim and an invert flag instead of baking an assumption in.
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t min_pulse_us;      /* pulse at 0 degrees */
    uint16_t max_pulse_us;      /* pulse at 180 degrees */
    int16_t  trim_deg;          /* added to every command (horn alignment) */
    bool     invert;            /* mirror the sweep */
    uint8_t  min_angle_deg;     /* software limits protect the linkage */
    uint8_t  max_angle_deg;
    uint16_t settle_base_ms;
    uint16_t settle_ms_per_deg;
} servo_cfg_t;

/* Limit an angle to the configured sweep. */
uint8_t servo_clamp_angle(const servo_cfg_t *cfg, int16_t angle_deg);

/* Pulse width for an angle, after clamp, trim and invert. */
uint16_t servo_pulse_us(const servo_cfg_t *cfg, int16_t angle_deg);

/* Time to wait after commanding a move before the horn can be trusted. */
uint16_t servo_settle_ms(const servo_cfg_t *cfg, int16_t from_deg, int16_t to_deg);

#endif /* SERVO_H */
