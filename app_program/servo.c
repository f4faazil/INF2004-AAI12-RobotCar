/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * servo.c
 * Scanning-servo mapping.  See servo.h.
 */
#include "servo.h"

#define SERVO_FULL_SWEEP_DEG    (180)

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

uint8_t servo_clamp_angle(const servo_cfg_t *cfg, int16_t angle_deg)
{
    return (uint8_t)clamp_i32(angle_deg, cfg->min_angle_deg, cfg->max_angle_deg);
}

uint16_t servo_pulse_us(const servo_cfg_t *cfg, int16_t angle_deg)
{
    int32_t angle = (int32_t)servo_clamp_angle(cfg, angle_deg) + cfg->trim_deg;
    int32_t span = (int32_t)cfg->max_pulse_us - (int32_t)cfg->min_pulse_us;

    if (cfg->invert) {
        angle = SERVO_FULL_SWEEP_DEG - angle;
    }
    angle = clamp_i32(angle, 0, SERVO_FULL_SWEEP_DEG);
    return (uint16_t)((int32_t)cfg->min_pulse_us + ((angle * span) / SERVO_FULL_SWEEP_DEG));
}

uint16_t servo_settle_ms(const servo_cfg_t *cfg, int16_t from_deg, int16_t to_deg)
{
    int32_t delta = (int32_t)servo_clamp_angle(cfg, to_deg) -
                    (int32_t)servo_clamp_angle(cfg, from_deg);

    if (delta < 0) {
        delta = -delta;
    }
    return (uint16_t)((int32_t)cfg->settle_base_ms + (delta * cfg->settle_ms_per_deg));
}
