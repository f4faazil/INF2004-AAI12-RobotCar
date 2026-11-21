/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * pid.h
 * Integer PID controller (the RP2040's Cortex-M0+ has no FPU).  Gains are
 * fixed-point with PID_GAIN_ONE = 1.0.  The derivative acts on the
 * measurement, not the error, so a setpoint step does not kick the output.
 * The integral term is clamped (anti-windup).
 *
 * Units are the caller's: for the wheel-speed loop the error is mm/s and the
 * output is signed PWM per-mille.  The default gains in robot_config.h are
 * starting values only; they must be tuned on the car.
 */
#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

#define PID_GAIN_ONE    (256)

typedef struct {
    int32_t kp;             /* gains in 1/256 units */
    int32_t ki;             /* per second of accumulated error */
    int32_t kd;             /* per (unit/second) of measurement rate */
    int32_t out_min;
    int32_t out_max;
    int32_t integral_limit; /* |integral| clamp, in error-units x ms */
    int32_t integral;
    int32_t prev_measurement;
    bool    has_prev;       /* false until the first update */
} pid_ctrl_t;

void pid_init(pid_ctrl_t *pid, int32_t kp, int32_t ki, int32_t kd,
              int32_t out_min, int32_t out_max, int32_t integral_limit);

/* Forget history (integral, previous measurement); keep the gains. */
void pid_reset(pid_ctrl_t *pid);

/* One update.  dt_ms must be non-zero; returns the clamped output. */
int32_t pid_update(pid_ctrl_t *pid, int32_t setpoint, int32_t measurement,
                   uint32_t dt_ms);

#endif /* PID_H */
