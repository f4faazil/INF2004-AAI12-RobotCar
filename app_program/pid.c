/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * pid.c
 * Integer PID controller.  See pid.h.
 */
#include "pid.h"

#define MS_PER_SECOND   (1000)

static int32_t clamp(int32_t value, int32_t low, int32_t high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

void pid_init(pid_ctrl_t *pid, int32_t kp, int32_t ki, int32_t kd,
              int32_t out_min, int32_t out_max, int32_t integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_limit = integral_limit;
    pid_reset(pid);
}

void pid_reset(pid_ctrl_t *pid)
{
    pid->integral = 0;
    pid->prev_measurement = 0;
    pid->has_prev = false;
}

int32_t pid_update(pid_ctrl_t *pid, int32_t setpoint, int32_t measurement,
                   uint32_t dt_ms)
{
    int32_t error = setpoint - measurement;
    int64_t sum = (int64_t)pid->kp * error;

    if (dt_ms != 0U) {
        int64_t rate;

        pid->integral = clamp(pid->integral + (error * (int32_t)dt_ms),
                              -pid->integral_limit, pid->integral_limit);
        sum += ((int64_t)pid->ki * pid->integral) / MS_PER_SECOND;

        if (pid->has_prev) {
            rate = ((int64_t)(measurement - pid->prev_measurement) *
                    MS_PER_SECOND) / (int64_t)dt_ms;
            sum -= (int64_t)pid->kd * rate;
        }
        pid->prev_measurement = measurement;
        pid->has_prev = true;
    }

    sum /= PID_GAIN_ONE;
    if (sum > (int64_t)pid->out_max) {
        return pid->out_max;
    }
    if (sum < (int64_t)pid->out_min) {
        return pid->out_min;
    }
    return (int32_t)sum;
}
