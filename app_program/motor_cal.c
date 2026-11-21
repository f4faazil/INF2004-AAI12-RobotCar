/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * motor_cal.c
 * Open-loop motor calibration state machine.  See motor_cal.h.
 */
#include "motor_cal.h"

#define MS_PER_S    (1000)

static const int16_t SPEED_LEVELS[MCAL_SPEED_LEVELS] = {
    150, 200, 250, 300, 350, 400, 450
};

static int32_t abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* The count of the wheel under test (left unless the right wheel is tested). */
static int32_t selected(const motor_cal_t *cal, int32_t left, int32_t right)
{
    return (cal->test == MCAL_DEADZONE_RIGHT) ? right : left;
}

static int32_t counts_per_second(int32_t delta, uint32_t window_ms)
{
    if (window_ms == 0U) {
        return 0;
    }
    return (delta * MS_PER_S) / (int32_t)window_ms;
}

static void enter(motor_cal_t *cal, mcal_phase_t phase, uint32_t now_ms,
                  int32_t left, int32_t right, int16_t level)
{
    cal->phase = phase;
    cal->phase_start_ms = now_ms;
    cal->window_start_ms = now_ms;
    cal->base_left = left;
    cal->base_right = right;
    cal->window_open = false;
    cal->level = level;
}

/* Apply 'level' to the wheel(s) under test; the other wheel stays off. */
static void drive(const motor_cal_t *cal, int16_t level, int16_t *left_pwm, int16_t *right_pwm)
{
    *left_pwm = (cal->test != MCAL_DEADZONE_RIGHT) ? level : (int16_t)0;
    *right_pwm = (cal->test != MCAL_DEADZONE_LEFT) ? level : (int16_t)0;
}

static void set_event(mcal_event_t *event, mcal_event_kind_t kind, int16_t pwm,
                      const motor_cal_t *cal, int32_t left, int32_t right, uint32_t window_ms)
{
    event->kind = kind;
    event->pwm = pwm;
    event->left_cps = counts_per_second(left - cal->base_left, window_ms);
    event->right_cps = counts_per_second(right - cal->base_right, window_ms);
}

int16_t motor_cal_speed_level(uint8_t index)
{
    return (index < MCAL_SPEED_LEVELS) ? SPEED_LEVELS[index] : (int16_t)0;
}

void motor_cal_start(motor_cal_t *cal, mcal_test_t test, uint32_t now_ms,
                     int32_t left_count, int32_t right_count)
{
    cal->test = test;
    cal->origin_left = left_count;
    cal->origin_right = right_count;
    cal->last_moving = 0;
    cal->speed_index = 0U;
    if (test == MCAL_SPEED_TABLE) {
        enter(cal, MCAL_PH_SPEED_SETTLE, now_ms, left_count, right_count, SPEED_LEVELS[0]);
    } else {
        enter(cal, MCAL_PH_UP_PULSE, now_ms, left_count, right_count, (int16_t)MCAL_PWM_FIRST);
    }
}

static bool abort_test(motor_cal_t *cal, int16_t pwm, mcal_event_t *event)
{
    event->kind = MCAL_EV_ABORT;
    event->pwm = pwm;
    cal->phase = MCAL_PH_IDLE;
    return false;
}

bool motor_cal_step(motor_cal_t *cal, uint32_t now_ms,
                    int32_t left_count, int32_t right_count,
                    int16_t *left_pwm, int16_t *right_pwm,
                    mcal_event_t *event)
{
    const uint32_t elapsed = now_ms - cal->phase_start_ms;
    const int32_t sel = selected(cal, left_count, right_count);
    const int32_t sel_base = selected(cal, cal->base_left, cal->base_right);

    *left_pwm = 0;
    *right_pwm = 0;
    event->kind = MCAL_EV_NONE;
    event->pwm = 0;
    event->left_cps = 0;
    event->right_cps = 0;

    switch (cal->phase) {
    case MCAL_PH_UP_PULSE:
        drive(cal, cal->level, left_pwm, right_pwm);
        if (elapsed >= MCAL_PULSE_MS) {
            const int16_t level = cal->level;
            const bool moved = abs32(sel - sel_base) >= MCAL_MOVE_COUNTS;

            set_event(event, moved ? MCAL_EV_BREAKAWAY : MCAL_EV_SAMPLE, level, cal,
                      left_count, right_count, elapsed);
            if (moved) {
                enter(cal, MCAL_PH_KICK, now_ms, left_count, right_count,
                      (int16_t)(level + MCAL_KICK_ABOVE));
            } else {
                enter(cal, MCAL_PH_UP_REST, now_ms, left_count, right_count, level);
            }
        }
        return true;

    case MCAL_PH_UP_REST:
        if (elapsed >= MCAL_REST_MS) {
            const int16_t next = (int16_t)(cal->level + MCAL_PWM_STEP);

            if (next > MCAL_PWM_LIMIT) {
                return abort_test(cal, cal->level, event);
            }
            enter(cal, MCAL_PH_UP_PULSE, now_ms, left_count, right_count, next);
        }
        return true;

    case MCAL_PH_KICK:
        drive(cal, cal->level, left_pwm, right_pwm);
        if (elapsed >= MCAL_KICK_MS) {
            cal->last_moving = cal->level;
            enter(cal, MCAL_PH_DOWN, now_ms, left_count, right_count, cal->level);
        }
        return true;

    case MCAL_PH_DOWN:
        if (abs32(sel - selected(cal, cal->origin_left, cal->origin_right)) > MCAL_TRAVEL_LIMIT) {
            return abort_test(cal, cal->level, event);
        }
        drive(cal, cal->level, left_pwm, right_pwm);
        if ((!cal->window_open) && (elapsed >= MCAL_DOWN_SETTLE_MS)) {
            cal->base_left = left_count;
            cal->base_right = right_count;
            cal->window_start_ms = now_ms;
            cal->window_open = true;
        }
        if (elapsed >= MCAL_DOWN_MS) {
            const uint32_t window = now_ms - cal->window_start_ms;
            const bool turning = abs32(sel - selected(cal, cal->base_left, cal->base_right))
                                 >= MCAL_STOP_COUNTS;
            const int16_t level = cal->level;

            if (!turning) {
                set_event(event, MCAL_EV_SUSTAIN, cal->last_moving, cal,
                          left_count, right_count, window);
                cal->phase = MCAL_PH_FINISH;
                *left_pwm = 0;
                *right_pwm = 0;
            } else {
                set_event(event, MCAL_EV_SAMPLE, level, cal, left_count, right_count, window);
                cal->last_moving = level;
                if (level <= MCAL_DOWN_STEP) {
                    return abort_test(cal, level, event);
                }
                enter(cal, MCAL_PH_DOWN, now_ms, left_count, right_count,
                      (int16_t)(level - MCAL_DOWN_STEP));
            }
        }
        return true;

    case MCAL_PH_SPEED_SETTLE:
        drive(cal, cal->level, left_pwm, right_pwm);
        if (elapsed >= MCAL_SPEED_SETTLE_MS) {
            enter(cal, MCAL_PH_SPEED_MEASURE, now_ms, left_count, right_count, cal->level);
        }
        return true;

    case MCAL_PH_SPEED_MEASURE:
        drive(cal, cal->level, left_pwm, right_pwm);
        if (elapsed >= MCAL_SPEED_MEASURE_MS) {
            set_event(event, MCAL_EV_SPEED, cal->level, cal, left_count, right_count, elapsed);
            enter(cal, MCAL_PH_SPEED_REST, now_ms, left_count, right_count, cal->level);
        }
        return true;

    case MCAL_PH_SPEED_REST:
        if (elapsed >= MCAL_SPEED_REST_MS) {
            cal->speed_index++;
            if (cal->speed_index >= MCAL_SPEED_LEVELS) {
                cal->phase = MCAL_PH_FINISH;
            } else {
                enter(cal, MCAL_PH_SPEED_SETTLE, now_ms, left_count, right_count,
                      SPEED_LEVELS[cal->speed_index]);
            }
        }
        return true;

    case MCAL_PH_FINISH:
        event->kind = MCAL_EV_DONE;
        cal->phase = MCAL_PH_IDLE;
        return false;

    case MCAL_PH_IDLE:
    default:
        return false;
    }
}
