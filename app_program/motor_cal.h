/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * motor_cal.h
 * Open-loop motor calibration (Buddy 2).  A small state machine, fed with the
 * encoder counts every control step, that decides which PWM to apply and what
 * to report.  It touches no hardware, so it is tested on the host; the car
 * runs it from motor_cal_task.c.  The reported numbers go into the drive model
 * of ROBOT_MOTION_CFG through tools/motor_cal_fit.py.
 *
 * Two tests:
 *
 *   dead zone (one wheel, the other stays off, so the car pivots in place):
 *     UP    short PWM pulses of rising strength, 400 ms on and 600 ms off.  The
 *           first pulse that moves the wheel is the BREAKAWAY PWM.
 *     DOWN  the wheel is kicked above breakaway, then the PWM is lowered in
 *           steps until it no longer turns.  The last PWM that still turned it
 *           is the SUSTAIN PWM.
 *
 *   speed table (both wheels together, open loop): a list of PWM levels, each
 *     held until the speed settles, giving counts per second per wheel.
 */
#ifndef MOTOR_CAL_H
#define MOTOR_CAL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MCAL_DEADZONE_LEFT = 0,
    MCAL_DEADZONE_RIGHT,
    MCAL_SPEED_TABLE
} mcal_test_t;

typedef enum {
    MCAL_EV_NONE = 0,
    MCAL_EV_SAMPLE,     /* one step of the dead-zone search: pwm, counts/s of both wheels */
    MCAL_EV_BREAKAWAY,  /* the wheel started to turn at this PWM */
    MCAL_EV_SUSTAIN,    /* lowest PWM that still kept the wheel turning */
    MCAL_EV_SPEED,      /* speed-table row: pwm, counts/s of both wheels */
    MCAL_EV_DONE,
    MCAL_EV_ABORT       /* no breakaway up to the PWM limit, or the car travelled too far */
} mcal_event_kind_t;

typedef struct {
    mcal_event_kind_t kind;
    int16_t           pwm;          /* per-mille */
    int32_t           left_cps;     /* encoder counts per second, signed (+ is forward) */
    int32_t           right_cps;
} mcal_event_t;

/* The numbers below are public so the tests and the log reader agree on them. */
#define MCAL_PWM_FIRST          (100)       /* first pulse: 10 % */
#define MCAL_PWM_STEP           (10)
#define MCAL_PWM_LIMIT          (450)       /* highest PWM the search will try */
#define MCAL_PULSE_MS           (400U)
#define MCAL_REST_MS            (600U)
#define MCAL_MOVE_COUNTS        (30)        /* counts in a pulse that mean "it moved" */
#define MCAL_KICK_ABOVE         (80)        /* kick this far above breakaway */
#define MCAL_KICK_MS            (600U)
#define MCAL_DOWN_STEP          (5)
#define MCAL_DOWN_MS            (400U)
#define MCAL_DOWN_SETTLE_MS     (100U)
#define MCAL_STOP_COUNTS        (10)        /* fewer counts than this in the window = stalled */
#define MCAL_TRAVEL_LIMIT       (20000)     /* counts (about 1.7 m of wheel): abort beyond this */
#define MCAL_SPEED_LEVELS       (7U)
#define MCAL_SPEED_SETTLE_MS    (600U)
#define MCAL_SPEED_MEASURE_MS   (500U)
#define MCAL_SPEED_REST_MS      (300U)

typedef enum {
    MCAL_PH_IDLE = 0,
    MCAL_PH_UP_PULSE,
    MCAL_PH_UP_REST,
    MCAL_PH_KICK,
    MCAL_PH_DOWN,
    MCAL_PH_SPEED_SETTLE,
    MCAL_PH_SPEED_MEASURE,
    MCAL_PH_SPEED_REST,
    MCAL_PH_FINISH
} mcal_phase_t;

typedef struct {
    mcal_test_t  test;
    mcal_phase_t phase;
    uint32_t     phase_start_ms;
    int32_t      base_left;         /* counts at the start of the phase window */
    int32_t      base_right;
    int32_t      origin_left;       /* counts when the test began */
    int32_t      origin_right;
    uint32_t     window_start_ms;   /* when base_* were taken */
    bool         window_open;       /* DOWN: base_* hold the settled reading */
    int16_t      level;             /* the PWM of the current phase */
    int16_t      last_moving;       /* DOWN: last level that still turned the wheel */
    uint8_t      speed_index;
} motor_cal_t;

void motor_cal_start(motor_cal_t *cal, mcal_test_t test, uint32_t now_ms,
                     int32_t left_count, int32_t right_count);

/*
 * One control step.  Writes the PWM to apply to both wheels (per-mille, signed)
 * and at most one event.  Returns false once the test has finished (the last
 * call carries MCAL_EV_DONE or MCAL_EV_ABORT, with both PWM at 0).
 */
bool motor_cal_step(motor_cal_t *cal, uint32_t now_ms,
                    int32_t left_count, int32_t right_count,
                    int16_t *left_pwm, int16_t *right_pwm,
                    mcal_event_t *event);

/* The PWM of speed-table row 'index' (0..MCAL_SPEED_LEVELS-1). */
int16_t motor_cal_speed_level(uint8_t index);

#endif /* MOTOR_CAL_H */
