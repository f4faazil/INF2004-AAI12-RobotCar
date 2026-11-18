/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * obstacle_test_task.c
 * Ultrasonic and servo bring-up on the car, running the same code as the robot
 * (hal_us_*, us_convert, the scan sequencer, the profile and the planner).
 * Press START (GP20) to run the step shown; STOP (GP21) in the menu skips to
 * the next step, and STOP during a run ends it.  Lines start with "[obs]".
 *
 *   0  RANGE     the servo's usable sweep BY PULSE, between the two stops measured on the
 *                car (about 1312 us right and 2250 us left): from the right stop to the left
 *                stop in 30 us steps, 1.4 s each.  The sonar measures a flat wall in front at
 *                every step.  The wall is square-on where the median is smallest, and the
 *                angle off that point follows from range = distance / cos(angle), so the log
 *                gives the pulse for straight ahead and the microseconds per degree without
 *                a protractor.  STOP ends it.  The motors are not driven.
 *   1  DISTANCE  a flat object at 100, 200, 300, 400 then 600 mm: 5 s of readings,
 *                then the count, mean, median, min, max and the error against
 *                the distance you set up
 *   2  NO ECHO   point the sensor at open space (nothing within 4 m) or at a soft
 *                surface: how the no-echo handling reports it, and when it would
 *                raise the sensor fault
 *   3  SERVO     centre, left limit, centre, right limit, centre, 3 s each: check
 *                the angles with a protractor
 *   4  LOAD      WHEELS LIFTED OFF THE FLOOR: both motors at 25 % for 10 s while the
 *                servo sweeps and the sensor keeps measuring.  Shows whether the
 *                servo, the sensor or the board is upset by the motors (a reset
 *                prints the banner again)
 *   5  SCAN      the real scan, width profile and bypass plan for an obstacle in
 *                front: compare the width and the closest distance with a ruler
 *
 * Only step 4 drives the motors.
 *
 * NOT YET RUN ON THE CAR.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "hal_rp2040.h"
#include "obstacle.h"
#include "obstacle_test_task.h"
#include "robot_config.h"
#include "servo.h"
#include "ultrasonic.h"

#define OBS_BUTTON_SAMPLES      (6U)        /* 6 x 5 ms = 30 ms debounce */
#define OBS_WINDOW_MS           (5000U)     /* length of one distance run */
#define OBS_MAX_SAMPLES         (96U)
#define OBS_SERVO_HOLD_MS       (3000U)
#define OBS_SCAN_TIMEOUT_MS     (60000U)
#define OBS_INVALID_LIMIT       (8U)        /* the robot's value: invalid readings in a row */
#define OBS_CONFIRM             (2U)
#define OBS_TARGET_COUNT        (5U)
#define OBS_LOAD_MS             (10000U)    /* the motors-running test */
#define OBS_LOAD_PWM            (250)       /* per-mille: 25 %, wheels lifted */
#define OBS_LOAD_SWEEP_MS       (1000U)

typedef enum {
    STEP_RANGE = 0,
    STEP_DISTANCE,
    STEP_NOECHO,
    STEP_SERVO,
    STEP_LOAD,
    STEP_SCAN,
    STEP_COUNT
} obs_step_t;

typedef struct {
    uint8_t count;
    bool    state;
} button_t;

static ID g_obs_flag;

static const uint16_t TARGET_MM[OBS_TARGET_COUNT] = { 100U, 200U, 300U, 400U, 600U };

/* servo test: angles from the car's right, 90 = straight ahead */
#define OBS_SERVO_STEPS         (5U)

/* range test: pulses from just inside the measured right stop to just inside the left stop */
#define RANGE_FIRST_US          (1332U)     /* the right stop is 1312 us (measured on the car) */
#define RANGE_LAST_US           (2230U)     /* the left stop is 2250 us */
#define RANGE_STEP_US           (30U)
#define RANGE_COUNT             (((RANGE_LAST_US - RANGE_FIRST_US) / RANGE_STEP_US) + 1U)
#define RANGE_REST_US           (1800U)     /* about straight ahead: where it ends up */
#define RANGE_HOLD_MS           (1400U)
#define RANGE_SETTLE_MS         (700U)      /* the readings in the rest of the hold count */

static bool button_pressed(button_t *b, bool raw)
{
    bool edge = false;

    if (raw == b->state) {
        b->count = 0U;
    } else {
        b->count++;
        if (b->count >= OBS_BUTTON_SAMPLES) {
            b->state = raw;
            b->count = 0U;
            edge = raw;
        }
    }
    return edge;
}

static void servo_to(uint8_t angle_deg)
{
    hal_servo_set_pulse_us(servo_pulse_us(&ROBOT_OBSTACLE_CFG.servo, angle_deg));
}

static const char *status_name(us_status_t s)
{
    switch (s) {
    case US_VALID:        return "valid";
    case US_NO_ECHO:      return "no-echo";
    case US_TOO_CLOSE:    return "too-close";
    case US_OUT_OF_RANGE: return "out-of-range";
    default:              return "error";
    }
}

static const char *kind_name(scan_point_kind_t k)
{
    switch (k) {
    case SCAN_POINT_OBSTACLE: return "obstacle";
    case SCAN_POINT_FAR:      return "far";
    case SCAN_POINT_NOECHO:   return "no-echo";
    default:                  return "invalid";
    }
}

static const char *action_name(plan_action_t a)
{
    switch (a) {
    case PLAN_CONTINUE:     return "continue";
    case PLAN_BYPASS_LEFT:  return "bypass-left";
    case PLAN_BYPASS_RIGHT: return "bypass-right";
    case PLAN_REVERSE:      return "reverse";
    default:                return "stop";
    }
}

static const char *reason_name(plan_reason_t r)
{
    switch (r) {
    case PLAN_REASON_OK:              return "ok";
    case PLAN_REASON_NO_OBSTACLE:     return "no-obstacle";
    case PLAN_REASON_INVALID_SCAN:    return "invalid-scan";
    case PLAN_REASON_NO_CLEARANCE:    return "no-clearance";
    case PLAN_REASON_DETOUR_TOO_LARGE: return "detour-too-large";
    default:                          return "too-close";
    }
}

static void print_prompt(obs_step_t step, uint8_t target_idx)
{
    switch (step) {
    case STEP_RANGE:
        tm_printf((UB *)"[obs] step 0 RANGE: put a broad flat wall (or a big box face) 400 mm straight in front of "
                        "the sensor, square to the car, keep the horn and cables clear.  Press START: the servo "
                        "sweeps from its right stop to its left stop in %d steps (about %d s).  Press STOP at once "
                        "if it hums or stalls.  STOP skips to step 1\n",
                  (INT)RANGE_COUNT, (INT)((RANGE_COUNT * RANGE_HOLD_MS) / 1000U));
        break;
    case STEP_DISTANCE:
        tm_printf((UB *)"[obs] step 1 DISTANCE: put a broad flat hard object %d mm from the sensor's FRONT FACE "
                        "(the two metal cylinders; use a ruler), keep it still, press START.  STOP skips to step 2\n",
                  (INT)TARGET_MM[target_idx]);
        break;
    case STEP_NOECHO:
        tm_printf((UB *)"[obs] step 2 NO ECHO: point the sensor at open space (nothing within 4 m), "
                        "press START.  STOP skips to step 3\n");
        break;
    case STEP_SERVO:
        tm_printf((UB *)"[obs] step 3 SERVO: keep the horn clear, press START.  STOP skips to step 4\n");
        break;
    case STEP_LOAD:
        tm_printf((UB *)"[obs] step 4 LOAD: LIFT THE WHEELS OFF THE FLOOR (the motors will spin for 10 s), "
                        "press START.  STOP ends it, or skips to step 5\n");
        break;
    default:
        tm_printf((UB *)"[obs] step 5 SCAN: put one box, centred, 650 mm from the sensor face (the distance the "
                        "robot starts a scan from), press START.  Repeat with a 150 mm box and at 500 mm.  "
                        "STOP skips to step 0\n");
        break;
    }
}

static void sort_u16(uint16_t *a, uint8_t n)
{
    uint8_t i;

    for (i = 1U; i < n; i++) {
        uint16_t v = a[i];
        int16_t j = (int16_t)i - 1;

        while ((j >= 0) && (a[j] > v)) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = v;
    }
}

static void obstacle_test_task(INT stacd, void *exinf)
{
    static us_monitor_t mon;
    static scan_t scan;
    static us_reading_t reading;
    static uint16_t samples[OBS_MAX_SAMPLES];
    static obstacle_profile_t profile;
    static bypass_plan_t plan;
    button_t start_btn = { 0U, false };
    button_t stop_btn = { 0U, false };
    obs_step_t step = STEP_RANGE;
    uint8_t target_idx = 0U;
    uint8_t range_idx = 0U;
    uint32_t range_since_ms = 0U;
    static uint16_t range_median[RANGE_COUNT];
    bool running = false;
    bool measuring = false;
    uint32_t last_start_ms = 0U;
    uint32_t run_start_ms = 0U;
    uint8_t n_valid = 0U;
    uint16_t n_total = 0U;
    uint16_t n_noecho = 0U;
    uint16_t n_other = 0U;
    uint8_t servo_idx = 0U;
    uint16_t n_ok = 0U;
    uint32_t servo_since_ms = 0U;
    const uint8_t servo_angles[OBS_SERVO_STEPS] = {
        90U, ROBOT_OBSTACLE_CFG.servo.max_angle_deg, 90U,
        ROBOT_OBSTACLE_CFG.servo.min_angle_deg, 90U
    };

    (void)stacd;
    (void)exinf;
    hal_motor_set(0, 0);
    servo_to(90U);
    us_monitor_init(&mon, (uint8_t)OBS_CONFIRM, (uint8_t)OBS_INVALID_LIMIT);
    reading.status = US_ERROR;
    reading.seq = 1U;
    tm_printf((UB *)"[obs] ultrasonic and servo bring-up.  Servo limits %d to %d deg, pulses %d to %d us.  "
                    "START = run, STOP = skip or end.\n",
              (INT)ROBOT_OBSTACLE_CFG.servo.min_angle_deg, (INT)ROBOT_OBSTACLE_CFG.servo.max_angle_deg,
              (INT)servo_pulse_us(&ROBOT_OBSTACLE_CFG.servo, ROBOT_OBSTACLE_CFG.servo.min_angle_deg),
              (INT)servo_pulse_us(&ROBOT_OBSTACLE_CFG.servo, ROBOT_OBSTACLE_CFG.servo.max_angle_deg));
    print_prompt(step, target_idx);

    for (;;) {
        UINT pattern = 0U;
        uint32_t now;
        bool start_edge;
        bool stop_edge;
        bool fresh = false;
        int16_t pwm = 0;

        (void)tk_wai_flg(g_obs_flag, HAL_TICK_MOTION, TWF_ORW | TWF_BITCLR, &pattern, TMO_FEVR);
        hal_tick_ack(HAL_TICK_MOTION);
        now = hal_time_ms();
        start_edge = button_pressed(&start_btn, hal_button_start());
        stop_edge = button_pressed(&stop_btn, hal_button_stop());

        /* The measurement engine: the same trigger, poll and convert as the robot. */
        if (measuring) {
            hal_echo_t echo = hal_us_poll();

            if (echo.finished) {
                uint16_t mm = 0U;

                reading.status = us_convert(echo.echo_us, echo.complete, &mm);
                reading.distance_mm = (reading.status == US_VALID) ? mm : 0U;
                reading.timestamp_ms = now;
                reading.seq++;
                measuring = false;
                fresh = true;
            }
        }

        if (!running) {
            if (stop_edge) {
                step = (obs_step_t)(((uint32_t)step + 1U) % (uint32_t)STEP_COUNT);
                print_prompt(step, target_idx);
            } else if (start_edge) {
                running = true;
                run_start_ms = now;
                last_start_ms = now - ROBOT_US_PERIOD_MS;
                n_valid = 0U;
                n_total = 0U;
                n_noecho = 0U;
                n_other = 0U;
                n_ok = 0U;
                us_monitor_init(&mon, (uint8_t)OBS_CONFIRM, (uint8_t)OBS_INVALID_LIMIT);
                if (step == STEP_RANGE) {
                    range_idx = 0U;
                    range_since_ms = now;
                    hal_servo_set_pulse_us(RANGE_FIRST_US);
                    tm_printf((UB *)"[obs] RANGE start: pulses %d to %d us in %d us steps.  STOP ends it\n",
                              (INT)RANGE_FIRST_US, (INT)RANGE_LAST_US, (INT)RANGE_STEP_US);
                } else if (step == STEP_LOAD) {
                    servo_idx = 0U;
                    servo_since_ms = now;
                    servo_to(ROBOT_OBSTACLE_CFG.servo.min_angle_deg);
                    tm_printf((UB *)"[obs] LOAD: motors on at %d per-mille for %d s, servo sweeping\n",
                              (INT)OBS_LOAD_PWM, (INT)(OBS_LOAD_MS / 1000U));
                } else if (step == STEP_SERVO) {
                    servo_idx = 0U;
                    servo_since_ms = now;
                    servo_to(servo_angles[0]);
                    tm_printf((UB *)"[obs] SERVO angle=%d deg pulse=%d us\n", (INT)servo_angles[0],
                              (INT)servo_pulse_us(&ROBOT_OBSTACLE_CFG.servo, servo_angles[0]));
                } else if (step == STEP_SCAN) {
                    servo_to(90U);
                    scan_start(&scan, &ROBOT_OBSTACLE_CFG, now);
                    tm_printf((UB *)"[obs] SCAN started (coarse %d to %d deg step %d, fine step %d)\n",
                              (INT)ROBOT_OBSTACLE_CFG.coarse_min_deg, (INT)ROBOT_OBSTACLE_CFG.coarse_max_deg,
                              (INT)ROBOT_OBSTACLE_CFG.coarse_step_deg, (INT)ROBOT_OBSTACLE_CFG.fine_step_deg);
                } else {
                    servo_to(90U);
                    tm_printf((UB *)"[obs] measuring for 5 s ...\n");
                }
            } else {
                /* idle */
            }
        } else if (step == STEP_RANGE) {
            if (fresh && ((now - range_since_ms) >= RANGE_SETTLE_MS)) {
                n_total++;
                if ((reading.status == US_VALID) && (n_valid < OBS_MAX_SAMPLES)) {
                    samples[n_valid] = reading.distance_mm;
                    n_valid++;
                } else if (reading.status == US_NO_ECHO) {
                    n_noecho++;
                } else {
                    n_other++;
                }
            }
            if (!measuring && ((now - last_start_ms) >= ROBOT_US_PERIOD_MS)) {
                hal_us_trigger();
                measuring = true;
                last_start_ms = now;
            }
            if (stop_edge || (range_idx >= RANGE_COUNT)) {
                uint16_t best = 0U;
                uint8_t first_best = 0U;
                uint8_t last_best = 0U;
                uint8_t i;

                hal_servo_set_pulse_us(RANGE_REST_US);
                for (i = 0U; i < range_idx; i++) {
                    if ((range_median[i] > 0U) && ((best == 0U) || (range_median[i] < best))) {
                        best = range_median[i];
                        first_best = i;
                    }
                }
                for (i = 0U; i < range_idx; i++) {
                    if ((best != 0U) && (range_median[i] <= (best + 3U))) {
                        last_best = i;          /* within 3 mm of the smallest: square-on */
                    }
                }
                if (best != 0U) {
                    tm_printf((UB *)"[obs] RESULT RANGE %s: the smallest median was %d mm at pulses %d to %d us "
                                    "(the wall is square-on there: that pulse points straight ahead, if the wall "
                                    "is square to the car).  The angle at any other pulse is acos(%d / median)\n",
                              stop_edge ? (UB *)"stopped" : (UB *)"finished", (INT)best,
                              (INT)(RANGE_FIRST_US + ((uint32_t)first_best * RANGE_STEP_US)),
                              (INT)(RANGE_FIRST_US + ((uint32_t)last_best * RANGE_STEP_US)), (INT)best);
                } else {
                    tm_printf((UB *)"[obs] RESULT RANGE %s: no valid echo at any pulse - is the wall in front?\n",
                              stop_edge ? (UB *)"stopped" : (UB *)"finished");
                }
                running = false;
                measuring = false;
                step = STEP_DISTANCE;
                print_prompt(step, target_idx);
            } else if ((now - range_since_ms) >= RANGE_HOLD_MS) {
                /* the result for the pulse just held */
                uint16_t median = 0U;

                if (n_valid > 0U) {
                    sort_u16(samples, n_valid);
                    median = samples[n_valid / 2U];
                }
                range_median[range_idx] = median;
                tm_printf((UB *)"[obs] RANGE pulse=%d us  readings=%d valid=%d no-echo=%d  median=%d mm\n",
                          (INT)(RANGE_FIRST_US + ((uint32_t)range_idx * RANGE_STEP_US)), (INT)n_total,
                          (INT)n_valid, (INT)n_noecho, (INT)median);
                range_idx++;
                range_since_ms = now;
                n_valid = 0U;
                n_total = 0U;
                n_noecho = 0U;
                n_other = 0U;
                if (range_idx < RANGE_COUNT) {
                    hal_servo_set_pulse_us((uint16_t)(RANGE_FIRST_US + ((uint32_t)range_idx * RANGE_STEP_US)));
                }
            } else {
                /* holding */
            }
        } else if ((step == STEP_DISTANCE) || (step == STEP_NOECHO)) {
            if (fresh) {
                n_total++;
                if (reading.status == US_VALID) {
                    if (n_valid < OBS_MAX_SAMPLES) {
                        samples[n_valid] = reading.distance_mm;
                        n_valid++;
                    }
                } else if (reading.status == US_NO_ECHO) {
                    n_noecho++;
                } else {
                    n_other++;
                }
                (void)us_monitor_update(&mon, &reading, 0U, 0U, 0U);
                if ((n_total % 5U) == 1U) {
                    tm_printf((UB *)"[obs] reading %d mm (%s)\n", (INT)reading.distance_mm,
                              (UB *)status_name(reading.status));
                }
            }
            if (!measuring && ((now - last_start_ms) >= ROBOT_US_PERIOD_MS)) {
                hal_us_trigger();
                measuring = true;
                last_start_ms = now;
            }
            if (stop_edge || ((now - run_start_ms) >= OBS_WINDOW_MS)) {
                if (n_valid > 0U) {
                    uint32_t sum = 0U;
                    uint16_t lo;
                    uint16_t hi;
                    uint8_t i;
                    uint16_t median;

                    sort_u16(samples, n_valid);
                    lo = samples[0];
                    hi = samples[n_valid - 1U];
                    median = samples[n_valid / 2U];
                    for (i = 0U; i < n_valid; i++) {
                        sum += samples[i];
                    }
                    if (step == STEP_DISTANCE) {
                        tm_printf((UB *)"[obs] RESULT DISTANCE target=%d mm: readings=%d valid=%d no-echo=%d other=%d "
                                        "mean=%d median=%d min=%d max=%d error(median)=%d mm\n",
                                  (INT)TARGET_MM[target_idx], (INT)n_total, (INT)n_valid, (INT)n_noecho,
                                  (INT)n_other, (INT)(sum / n_valid), (INT)median, (INT)lo, (INT)hi,
                                  (INT)median - (INT)TARGET_MM[target_idx]);
                    } else {
                        tm_printf((UB *)"[obs] RESULT NO ECHO: readings=%d valid=%d no-echo=%d other=%d "
                                        "(valid readings at open space: the median was %d mm)\n",
                                  (INT)n_total, (INT)n_valid, (INT)n_noecho, (INT)n_other, (INT)median);
                    }
                } else {
                    tm_printf((UB *)"[obs] RESULT %s: readings=%d valid=0 no-echo=%d other=%d  "
                                    "sensor fault flag=%d (it is raised after %d invalid readings in a row)\n",
                              (step == STEP_DISTANCE) ? (UB *)"DISTANCE" : (UB *)"NO ECHO",
                              (INT)n_total, (INT)n_noecho, (INT)n_other,
                              (INT)us_monitor_sensor_fault(&mon), (INT)OBS_INVALID_LIMIT);
                }
                running = false;
                measuring = false;
                if (step == STEP_DISTANCE) {
                    target_idx++;
                    if (target_idx >= OBS_TARGET_COUNT) {
                        target_idx = 0U;
                        step = STEP_NOECHO;
                    }
                } else {
                    step = STEP_SERVO;
                }
                print_prompt(step, target_idx);
            }
        } else if (step == STEP_SERVO) {
            if ((now - servo_since_ms) >= OBS_SERVO_HOLD_MS) {
                servo_idx++;
                if ((servo_idx >= OBS_SERVO_STEPS) || stop_edge) {
                    servo_to(90U);
                    tm_printf((UB *)"[obs] RESULT SERVO done: check the left limit is about 28 deg left of centre, "
                                    "the right limit about 32 deg right\n");
                    running = false;
                    step = STEP_LOAD;
                    print_prompt(step, target_idx);
                } else {
                    servo_since_ms = now;
                    servo_to(servo_angles[servo_idx]);
                    tm_printf((UB *)"[obs] SERVO angle=%d deg pulse=%d us\n", (INT)servo_angles[servo_idx],
                              (INT)servo_pulse_us(&ROBOT_OBSTACLE_CFG.servo, servo_angles[servo_idx]));
                }
            } else if (stop_edge) {
                servo_to(90U);
                running = false;
                step = STEP_LOAD;
                print_prompt(step, target_idx);
            } else {
                /* holding */
            }
        } else if (step == STEP_LOAD) {
            pwm = OBS_LOAD_PWM;
            if (fresh) {
                n_total++;
                if (reading.status == US_VALID) {
                    n_ok++;
                } else if (reading.status == US_NO_ECHO) {
                    n_noecho++;
                } else {
                    n_other++;
                }
            }
            if (!measuring && ((now - last_start_ms) >= ROBOT_US_PERIOD_MS)) {
                hal_us_trigger();
                measuring = true;
                last_start_ms = now;
            }
            if ((now - servo_since_ms) >= OBS_LOAD_SWEEP_MS) {
                servo_since_ms = now;
                servo_idx++;
                servo_to(((servo_idx & 1U) != 0U) ? ROBOT_OBSTACLE_CFG.servo.max_angle_deg
                                                  : ROBOT_OBSTACLE_CFG.servo.min_angle_deg);
            }
            if (stop_edge || ((now - run_start_ms) >= OBS_LOAD_MS)) {
                pwm = 0;
                servo_to(90U);
                tm_printf((UB *)"[obs] RESULT LOAD: %d s, servo moves=%d, readings=%d valid=%d no-echo=%d other=%d.  "
                                "No banner since START means no reset.  Compare valid with a run at rest\n",
                          (INT)((now - run_start_ms) / 1000U), (INT)servo_idx, (INT)n_total, (INT)n_ok,
                          (INT)n_noecho, (INT)n_other);
                running = false;
                measuring = false;
                step = STEP_SCAN;
                print_prompt(step, target_idx);
            }
        } else {
            /* STEP_SCAN: exactly what the robot's Obstacle task does */
            scan_action_t act;
            bool done = scan_update(&scan, now, &reading, &act);

            if (act.servo_move) {
                servo_to(act.servo_angle);
            }
            if (act.request_measure && !measuring) {
                hal_us_trigger();
                measuring = true;
            }
            if (stop_edge || (!done && ((now - run_start_ms) >= OBS_SCAN_TIMEOUT_MS))) {
                servo_to(90U);
                tm_printf((UB *)"[obs] SCAN stopped after %d ms\n", (INT)(now - run_start_ms));
                running = false;
                measuring = false;
                step = STEP_DISTANCE;
                print_prompt(step, target_idx);
            } else if (done) {
                uint8_t i;

                obstacle_profile_build(&ROBOT_OBSTACLE_CFG, scan.points, scan.point_count, &profile);
                obstacle_plan(&ROBOT_OBSTACLE_CFG, &profile, &plan);
                for (i = 0U; i < scan.point_count; i++) {
                    tm_printf((UB *)"[obs] POINT angle=%d deg  %s  %d mm\n", (INT)scan.points[i].angle_deg,
                              (UB *)kind_name(scan.points[i].kind), (INT)scan.points[i].distance_mm);
                }
                tm_printf((UB *)"[obs] RESULT SCAN: %d ms, valid=%d obstacle=%d closest=%d mm at %d deg, "
                                "forward=%d mm, width=%d mm, centre x=%d mm, left clear=%d mm, right clear=%d mm\n",
                          (INT)(now - run_start_ms), (INT)profile.valid, (INT)profile.obstacle_found,
                          (INT)profile.closest_mm, (INT)profile.closest_angle_deg, (INT)profile.forward_mm,
                          (INT)profile.width_mm, (INT)profile.centre_x_mm,
                          (INT)profile.left_clearance_mm, (INT)profile.right_clearance_mm);
                tm_printf((UB *)"[obs] RESULT PLAN: action=%s reason=%s (width, closest and centre x are what to compare "
                                "with a ruler)\n",
                          (UB *)action_name(plan.action), (UB *)reason_name(plan.reason));
                running = false;
                measuring = false;
                step = STEP_DISTANCE;
                print_prompt(step, target_idx);
            } else {
                /* scanning */
            }
        }
        hal_motor_set(pwm, pwm);
    }
}

static ER make_obs_task(void)
{
    T_CTSK ct;
    ID id;

    ct.exinf = NULL;
    ct.tskatr = TA_HLNG | TA_RNG3;
    ct.task = obstacle_test_task;
    ct.itskpri = ROBOT_PRI_OBSTACLE;
    ct.stksz = ROBOT_STACK_BYTES * 2;
#if TK_SUPPORT_SMP
    /* Peripheral interrupts are owned by processor 1 (RP2040 core 0). */
    ct.tskatr |= TA_ASSPRC;
    ct.assprc = TP_PRC1;
#endif
    id = tk_cre_tsk(&ct);
    if (id < E_OK) {
        return id;
    }
    return tk_sta_tsk(id, 0);
}

ER obstacle_test_start_task(void)
{
    T_CFLG cflg;
    ER err;

    hal_init();
    cflg.flgatr = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    g_obs_flag = tk_cre_flg(&cflg);
    if (g_obs_flag < E_OK) {
        return g_obs_flag;
    }
    err = make_obs_task();
    if (err < E_OK) {
        return err;
    }
    err = hal_gpio_irq_start();
    if (err < E_OK) {
        return err;
    }
    return hal_tick_start(g_obs_flag);
}
