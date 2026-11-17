/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * freeroam_task.c
 * Open-space obstacle demo.  One task runs the real Motion module, the real
 * ultrasonic monitor, the real scan sequencer and the real planner.
 *
 *   START (GP20)  3 s countdown, then drive straight (up to 3 m)
 *   obstacle      the sonar sees something nearer than 500 mm: stop
 *   scan          the servo sweeps, the profile and plan are printed
 *   bypass        turn 90, move sideways, turn back, move past the obstacle, turn,
 *                 move back to the original path, turn to the original heading,
 *                 then drive 600 mm more and stop
 *   STOP (GP21)   aborts at any moment and stops the motors
 *
 * The side is chosen by the planner with the "open to the sweep limit" rule
 * (sweep_clear_min_rays = 2) and no echo counted as free space: the HC-SR04's beam
 * is too wide for the sonar to confirm room beside a box.  That is only safe in a
 * controlled open arena, so keep a hand near the car and clear 2 m ahead and 1 m to
 * each side.  Lines start with "[free]".
 *
 * NOT YET RUN ON THE CAR.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "freeroam_task.h"
#include "hal_rp2040.h"
#include "motion.h"
#include "obstacle.h"
#include "robot_config.h"
#include "servo.h"
#include "ultrasonic.h"

#define FR_BUTTON_SAMPLES       (6U)        /* 6 x 5 ms = 30 ms debounce */
#define FR_COUNTDOWN_MS         (3000U)
#define FR_DRIVE_MM             (3000)      /* the straight run */
#define FR_AFTER_MM             (1500)      /* the run after the bypass: it keeps looking for the next obstacle */
/* 1 = after passing the obstacle, cross back to the original path.  0 = stay in the new
 * lane and carry on straight.  With objects close together the return can put the car
 * right in front of the next one (seen on the car: 116 mm), because the sonar cannot see
 * ahead while the car is beside the box or turning back. */
#define FR_RETURN_TO_PATH       (1)
#define FR_SPEED_MM_S           (120)
#define FR_SCAN_MM              (500U)      /* stop and scan when the sonar sees this */
#define FR_HYST_MM              (30U)
#define FR_EMERGENCY_MM         (120U)      /* during a move: anything this near aborts */
#define FR_STOP_WAIT_MS         (2000U)
#define FR_SCAN_TIMEOUT_MS      (20000U)
#define FR_LEASE_MS             (40000U)
#define FR_TURN_TIMEOUT_MS      (8000U)
#define FR_BLANK_MS             (3000U)     /* after a "nothing there" scan, ignore the sonar */
#define FR_MAX_STEPS            (9U)
#define FR_SENSOR_CONFIRM       (2U)
#define FR_INVALID_LIMIT        (8U)
/* 1 = when the scan shows no open side (the box fills the whole sweep), go round
 * anyway toward the side whose outer rays read farther.  The sonar cannot see the
 * sides then, so only use it with nothing beside the path; the sonar still aborts a
 * move if anything comes within FR_EMERGENCY_MM.  0 = stop and wait instead. */
#define FR_BLIND_FALLBACK       (1)

typedef enum {
    FR_IDLE = 0,
    FR_COUNTDOWN,
    FR_RUN,
    FR_STOPPING,
    FR_SCAN
} fr_state_t;

typedef struct {
    bool    turn;           /* true: turn by 'value' degrees (+ = left); false: move 'value' mm */
    int16_t value;
    bool    watch;          /* look for an obstacle ahead during this step */
} fr_step_t;

typedef struct {
    uint8_t count;
    bool    state;
} button_t;

static ID g_fr_flag;

static bool button_pressed(button_t *b, bool raw)
{
    bool edge = false;

    if (raw == b->state) {
        b->count = 0U;
    } else {
        b->count++;
        if (b->count >= FR_BUTTON_SAMPLES) {
            b->state = raw;
            b->count = 0U;
            edge = raw;
        }
    }
    return edge;
}

static void servo_to(const obstacle_config_t *cfg, uint8_t angle_deg)
{
    hal_servo_set_pulse_us(servo_pulse_us(&cfg->servo, angle_deg));
}

static int32_t abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static uint32_t step_timeout_ms(const fr_step_t *s)
{
    if (s->turn) {
        return FR_TURN_TIMEOUT_MS;
    }
    return ((uint32_t)abs32(s->value) * 1000U * 16U) / ((uint32_t)FR_SPEED_MM_S * 10U) + 3000U;
}

static void submit_step(motion_t *motion, uint16_t id, const fr_step_t *s, uint32_t now)
{
    motion_cmd_t cmd;

    cmd.id = id;
    cmd.diff_mm_s = 0;
    cmd.lease_ms = (uint16_t)FR_LEASE_MS;
    if (s->turn) {
        cmd.type = MOTION_CMD_TURN;
        cmd.speed_mm_s = 0;
        cmd.distance_mm = 0;
        cmd.angle_deg = s->value;
    } else {
        cmd.type = MOTION_CMD_MOVE;
        cmd.speed_mm_s = (int16_t)FR_SPEED_MM_S;
        cmd.distance_mm = s->value;
        cmd.angle_deg = 0;
    }
    (void)motion_submit(motion, &cmd, now);
}

static void submit_stop(motion_t *motion, uint16_t id, uint32_t now)
{
    motion_cmd_t cmd;

    cmd.id = id;
    cmd.type = MOTION_CMD_STOP;
    cmd.speed_mm_s = 0;
    cmd.diff_mm_s = 0;
    cmd.distance_mm = 0;
    cmd.angle_deg = 0;
    cmd.lease_ms = 0U;
    (void)motion_submit(motion, &cmd, now);
}

static const char *action_name(plan_action_t a)
{
    switch (a) {
    case PLAN_CONTINUE:     return "continue";
    case PLAN_BYPASS_LEFT:  return "bypass LEFT";
    case PLAN_BYPASS_RIGHT: return "bypass RIGHT";
    case PLAN_REVERSE:      return "too close, would reverse";
    default:                return "stop";
    }
}

static const char *reason_name(plan_reason_t r)
{
    switch (r) {
    case PLAN_REASON_OK:               return "ok";
    case PLAN_REASON_NO_OBSTACLE:      return "no obstacle";
    case PLAN_REASON_INVALID_SCAN:     return "invalid scan";
    case PLAN_REASON_NO_CLEARANCE:     return "no clear side";
    case PLAN_REASON_DETOUR_TOO_LARGE: return "detour too large";
    default:                           return "too close";
    }
}

static const char *kind_name(scan_point_kind_t k)
{
    switch (k) {
    case SCAN_POINT_OBSTACLE: return "obstacle";
    case SCAN_POINT_FAR:      return "clear";
    case SCAN_POINT_NOECHO:   return "no-echo";
    default:                  return "invalid";
    }
}

/* The whole manoeuvre, as steps, from the planner's lateral and forward distances. */
static uint8_t build_bypass(fr_step_t *steps, const bypass_plan_t *plan)
{
    const int16_t s = (plan->action == PLAN_BYPASS_LEFT) ? 1 : -1;
    const int16_t lateral = plan->steps[1].value;
    const int16_t forward = plan->steps[3].value;      /* includes forward_extra_mm */

    steps[0].turn = true;  steps[0].value = (int16_t)(s * 90);   steps[0].watch = false;
    steps[1].turn = false; steps[1].value = lateral;             steps[1].watch = false;
    steps[2].turn = true;  steps[2].value = (int16_t)(-s * 90);  steps[2].watch = false;
    steps[3].turn = false; steps[3].value = forward;             steps[3].watch = false;
#if FR_RETURN_TO_PATH
    steps[4].turn = true;  steps[4].value = (int16_t)(-s * 90);  steps[4].watch = false;
    steps[5].turn = false; steps[5].value = lateral;             steps[5].watch = false;
    steps[6].turn = true;  steps[6].value = (int16_t)(s * 90);   steps[6].watch = false;
    steps[7].turn = false; steps[7].value = (int16_t)FR_AFTER_MM; steps[7].watch = true;
    return 8U;
#else
    /* Stay in the new lane: the car already faces its original heading, and the next
     * object is often only half a metre beyond this one, in the old lane. */
    steps[4].turn = false; steps[4].value = (int16_t)FR_AFTER_MM; steps[4].watch = true;
    return 5U;
#endif
}

static void freeroam_task(INT stacd, void *exinf)
{
    static motion_t motion;
    static us_monitor_t mon;
    static scan_t scan;
    static us_reading_t reading;
    static obstacle_profile_t profile;
    static bypass_plan_t plan;
    static fr_step_t steps[FR_MAX_STEPS];
    obstacle_config_t cfg = ROBOT_OBSTACLE_CFG;
    encoder_snapshot_t left;
    encoder_snapshot_t right;
    motion_output_t mo;
    button_t start_btn = { 0U, false };
    button_t stop_btn = { 0U, false };
    fr_state_t state = FR_IDLE;
    uint8_t step_count = 0U;
    uint8_t step_idx = 0U;
    uint16_t cmd_id = 0U;
    uint32_t state_since_ms = 0U;
    uint32_t step_since_ms = 0U;
    uint32_t last_start_ms = 0U;
    uint32_t blank_until_ms = 0U;
    int32_t run_start_mm = 0;
    bool measuring = false;

    (void)stacd;
    (void)exinf;
    motion_init(&motion, &ROBOT_MOTION_CFG);
    hal_motor_set(0, 0);
    servo_to(&cfg, 90U);
    reading.status = US_ERROR;
    reading.seq = 1U;
    tm_printf((UB *)"[free] open-space obstacle bypass.  Clear 2 m ahead and 1 m each side, a box about 1 m "
                    "ahead on the car's path.  START = go (3 s countdown), STOP = abort at any time.\n");

    for (;;) {
        UINT pattern = 0U;
        uint32_t now;
        bool start_edge;
        bool stop_edge;
        bool fresh = false;
        bool abort_run = false;
        const char *abort_why = "";
        const motion_status_t *st;
        int16_t pwm_l = 0;
        int16_t pwm_r = 0;

        (void)tk_wai_flg(g_fr_flag, HAL_TICK_MOTION, TWF_ORW | TWF_BITCLR, &pattern, TMO_FEVR);
        hal_tick_ack(HAL_TICK_MOTION);
        now = hal_time_ms();
        hal_encoders_snapshot(&left, &right);
        motion_step(&motion, &left, &right, now, hal_time_us(), &mo);
        st = motion_status(&motion);
        start_edge = button_pressed(&start_btn, hal_button_start());
        stop_edge = button_pressed(&stop_btn, hal_button_stop());

        /* the ultrasonic engine: same trigger, poll and convert as the robot */
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

        switch (state) {
        case FR_IDLE:
            if (start_edge) {
                state = FR_COUNTDOWN;
                state_since_ms = now;
                tm_printf((UB *)"[free] starting in 3 s: step back, keep clear of the wheels\n");
            }
            break;

        case FR_COUNTDOWN:
            if (stop_edge) {
                state = FR_IDLE;
                tm_printf((UB *)"[free] cancelled\n");
            } else if ((now - state_since_ms) >= FR_COUNTDOWN_MS) {
                steps[0].turn = false;
                steps[0].value = (int16_t)FR_DRIVE_MM;
                steps[0].watch = true;
                step_count = 1U;
                step_idx = 0U;
                blank_until_ms = 0U;
                run_start_mm = st->distance_mm;
                us_monitor_init(&mon, (uint8_t)FR_SENSOR_CONFIRM, (uint8_t)FR_INVALID_LIMIT);
                cmd_id++;
                submit_step(&motion, cmd_id, &steps[0], now);
                step_since_ms = now;
                state = FR_RUN;
                tm_printf((UB *)"[free] DRIVE straight, up to %d mm at %d mm/s\n", (INT)FR_DRIVE_MM, (INT)FR_SPEED_MM_S);
            } else {
                /* counting down */
            }
            break;

        case FR_RUN: {
            const fr_step_t *cur = &steps[step_idx];
            const bool done = (st->state == MOTION_DONE) && (st->cmd_id == cmd_id);

            pwm_l = mo.left_pwm;
            pwm_r = mo.right_pwm;
            if (!cur->turn && !measuring && ((now - last_start_ms) >= ROBOT_US_PERIOD_MS)) {
                hal_us_trigger();
                measuring = true;
                last_start_ms = now;
            }
            if (stop_edge) {
                abort_run = true;
                abort_why = "STOP button";
            } else if (!cur->turn && fresh && (reading.status == US_VALID) &&
                       (reading.distance_mm < FR_EMERGENCY_MM)) {
                abort_run = true;
                abort_why = "something is right in front of the car";
                tm_printf((UB *)"[free] sonar reads %d mm\n", (INT)reading.distance_mm);
            } else if (cur->watch && fresh) {
                const bool obstacle = us_monitor_update(&mon, &reading, (uint16_t)FR_SCAN_MM,
                                                        (uint16_t)FR_HYST_MM, (uint16_t)(FR_SCAN_MM / 2U));

                if (obstacle && ((int32_t)(now - blank_until_ms) >= 0)) {
                    tm_printf((UB *)"[free] OBSTACLE: sonar reads %d mm, travelled %d mm.  Stopping.\n",
                              (INT)reading.distance_mm, (INT)(st->distance_mm - run_start_mm));
                    cmd_id++;
                    submit_stop(&motion, cmd_id, now);
                    state = FR_STOPPING;
                    state_since_ms = now;
                }
            } else if (done) {
                step_idx++;
                if (step_idx >= step_count) {
                    cmd_id++;
                    submit_stop(&motion, cmd_id, now);
                    tm_printf((UB *)"[free] DONE.  The run is finished.  START to run again.\n");
                    state = FR_IDLE;
                } else {
                    if (steps[step_idx].watch) {
                        /* a fresh monitor: the old one still says "obstacle" from the stop */
                        us_monitor_init(&mon, (uint8_t)FR_SENSOR_CONFIRM, (uint8_t)FR_INVALID_LIMIT);
                    }
                    cmd_id++;
                    submit_step(&motion, cmd_id, &steps[step_idx], now);
                    step_since_ms = now;
                    tm_printf((UB *)"[free] step %d of %d: %s %d %s\n", (INT)(step_idx + 1U), (INT)step_count,
                              steps[step_idx].turn ? (UB *)"TURN" : (UB *)"MOVE", (INT)steps[step_idx].value,
                              steps[step_idx].turn ? (UB *)"deg (+ is left)" : (UB *)"mm");
                }
            } else if ((now - step_since_ms) >= step_timeout_ms(cur)) {
                abort_run = true;
                abort_why = "a step took too long";
            } else {
                /* running */
            }
            break;
        }

        case FR_STOPPING:
            if (stop_edge) {
                abort_run = true;
                abort_why = "STOP button";
            } else if (((st->flags & MOTION_FLAG_STOPPED) != 0U) || ((now - state_since_ms) >= FR_STOP_WAIT_MS)) {
                measuring = false;
                scan_start(&scan, &cfg, now);
                state = FR_SCAN;
                state_since_ms = now;
                tm_printf((UB *)"[free] stopped.  SCAN (rays %d to %d deg)\n", (INT)cfg.coarse_min_deg,
                          (INT)cfg.coarse_max_deg);
            } else {
                pwm_l = mo.left_pwm;
                pwm_r = mo.right_pwm;
            }
            break;

        case FR_SCAN: {
            scan_action_t act;
            const bool done = scan_update(&scan, now, &reading, &act);

            if (act.servo_move) {
                servo_to(&cfg, act.servo_angle);
            }
            if (act.request_measure && !measuring) {
                hal_us_trigger();
                measuring = true;
            }
            if (stop_edge) {
                abort_run = true;
                abort_why = "STOP button";
            } else if (!done && ((now - state_since_ms) >= FR_SCAN_TIMEOUT_MS)) {
                abort_run = true;
                abort_why = "the scan took too long";
            } else if (done) {
                uint8_t i;

                obstacle_profile_build(&cfg, scan.points, scan.point_count, &profile);
                obstacle_plan(&cfg, &profile, &plan);
                for (i = 0U; i < scan.point_count; i++) {
                    tm_printf((UB *)"[free] POINT angle=%d deg  %s  %d mm\n", (INT)scan.points[i].angle_deg,
                              (UB *)kind_name(scan.points[i].kind), (INT)scan.points[i].distance_mm);
                }
                tm_printf((UB *)"[free] SCAN RESULT: closest=%d mm, width=%d mm, centre x=%d mm, "
                                "clear left=%d mm, clear right=%d mm\n",
                          (INT)profile.closest_mm, (INT)profile.width_mm, (INT)profile.centre_x_mm,
                          (INT)profile.left_clearance_mm, (INT)profile.right_clearance_mm);
                tm_printf((UB *)"[free] PLAN: %s (%s)\n", (UB *)action_name(plan.action), (UB *)reason_name(plan.reason));
#if FR_BLIND_FALLBACK
                if ((plan.action == PLAN_STOP) && (plan.reason == PLAN_REASON_NO_CLEARANCE) &&
                    obstacle_blind_plan(&cfg, scan.points, scan.point_count, &profile, &plan)) {
                    tm_printf((UB *)"[free] no open side was seen: going round %s ANYWAY (blind)\n",
                              (plan.action == PLAN_BYPASS_LEFT) ? (UB *)"LEFT" : (UB *)"RIGHT");
                }
#endif
                if (((plan.action == PLAN_BYPASS_LEFT) || (plan.action == PLAN_BYPASS_RIGHT)) &&
                    (plan.step_count >= 4U)) {
                    step_count = build_bypass(steps, &plan);
                    step_idx = 0U;
                    tm_printf((UB *)"[free] BYPASS %s: sideways %d mm, forward %d mm\n",
                              (plan.action == PLAN_BYPASS_LEFT) ? (UB *)"LEFT" : (UB *)"RIGHT",
                              (INT)plan.steps[1].value, (INT)plan.steps[3].value);
                    cmd_id++;
                    submit_step(&motion, cmd_id, &steps[0], now);
                    step_since_ms = now;
                    state = FR_RUN;
                    tm_printf((UB *)"[free] step 1 of %d: TURN %d deg (+ is left)\n", (INT)step_count, (INT)steps[0].value);
                } else if (plan.action == PLAN_CONTINUE) {
                    steps[0].turn = false;
                    steps[0].value = (int16_t)FR_DRIVE_MM;
                    steps[0].watch = true;
                    step_count = 1U;
                    step_idx = 0U;
                    blank_until_ms = now + FR_BLANK_MS;
                    us_monitor_init(&mon, (uint8_t)FR_SENSOR_CONFIRM, (uint8_t)FR_INVALID_LIMIT);
                    cmd_id++;
                    submit_step(&motion, cmd_id, &steps[0], now);
                    step_since_ms = now;
                    state = FR_RUN;
                    tm_printf((UB *)"[free] nothing in the way: driving on\n");
                } else {
                    tm_printf((UB *)"[free] no safe way round: the car stays stopped.  START to try again.\n");
                    state = FR_IDLE;
                }
            } else {
                /* scanning */
            }
            break;
        }

        default:
            state = FR_IDLE;
            break;
        }

        if (abort_run) {
            cmd_id++;
            submit_stop(&motion, cmd_id, now);
            servo_to(&cfg, 90U);
            measuring = false;
            state = FR_IDLE;
            tm_printf((UB *)"[free] ABORTED: %s.  The motors are stopped.  START to run again.\n", (UB *)abort_why);
            pwm_l = 0;
            pwm_r = 0;
        }
        hal_motor_set(pwm_l, pwm_r);
    }
}

static ER make_fr_task(void)
{
    T_CTSK ct;
    ID id;

    ct.exinf = NULL;
    ct.tskatr = TA_HLNG | TA_RNG3;
    ct.task = freeroam_task;
    ct.itskpri = ROBOT_PRI_MOTION;
    ct.stksz = ROBOT_STACK_BYTES * 3;
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

ER freeroam_start_task(void)
{
    T_CFLG cflg;
    ER err;

    hal_init();
    cflg.flgatr = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    g_fr_flag = tk_cre_flg(&cflg);
    if (g_fr_flag < E_OK) {
        return g_fr_flag;
    }
    err = make_fr_task();
    if (err < E_OK) {
        return err;
    }
    err = hal_gpio_irq_start();
    if (err < E_OK) {
        return err;
    }
    return hal_tick_start(g_fr_flag);
}
