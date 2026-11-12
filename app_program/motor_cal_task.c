/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * motor_cal_task.c
 * Motor calibration on the car.  Press START (GP20) to run the next test; each
 * test prints "[cal]" lines on the USB console.  Copy the whole log into
 * tools/motor_cal_fit.py to get the numbers for ROBOT_MOTION_CFG.
 *
 *   1  dead zone, left wheel    (the car pivots on the right wheel)
 *   2  dead zone, right wheel   (the car pivots on the left wheel)
 *   3  speed table, both wheels open loop (needs about 2 m of clear floor)
 *   4  1000 mm drive, closed loop (check the distance with a tape measure)
 *   5  360 degree turn left, closed loop (measure the real angle)
 *   6  360 degree turn right
 *
 * Tests 4-6 use the real motion module with the current ROBOT_MOTION_CFG, so
 * they also check the calibration.  The STOP button (GP21) ends any test.
 *
 * NOT YET RUN ON THE CAR.  The state machine in motor_cal.c is tested on the
 * host; this file is glue and has only been read, not compiled for the board.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "hal_rp2040.h"
#include "motion.h"
#include "motor_cal.h"
#include "motor_cal_task.h"
#include "robot_config.h"

#define CAL_BUTTON_SAMPLES      (6U)        /* 6 x 5 ms = 30 ms debounce */
#define CAL_CLOSED_LEASE_MS     (8000U)
#define CAL_CLOSED_TIMEOUT_MS   (12000U)
#define CAL_CLOSED_REPORT_MS    (250U)
#define CAL_MOVE_MM             (1000)
#define CAL_MOVE_SPEED_MM_S     (120)
#define CAL_TURN_DEG            (360)

typedef enum {
    CAL_STEP_DZ_LEFT = 0,
    CAL_STEP_DZ_RIGHT,
    CAL_STEP_SPEED,
    CAL_STEP_MOVE,
    CAL_STEP_TURN_LEFT,
    CAL_STEP_TURN_RIGHT,
    CAL_STEP_COUNT
} cal_step_t;

typedef enum {
    CAL_MODE_WAIT = 0,
    CAL_MODE_OPEN,
    CAL_MODE_CLOSED
} cal_mode_t;

typedef struct {
    uint8_t count;
    bool    state;
} button_t;

static ID g_cal_flag;

static const char *const STEP_TEXT[CAL_STEP_COUNT] = {
    "1 dead zone, LEFT wheel (car pivots on the right wheel)",
    "2 dead zone, RIGHT wheel (car pivots on the left wheel)",
    "3 speed table, both wheels (about 2 m of clear floor)",
    "4 closed-loop drive 1000 mm",
    "5 closed-loop turn 360 deg LEFT",
    "6 closed-loop turn 360 deg RIGHT"
};

/* Debounced press: true on the sample where the button goes from up to down. */
static bool button_pressed(button_t *b, bool raw)
{
    bool edge = false;

    if (raw == b->state) {
        b->count = 0U;
    } else {
        b->count++;
        if (b->count >= CAL_BUTTON_SAMPLES) {
            b->state = raw;
            b->count = 0U;
            edge = raw;
        }
    }
    return edge;
}

static void print_prompt(cal_step_t step)
{
    tm_printf((UB *)"[cal] ready: press START to run test %s\n", (UB *)STEP_TEXT[step]);
}

static void print_event(const mcal_event_t *ev)
{
    switch (ev->kind) {
    case MCAL_EV_SAMPLE:
        tm_printf((UB *)"[cal] SAMPLE pwm=%d l=%d r=%d\n",
                  (INT)ev->pwm, (INT)ev->left_cps, (INT)ev->right_cps);
        break;
    case MCAL_EV_BREAKAWAY:
        tm_printf((UB *)"[cal] BREAKAWAY pwm=%d l=%d r=%d\n",
                  (INT)ev->pwm, (INT)ev->left_cps, (INT)ev->right_cps);
        break;
    case MCAL_EV_SUSTAIN:
        tm_printf((UB *)"[cal] SUSTAIN pwm=%d l=%d r=%d\n",
                  (INT)ev->pwm, (INT)ev->left_cps, (INT)ev->right_cps);
        break;
    case MCAL_EV_SPEED:
        tm_printf((UB *)"[cal] SPEED pwm=%d l=%d r=%d\n",
                  (INT)ev->pwm, (INT)ev->left_cps, (INT)ev->right_cps);
        break;
    case MCAL_EV_DONE:
        tm_printf((UB *)"[cal] DONE\n");
        break;
    case MCAL_EV_ABORT:
        tm_printf((UB *)"[cal] ABORT pwm=%d (no breakaway up to the limit, or travelled too far)\n",
                  (INT)ev->pwm);
        break;
    default:
        break;
    }
}

static void submit_closed_loop(motion_t *motion, cal_step_t step, uint16_t id, uint32_t now_ms)
{
    motion_cmd_t cmd;

    cmd.id = id;
    cmd.speed_mm_s = 0;
    cmd.diff_mm_s = 0;
    cmd.distance_mm = 0;
    cmd.angle_deg = 0;
    cmd.lease_ms = (uint16_t)CAL_CLOSED_LEASE_MS;
    if (step == CAL_STEP_MOVE) {
        cmd.type = MOTION_CMD_MOVE;
        cmd.speed_mm_s = (int16_t)CAL_MOVE_SPEED_MM_S;
        cmd.distance_mm = (int16_t)CAL_MOVE_MM;
    } else {
        cmd.type = MOTION_CMD_TURN;
        cmd.angle_deg = (step == CAL_STEP_TURN_LEFT) ? (int16_t)CAL_TURN_DEG : (int16_t)(-CAL_TURN_DEG);
    }
    (void)motion_submit(motion, &cmd, now_ms);
}

static void print_closed_result(cal_step_t step, const motion_status_t *st, bool timed_out,
                                uint32_t elapsed_ms, int32_t dl, int32_t dr)
{
    tm_printf((UB *)"[cal] %s t=%dms enc L=%d R=%d dist=%dmm\n",
              timed_out ? (UB *)"CLOSED-TIMEOUT" : (UB *)"CLOSED-DONE",
              (INT)elapsed_ms, (INT)dl, (INT)dr, (INT)st->distance_mm);
    if (step == CAL_STEP_MOVE) {
        tm_printf((UB *)"[cal] measure the real distance D (mm): new counts_per_rev = %d x %d / D\n",
                  (INT)ROBOT_MOTION_CFG.counts_per_rev, (INT)CAL_MOVE_MM);
    } else {
        tm_printf((UB *)"[cal] measure the real angle A (deg): new track_width = %d x 360 / A\n",
                  (INT)ROBOT_MOTION_CFG.track_width_mm);
    }
}

static void motor_cal_task(INT stacd, void *exinf)
{
    static motion_t motion;
    static motor_cal_t cal;
    encoder_snapshot_t left;
    encoder_snapshot_t right;
    motion_output_t mo;
    button_t start_btn = { 0U, false };
    button_t stop_btn = { 0U, false };
    cal_mode_t mode = CAL_MODE_WAIT;
    cal_step_t step = CAL_STEP_DZ_LEFT;
    uint16_t cmd_id = 0U;
    uint32_t closed_start_ms = 0U;
    uint32_t last_report_ms = 0U;
    int32_t closed_base_l = 0;
    int32_t closed_base_r = 0;

    (void)stacd;
    (void)exinf;
    motion_init(&motion, &ROBOT_MOTION_CFG);
    hal_motor_set(0, 0);
    tm_printf((UB *)"[cal] motor calibration. START = run next test, STOP = abort the running test.\n");
    print_prompt(step);

    for (;;) {
        UINT pattern;
        int16_t pwm_l = 0;
        int16_t pwm_r = 0;
        uint32_t now;
        bool start_edge;
        bool stop_edge;

        (void)tk_wai_flg(g_cal_flag, HAL_TICK_MOTION, TWF_ORW | TWF_BITCLR, &pattern, TMO_FEVR);
        hal_tick_ack(HAL_TICK_MOTION);
        now = hal_time_ms();
        hal_encoders_snapshot(&left, &right);
        /* Always stepped, so its speed estimate is ready when a closed-loop test starts. */
        motion_step(&motion, &left, &right, now, hal_time_us(), &mo);
        start_edge = button_pressed(&start_btn, hal_button_start());
        stop_edge = button_pressed(&stop_btn, hal_button_stop());

        switch (mode) {
        case CAL_MODE_WAIT:
            if (start_edge) {
                tm_printf((UB *)"[cal] RUN %s\n", (UB *)STEP_TEXT[step]);
                if (step <= CAL_STEP_SPEED) {
                    const mcal_test_t test = (step == CAL_STEP_DZ_LEFT) ? MCAL_DEADZONE_LEFT :
                                             (step == CAL_STEP_DZ_RIGHT) ? MCAL_DEADZONE_RIGHT :
                                             MCAL_SPEED_TABLE;

                    motor_cal_start(&cal, test, now, left.count, right.count);
                    mode = CAL_MODE_OPEN;
                } else {
                    cmd_id++;
                    closed_start_ms = now;
                    last_report_ms = now;
                    closed_base_l = left.count;
                    closed_base_r = right.count;
                    submit_closed_loop(&motion, step, cmd_id, now);
                    mode = CAL_MODE_CLOSED;
                }
            }
            break;

        case CAL_MODE_OPEN: {
            mcal_event_t ev;
            const bool running = motor_cal_step(&cal, now, left.count, right.count,
                                                &pwm_l, &pwm_r, &ev);

            print_event(&ev);
            if (stop_edge) {
                pwm_l = 0;
                pwm_r = 0;
                tm_printf((UB *)"[cal] STOPPED by button\n");
                mode = CAL_MODE_WAIT;
                print_prompt(step);
            } else if (!running) {
                mode = CAL_MODE_WAIT;
                step = (cal_step_t)((uint32_t)step + 1U);
                print_prompt(step);
            } else {
                /* keep running */
            }
            break;
        }

        case CAL_MODE_CLOSED: {
            const motion_status_t *st = motion_status(&motion);
            const uint32_t elapsed = now - closed_start_ms;
            const bool done = (st->state == MOTION_DONE) && (st->cmd_id == cmd_id);
            const bool timed_out = elapsed >= CAL_CLOSED_TIMEOUT_MS;

            pwm_l = mo.left_pwm;
            pwm_r = mo.right_pwm;
            if ((now - last_report_ms) >= CAL_CLOSED_REPORT_MS) {
                last_report_ms = now;
                tm_printf((UB *)"[cal] RUN t=%d v=%d/%d mm/s dist=%dmm pwm=%d/%d\n",
                          (INT)elapsed, (INT)st->left_mm_s, (INT)st->right_mm_s,
                          (INT)st->distance_mm, (INT)pwm_l, (INT)pwm_r);
            }
            if (stop_edge || done || timed_out) {
                motion_cmd_t stop;

                stop.id = (uint16_t)(cmd_id + 1U);
                stop.type = MOTION_CMD_STOP;
                stop.speed_mm_s = 0;
                stop.diff_mm_s = 0;
                stop.distance_mm = 0;
                stop.angle_deg = 0;
                stop.lease_ms = 0U;
                cmd_id++;
                (void)motion_submit(&motion, &stop, now);
                pwm_l = 0;
                pwm_r = 0;
                if (stop_edge) {
                    tm_printf((UB *)"[cal] STOPPED by button\n");
                } else {
                    print_closed_result(step, st, timed_out, elapsed,
                                        left.count - closed_base_l,
                                        right.count - closed_base_r);
                }
                mode = CAL_MODE_WAIT;
                step = (cal_step_t)((uint32_t)step + 1U);
                if (step >= CAL_STEP_COUNT) {
                    step = CAL_STEP_DZ_LEFT;
                    tm_printf((UB *)"[cal] all tests done: copy the log into tools/motor_cal_fit.py\n");
                }
                print_prompt(step);
            }
            break;
        }

        default:
            break;
        }

        hal_motor_set(pwm_l, pwm_r);
    }
}

static ER make_cal_task(void)
{
    T_CTSK ct;
    ID id;

    ct.exinf = NULL;
    ct.tskatr = TA_HLNG | TA_RNG3;
    ct.task = motor_cal_task;
    ct.itskpri = ROBOT_PRI_MOTION;
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

ER motor_cal_start_task(void)
{
    T_CFLG cflg;
    ER err;

    hal_init();
    cflg.flgatr = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    g_cal_flag = tk_cre_flg(&cflg);
    if (g_cal_flag < E_OK) {
        return g_cal_flag;
    }
    err = make_cal_task();
    if (err < E_OK) {
        return err;
    }
    err = hal_gpio_irq_start();
    if (err < E_OK) {
        return err;
    }
    return hal_tick_start(g_cal_flag);
}
