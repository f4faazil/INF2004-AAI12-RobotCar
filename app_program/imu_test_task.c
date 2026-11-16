/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * imu_test_task.c
 * IMU bring-up on the car.  Press START (GP20) to move to the next step; each
 * step prints "[imu]" lines on the USB console.  STOP (GP21) aborts a drive.
 *
 *   1  REST   car level and still: reads the three axes, checks Z is +1 g
 *   2  NOSE   lift the nose about 20 degrees and hold: finds which axis is
 *             forward and the sign that makes "nose up" a positive pitch
 *   3  LIVE   prints the pitch every 200 ms with the sign just found, so a
 *             nose-up lift must read positive
 *   4  HUMP   START again: rest calibration, then a closed-loop 1000 mm drive
 *             at 120 mm/s with the real hump detector running.  Put a hump of
 *             known height in the path and compare.  Repeat as often as needed.
 *
 * Only the forward axis, its sign and the hump height are results.  The values
 * below the "RESULT" lines are the ones to put in robot_config.h.
 *
 * NOT YET RUN ON THE CAR.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "hal_rp2040.h"
#include "imu.h"
#include "imu_test_task.h"
#include "motion.h"
#include "robot_config.h"
#include "robot_tasks.h"

#define TEST_BUTTON_SAMPLES     (6U)        /* 6 x 5 ms = 30 ms debounce */
#define TEST_AVG_SAMPLES        (50U)       /* 1 s at 50 Hz */
#define TEST_PRINT_MS           (200U)
#define TEST_DRIVE_PRINT_MS     (100U)      /* a 19 cm hump takes about 1.6 s at 120 mm/s */
#define TEST_REST_MIN_Z_MG      (700)       /* flat: Z carries gravity, up (+) or upside down (-) */
#define TEST_REST_MAX_XY_MG     (250)       /* about 15 degrees of tilt */
#define TEST_MAG_LOW_MG         (900)       /* |g| outside 900..1100 is a scale or offset error */
#define TEST_MAG_HIGH_MG        (1100)
#define TEST_NOSE_MIN_MG        (100)       /* at least about 6 degrees of lift */
#define TEST_MOVE_MM            (1000)
#define TEST_MOVE_SPEED_MM_S    (120)
#define TEST_MOVE_LEASE_MS      (15000U)
#define TEST_MOVE_TIMEOUT_MS    (16000U)
#define TEST_SETTLE_MS          (3000U)     /* total settle time after START */
#define TEST_IGNORE_MS          (1200U)     /* the START press shakes the car: ignore this long */
#define TEST_CAL_SPREAD_MG      (250)       /* floor vibration is about +-50 mg; a hand is far more */

typedef enum {
    STEP_REST = 0,
    STEP_NOSE,
    STEP_LIVE,
    STEP_COUNT
} test_step_t;

typedef enum {
    PHASE_MENU = 0,     /* waiting for START */
    PHASE_AVERAGE,      /* averaging samples for REST or NOSE */
    PHASE_SETTLE,       /* HUMP: calibrating at rest */
    PHASE_DRIVE         /* HUMP: driving */
} test_phase_t;

typedef struct {
    uint8_t count;
    bool    state;
} button_t;

static ID g_test_flag;

/* Debounced press: true on the sample where the button goes from up to down. */
static bool button_pressed(button_t *b, bool raw)
{
    bool edge = false;

    if (raw == b->state) {
        b->count = 0U;
    } else {
        b->count++;
        if (b->count >= TEST_BUTTON_SAMPLES) {
            b->state = raw;
            b->count = 0U;
            edge = raw;
        }
    }
    return edge;
}

static int16_t axis_value(const imu_vec3_t *v, uint8_t axis)
{
    return (axis == IMU_AXIS_X) ? v->x : ((axis == IMU_AXIS_Y) ? v->y : v->z);
}

static char axis_name(uint8_t axis)
{
    return (axis == IMU_AXIS_X) ? 'X' : ((axis == IMU_AXIS_Y) ? 'Y' : 'Z');
}

static int32_t abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* The sideways axis: not the forward axis, and not the one carrying gravity. */
static uint8_t lateral_axis(uint8_t fwd_axis, const imu_vec3_t *rest)
{
    uint8_t vertical = IMU_AXIS_Z;
    int32_t best = 0;
    uint8_t axis;

    for (axis = 0U; axis < 3U; axis++) {
        const int32_t a = abs32(axis_value(rest, axis));

        if ((axis != fwd_axis) && (a > best)) {
            best = a;
            vertical = axis;
        }
    }
    for (axis = 0U; axis < 3U; axis++) {
        if ((axis != fwd_axis) && (axis != vertical)) {
            return axis;
        }
    }
    return IMU_AXIS_Y;
}

static int32_t isqrt_i32(int32_t v)
{
    int32_t r = 0;

    while (((r + 1) * (r + 1)) <= v) {
        r++;
    }
    return r;
}

static void print_prompt(test_step_t step)
{
    switch (step) {
    case STEP_REST:
        tm_printf((UB *)"[imu] step 1 REST: put the car level and still, then press START\n");
        break;
    case STEP_NOSE:
        tm_printf((UB *)"[imu] step 2 NOSE: lift the nose about 20 degrees and hold it, then press START\n");
        break;
    default:
        tm_printf((UB *)"[imu] READY: put the car FLAT on the floor, wheels down, hands off, hump about 30 cm ahead. "
                        "Press START (GP20) to run the 1 m hump drive, STOP (GP21) only aborts a drive.\n");
        break;
    }
}

static void submit_move(motion_t *motion, uint16_t id, uint32_t now_ms)
{
    motion_cmd_t cmd;

    cmd.id = id;
    cmd.type = MOTION_CMD_MOVE;
    cmd.speed_mm_s = (int16_t)TEST_MOVE_SPEED_MM_S;
    cmd.diff_mm_s = 0;
    cmd.distance_mm = (int16_t)TEST_MOVE_MM;
    cmd.angle_deg = 0;
    cmd.lease_ms = (uint16_t)TEST_MOVE_LEASE_MS;
    (void)motion_submit(motion, &cmd, now_ms);
}

static void submit_stop(motion_t *motion, uint16_t id, uint32_t now_ms)
{
    motion_cmd_t cmd;

    cmd.id = id;
    cmd.type = MOTION_CMD_STOP;
    cmd.speed_mm_s = 0;
    cmd.diff_mm_s = 0;
    cmd.distance_mm = 0;
    cmd.angle_deg = 0;
    cmd.lease_ms = 0U;
    (void)motion_submit(motion, &cmd, now_ms);
}

static void imu_test_task(INT stacd, void *exinf)
{
    static motion_t motion;
    static imu_cal_t cal;
    static imu_cal_t cal_roll;
    int16_t roll_peak = 0;
    int16_t pitch_peak = 0;
    static hump_detector_t hump;
    encoder_snapshot_t left;
    encoder_snapshot_t right;
    motion_output_t mo;
    button_t start_btn = { 0U, false };
    button_t stop_btn = { 0U, false };
    /* ROBOT_IMU_TEST_SKIP_SETUP: start at the hump drive, with the forward axis
     * and sign from robot_config.h (already measured), so steps 1 and 2 are not needed. */
    test_step_t step = ROBOT_IMU_TEST_SKIP_SETUP ? STEP_LIVE : STEP_REST;
    test_phase_t phase = PHASE_MENU;
    imu_variant_t variant = IMU_VARIANT_LSM303D;
    uint8_t addr = 0U;
    bool present = false;
    uint8_t fail_count = 0U;
    uint8_t fwd_axis = (uint8_t)ROBOT_IMU_FORWARD_AXIS;
    int8_t fwd_sign = (int8_t)ROBOT_IMU_FORWARD_SIGN;
    imu_vec3_t rest_mean = { 0, 0, 0 };
    bool rest_done = false;
    int32_t sum[3] = { 0, 0, 0 };
    uint16_t avg_n = 0U;
    uint16_t cmd_id = 0U;
    uint32_t phase_start_ms = 0U;
    uint32_t last_print_ms = 0U;
    int32_t base_dist_mm = 0;
    uint8_t humps_before = 0U;

    (void)stacd;
    (void)exinf;
    motion_init(&motion, &ROBOT_MOTION_CFG);
    hal_motor_set(0, 0);
    hump_init(&hump, &ROBOT_HUMP_CFG);
    tm_printf((UB *)"[imu] IMU bring-up. START = next step, STOP = abort the drive.\n");
    print_prompt(step);

    for (;;) {
        UINT pattern = 0U;
        uint32_t now;
        bool start_edge;
        bool stop_edge;
        bool imu_tick;
        int16_t pwm_l = 0;
        int16_t pwm_r = 0;
        imu_vec3_t g = { 0, 0, 0 };
        bool have_sample = false;

        (void)tk_wai_flg(g_test_flag, HAL_TICK_MOTION | HAL_TICK_IMU, TWF_ORW | TWF_BITCLR,
                         &pattern, TMO_FEVR);
        imu_tick = (pattern & HAL_TICK_IMU) != 0U;
        if ((pattern & HAL_TICK_MOTION) != 0U) {
            hal_tick_ack(HAL_TICK_MOTION);
        }
        if (imu_tick) {
            hal_tick_ack(HAL_TICK_IMU);
        }
        now = hal_time_ms();
        hal_encoders_snapshot(&left, &right);
        motion_step(&motion, &left, &right, now, hal_time_us(), &mo);
        start_edge = button_pressed(&start_btn, hal_button_start());
        stop_edge = button_pressed(&stop_btn, hal_button_stop());

        /* Read the accelerometer every 20 ms. */
        if (imu_tick) {
            if (!present) {
                if (robot_imu_probe(&variant, &addr)) {
                    present = true;
                    tm_printf((UB *)"[imu] found %s at 0x%x\n",
                              (variant == IMU_VARIANT_LSM303D) ? (UB *)"LSM303D" : (UB *)"LSM303DLHC",
                              (INT)addr);
                } else if ((now - last_print_ms) >= 1000U) {
                    last_print_ms = now;
                    tm_printf((UB *)"[imu] NO IMU: nothing answers on I2C1 (GP26 SDA / GP27 SCL) at 0x1D or 0x19\n");
                }
            }
            if (present) {
                uint8_t raw[6];

                if (hal_i2c_read(addr, (uint8_t)(0x28U | 0x80U), raw, 6U)) {
                    imu_decode_accel(variant, raw, &g);
                    have_sample = true;
                    fail_count = 0U;
                } else {
                    fail_count++;
                    if (fail_count >= 3U) {
                        present = false;
                        tm_printf((UB *)"[imu] read failed 3 times: probing again\n");
                    }
                }
            }
        }

        switch (phase) {
        case PHASE_MENU:
            if (start_edge && present) {
                if (step == STEP_LIVE) {
                    /* HUMP run: calibrate at rest with the sign found above. */
                    imu_cal_init(&cal, fwd_axis, fwd_sign, ROBOT_IMU_CAL_SAMPLES,
                                 TEST_CAL_SPREAD_MG);
                    /* One wheel on the hump tilts the car sideways (roll) more than
                     * it pitches it, so the sideways tilt is measured as well. */
                    imu_cal_init(&cal_roll, lateral_axis(fwd_axis, &rest_mean), 1,
                                 ROBOT_IMU_CAL_SAMPLES, TEST_CAL_SPREAD_MG);
                    hump_init(&hump, &ROBOT_HUMP_CFG);
                    phase = PHASE_SETTLE;
                    phase_start_ms = now;
                    tm_printf((UB *)"[imu] RUN hump drive: calibrating at rest (axis %c sign %d). Keep the car still.\n",
                              axis_name(fwd_axis), (INT)fwd_sign);
                } else {
                    phase = PHASE_AVERAGE;
                    avg_n = 0U;
                    sum[0] = 0;
                    sum[1] = 0;
                    sum[2] = 0;
                }
            } else if (start_edge) {
                tm_printf((UB *)"[imu] cannot start: the IMU is not responding\n");
            } else if (have_sample && (step == STEP_LIVE) && ((now - last_print_ms) >= TEST_PRINT_MS)) {
                imu_cal_t live;
                int16_t pitch = 0;

                last_print_ms = now;
                if (rest_done) {
                    /* Live pitch against the REST baseline, with the sign found in NOSE. */
                    imu_cal_init(&live, fwd_axis, fwd_sign, 1U, 32000);
                    (void)imu_cal_feed(&live, &rest_mean);
                    pitch = imu_pitch_permille(&live, &g);
                }
                tm_printf((UB *)"[imu] LIVE x=%d y=%d z=%d mg  pitch=%d permille\n",
                          (INT)g.x, (INT)g.y, (INT)g.z, (INT)pitch);
            } else {
                /* idle */
            }
            break;

        case PHASE_AVERAGE:
            if (have_sample) {
                sum[0] += g.x;
                sum[1] += g.y;
                sum[2] += g.z;
                avg_n++;
            }
            if (avg_n >= TEST_AVG_SAMPLES) {
                imu_vec3_t mean;

                mean.x = (int16_t)(sum[0] / (int32_t)avg_n);
                mean.y = (int16_t)(sum[1] / (int32_t)avg_n);
                mean.z = (int16_t)(sum[2] / (int32_t)avg_n);
                if (step == STEP_REST) {
                    rest_mean = mean;
                    tm_printf((UB *)"[imu] RESULT rest x=%d y=%d z=%d mg\n",
                              (INT)mean.x, (INT)mean.y, (INT)mean.z);
                    {
                        const int32_t mag = isqrt_i32(((int32_t)mean.x * mean.x) + ((int32_t)mean.y * mean.y) +
                                                      ((int32_t)mean.z * mean.z));

                        tm_printf((UB *)"[imu] RESULT |g| = %d mg\n", (INT)mag);
                        if ((abs32(mean.z) < TEST_REST_MIN_Z_MG) ||
                            (abs32(mean.x) > TEST_REST_MAX_XY_MG) || (abs32(mean.y) > TEST_REST_MAX_XY_MG)) {
                            tm_printf((UB *)"[imu] WRONG: gravity is not mostly on Z, so the board or the car is tilted. Level it and try again\n");
                        } else {
                            rest_done = true;
                            if (mean.z < 0) {
                                tm_printf((UB *)"[imu] OK: Z is negative, so the board is mounted upside down. That is fine: the pitch maths allows for it\n");
                            } else {
                                tm_printf((UB *)"[imu] OK: Z reads +1 g, the board is flat and the right way up\n");
                            }
                            if ((mag < TEST_MAG_LOW_MG) || (mag > TEST_MAG_HIGH_MG)) {
                                tm_printf((UB *)"[imu] NOTE: |g| is %d mg, not about 1000. The scale or offset is off. Pitch and hump height use ratios and the measured |g|, so they are not affected\n",
                                          (INT)mag);
                            }
                            step = STEP_NOSE;
                        }
                    }
                } else {
                    /* NOSE: the axis that changed most is the forward axis. */
                    int32_t best = 0;
                    uint8_t axis;
                    uint8_t best_axis = 0U;
                    int32_t best_delta = 0;

                    for (axis = 0U; axis < 3U; axis++) {
                        const int32_t d = (int32_t)axis_value(&mean, axis) - (int32_t)axis_value(&rest_mean, axis);

                        tm_printf((UB *)"[imu] NOSE delta %c = %d mg\n", axis_name(axis), (INT)d);
                        if (abs32(d) > best) {
                            best = abs32(d);
                            best_axis = axis;
                            best_delta = d;
                        }
                    }
                    if (best < TEST_NOSE_MIN_MG) {
                        tm_printf((UB *)"[imu] WRONG: no axis moved by 100 mg. Lift the nose higher (about 20 degrees) and try again\n");
                    } else if (best_axis == IMU_AXIS_Z) {
                        tm_printf((UB *)"[imu] WRONG: Z changed most. Lift the NOSE (front) and keep the sides level\n");
                    } else {
                        /*
                         * The pitch maths treats nose up as the reaction vector moving
                         * toward minus forward, so the sign is the opposite of the way
                         * the forward-axis reading moved.
                         */
                        fwd_axis = best_axis;
                        fwd_sign = (best_delta > 0) ? (int8_t)-1 : (int8_t)1;
                        tm_printf((UB *)"[imu] RESULT forward axis = %c, sign = %d (lift = %d mg, about %d deg)\n",
                                  axis_name(fwd_axis), (INT)fwd_sign, (INT)best,
                                  (INT)((best * 57) / 1000));
                        tm_printf((UB *)"[imu] put in robot_config.h: ROBOT_IMU_FORWARD_AXIS (IMU_AXIS_%c)  ROBOT_IMU_FORWARD_SIGN (%d)\n",
                                  axis_name(fwd_axis), (INT)fwd_sign);
                        step = STEP_LIVE;
                    }
                }
                phase = PHASE_MENU;
                print_prompt(step);
            }
            break;

        case PHASE_SETTLE:
            /* Pressing START shakes the car, so the first second of readings is thrown away. */
            if (have_sample && ((now - phase_start_ms) >= TEST_IGNORE_MS)) {
                (void)imu_cal_feed(&cal, &g);
                (void)imu_cal_feed(&cal_roll, &g);
            }
            if ((cal.state == IMU_CAL_FAILED_MOVING) || (cal_roll.state == IMU_CAL_FAILED_MOVING)) {
                tm_printf((UB *)"[imu] WRONG: the car moved during calibration. Keep it still and press START again\n");
                phase = PHASE_MENU;
            } else if ((cal.state == IMU_CAL_DONE) && (cal_roll.state == IMU_CAL_DONE) &&
                       ((now - phase_start_ms) >= TEST_SETTLE_MS)) {
                roll_peak = 0;
                pitch_peak = 0;
                cmd_id++;
                base_dist_mm = motion_status(&motion)->distance_mm;
                phase_start_ms = now;
                last_print_ms = now;
                humps_before = (uint8_t)hump.humps_seen;
#if ROBOT_IMU_TEST_HAND
                tm_printf((UB *)"[imu] PUSH the car over the hump by hand now, slowly and steadily "
                                "(about 12 cm per second). Press START when it is past.\n");
#else
                submit_move(&motion, cmd_id, now);
                tm_printf((UB *)"[imu] DRIVE 1000 mm at 120 mm/s\n");
#endif
                phase = PHASE_DRIVE;
            } else if ((now - phase_start_ms) >= 6000U) {
                tm_printf((UB *)"[imu] WRONG: calibration did not finish. Check the IMU reads\n");
                phase = PHASE_MENU;
            } else {
                /* keep calibrating */
            }
            break;

        case PHASE_DRIVE: {
            const motion_status_t *st = motion_status(&motion);
            const uint32_t elapsed = now - phase_start_ms;
#if ROBOT_IMU_TEST_HAND
            const bool done = start_edge;               /* pushed by hand: START ends the run */
#else
            const bool done = (st->state == MOTION_DONE) && (st->cmd_id == cmd_id);
#endif
            const bool timed_out = elapsed >= TEST_MOVE_TIMEOUT_MS;

#if ROBOT_IMU_TEST_HAND
            pwm_l = 0;
            pwm_r = 0;
#else
            pwm_l = mo.left_pwm;
            pwm_r = mo.right_pwm;
#endif
            if (have_sample) {
                hump_input_t hin;
                hump_event_t ev;

                hin.pitch_permille = imu_pitch_permille(&cal, &g);
                hin.disturbance_mg = imu_disturbance_mg(&cal, &g);
                hin.distance_mm = st->distance_mm;
                hin.distance_valid = true;
                hin.imu_ok = true;
                hin.now_ms = now;
                ev = hump_update(&hump, &hin);
                {
                    const int16_t roll = imu_pitch_permille(&cal_roll, &g);

                    if (abs32(roll) > abs32(roll_peak)) {
                        roll_peak = roll;
                    }
                    if (abs32(hin.pitch_permille) > abs32(pitch_peak)) {
                        pitch_peak = hin.pitch_permille;
                    }
                    if ((now - last_print_ms) >= TEST_DRIVE_PRINT_MS) {
                        last_print_ms = now;
                        tm_printf((UB *)"[imu] DRIVE d=%dmm pitch=%d roll=%d dist-mg=%d state=%d\n",
                                  (INT)(st->distance_mm - base_dist_mm), (INT)hin.pitch_permille,
                                  (INT)roll, (INT)hin.disturbance_mg, (INT)hump.state);
                    }
                }
                if (ev == HUMP_EVENT_FINISHED) {
                    hump_state_t hs;
                    uint8_t hcode;
                    hump_validity_t hv;
                    bool hfault;

                    hump_event_unpack(hump.last_event_packed, &hs, &hcode, &hv, &hfault);
                    tm_printf((UB *)"[imu] RESULT hump finished: height=%dmm validity=%d (0=valid 1=n/a 2=low confidence)\n",
                              (INT)hcode, (INT)hv);
                }
            }
            if (stop_edge || done || timed_out) {
                cmd_id++;
                submit_stop(&motion, cmd_id, now);
                pwm_l = 0;
                pwm_r = 0;
                if (stop_edge) {
                    tm_printf((UB *)"[imu] STOPPED by button\n");
                }
                /*
                 * One wheel on the hump lifts that wheel by h, which rolls the car by
                 * sin(roll) = h / track.  So the peak roll gives the height directly,
                 * with no speed or distance in it.
                 */
                tm_printf((UB *)"[imu] RESULT peak roll=%d permille, peak pitch=%d permille -> height from roll = %d mm (track %d mm)\n",
                          (INT)roll_peak, (INT)pitch_peak,
                          (INT)((abs32(roll_peak) * (int32_t)ROBOT_MOTION_CFG.track_width_mm) / 1000),
                          (INT)ROBOT_MOTION_CFG.track_width_mm);
                tm_printf((UB *)"[imu] RESULT drive over: humps seen=%d highest valid=%dmm (-1 = none)\n",
                          (INT)((uint8_t)hump.humps_seen - humps_before),
                          (INT)hump_highest_valid_mm(&hump));
                phase = PHASE_MENU;
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

static ER make_test_task(void)
{
    T_CTSK ct;
    ID id;

    ct.exinf = NULL;
    ct.tskatr = TA_HLNG | TA_RNG3;
    ct.task = imu_test_task;
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

ER imu_test_start_task(void)
{
    T_CFLG cflg;
    ER err;

    hal_init();
    cflg.flgatr = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    g_test_flag = tk_cre_flg(&cflg);
    if (g_test_flag < E_OK) {
        return g_test_flag;
    }
    err = make_test_task();
    if (err < E_OK) {
        return err;
    }
    err = hal_gpio_irq_start();
    if (err < E_OK) {
        return err;
    }
    return hal_tick_start(g_test_flag);
}
