/*
 * test_motion.c
 * Host tests for the motion controller.  The controller is exercised against
 * a simple simulated drivetrain (first-order wheel response, quantised
 * encoder counts) so the closed loop, command lease, STOP precedence and
 * stale-command rules are all checked together.
 *
 * SIMULATION ONLY: the plant constants (0.4 mm/s per PWM per-mille, 50 ms
 * lag, 65 mm wheels, 1000 counts/rev) are stand-ins.  They show the logic is
 * sound; the real gains still have to be tuned on the car.
 */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "motion.h"
#include "test_support.h"
#include "test_port.h"

#define PI_D            (3.14159265358979)
#define STEP_MS         (5U)
#define WHEEL_MM        (65U)
#define TRACK_MM        (150U)
#define CPR             (1000U)

static motion_config_t make_config(void)
{
    motion_config_t c;

    c.counts_per_rev = CPR;
    c.wheel_diameter_mm = WHEEL_MM;
    c.track_width_mm = TRACK_MM;
    c.kp = 256;
    c.ki = 512;
    c.kd = 0;
    c.ff_num = 5;
    c.ff_den = 2;                   /* pwm = speed / 0.4 */
    c.balance_mm_s_per_mm = 2;
    c.default_lease_ms = 50U;
    c.max_speed_mm_s = 300U;
    c.creep_speed_mm_s = 40U;
    c.turn_speed_mm_s = 120U;
    c.decel_per_s = 6U;
    c.speed_window_ms = 20U;
    c.stopped_speed_mm_s = 8U;
    c.stopped_confirm_ms = 40U;
    c.encoder_fault_ms = 300U;
    c.encoder_fault_pwm = 300U;
    c.ff_right_num = 0;
    c.ff_right_den = 0;             /* right wheel shares the left feed-forward */
    c.pwm_offset_left = 0;
    c.pwm_offset_right = 0;
    c.pwm_limit = 0;                /* full range */
    return c;
}

/* ---- simulated drivetrain ------------------------------------------- */

typedef struct {
    double   v_l, v_r;              /* mm/s */
    double   pos_l, pos_r;          /* mm, signed */
    double   gain_l, gain_r;        /* mm/s per pwm per-mille */
    double   tau_ms;
    int32_t  cnt_l, cnt_r;
    uint32_t edge_l_us, edge_r_us;
    bool     freeze_encoders;
} plant_t;

typedef struct {
    motion_t m;
    plant_t  p;
    uint32_t now_ms;
    uint32_t now_us;
    motion_output_t out;
} rig_t;

static void rig_init(rig_t *r)
{
    motion_config_t cfg = make_config();

    motion_init(&r->m, &cfg);
    r->p.v_l = 0.0; r->p.v_r = 0.0;
    r->p.pos_l = 0.0; r->p.pos_r = 0.0;
    r->p.gain_l = 0.4; r->p.gain_r = 0.4;
    r->p.tau_ms = 50.0;
    r->p.cnt_l = 0; r->p.cnt_r = 0;
    r->p.edge_l_us = 0U; r->p.edge_r_us = 0U;
    r->p.freeze_encoders = false;
    r->now_ms = 1000U;
    r->now_us = 1000000U;
    r->out.left_pwm = 0;
    r->out.right_pwm = 0;
}

static void plant_advance_1ms(plant_t *p, int pwm_l, int pwm_r, uint32_t now_us)
{
    double a = 1.0 / p->tau_ms;
    double counts_per_mm = (double)CPR / (PI_D * (double)WHEEL_MM);
    int32_t nl;
    int32_t nr;

    p->v_l += (p->gain_l * (double)pwm_l - p->v_l) * a;
    p->v_r += (p->gain_r * (double)pwm_r - p->v_r) * a;
    p->pos_l += p->v_l * 0.001;
    p->pos_r += p->v_r * 0.001;
    if (!p->freeze_encoders) {
        nl = (int32_t)floor(p->pos_l * counts_per_mm);
        nr = (int32_t)floor(p->pos_r * counts_per_mm);
        if (nl != p->cnt_l) { p->cnt_l = nl; p->edge_l_us = now_us; }
        if (nr != p->cnt_r) { p->cnt_r = nr; p->edge_r_us = now_us; }
    }
}

/* One 5 ms control step: snapshot, motion_step, apply PWM for 5 ms. */
static void rig_step(rig_t *r)
{
    encoder_snapshot_t sl;
    encoder_snapshot_t sr;
    unsigned i;

    sl.count = r->p.cnt_l; sl.last_edge_us = r->p.edge_l_us; sl.illegal = 0U;
    sr.count = r->p.cnt_r; sr.last_edge_us = r->p.edge_r_us; sr.illegal = 0U;
    motion_step(&r->m, &sl, &sr, r->now_ms, r->now_us, &r->out);
    for (i = 0U; i < STEP_MS; i++) {
        r->now_us += 1000U;
        plant_advance_1ms(&r->p, r->out.left_pwm, r->out.right_pwm, r->now_us);
    }
    r->now_ms += STEP_MS;
}

/* Run for duration_ms, refreshing 'cmd' every 10 ms when non-NULL. */
static void rig_run(rig_t *r, uint32_t duration_ms, const motion_cmd_t *cmd)
{
    uint32_t end = r->now_ms + duration_ms;

    while (r->now_ms < end) {
        if ((cmd != NULL) && ((r->now_ms % 10U) == 0U)) {
            (void)motion_submit(&r->m, cmd, r->now_ms);
        }
        rig_step(r);
    }
}

static motion_cmd_t drive(uint16_t id, int16_t speed, int16_t diff)
{
    motion_cmd_t c = { id, MOTION_CMD_DRIVE, speed, diff, 0, 0, 0U };
    return c;
}

static motion_cmd_t move(uint16_t id, int16_t mm)
{
    motion_cmd_t c = { id, MOTION_CMD_MOVE, 150, 0, mm, 0, 0U };
    return c;
}

static motion_cmd_t turn(uint16_t id, int16_t deg)
{
    motion_cmd_t c = { id, MOTION_CMD_TURN, 0, 0, 0, deg, 0U };
    return c;
}

static motion_cmd_t stop_cmd(uint16_t id)
{
    motion_cmd_t c = { id, MOTION_CMD_STOP, 0, 0, 0, 0, 0U };
    return c;
}

static double yaw_deg(const rig_t *r)
{
    return ((r->p.pos_r - r->p.pos_l) / (double)TRACK_MM) * (180.0 / PI_D);
}

/* ---- tests ---------------------------------------------------------- */

/* A DRIVE command settles on its commanded speed.  Catches: a broken speed
 * loop (wrong sign, no feed-forward, estimate off by the unit conversion). */
static void test_drive_reaches_commanded_speed(void)
{
    rig_t r;
    motion_cmd_t c = drive(1U, 150, 0);
    const motion_status_t *s;

    rig_init(&r);
    rig_run(&r, 1500U, &c);
    s = motion_status(&r.m);
    CHECK(fabs(r.p.v_l - 150.0) < 8.0);
    CHECK(fabs(r.p.v_r - 150.0) < 8.0);
    CHECK(abs(s->left_mm_s - 150) < 12);       /* the estimate agrees */
    CHECK_INT(MOTION_RUNNING, s->state);
}

/* A finite MOVE stops within 5% of the requested distance and reports DONE.
 * Tribunal target: 1 m within +/-50 mm.  It must also arrive slowly: the wheel
 * speed when the move completes is near the creep speed, not the cruise
 * speed.  Catches: no deceleration (the car coasts onto the target at full
 * speed), wrong distance conversion, or never finishing. */
static void test_move_stops_near_the_target_distance(void)
{
    rig_t r;
    motion_cmd_t c = move(2U, 500);
    const motion_status_t *s;
    double speed_at_done = -1.0;

    rig_init(&r);
    while ((r.now_ms < 5000U) && (speed_at_done < 0.0)) {
        if ((r.now_ms % 10U) == 0U) {
            (void)motion_submit(&r.m, &c, r.now_ms);
        }
        rig_step(&r);
        if (motion_status(&r.m)->state == MOTION_DONE) {
            speed_at_done = r.p.v_l;
        }
    }
    rig_run(&r, 1000U, &c);
    s = motion_status(&r.m);
    CHECK_INT(MOTION_DONE, s->state);
    CHECK((s->flags & MOTION_FLAG_DONE) != 0U);
    CHECK(speed_at_done >= 0.0);
    CHECK(speed_at_done < 80.0);              /* cruise is 150 mm/s */
    CHECK(fabs(r.p.pos_l - 500.0) < 25.0);
    CHECK(fabs(r.p.pos_r - 500.0) < 25.0);
    CHECK(abs(s->distance_mm - 500) < 25);
}

/* A reverse MOVE (negative distance) drives backwards and reports negative
 * distance.  Catches: sign loss, which the bypass/recovery path needs. */
static void test_reverse_move(void)
{
    rig_t r;
    motion_cmd_t c = move(3U, -200);
    const motion_status_t *s;

    rig_init(&r);
    rig_run(&r, 3000U, &c);
    s = motion_status(&r.m);
    CHECK_INT(MOTION_DONE, s->state);
    CHECK(fabs(r.p.pos_l + 200.0) < 15.0);
    CHECK(s->distance_mm < -180);
}

/* An in-place left turn of 90 degrees ends within +/-10 degrees (Tribunal
 * target), with the wheels turning opposite ways.  Catches: wrong track-width
 * arithmetic, wrong turn direction. */
static void test_left_turn_90_degrees(void)
{
    rig_t r;
    motion_cmd_t c = turn(4U, 90);
    const motion_status_t *s;

    rig_init(&r);
    rig_run(&r, 4000U, &c);
    s = motion_status(&r.m);
    CHECK_INT(MOTION_DONE, s->state);
    CHECK(fabs(yaw_deg(&r) - 90.0) < 10.0);
    CHECK(r.p.pos_r > 0.0);
    CHECK(r.p.pos_l < 0.0);
}

/* A right turn is the mirror image. */
static void test_right_turn_90_degrees(void)
{
    rig_t r;
    motion_cmd_t c = turn(5U, -90);

    rig_init(&r);
    rig_run(&r, 4000U, &c);
    CHECK(fabs(yaw_deg(&r) + 90.0) < 10.0);
}

/* A U-turn (180) also lands within tolerance. */
static void test_u_turn(void)
{
    rig_t r;
    motion_cmd_t c = turn(6U, 180);

    rig_init(&r);
    rig_run(&r, 6000U, &c);
    CHECK(fabs(yaw_deg(&r) - 180.0) < 12.0);
}

/* Refreshing the same finite command every 10 ms must not restart it: the
 * car travels the requested distance once, not forever.  Catches: a refresh
 * re-capturing the start point (Tribunal: repeated refreshes must not restart
 * the manoeuvre). */
static void test_refreshing_a_move_does_not_restart_it(void)
{
    rig_t r;
    motion_cmd_t c = move(7U, 200);

    rig_init(&r);
    rig_run(&r, 6000U, &c);       /* far longer than one 200 mm move needs */
    CHECK(r.p.pos_l < 225.0);
    CHECK(r.p.pos_l > 175.0);
    CHECK_INT(MOTION_DONE, motion_status(&r.m)->state);
}

/* When refreshes stop, the lease lapses: PWM goes to zero and the status
 * says EXPIRED.  A late refresh of the same id does not restart movement.
 * Catches: a car that keeps driving on a stale command. */
static void test_lapsed_lease_stops_and_cannot_be_resumed(void)
{
    rig_t r;
    motion_cmd_t c = drive(8U, 150, 0);
    const motion_status_t *s;

    rig_init(&r);
    rig_run(&r, 500U, &c);
    rig_run(&r, 100U, NULL);                    /* no refresh: > 50 ms lease */
    s = motion_status(&r.m);
    CHECK_INT(MOTION_EXPIRED, s->state);
    CHECK((s->flags & MOTION_FLAG_EXPIRED) != 0U);
    CHECK_INT(0, r.out.left_pwm);
    CHECK_INT(0, r.out.right_pwm);

    CHECK(!motion_submit(&r.m, &c, r.now_ms));  /* same id: refused */
    rig_run(&r, 100U, NULL);
    CHECK_INT(0, r.out.left_pwm);
}

/* The lease is honoured but not early: a command refreshed within the lease
 * keeps running.  Catches: expiry measured from the wrong reference. */
static void test_command_refreshed_inside_the_lease_keeps_running(void)
{
    rig_t r;
    motion_cmd_t c = drive(9U, 120, 0);

    rig_init(&r);
    rig_run(&r, 1000U, &c);                     /* refresh every 10 ms */
    CHECK_INT(MOTION_RUNNING, motion_status(&r.m)->state);
    CHECK(r.out.left_pwm > 0);
}

/* STOP zeroes the PWM on the next step and a refresh of the cancelled
 * command cannot restart the car.  Catches: old queued commands restarting
 * motion after a stop (Tribunal rule). */
static void test_stop_cancels_and_old_command_cannot_restart(void)
{
    rig_t r;
    motion_cmd_t c = drive(10U, 150, 0);
    motion_cmd_t s = stop_cmd(11U);

    rig_init(&r);
    rig_run(&r, 600U, &c);
    CHECK(r.out.left_pwm > 0);

    CHECK(motion_submit(&r.m, &s, r.now_ms));
    rig_step(&r);
    CHECK_INT(0, r.out.left_pwm);
    CHECK_INT(0, r.out.right_pwm);

    CHECK(!motion_submit(&r.m, &c, r.now_ms));  /* old id: refused */
    rig_run(&r, 200U, &c);
    CHECK_INT(0, r.out.left_pwm);
}

/* After a STOP the status names the STOP's id, and STOPPED is reported only
 * for it.  The Vehicle Controller relies on this to know that the stop it
 * asked for (not an older one) has taken effect.  Catches: a stale STOPPED
 * flag from before the STOP being mistaken for confirmation. */
static void test_stop_status_names_the_stop_command(void)
{
    rig_t r;
    motion_cmd_t c = drive(15U, 150, 0);
    motion_cmd_t s = stop_cmd(16U);

    rig_init(&r);
    rig_run(&r, 600U, &c);
    CHECK_INT(15, motion_status(&r.m)->cmd_id);
    (void)motion_submit(&r.m, &s, r.now_ms);
    rig_step(&r);
    CHECK_INT(16, motion_status(&r.m)->cmd_id);
    CHECK_INT(MOTION_IDLE, motion_status(&r.m)->state);
}

/* A new command after a stop is accepted. */
static void test_new_command_after_stop_is_accepted(void)
{
    rig_t r;
    motion_cmd_t c = drive(12U, 150, 0);
    motion_cmd_t s = stop_cmd(13U);
    motion_cmd_t d = drive(14U, 100, 0);

    rig_init(&r);
    rig_run(&r, 300U, &c);
    (void)motion_submit(&r.m, &s, r.now_ms);
    rig_run(&r, 200U, NULL);
    CHECK(motion_submit(&r.m, &d, r.now_ms));
    rig_run(&r, 800U, &d);
    CHECK(fabs(r.p.v_l - 100.0) < 8.0);
}

/* A command older than the newest accepted id is rejected, including across
 * the 16-bit wrap.  Catches: reordered/stale commands taking effect. */
static void test_stale_and_wrapped_ids(void)
{
    rig_t r;
    motion_cmd_t a = drive(100U, 100, 0);
    motion_cmd_t old = drive(99U, 200, 0);
    motion_cmd_t hi = drive(65535U, 100, 0);
    motion_cmd_t wrapped = drive(0U, 100, 0);
    motion_cmd_t before_wrap = drive(65534U, 100, 0);

    rig_init(&r);
    CHECK(motion_submit(&r.m, &a, r.now_ms));
    CHECK(!motion_submit(&r.m, &old, r.now_ms));
    CHECK_INT(1, r.m.rejected_stale);

    rig_init(&r);
    CHECK(motion_submit(&r.m, &hi, r.now_ms));
    CHECK(motion_submit(&r.m, &wrapped, r.now_ms));    /* 0 is newer than 65535 */
    CHECK(!motion_submit(&r.m, &before_wrap, r.now_ms)); /* and 65534 is older */
}

/* While the safety override is set, the car does not move and new commands
 * are rejected; releasing it does not resume the cancelled command. */
static void test_override_blocks_and_does_not_resume(void)
{
    rig_t r;
    motion_cmd_t c = drive(20U, 150, 0);
    motion_cmd_t d = drive(21U, 150, 0);
    const motion_status_t *s;

    rig_init(&r);
    rig_run(&r, 500U, &c);
    motion_set_override(&r.m, true);
    rig_step(&r);
    s = motion_status(&r.m);
    CHECK_INT(0, r.out.left_pwm);
    CHECK_INT(MOTION_OVERRIDDEN, s->state);
    CHECK((s->flags & MOTION_FLAG_OVERRIDE) != 0U);
    CHECK(!motion_submit(&r.m, &d, r.now_ms));

    motion_set_override(&r.m, false);
    rig_run(&r, 200U, &c);                      /* the old command */
    CHECK_INT(0, r.out.left_pwm);
    CHECK(motion_submit(&r.m, &d, r.now_ms));   /* but a new one works */
}

/* STOPPED is only reported after the wheels have actually stopped for the
 * confirmation time, not when PWM first reaches zero.  Catches: the obstacle
 * scan starting while the car is still coasting (Tribunal: PWM zero is not
 * the same as stopped). */
static void test_stopped_flag_waits_for_the_wheels_to_stop(void)
{
    rig_t r;
    motion_cmd_t c = drive(30U, 250, 0);
    motion_cmd_t s = stop_cmd(31U);
    int stopped_early = 0;
    int stopped_late = 0;
    uint32_t t;

    rig_init(&r);
    rig_run(&r, 1500U, &c);
    CHECK((motion_status(&r.m)->flags & MOTION_FLAG_STOPPED) == 0U);

    (void)motion_submit(&r.m, &s, r.now_ms);
    for (t = 0U; t < 1000U; t += STEP_MS) {
        rig_step(&r);
        if ((motion_status(&r.m)->flags & MOTION_FLAG_STOPPED) != 0U) {
            if (fabs(r.p.v_l) > 20.0) {
                stopped_early = 1;
            }
            stopped_late = 1;
        }
    }
    CHECK_INT(0, stopped_early);
    CHECK_INT(1, stopped_late);
}

/* STOPPED needs BOTH wheels still: with one wheel driving (a pivot) the car
 * is not stopped.  Catches: a check on one wheel only, which would let the
 * obstacle scan start while the car is still rotating. */
static void test_stopped_requires_both_wheels_to_be_still(void)
{
    rig_t r;
    motion_cmd_t left_moving = drive(32U, 60, -120);    /* left 120, right 0 */
    motion_cmd_t right_moving = drive(33U, 60, 120);    /* left 0, right 120 */

    rig_init(&r);
    rig_run(&r, 1500U, &left_moving);
    CHECK(r.p.v_l > 80.0);
    CHECK(fabs(r.p.v_r) < 10.0);
    CHECK((motion_status(&r.m)->flags & MOTION_FLAG_STOPPED) == 0U);

    rig_init(&r);
    rig_run(&r, 1500U, &right_moving);
    CHECK(r.p.v_r > 80.0);
    CHECK(fabs(r.p.v_l) < 10.0);
    CHECK((motion_status(&r.m)->flags & MOTION_FLAG_STOPPED) == 0U);
}

/* PWM applied but the encoder never counts: flagged as an encoder fault.
 * Catches: a dead encoder silently running the car open-loop. */
static void test_dead_encoder_is_flagged(void)
{
    rig_t r;
    motion_cmd_t c = drive(40U, 150, 0);

    rig_init(&r);
    r.p.freeze_encoders = true;
    rig_run(&r, 800U, &c);
    CHECK((motion_status(&r.m)->flags & MOTION_FLAG_ENCODER_FAULT) != 0U);
}

/* Healthy encoders never raise the fault. */
static void test_working_encoders_do_not_fault(void)
{
    rig_t r;
    motion_cmd_t c = drive(41U, 150, 0);

    rig_init(&r);
    rig_run(&r, 2000U, &c);
    CHECK((motion_status(&r.m)->flags & MOTION_FLAG_ENCODER_FAULT) == 0U);
}

/* One motor 15% weaker still drives straight, because each wheel is speed
 * controlled and the straight-line correction removes residual drift.
 * Catches: open-loop veering. */
static void test_weak_motor_still_drives_straight(void)
{
    rig_t r;
    motion_cmd_t c = move(50U, 600);

    rig_init(&r);
    r.p.gain_r = 0.34;
    rig_run(&r, 6000U, &c);
    CHECK(fabs(yaw_deg(&r)) < 3.0);
}

/* A differential DRIVE veers: positive diff makes the right wheel faster. */
static void test_drive_differential_veers_left_for_positive_diff(void)
{
    rig_t r;
    motion_cmd_t c = drive(60U, 120, 40);

    rig_init(&r);
    rig_run(&r, 1500U, &c);
    CHECK(r.p.v_r > r.p.v_l + 25.0);
}

/* A zero-speed DRIVE holds the motors still without dithering. */
static void test_zero_speed_drive_outputs_zero(void)
{
    rig_t r;
    motion_cmd_t c = drive(70U, 0, 0);

    rig_init(&r);
    rig_run(&r, 500U, &c);
    CHECK_INT(0, r.out.left_pwm);
    CHECK_INT(0, r.out.right_pwm);
}

/* PWM never exceeds the per-mille limit even for an absurd request. */
static void test_pwm_is_limited(void)
{
    rig_t r;
    motion_cmd_t c = drive(80U, 32000, 0);
    int i;

    rig_init(&r);
    for (i = 0; i < 200; i++) {
        (void)motion_submit(&r.m, &c, r.now_ms);
        rig_step(&r);
        CHECK(r.out.left_pwm <= MOTION_PWM_MAX);
        CHECK(r.out.left_pwm >= -MOTION_PWM_MAX);
    }
    CHECK(r.p.v_l <= 305.0);                    /* held to max_speed_mm_s */
}

/* Counts <-> micrometres round-trips, and one wheel revolution is pi * D. */
static void test_unit_conversions(void)
{
    motion_config_t cfg = make_config();

    CHECK(abs(motion_counts_to_um(&cfg, (int32_t)CPR) - 204204) < 5);
    CHECK(abs(motion_um_to_counts(&cfg, 204204) - (int32_t)CPR) <= 1);
    CHECK_INT(-motion_counts_to_um(&cfg, 777), motion_counts_to_um(&cfg, -777));
}

/*
 * Open-loop drive model: with the PID switched off the PWM is exactly
 * offset + speed x ff, per wheel, in the direction of the target.
 */
static void rig_init_open_loop(rig_t *r, motion_config_t *cfg)
{
    rig_init(r);
    cfg->kp = 0;
    cfg->ki = 0;
    cfg->kd = 0;
    cfg->balance_mm_s_per_mm = 0;
    motion_init(&r->m, cfg);
}

static void test_offset_is_added_in_the_direction_of_the_target(void)
{
    rig_t r;
    motion_config_t cfg = make_config();
    motion_cmd_t fwd = drive(90U, 80, 0);
    motion_cmd_t rev = drive(91U, -80, 0);

    cfg.pwm_offset_left = 100;
    cfg.pwm_offset_right = 60;
    rig_init_open_loop(&r, &cfg);
    rig_run(&r, 300U, &fwd);
    CHECK_INT(300, r.out.left_pwm);             /* 80 x 5/2 + 100 */
    CHECK_INT(260, r.out.right_pwm);            /* 80 x 5/2 + 60 */

    rig_init_open_loop(&r, &cfg);
    rig_run(&r, 300U, &rev);
    CHECK_INT(-300, r.out.left_pwm);
    CHECK_INT(-260, r.out.right_pwm);
}

static void test_right_wheel_has_its_own_feed_forward(void)
{
    rig_t r;
    motion_config_t cfg = make_config();
    motion_cmd_t fwd = drive(92U, 80, 0);

    cfg.ff_right_num = 3;
    cfg.ff_right_den = 1;
    cfg.pwm_offset_right = 10;
    rig_init_open_loop(&r, &cfg);
    rig_run(&r, 300U, &fwd);
    CHECK_INT(200, r.out.left_pwm);             /* left keeps 5/2 */
    CHECK_INT(250, r.out.right_pwm);            /* 80 x 3 + 10 */
}

static void test_zero_right_den_means_shared_feed_forward(void)
{
    rig_t r;
    motion_config_t cfg = make_config();
    motion_cmd_t fwd = drive(93U, 80, 0);

    cfg.ff_right_num = 7;                       /* ignored: den is 0 */
    cfg.ff_right_den = 0;
    rig_init_open_loop(&r, &cfg);
    rig_run(&r, 300U, &fwd);
    CHECK_INT(200, r.out.left_pwm);
    CHECK_INT(200, r.out.right_pwm);
}

static void test_configured_pwm_limit_caps_both_wheels(void)
{
    rig_t r;
    motion_config_t cfg = make_config();
    motion_cmd_t c = drive(94U, 250, 0);
    int i;

    cfg.pwm_limit = 400;
    cfg.pwm_offset_left = 150;
    rig_init(&r);
    motion_init(&r.m, &cfg);
    for (i = 0; i < 200; i++) {
        (void)motion_submit(&r.m, &c, r.now_ms);
        rig_step(&r);
        CHECK(r.out.left_pwm <= 400);
        CHECK(r.out.right_pwm <= 400);
        CHECK(r.out.left_pwm >= -400);
    }
}

int main(void)
{
    RUN(test_drive_reaches_commanded_speed);
    RUN(test_move_stops_near_the_target_distance);
    RUN(test_reverse_move);
    RUN(test_left_turn_90_degrees);
    RUN(test_right_turn_90_degrees);
    RUN(test_u_turn);
    RUN(test_refreshing_a_move_does_not_restart_it);
    RUN(test_lapsed_lease_stops_and_cannot_be_resumed);
    RUN(test_command_refreshed_inside_the_lease_keeps_running);
    RUN(test_stop_cancels_and_old_command_cannot_restart);
    RUN(test_stop_status_names_the_stop_command);
    RUN(test_new_command_after_stop_is_accepted);
    RUN(test_stale_and_wrapped_ids);
    RUN(test_override_blocks_and_does_not_resume);
    RUN(test_stopped_flag_waits_for_the_wheels_to_stop);
    RUN(test_stopped_requires_both_wheels_to_be_still);
    RUN(test_dead_encoder_is_flagged);
    RUN(test_working_encoders_do_not_fault);
    RUN(test_weak_motor_still_drives_straight);
    RUN(test_drive_differential_veers_left_for_positive_diff);
    RUN(test_zero_speed_drive_outputs_zero);
    RUN(test_pwm_is_limited);
    RUN(test_unit_conversions);
    RUN(test_offset_is_added_in_the_direction_of_the_target);
    RUN(test_right_wheel_has_its_own_feed_forward);
    RUN(test_zero_right_den_means_shared_feed_forward);
    RUN(test_configured_pwm_limit_caps_both_wheels);
    return TEST_REPORT();
}
