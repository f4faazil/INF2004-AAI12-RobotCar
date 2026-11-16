/*
 * test_motor_cal.c
 * Host tests for the open-loop motor calibration state machine.  The plant is
 * a motor with stiction: it needs more PWM to start turning (breakaway) than
 * to keep turning (sustain), and above that its speed is linear in the PWM.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "motor_cal.h"
#include "test_support.h"

#define STEP_MS     (5U)

typedef struct {
    int16_t breakaway;      /* PWM needed to start from rest */
    int16_t sustain;        /* PWM below which a turning wheel stops */
    int16_t offset;         /* speed = (pwm - offset) x gain */
    double  gain;           /* counts/s per per-mille */
    bool    moving;
    double  pos;            /* counts */
    bool    dead;           /* encoder never counts */
} wheel_t;

typedef struct {
    wheel_t  l;
    wheel_t  r;
    uint32_t now_ms;
} sim_t;

static void wheel_advance(wheel_t *w, int pwm, uint32_t ms)
{
    const int mag = abs(pwm);

    if (mag >= w->breakaway) {
        w->moving = true;
    } else if (mag < w->sustain) {
        w->moving = false;
    } else {
        /* between sustain and breakaway: keeps its state */
    }
    if (w->moving && (!w->dead)) {
        w->pos += (double)(mag - w->offset) * w->gain * ((double)ms / 1000.0);
    }
}

static void sim_init(sim_t *s)
{
    s->l.breakaway = 230;  s->l.sustain = 190;  s->l.offset = 150;
    s->l.gain = 8.0;       s->l.moving = false; s->l.pos = 0.0;  s->l.dead = false;
    s->r.breakaway = 250;  s->r.sustain = 210;  s->r.offset = 170;
    s->r.gain = 7.5;       s->r.moving = false; s->r.pos = 0.0;  s->r.dead = false;
    s->now_ms = 5000U;
}

typedef struct {
    mcal_event_t ev[64];
    unsigned     n;
    bool         left_off;      /* the unused wheel was never driven */
    bool         right_off;
    bool         finished;
} run_t;

static run_t run_test(sim_t *s, mcal_test_t test, uint32_t limit_ms)
{
    run_t out;
    motor_cal_t cal;
    uint32_t t0 = s->now_ms;
    bool running = true;

    out.n = 0U;
    out.left_off = true;
    out.right_off = true;
    out.finished = false;
    motor_cal_start(&cal, test, s->now_ms, (int32_t)s->l.pos, (int32_t)s->r.pos);

    while (running && ((s->now_ms - t0) < limit_ms)) {
        int16_t lp;
        int16_t rp;
        mcal_event_t ev;

        running = motor_cal_step(&cal, s->now_ms, (int32_t)s->l.pos, (int32_t)s->r.pos,
                                 &lp, &rp, &ev);
        if (ev.kind != MCAL_EV_NONE) {
            if (out.n < (sizeof(out.ev) / sizeof(out.ev[0]))) {
                out.ev[out.n] = ev;
                out.n++;
            }
        }
        if (lp != 0) { out.left_off = false; }
        if (rp != 0) { out.right_off = false; }
        wheel_advance(&s->l, lp, STEP_MS);
        wheel_advance(&s->r, rp, STEP_MS);
        s->now_ms += STEP_MS;
    }
    out.finished = !running;
    return out;
}

static const mcal_event_t *find(const run_t *r, mcal_event_kind_t kind)
{
    unsigned i;

    for (i = 0U; i < r->n; i++) {
        if (r->ev[i].kind == kind) {
            return &r->ev[i];
        }
    }
    return NULL;
}

static void test_left_wheel_breakaway_and_sustain(void)
{
    sim_t s;
    run_t r;
    const mcal_event_t *b;
    const mcal_event_t *u;

    sim_init(&s);
    r = run_test(&s, MCAL_DEADZONE_LEFT, 120000U);

    CHECK(r.finished);
    CHECK(r.right_off);                         /* the right wheel was never driven */
    b = find(&r, MCAL_EV_BREAKAWAY);
    u = find(&r, MCAL_EV_SUSTAIN);
    CHECK(b != NULL);
    CHECK(u != NULL);
    if ((b != NULL) && (u != NULL)) {
        CHECK_INT(230, b->pwm);                 /* first pulse level at or above 230 */
        CHECK_INT(190, u->pwm);                 /* lowest level that still turned it */
    }
    CHECK(find(&r, MCAL_EV_DONE) != NULL);
    CHECK(find(&r, MCAL_EV_ABORT) == NULL);
}

static void test_right_wheel_is_tested_on_its_own(void)
{
    sim_t s;
    run_t r;
    const mcal_event_t *b;
    const mcal_event_t *u;

    sim_init(&s);
    r = run_test(&s, MCAL_DEADZONE_RIGHT, 120000U);

    CHECK(r.finished);
    CHECK(r.left_off);
    b = find(&r, MCAL_EV_BREAKAWAY);
    u = find(&r, MCAL_EV_SUSTAIN);
    CHECK(b != NULL);
    CHECK(u != NULL);
    if ((b != NULL) && (u != NULL)) {
        CHECK_INT(250, b->pwm);
        CHECK_INT(210, u->pwm);
    }
}

/* The search steps up by 10, so a breakaway at 233 is reported as 240. */
static void test_breakaway_is_the_first_pulse_that_moves(void)
{
    sim_t s;
    run_t r;
    const mcal_event_t *b;

    sim_init(&s);
    s.l.breakaway = 233;
    r = run_test(&s, MCAL_DEADZONE_LEFT, 120000U);
    b = find(&r, MCAL_EV_BREAKAWAY);
    CHECK(b != NULL);
    if (b != NULL) {
        CHECK_INT(240, b->pwm);
    }
}

static void test_a_wheel_that_never_moves_aborts(void)
{
    sim_t s;
    run_t r;

    sim_init(&s);
    s.l.dead = true;                            /* encoder (or motor) is dead */
    r = run_test(&s, MCAL_DEADZONE_LEFT, 120000U);

    CHECK(r.finished);
    CHECK(find(&r, MCAL_EV_ABORT) != NULL);
    CHECK(find(&r, MCAL_EV_BREAKAWAY) == NULL);
    CHECK(find(&r, MCAL_EV_DONE) == NULL);
}

static void test_motor_that_never_stops_aborts_on_travel(void)
{
    sim_t s;
    run_t r;

    sim_init(&s);
    s.l.sustain = 0;                            /* never stalls, however low the PWM */
    s.l.gain = 40.0;                            /* fast: reaches the travel limit */
    r = run_test(&s, MCAL_DEADZONE_LEFT, 240000U);

    CHECK(r.finished);
    CHECK(find(&r, MCAL_EV_ABORT) != NULL);
    CHECK(find(&r, MCAL_EV_SUSTAIN) == NULL);
}

static void test_speed_table_reports_each_level_for_both_wheels(void)
{
    sim_t s;
    run_t r;
    unsigned i;
    unsigned rows = 0U;

    sim_init(&s);
    r = run_test(&s, MCAL_SPEED_TABLE, 60000U);

    CHECK(r.finished);
    for (i = 0U; i < r.n; i++) {
        if (r.ev[i].kind == MCAL_EV_SPEED) {
            const int pwm = r.ev[i].pwm;
            /* from rest a wheel only starts above its breakaway PWM (250 for the
             * right wheel here), so the two lowest rows read zero */
            const bool turns = pwm >= 250;
            const double exp_l = turns ? (double)(pwm - s.l.offset) * s.l.gain : 0.0;
            const double exp_r = turns ? (double)(pwm - s.r.offset) * s.r.gain : 0.0;

            CHECK_INT(motor_cal_speed_level((uint8_t)rows), pwm);
            CHECK(fabs((double)r.ev[i].left_cps - exp_l) <= 20.0);
            CHECK(fabs((double)r.ev[i].right_cps - exp_r) <= 20.0);
            rows++;
        }
    }
    CHECK_INT((int)MCAL_SPEED_LEVELS, (int)rows);
    CHECK(find(&r, MCAL_EV_DONE) != NULL);
}

/* The test never leaves a wheel driven after it ends. */
static void test_pwm_is_zero_when_finished(void)
{
    sim_t s;
    motor_cal_t cal;
    int16_t lp = 1;
    int16_t rp = 1;
    mcal_event_t ev;
    bool running = true;
    uint32_t guard = 0U;

    sim_init(&s);
    motor_cal_start(&cal, MCAL_SPEED_TABLE, s.now_ms, 0, 0);
    while (running && (guard < 20000U)) {
        running = motor_cal_step(&cal, s.now_ms, (int32_t)s.l.pos, (int32_t)s.r.pos,
                                 &lp, &rp, &ev);
        wheel_advance(&s.l, lp, STEP_MS);
        wheel_advance(&s.r, rp, STEP_MS);
        s.now_ms += STEP_MS;
        guard++;
    }
    CHECK(!running);
    CHECK_INT(0, lp);
    CHECK_INT(0, rp);
    CHECK(!motor_cal_step(&cal, s.now_ms, 0, 0, &lp, &rp, &ev));    /* stays finished */
}

int main(void)
{
    RUN(test_left_wheel_breakaway_and_sustain);
    RUN(test_right_wheel_is_tested_on_its_own);
    RUN(test_breakaway_is_the_first_pulse_that_moves);
    RUN(test_a_wheel_that_never_moves_aborts);
    RUN(test_motor_that_never_stops_aborts_on_travel);
    RUN(test_speed_table_reports_each_level_for_both_wheels);
    RUN(test_pwm_is_zero_when_finished);
    return TEST_REPORT();
}
