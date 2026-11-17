/*
 * test_obstacle.c
 * Host tests for the scan sequencer, obstacle profile and bypass planner.
 * The scan is run against a ray-cast world with a servo that takes real time
 * to move, so a sequencer that measures before the servo settles produces a
 * wrong profile and fails.
 *
 * SIMULATION ONLY: ideal range returns, no beam width, no multipath.  It
 * shows the logic is sound; real scan accuracy needs the bench tests the
 * Tribunal lists (known distances/widths, +/-20 mm over 100-500 mm).
 */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "obstacle.h"
#include "test_support.h"

#define PI_D    (3.14159265358979)

/* ---- the simulated world --------------------------------------------- */

typedef struct {
    bool   has_box;
    double box_cx, box_w, box_y, box_depth;     /* x: right positive */
    bool   has_right_wall;
    double right_wall_x;
    bool   has_left_wall;
    double left_wall_x;                         /* positive number, wall at -x */
    bool   has_back_wall;
    double back_y;
} world_t;

static double hit_box(const world_t *w, double dx, double dy)
{
    double x0 = w->box_cx - (w->box_w / 2.0);
    double x1 = w->box_cx + (w->box_w / 2.0);
    double y0 = w->box_y;
    double y1 = w->box_y + w->box_depth;
    double tmin = 0.0;
    double tmax = 1.0e9;

    if (fabs(dx) < 1.0e-9) {
        if ((0.0 < x0) || (0.0 > x1)) { return -1.0; }
    } else {
        double a = x0 / dx;
        double b = x1 / dx;
        double lo = (a < b) ? a : b;
        double hi = (a < b) ? b : a;
        tmin = (lo > tmin) ? lo : tmin;
        tmax = (hi < tmax) ? hi : tmax;
    }
    if (fabs(dy) < 1.0e-9) {
        return -1.0;
    } else {
        double a = y0 / dy;
        double b = y1 / dy;
        double lo = (a < b) ? a : b;
        double hi = (a < b) ? b : a;
        tmin = (lo > tmin) ? lo : tmin;
        tmax = (hi < tmax) ? hi : tmax;
    }
    return (tmin <= tmax) ? tmin : -1.0;
}

/* Distance along the ray at servo angle 'deg' (from the right; 90 ahead), or
 * -1 for nothing in range.  'left_high' says whether angles above 90 are on
 * the left. */
static double raycast(const world_t *w, double deg, bool left_high)
{
    double th = deg * PI_D / 180.0;
    double dx = left_high ? cos(th) : -cos(th);
    double dy = sin(th);
    double best = -1.0;
    double t;

    if (w->has_box) {
        t = hit_box(w, dx, dy);
        if ((t > 0.0) && ((best < 0.0) || (t < best))) { best = t; }
    }
    if (w->has_right_wall && (dx > 1.0e-9)) {
        t = w->right_wall_x / dx;
        if ((t > 0.0) && (t * dy > 0.0) && ((best < 0.0) || (t < best))) { best = t; }
    }
    if (w->has_left_wall && (dx < -1.0e-9)) {
        t = -w->left_wall_x / dx;
        if ((t > 0.0) && (t * dy > 0.0) && ((best < 0.0) || (t < best))) { best = t; }
    }
    if (w->has_back_wall && (dy > 1.0e-9)) {
        t = w->back_y / dy;
        if ((best < 0.0) || (t < best)) { best = t; }
    }
    return (best > 4000.0) ? -1.0 : best;
}

/* ---- the simulated HAL ------------------------------------------------ */

#define MEASURE_MS      (60U)
#define SERVO_MS_PER_DEG (4.0)

typedef struct {
    world_t           world;
    obstacle_config_t cfg;
    scan_t            scan;
    double            servo_deg;
    double            servo_target;
    bool              pending;
    uint32_t          done_ms;
    us_reading_t      reading;
    uint32_t          now;
    uint8_t           visited[128];
    size_t            visited_count;
    bool              dead_sensor;
    uint32_t          finished_ms;
} sim_t;

static obstacle_config_t make_cfg(void)
{
    obstacle_config_t c;

    c.servo.min_pulse_us = 500U;
    c.servo.max_pulse_us = 2500U;
    c.servo.trim_deg = 0;
    c.servo.invert = false;
    c.servo.min_angle_deg = 20U;
    c.servo.max_angle_deg = 160U;
    c.servo.settle_base_ms = 60U;
    c.servo.settle_ms_per_deg = 4U;
    c.coarse_min_deg = 30U;
    c.coarse_max_deg = 150U;
    c.coarse_step_deg = 30U;
    c.fine_step_deg = 5U;
    c.detect_range_mm = 600U;
    c.samples_per_point = 3U;
    c.measure_timeout_ms = 150U;
    c.no_echo_is_clear = false;
    c.left_is_high_angle = true;
    c.robot_width_mm = 140U;
    c.robot_length_mm = 200U;
    c.margin_mm = 30U;
    c.assumed_depth_mm = 150U;
    c.min_turn_distance_mm = 120U;
    c.recovery_limit_mm = 500U;
    c.sweep_clear_min_rays = 0U;
    return c;
}

static world_t empty_room(void)
{
    world_t w;

    w.has_box = false;
    w.box_cx = 0.0; w.box_w = 0.0; w.box_y = 0.0; w.box_depth = 0.0;
    w.has_right_wall = false; w.right_wall_x = 0.0;
    w.has_left_wall = false; w.left_wall_x = 0.0;
    w.has_back_wall = true; w.back_y = 1500.0;
    return w;
}

static world_t box_at(double cx, double width, double y)
{
    world_t w = empty_room();

    w.has_box = true;
    w.box_cx = cx;
    w.box_w = width;
    w.box_y = y;
    w.box_depth = 100.0;
    return w;
}

static void sim_init(sim_t *s, const world_t *w, const obstacle_config_t *cfg)
{
    s->world = *w;
    s->cfg = *cfg;
    s->servo_deg = 90.0;
    s->servo_target = 90.0;
    s->pending = false;
    s->done_ms = 0U;
    s->reading.status = US_ERROR;
    s->reading.distance_mm = 0U;
    s->reading.timestamp_ms = 0U;
    s->reading.seq = 1U;
    s->now = 1000U;
    s->visited_count = 0U;
    s->dead_sensor = false;
    s->finished_ms = 0U;
}

/* Run the whole scan to completion; returns false if it never finishes. */
static bool sim_run(sim_t *s)
{
    bool done = false;
    uint32_t t0;

    scan_start(&s->scan, &s->cfg, s->now);
    t0 = s->now;
    while (!done && ((s->now - t0) < 60000U)) {
        scan_action_t act;
        double step = 10.0 / SERVO_MS_PER_DEG;      /* degrees per 10 ms */

        if (fabs(s->servo_target - s->servo_deg) <= step) {
            s->servo_deg = s->servo_target;
        } else {
            s->servo_deg += (s->servo_target > s->servo_deg) ? step : -step;
        }
        if (s->pending && !s->dead_sensor && (s->now >= s->done_ms)) {
            double d = raycast(&s->world, s->servo_deg, s->cfg.left_is_high_angle);
            uint16_t mm = 0U;

            s->reading.seq++;
            s->reading.timestamp_ms = s->now;
            if (d < 0.0) {
                s->reading.status = US_NO_ECHO;
            } else {
                s->reading.status = us_convert((uint32_t)((d * 2000.0) / 343.0), true, &mm);
                s->reading.distance_mm = mm;
            }
            s->pending = false;
        }

        done = scan_update(&s->scan, s->now, &s->reading, &act);
        if (act.servo_move) {
            s->servo_target = (double)act.servo_angle;
            if (s->visited_count < sizeof(s->visited)) {
                s->visited[s->visited_count++] = act.servo_angle;
            }
        }
        if (act.request_measure) {
            s->pending = true;
            s->done_ms = s->now + MEASURE_MS;
        }
        s->now += 10U;
    }
    s->finished_ms = s->now - t0;
    return done;
}

static void analyse(const sim_t *s, obstacle_profile_t *p, bypass_plan_t *plan)
{
    obstacle_profile_build(&s->cfg, s->scan.points, s->scan.point_count, p);
    obstacle_plan(&s->cfg, p, plan);
}

/* ---- trig ------------------------------------------------------------- */

/* Table sin/cos in per-mille over 0..180.  Catches: a table typo, or wrong
 * symmetry for the second quadrant (angles to the left of straight ahead). */
static void test_trig_tables(void)
{
    int d;

    CHECK_INT(0, trig_sin_permille(0));
    CHECK_INT(500, trig_sin_permille(30));
    CHECK_INT(866, trig_sin_permille(60));
    CHECK_INT(1000, trig_sin_permille(90));
    CHECK_INT(866, trig_sin_permille(120));
    CHECK_INT(0, trig_sin_permille(180));
    CHECK_INT(1000, trig_cos_permille(0));
    CHECK_INT(500, trig_cos_permille(60));
    CHECK_INT(0, trig_cos_permille(90));
    CHECK_INT(-500, trig_cos_permille(120));
    CHECK_INT(-1000, trig_cos_permille(180));
    for (d = 0; d <= 180; d += 7) {
        CHECK(abs(trig_sin_permille((int16_t)d) -
                  (int)lround(1000.0 * sin(d * PI_D / 180.0))) <= 1);
        CHECK(abs(trig_cos_permille((int16_t)d) -
                  (int)lround(1000.0 * cos(d * PI_D / 180.0))) <= 1);
    }
}

/* ---- sequencer and profile ------------------------------------------ */

/* A 100 mm box 300 mm ahead is located, measured and planned around: coarse
 * angles first, then a fine scan around the hit, and the servo returns to
 * straight ahead.  Catches: measuring before the servo settles, wrong order,
 * not returning the servo. */
static void test_scan_visits_coarse_then_fine_and_returns_ahead(void)
{
    sim_t s;
    static const uint8_t COARSE[] = { 30U, 60U, 90U, 120U, 150U };
    size_t i;
    world_t w = box_at(0.0, 100.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    for (i = 0U; i < sizeof(COARSE); i++) {
        CHECK_INT(COARSE[i], s.visited[i]);
    }
    CHECK_INT(60, s.visited[5]);                /* fine region starts 30 deg out */
    CHECK_INT(65, s.visited[6]);
    CHECK_INT(90, s.visited[s.visited_count - 1U]);
    CHECK(s.finished_ms < 15000U);
}

/* The profile locates the box: nearest distance, centre offset and width. */
static void test_profile_of_a_centred_box(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(0.0, 100.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK(p.valid);
    CHECK(p.obstacle_found);
    CHECK(abs((int)p.forward_mm - 300) <= 12);
    CHECK(abs(p.centre_x_mm) <= 25);
    CHECK(abs((int)p.width_mm - 100) <= 40);
    CHECK(p.left_clearance_mm > 200U);
    CHECK(p.right_clearance_mm > 200U);
}

/* An off-centre box is placed on the correct side (right positive).  The box
 * spans x = -20..+80 at 300 mm, so the 90-degree ray hits it. */
static void test_profile_of_an_offset_box(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(30.0, 100.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK(p.obstacle_found);
    CHECK(p.centre_x_mm > 10);
    CHECK(p.centre_x_mm < 50);
}

/* LIMIT, documented: with 30-degree coarse spacing, a 100 mm box 300 mm ahead
 * and 60 mm to the right falls between the 60 and 90 degree rays and is never
 * seen.  The scan reports a clear centre rather than an obstacle.  This is why
 * the robot configuration uses a finer coarse step; see the next test. */
static void test_narrow_offset_object_can_hide_between_coarse_rays(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(60.0, 100.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK(!p.obstacle_found);
}

/* A 15-degree coarse step finds the same box, and the plan clears its right
 * edge.  Catches: choosing a coarse step too wide for the obstacle size. */
static void test_finer_coarse_step_finds_the_offset_object(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(60.0, 100.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    cfg.coarse_step_deg = 15U;
    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK(p.obstacle_found);
    CHECK(p.centre_x_mm > 30);
    CHECK(p.centre_x_mm < 90);
    CHECK_INT(PLAN_BYPASS_LEFT, plan.action);       /* the open side is the left */
}

/* With both sides open, the plan is a left bypass with the five-step
 * rectangle: turn, lateral move, turn back, forward past, turn to the line.
 * The lateral distance clears the obstacle edge by half the car width plus the
 * margin.  Catches: wrong turn signs, wrong distances. */
static void test_open_room_gives_a_left_bypass(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(0.0, 100.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK_INT(PLAN_BYPASS_LEFT, plan.action);
    CHECK_INT(5, plan.step_count);
    CHECK(plan.steps[0].is_turn);
    CHECK_INT(90, plan.steps[0].value);
    CHECK(!plan.steps[1].is_turn);
    CHECK(abs(plan.steps[1].value - 140) <= 35);    /* 70 + 30 + ~40 */
    CHECK(plan.steps[2].is_turn);
    CHECK_INT(-90, plan.steps[2].value);
    CHECK(!plan.steps[3].is_turn);
    CHECK(abs(plan.steps[3].value - 680) <= 30);    /* 300 + 150 + 200 + 30 */
    CHECK(plan.steps[4].is_turn);
    CHECK_INT(-90, plan.steps[4].value);
    CHECK_INT(plan.steps[1].value, plan.return_leg_mm);
}

/* A wall close on the right leaves no room that side: the plan goes left.
 * Catches: ignoring clearance, which would drive the car into the wall. */
static void test_wall_on_the_right_sends_the_car_left(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(0.0, 100.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    w.has_right_wall = true;
    w.right_wall_x = 120.0;
    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK(p.right_clearance_mm < p.left_clearance_mm);
    CHECK_INT(PLAN_BYPASS_LEFT, plan.action);
}

/* The mirror image: a wall on the left sends the car right, with the turn
 * signs mirrored.  Catches: a planner that always prefers left. */
static void test_wall_on_the_left_sends_the_car_right(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(0.0, 100.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    w.has_left_wall = true;
    w.left_wall_x = 120.0;
    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK_INT(PLAN_BYPASS_RIGHT, plan.action);
    CHECK_INT(-90, plan.steps[0].value);
    CHECK_INT(90, plan.steps[2].value);
    CHECK_INT(90, plan.steps[4].value);
}

/* With room on both sides the shorter detour wins: a box left of centre is
 * passed on its right.  Catches: a planner that always prefers one side,
 * which would add needless deviation from the line (Tribunal: minimise). */
static void test_the_shorter_detour_side_is_chosen(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(-40.0, 100.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK(p.centre_x_mm < -10);
    CHECK_INT(PLAN_BYPASS_RIGHT, plan.action);
    CHECK_INT(-90, plan.steps[0].value);
}

/* An object that IS seen but lies beside the car's path (its near edge is
 * outside half the car width plus the margin) does not need a bypass.
 * Catches: treating every detected object as blocking, which would send the
 * car round things that are not in its way. */
static void test_visible_object_beside_the_path_does_not_block_it(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(173.0, 60.0, 250.0);     /* x = 143..203, hit at ~60 deg */
    obstacle_config_t cfg = make_cfg();

    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK(p.obstacle_found);                    /* it was seen... */
    CHECK(p.left_edge_x_mm > 100);              /* ...and is clear of the path */
    CHECK_INT(PLAN_CONTINUE, plan.action);
}

/* Walls both sides: no safe route, the car stays stopped (Tribunal rule). */
static void test_corridor_with_no_room_gives_stop(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(0.0, 100.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    w.has_right_wall = true;
    w.right_wall_x = 120.0;
    w.has_left_wall = true;
    w.left_wall_x = 120.0;
    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK_INT(PLAN_STOP, plan.action);
    CHECK_INT(PLAN_REASON_NO_CLEARANCE, plan.reason);
    CHECK_INT(0, plan.step_count);
}

/* Nothing in the way: continue, and the scan ends after the coarse pass. */
static void test_empty_room_continues(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = empty_room();
    obstacle_config_t cfg = make_cfg();

    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK(p.valid);
    CHECK(!p.obstacle_found);
    CHECK(p.centre_clear);
    CHECK_INT(PLAN_CONTINUE, plan.action);
    CHECK_INT(PLAN_REASON_NO_OBSTACLE, plan.reason);
    CHECK_INT(5 + 1, s.visited_count);          /* five angles + back to 90 */
}

/* An object outside the car's corridor is not an obstacle to bypass. */
static void test_object_outside_the_corridor_is_ignored(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(300.0, 100.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK_INT(PLAN_CONTINUE, plan.action);
}

/* No echoes at all is NOT free space: the scan is invalid and the plan is to
 * stay stopped.  Catches: treating silence as an open road (Tribunal rule). */
static void test_no_echo_is_not_free_space(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = empty_room();
    obstacle_config_t cfg = make_cfg();

    w.has_back_wall = false;
    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK(!p.valid);
    CHECK_INT(PLAN_STOP, plan.action);
    CHECK_INT(PLAN_REASON_INVALID_SCAN, plan.reason);
}

/* ...unless the team explicitly configures no-echo as clear. */
static void test_no_echo_can_be_configured_as_clear(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = empty_room();
    obstacle_config_t cfg = make_cfg();

    w.has_back_wall = false;
    cfg.no_echo_is_clear = true;
    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK(p.valid);
    CHECK_INT(PLAN_CONTINUE, plan.action);
}

/* A dead sensor: every reading times out, the scan still terminates in
 * bounded time and the plan is to stop.  Catches: waiting forever on a dead
 * ultrasonic sensor. */
static void test_dead_sensor_terminates_and_stops(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(0.0, 100.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    sim_init(&s, &w, &cfg);
    s.dead_sensor = true;
    CHECK(sim_run(&s));
    CHECK(s.finished_ms < 30000U);
    analyse(&s, &p, &plan);
    CHECK_INT(PLAN_STOP, plan.action);
    CHECK_INT(PLAN_REASON_INVALID_SCAN, plan.reason);
}

/* Too close to turn: back up first, by enough to reach the minimum distance
 * plus a margin. */
static void test_obstacle_too_close_to_turn_gives_reverse(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(0.0, 100.0, 80.0);
    obstacle_config_t cfg = make_cfg();

    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK_INT(PLAN_REVERSE, plan.action);
    CHECK_INT(PLAN_REASON_TOO_CLOSE, plan.reason);
    CHECK_INT(1, plan.step_count);
    CHECK(!plan.steps[0].is_turn);
    CHECK(plan.steps[0].value < 0);
    CHECK(abs(plan.steps[0].value + 70) <= 15);     /* -(120 - 80 + 30) */
}

/* A detour longer than the allowed return leg (0.5 m recovery limit) is
 * refused even when there is room.  Catches: a bypass the car could never
 * recover from. */
static void test_detour_longer_than_the_recovery_limit_is_refused(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(0.0, 200.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    cfg.recovery_limit_mm = 150U;
    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK_INT(PLAN_STOP, plan.action);
    CHECK_INT(PLAN_REASON_DETOUR_TOO_LARGE, plan.reason);
}

/* Mirrored servo mounting: a world mirrored left-right gives the mirrored
 * plan when left_is_high_angle is false.  Catches: a hard-coded left/right
 * convention (the bench found the servo horn not aligned). */
static void test_servo_mounting_direction_is_configurable(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(0.0, 100.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    w.has_right_wall = true;
    w.right_wall_x = 120.0;             /* physical wall on the car's right */
    cfg.left_is_high_angle = false;     /* high angles now point RIGHT */
    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK_INT(PLAN_BYPASS_LEFT, plan.action);
    CHECK_INT(90, plan.steps[0].value);
}

/* With the servo mounted the other way round, an object on the car's right
 * must still be reported on the right (positive x).  Catches: a lateral
 * position that ignores the mounting, which would send the car into the
 * obstacle instead of round it. */
static void test_servo_mounting_mirror_keeps_the_object_on_the_correct_side(void)
{
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;
    world_t w = box_at(40.0, 100.0, 300.0);     /* physically right of centre */
    obstacle_config_t cfg = make_cfg();

    cfg.left_is_high_angle = false;
    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK(p.centre_x_mm > 10);
    CHECK_INT(PLAN_BYPASS_LEFT, plan.action);   /* left is the shorter detour */
}

/* The overall scan duration stays bounded for a typical scene. */
static void test_scan_time_is_bounded(void)
{
    sim_t s;
    world_t w = box_at(0.0, 100.0, 300.0);
    obstacle_config_t cfg = make_cfg();

    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    CHECK(s.finished_ms > 1000U);               /* it really measured */
    CHECK(s.finished_ms < 12000U);
}

/* ---- the real car and the measured servo ----------------------------- */

/* The car (200 x 250 mm) and the servo sweep measured by Buddy 5 (58 to 118
 * degrees), as in robot_config.c. */
static obstacle_config_t car_cfg(void)
{
    obstacle_config_t c = make_cfg();

    c.servo.min_pulse_us = 375U;
    c.servo.max_pulse_us = 2625U;
    c.servo.min_angle_deg = 58U;
    c.servo.max_angle_deg = 118U;
    c.coarse_min_deg = 62U;
    c.coarse_max_deg = 118U;
    c.coarse_step_deg = 14U;
    c.detect_range_mm = 800U;
    c.robot_width_mm = 200U;
    c.robot_length_mm = 250U;
    return c;
}

/* From the scan distance the robot now uses (650 mm) the farthest ray reaches far
 * enough to confirm room for the 200 mm car beside the box, so a bypass is
 * planned.  Catches: a scan distance, detect range or sweep that cannot find a
 * path. */
static void test_car_finds_a_path_when_the_scan_starts_far_enough(void)
{
    obstacle_config_t cfg = car_cfg();
    world_t w;
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;

    w = box_at(0.0, 80.0, 650.0);
    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK(p.obstacle_found);
    CHECK((plan.action == PLAN_BYPASS_LEFT) || (plan.action == PLAN_BYPASS_RIGHT));
    CHECK_INT(PLAN_REASON_OK, plan.reason);

    w = box_at(0.0, 150.0, 700.0);
    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK((plan.action == PLAN_BYPASS_LEFT) || (plan.action == PLAN_BYPASS_RIGHT));
}

/* From the old stop distance (250 mm) the same sweep confirms at most about
 * 100 mm beside the box, so the planner correctly refuses: this is why the scan
 * must start far away.  Catches: the planner inventing room it has not seen. */
static void test_car_cannot_confirm_a_path_from_the_stop_distance(void)
{
    obstacle_config_t cfg = car_cfg();
    world_t w = box_at(0.0, 80.0, 250.0);
    sim_t s;
    obstacle_profile_t p;
    bypass_plan_t plan;

    sim_init(&s, &w, &cfg);
    CHECK(sim_run(&s));
    analyse(&s, &p, &plan);
    CHECK(p.obstacle_found);
    CHECK_INT(PLAN_STOP, plan.action);
    CHECK_INT(PLAN_REASON_NO_CLEARANCE, plan.reason);
    CHECK(p.left_clearance_mm < 130U);
    CHECK(p.right_clearance_mm < 130U);
}

/* ---- the real scans (felt tubes on the HC-SR04) ---------------------- */

typedef struct { uint8_t angle; bool clear; uint16_t mm; } real_point_t;

static void plan_real_scan(const real_point_t *r, uint8_t n, uint8_t min_rays,
                           obstacle_profile_t *p, bypass_plan_t *plan)
{
    obstacle_config_t cfg = car_cfg();
    scan_point_t pts[16];
    uint8_t i;

    cfg.sweep_clear_min_rays = min_rays;
    for (i = 0U; i < n; i++) {
        pts[i].angle_deg = r[i].angle;
        pts[i].kind = r[i].clear ? SCAN_POINT_FAR : SCAN_POINT_OBSTACLE;
        pts[i].distance_mm = r[i].clear ? US_MAX_RANGE_MM : r[i].mm;
    }
    obstacle_profile_build(&cfg, pts, n, p);
    obstacle_plan(&cfg, p, plan);
}

/* Scans measured on the car, 14 Nov: the beam is wide, so the box is seen across
 * most of the sweep and only the end rays can be clear.  A clear point is a no echo
 * counted as free space (no_echo_is_clear). */
static const real_point_t REAL_80_AT_500[] = {
    { 62U, true, 0U }, { 67U, true, 0U }, { 72U, false, 539U }, { 77U, false, 517U },
    { 82U, false, 513U }, { 87U, false, 513U }, { 92U, false, 504U }, { 97U, false, 513U },
    { 102U, false, 510U }, { 107U, false, 514U }, { 112U, false, 514U }, { 117U, false, 526U } };
static const real_point_t REAL_150_AT_500[] = {
    { 62U, true, 0U }, { 76U, false, 513U }, { 81U, false, 500U }, { 86U, false, 496U },
    { 91U, false, 496U }, { 96U, false, 509U }, { 101U, false, 509U }, { 106U, true, 0U },
    { 111U, true, 0U }, { 116U, true, 0U } };
static const real_point_t REAL_80_AT_650[] = {
    { 62U, true, 0U }, { 67U, true, 0U }, { 72U, false, 718U }, { 77U, false, 689U },
    { 82U, false, 671U }, { 87U, false, 663U }, { 92U, false, 663U }, { 97U, false, 664U },
    { 102U, false, 676U }, { 107U, false, 664U }, { 112U, false, 681U }, { 117U, true, 0U } };
static const real_point_t REAL_150_AT_650[] = {
    { 58U, false, 656U }, { 63U, false, 652U }, { 68U, false, 656U }, { 73U, false, 648U },
    { 78U, false, 647U }, { 83U, false, 647U }, { 88U, false, 639U }, { 93U, false, 634U },
    { 98U, false, 639U }, { 103U, false, 639U }, { 108U, false, 640U }, { 113U, false, 652U },
    { 118U, false, 657U } };

#define COUNT_OF(a) ((uint8_t)(sizeof(a) / sizeof((a)[0])))

/* With the rule as the Tribunal wrote it, none of the four real scans can be
 * planned: the sonar confirms far too little room beside the box.  Catches: the
 * default silently becoming permissive. */
static void test_real_scans_are_refused_by_the_confirmed_clearance_rule(void)
{
    obstacle_profile_t p;
    bypass_plan_t plan;

    plan_real_scan(REAL_80_AT_500, COUNT_OF(REAL_80_AT_500), 0U, &p, &plan);
    CHECK_INT(PLAN_STOP, plan.action);
    plan_real_scan(REAL_150_AT_500, COUNT_OF(REAL_150_AT_500), 0U, &p, &plan);
    CHECK_INT(PLAN_STOP, plan.action);
    plan_real_scan(REAL_80_AT_650, COUNT_OF(REAL_80_AT_650), 0U, &p, &plan);
    CHECK_INT(PLAN_STOP, plan.action);
    plan_real_scan(REAL_150_AT_650, COUNT_OF(REAL_150_AT_650), 0U, &p, &plan);
    CHECK_INT(PLAN_STOP, plan.action);
}

/* With "two clear rays reaching the end of the sweep" the car goes round the side
 * that is open, and still stops when no side is.  Catches: a wrong side, or a
 * bypass with nothing clear. */
static void test_open_to_the_sweep_limit_picks_the_open_side(void)
{
    obstacle_profile_t p;
    bypass_plan_t plan;

    plan_real_scan(REAL_80_AT_500, COUNT_OF(REAL_80_AT_500), 2U, &p, &plan);
    CHECK_INT(PLAN_BYPASS_RIGHT, plan.action);        /* only the right side has two clear rays */
    CHECK_INT(PLAN_REASON_OK, plan.reason);
    plan_real_scan(REAL_150_AT_500, COUNT_OF(REAL_150_AT_500), 2U, &p, &plan);
    CHECK_INT(PLAN_BYPASS_LEFT, plan.action);         /* three clear rays on the left, one on the right */
    plan_real_scan(REAL_80_AT_650, COUNT_OF(REAL_80_AT_650), 2U, &p, &plan);
    CHECK_INT(PLAN_BYPASS_RIGHT, plan.action);        /* one clear ray on the left is not enough */
    plan_real_scan(REAL_150_AT_650, COUNT_OF(REAL_150_AT_650), 2U, &p, &plan);
    CHECK_INT(PLAN_STOP, plan.action);                /* the box fills the whole sweep: nothing clear */
    CHECK_INT(PLAN_REASON_NO_CLEARANCE, plan.reason);
}

/* A detour longer than the recovery limit is still refused when the side is only
 * open to the sweep limit.  Catches: the open-side rule bypassing the other checks. */
static void test_open_side_still_respects_the_recovery_limit(void)
{
    obstacle_profile_t p;
    bypass_plan_t plan;
    obstacle_config_t cfg = car_cfg();
    scan_point_t pts[8];
    uint8_t i;
    static const uint8_t ang[8] = { 62U, 67U, 72U, 77U, 82U, 87U, 92U, 97U };

    cfg.sweep_clear_min_rays = 2U;
    cfg.recovery_limit_mm = 150U;
    for (i = 0U; i < 8U; i++) {
        const bool clear = (i < 2U);

        pts[i].angle_deg = ang[i];
        pts[i].kind = clear ? SCAN_POINT_FAR : SCAN_POINT_OBSTACLE;
        pts[i].distance_mm = clear ? US_MAX_RANGE_MM : 520U;
    }
    obstacle_profile_build(&cfg, pts, 8U, &p);
    obstacle_plan(&cfg, &p, &plan);
    CHECK_INT(PLAN_STOP, plan.action);
}

/* ---- the blind bypass (no open side seen) ----------------------------- */

/* The scan from the free-roam run on the car, 17 Nov: the box fills all 13 rays. */
static const real_point_t REAL_FILLS_SWEEP[] = {
    { 58U, false, 494U }, { 63U, false, 498U }, { 68U, false, 495U }, { 73U, false, 495U },
    { 78U, false, 482U }, { 83U, false, 477U }, { 88U, false, 477U }, { 93U, false, 474U },
    { 98U, false, 470U }, { 103U, false, 474U }, { 108U, false, 483U }, { 113U, false, 479U },
    { 118U, false, 483U } };

static bool blind_for(const real_point_t *r, uint8_t n, obstacle_profile_t *p, bypass_plan_t *plan)
{
    obstacle_config_t cfg = car_cfg();
    scan_point_t pts[16];
    uint8_t i;

    cfg.sweep_clear_min_rays = 2U;
    cfg.margin_mm = 80U;
    for (i = 0U; i < n; i++) {
        pts[i].angle_deg = r[i].angle;
        pts[i].kind = r[i].clear ? SCAN_POINT_FAR : SCAN_POINT_OBSTACLE;
        pts[i].distance_mm = r[i].clear ? US_MAX_RANGE_MM : r[i].mm;
    }
    obstacle_profile_build(&cfg, pts, n, p);
    obstacle_plan(&cfg, p, plan);
    if (plan->action != PLAN_STOP) {
        return false;
    }
    return obstacle_blind_plan(&cfg, pts, n, p, plan);
}

/* The planner stops when the box fills the sweep; the blind plan then goes round the
 * side whose outer rays read farther (the right here), with a sensible distance.
 * Catches: a missing fallback, a wrong side, or a silly sideways distance. */
static void test_blind_plan_goes_round_a_box_that_fills_the_sweep(void)
{
    obstacle_profile_t p;
    bypass_plan_t plan;

    CHECK(blind_for(REAL_FILLS_SWEEP, COUNT_OF(REAL_FILLS_SWEEP), &p, &plan));
    CHECK_INT(PLAN_BYPASS_RIGHT, plan.action);
    CHECK_INT(5, plan.step_count);
    CHECK(plan.steps[0].is_turn && (plan.steps[0].value == -90));    /* turn right first */
    CHECK(!plan.steps[1].is_turn && (plan.steps[1].value > 300) && (plan.steps[1].value <= 500));
    CHECK(!plan.steps[3].is_turn && (plan.steps[3].value > 800) && (plan.steps[3].value < 1100));
}

/* The farther-reading end wins: the same box with the right end closer goes left.
 * Catches: a fixed side. */
static void test_blind_plan_takes_the_side_that_reads_farther(void)
{
    static const real_point_t FLIPPED[] = {
        { 58U, false, 470U }, { 63U, false, 470U }, { 68U, false, 480U }, { 73U, false, 480U },
        { 78U, false, 480U }, { 83U, false, 477U }, { 88U, false, 477U }, { 93U, false, 474U },
        { 98U, false, 480U }, { 103U, false, 480U }, { 108U, false, 490U }, { 113U, false, 500U },
        { 118U, false, 505U } };
    obstacle_profile_t p;
    bypass_plan_t plan;

    CHECK(blind_for(FLIPPED, COUNT_OF(FLIPPED), &p, &plan));
    CHECK_INT(PLAN_BYPASS_LEFT, plan.action);
    CHECK(plan.steps[0].is_turn && (plan.steps[0].value == 90));
}

/* No obstacle, or one too close to turn, gives no blind plan.  Catches: a bypass
 * round nothing. */
static void test_blind_plan_refuses_when_there_is_nothing_to_go_round(void)
{
    obstacle_config_t cfg = car_cfg();
    obstacle_profile_t p;
    bypass_plan_t plan;
    scan_point_t pts[3];

    pts[0].angle_deg = 62U; pts[0].kind = SCAN_POINT_FAR; pts[0].distance_mm = 1500U;
    pts[1].angle_deg = 90U; pts[1].kind = SCAN_POINT_FAR; pts[1].distance_mm = 1500U;
    pts[2].angle_deg = 118U; pts[2].kind = SCAN_POINT_FAR; pts[2].distance_mm = 1500U;
    obstacle_profile_build(&cfg, pts, 3U, &p);
    CHECK(!obstacle_blind_plan(&cfg, pts, 3U, &p, &plan));

    pts[1].kind = SCAN_POINT_OBSTACLE;
    pts[1].distance_mm = 90U;                       /* nearer than the turn distance */
    obstacle_profile_build(&cfg, pts, 3U, &p);
    CHECK(!obstacle_blind_plan(&cfg, pts, 3U, &p, &plan));
}

int main(void)
{
    RUN(test_trig_tables);
    RUN(test_scan_visits_coarse_then_fine_and_returns_ahead);
    RUN(test_profile_of_a_centred_box);
    RUN(test_profile_of_an_offset_box);
    RUN(test_narrow_offset_object_can_hide_between_coarse_rays);
    RUN(test_finer_coarse_step_finds_the_offset_object);
    RUN(test_open_room_gives_a_left_bypass);
    RUN(test_wall_on_the_right_sends_the_car_left);
    RUN(test_wall_on_the_left_sends_the_car_right);
    RUN(test_the_shorter_detour_side_is_chosen);
    RUN(test_visible_object_beside_the_path_does_not_block_it);
    RUN(test_corridor_with_no_room_gives_stop);
    RUN(test_empty_room_continues);
    RUN(test_object_outside_the_corridor_is_ignored);
    RUN(test_no_echo_is_not_free_space);
    RUN(test_no_echo_can_be_configured_as_clear);
    RUN(test_dead_sensor_terminates_and_stops);
    RUN(test_obstacle_too_close_to_turn_gives_reverse);
    RUN(test_detour_longer_than_the_recovery_limit_is_refused);
    RUN(test_servo_mounting_direction_is_configurable);
    RUN(test_servo_mounting_mirror_keeps_the_object_on_the_correct_side);
    RUN(test_scan_time_is_bounded);
    RUN(test_car_finds_a_path_when_the_scan_starts_far_enough);
    RUN(test_car_cannot_confirm_a_path_from_the_stop_distance);
    RUN(test_real_scans_are_refused_by_the_confirmed_clearance_rule);
    RUN(test_open_to_the_sweep_limit_picks_the_open_side);
    RUN(test_open_side_still_respects_the_recovery_limit);
    RUN(test_blind_plan_goes_round_a_box_that_fills_the_sweep);
    RUN(test_blind_plan_takes_the_side_that_reads_farther);
    RUN(test_blind_plan_refuses_when_there_is_nothing_to_go_round);
    return TEST_REPORT();
}
