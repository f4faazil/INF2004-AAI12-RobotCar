/*
 * obstacle_report.c
 * Not a test: a table of what the scan, profile and bypass planner decide for
 * the real car (200 x 250 mm) and the measured servo sweep (58 to 118 degrees),
 * for a box of a given width straight ahead at several distances.  It shows
 * why the planner reports "no clearance" and what would change that.
 *
 * SIMULATION ONLY: ideal ranges, no beam width.  The ray-cast world is the one
 * test_obstacle.c uses.
 *
 *   make obstacle_report && ./obstacle_report
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "obstacle.h"

#define PI_D (3.14159265358979)

typedef struct {
    bool   has_box;
    double box_cx, box_w, box_y, box_depth;
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

static double raycast(const world_t *w, double deg)
{
    double th = deg * PI_D / 180.0;
    double dx = cos(th);                    /* angles above 90 are on the left, x right positive */
    double dy = sin(th);
    double best = -1.0;
    double t;

    if (w->has_box) {
        t = hit_box(w, dx, dy);
        if ((t > 0.0) && ((best < 0.0) || (t < best))) { best = t; }
    }
    if (w->has_back_wall && (dy > 1.0e-9)) {
        t = w->back_y / dy;
        if ((best < 0.0) || (t < best)) { best = t; }
    }
    return (best > 4000.0) ? -1.0 : best;
}

static obstacle_config_t car_config(bool no_echo_is_clear, uint16_t margin, uint16_t detect_mm)
{
    obstacle_config_t c;

    c.servo.min_pulse_us = 375U;
    c.servo.max_pulse_us = 2625U;
    c.servo.trim_deg = 0;
    c.servo.invert = false;
    c.servo.min_angle_deg = 58U;
    c.servo.max_angle_deg = 118U;
    c.servo.settle_base_ms = 60U;
    c.servo.settle_ms_per_deg = 4U;
    c.coarse_min_deg = 62U;
    c.coarse_max_deg = 118U;
    c.coarse_step_deg = 14U;
    c.fine_step_deg = 5U;
    c.detect_range_mm = detect_mm;
    c.samples_per_point = 3U;
    c.measure_timeout_ms = 150U;
    c.no_echo_is_clear = no_echo_is_clear;
    c.left_is_high_angle = true;
    c.robot_width_mm = 200U;                /* measured, including protruding parts */
    c.robot_length_mm = 250U;
    c.margin_mm = margin;
    c.assumed_depth_mm = 150U;
    c.min_turn_distance_mm = 120U;
    c.recovery_limit_mm = 500U;
    c.sweep_clear_min_rays = 0U;
    c.max_width_mm = 0U;
    c.forward_extra_mm = 0U;
    c.blind_side = 0;
    return c;
}

/* Run the whole scan against the world; fills the profile and the plan. */
static void run_scan(const world_t *w, const obstacle_config_t *cfg,
                     obstacle_profile_t *prof, bypass_plan_t *plan)
{
    scan_t scan;
    us_reading_t reading;
    double servo_deg = 90.0;
    double servo_target = 90.0;
    bool pending = false;
    uint32_t done_ms = 0U;
    uint32_t now = 1000U;
    uint32_t t0;
    bool done = false;

    reading.status = US_ERROR;
    reading.distance_mm = 0U;
    reading.timestamp_ms = 0U;
    reading.seq = 1U;
    scan_start(&scan, cfg, now);
    t0 = now;
    while (!done && ((now - t0) < 60000U)) {
        scan_action_t act;
        double step = 10.0 / 4.0;

        if (fabs(servo_target - servo_deg) <= step) {
            servo_deg = servo_target;
        } else {
            servo_deg += (servo_target > servo_deg) ? step : -step;
        }
        if (pending && (now >= done_ms)) {
            double d = raycast(w, servo_deg);
            uint16_t mm = 0U;

            reading.seq++;
            reading.timestamp_ms = now;
            if (d < 0.0) {
                reading.status = US_NO_ECHO;
            } else {
                reading.status = us_convert((uint32_t)((d * 2000.0) / 343.0), true, &mm);
                reading.distance_mm = mm;
            }
            pending = false;
        }
        done = scan_update(&scan, now, &reading, &act);
        if (act.servo_move) {
            servo_target = (double)act.servo_angle;
        }
        if (act.request_measure) {
            pending = true;
            done_ms = now + 60U;
        }
        now += 10U;
    }
    obstacle_profile_build(cfg, scan.points, scan.point_count, prof);
    obstacle_plan(cfg, prof, plan);
}

static const char *action_name(plan_action_t a)
{
    switch (a) {
    case PLAN_CONTINUE:     return "continue";
    case PLAN_BYPASS_LEFT:  return "BYPASS LEFT";
    case PLAN_BYPASS_RIGHT: return "BYPASS RIGHT";
    case PLAN_REVERSE:      return "reverse";
    default:                return "STOP";
    }
}

static const char *reason_name(plan_reason_t r)
{
    switch (r) {
    case PLAN_REASON_OK:               return "ok";
    case PLAN_REASON_NO_OBSTACLE:      return "no-obstacle";
    case PLAN_REASON_INVALID_SCAN:     return "invalid-scan";
    case PLAN_REASON_NO_CLEARANCE:     return "no-clearance";
    case PLAN_REASON_DETOUR_TOO_LARGE: return "detour-too-large";
    default:                           return "too-close";
    }
}

static void table(const char *title, bool back_wall, bool no_echo_is_clear, uint16_t margin,
                  uint16_t detect_mm, const double *dists, unsigned int ndist)
{
    static const double widths[] = { 80.0, 150.0 };
    obstacle_config_t cfg = car_config(no_echo_is_clear, margin, detect_mm);
    unsigned int wi;
    unsigned int di;

    printf("\n%s\n", title);
    printf("  car needs %d mm of confirmed free space beside the box (width %d + 2 x margin %d)\n",
           (int)cfg.robot_width_mm + (2 * (int)margin), (int)cfg.robot_width_mm, (int)margin);
    printf("  box   dist | decision                       | width seen | clear L / R (mm)\n");
    for (wi = 0U; wi < 2U; wi++) {
        for (di = 0U; di < ndist; di++) {
            world_t w;
            obstacle_profile_t prof;
            bypass_plan_t plan;

            w.has_box = true;
            w.box_cx = 0.0;
            w.box_w = widths[wi];
            w.box_y = dists[di];
            w.box_depth = 50.0;
            w.has_back_wall = back_wall;
            w.back_y = 1500.0;
            run_scan(&w, &cfg, &prof, &plan);
            printf("  %3d  %4d | %-12s %-17s | %6d     | %4d / %4d\n", (int)widths[wi], (int)dists[di],
                   action_name(plan.action), reason_name(plan.reason), (int)prof.width_mm,
                   (int)prof.left_clearance_mm, (int)prof.right_clearance_mm);
        }
    }
}

int main(void)
{
    const double tan28 = tan(28.0 * PI_D / 180.0);

    printf("Real car 200 x 250 mm, servo sweep 62..118 degrees (28 degrees each side of centre).\n");
    printf("The farthest ray reaches %.0f%% of the distance to the side: at 300 mm that is %.0f mm, at 600 mm %.0f mm.\n",
           tan28 * 100.0, 300.0 * tan28, 600.0 * tan28);
    printf("The planner needs the free gap from the box edge to the farthest clear ray to be >= 260 mm.\n");
    printf("For an 80 mm box that needs the box at >= %.0f mm, for a 150 mm box >= %.0f mm.\n",
           (260.0 + 40.0) / tan28, (260.0 + 75.0) / tan28);

    static const double near_set[] = { 250.0, 300.0, 400.0, 500.0, 600.0 };
    static const double far_set[] = { 500.0, 600.0, 650.0, 700.0, 750.0 };

    table("A. as the robot is now (detect range 600 mm): room with a back wall 1.5 m ahead", true, false, 30U, 600U, near_set, 5U);
    table("B. as now, but open floor (side rays see nothing: no echo)", false, false, 30U, 600U, near_set, 5U);
    table("C. detect range 800 mm, scan from 600 to 750 mm: room with a back wall 1.5 m ahead", true, false, 30U, 800U, far_set, 5U);
    table("D. detect range 800 mm, open floor, no echo is NOT free (the Tribunal rule)", false, false, 30U, 800U, far_set, 5U);
    table("E. detect range 800 mm, open floor, no echo beside the box counts as free", false, true, 30U, 800U, far_set, 5U);
    return 0;
}
