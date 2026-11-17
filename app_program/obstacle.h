/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * obstacle.h
 * Obstacle scanning, profiling and bypass planning (Buddy 5).
 *
 *  1. scan_*     a sequencer that sweeps the servo, waits for it to settle and
 *                takes a median of ultrasonic readings at each angle: a coarse
 *                scan first, then a finer scan around the obstacle;
 *  2. profile    location, width, closest point and left/right clearance from
 *                the scan points;
 *  3. plan       stop / continue / bypass left / bypass right / reverse, with
 *                the manoeuvre as a list of turn and move steps for the
 *                Vehicle Controller to execute.
 *
 * Rules from the Tribunal: scanning starts only after the car is confirmed
 * stopped (the Vehicle Controller guarantees that); "no echo" is never taken
 * as free space unless explicitly configured; invalid scans or no safe route
 * keep the car stopped and report a fault; this module never drives the
 * wheels, it only produces a plan.
 *
 * Coordinates: x is lateral with RIGHT positive, y is forward.  Servo angles
 * are measured from the car's right: 90 is straight ahead.
 */
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>

#include "servo.h"
#include "ultrasonic.h"

#define SCAN_MAX_POINTS         (32U)
#define PLAN_MAX_STEPS          (6U)
#define SCAN_FORWARD_DEG        (90U)

typedef struct {
    servo_cfg_t servo;
    uint8_t     coarse_min_deg;
    uint8_t     coarse_max_deg;
    uint8_t     coarse_step_deg;
    uint8_t     fine_step_deg;
    uint16_t    detect_range_mm;        /* closer than this is an obstacle */
    uint8_t     samples_per_point;      /* readings per angle (median) */
    uint16_t    measure_timeout_ms;     /* give up on one reading after this */
    bool        no_echo_is_clear;       /* default false: no echo is unknown */
    bool        left_is_high_angle;     /* servo angle > 90 points left */

    uint16_t    robot_width_mm;
    uint16_t    robot_length_mm;
    uint16_t    margin_mm;
    uint16_t    assumed_depth_mm;       /* the ultrasonic sees only the front face */
    uint16_t    min_turn_distance_mm;   /* nearer than this: reverse first */
    uint16_t    recovery_limit_mm;      /* longest allowed return leg (0.5 m) */

    /*
     * 0 = off: free space beside the obstacle must be confirmed out to the
     * clearance the car needs.  The HC-SR04's beam is wide and the servo sweeps only
     * about 28 degrees each side, so on the real car the sonar can confirm only 40 to
     * 120 mm beside a box and that rule always stops the car.  N > 0: a side counts as
     * open when at least N rays in a row beyond the obstacle, out to the end of the
     * sweep, are clear (a far echo, or with no_echo_is_clear a no echo).  The car then
     * drives up to a car-width into space the sonar never saw.  Only for a controlled
     * arena: it weakens the "confirmed free space" rule.
     */
    uint8_t     sweep_clear_min_rays;

    /*
     * 0 = off.  The sonar makes a box look much wider than it is (the beam is wide: an
     * 80 to 150 mm box reads 300 to 650 mm), which makes the sideways leg of a bypass
     * far too long.  The scanned width is clamped to this, about its centre, for the
     * profile's edges.  Set it to a little more than the widest real obstacle.
     */
    uint16_t    max_width_mm;

    /* Added to the forward leg of a bypass, so the car does not turn back close to the
     * far side of an obstacle deeper than assumed_depth_mm. */
    uint16_t    forward_extra_mm;

    /* The side the blind bypass takes: +1 left, -1 right, 0 = the side whose outer rays read
     * farther.  When the box fills the sweep that choice is close to noise, so set this to
     * the side the course leaves free.  A forced side that needs more than recovery_limit_mm
     * gives no plan. */
    int8_t      blind_side;

    /*
     * The forward leg of a bypass, from the car's real geometry.  The axle moves from the stop
     * until the car's REAR is past the obstacle's back edge:
     *     forward = distance to the obstacle's front (sonar) + sensor_to_axle_mm
     *             + assumed_depth_mm + axle_to_rear_mm + forward_margin_mm + forward_extra_mm.
     * Set axle_to_rear_mm to 0 for the older, much longer estimate that adds the car's whole
     * length (robot_length_mm) and margin_mm instead.
     */
    uint16_t    sensor_to_axle_mm;      /* the sonar face is this far ahead of the wheel axle */
    uint16_t    axle_to_rear_mm;        /* the axle to the car's rear edge */
    uint16_t    forward_margin_mm;      /* clearance of the car's rear past the obstacle's back edge */
} obstacle_config_t;


/* ---- scan sequencer --------------------------------------------------- */

typedef enum {
    SCAN_POINT_OBSTACLE = 0,    /* valid echo closer than the detect range */
    SCAN_POINT_FAR,             /* valid echo beyond it: confirmed free */
    SCAN_POINT_NOECHO,          /* no echo: unknown */
    SCAN_POINT_INVALID          /* sensor error / inconsistent */
} scan_point_kind_t;

typedef struct {
    uint8_t           angle_deg;
    scan_point_kind_t kind;
    uint16_t          distance_mm;
} scan_point_t;

typedef enum {
    SCAN_IDLE = 0,
    SCAN_MOVING,                /* waiting for the servo to settle */
    SCAN_MEASURING,
    SCAN_RETURNING,             /* putting the servo back to straight ahead */
    SCAN_DONE
} scan_state_t;

typedef struct {
    bool    servo_move;         /* command servo_angle now */
    uint8_t servo_angle;
    bool    request_measure;    /* trigger one ultrasonic measurement now */
} scan_action_t;

typedef struct {
    obstacle_config_t cfg;
    scan_state_t      state;
    bool              fine_phase;
    uint8_t           angles[SCAN_MAX_POINTS];
    uint8_t           angle_count;
    uint8_t           angle_index;
    uint8_t           servo_angle;      /* last commanded */
    uint32_t          state_since_ms;
    uint16_t          settle_ms;
    uint32_t          request_seq;
    bool              measure_requested;
    uint16_t          samples[8];
    uint8_t           sample_count;
    uint8_t           valid_count;
    uint8_t           noecho_count;
    uint8_t           taken;
    scan_point_t      coarse[SCAN_MAX_POINTS];
    uint8_t           coarse_count;
    scan_point_t      points[SCAN_MAX_POINTS];
    uint8_t           point_count;
    uint32_t          started_ms;
    bool              move_issued;      /* the move for angle_index was commanded */
    uint32_t          request_ms;
    uint8_t           fine_lo;
    uint8_t           fine_hi;
} scan_t;

void scan_start(scan_t *s, const obstacle_config_t *cfg, uint32_t now_ms);

/*
 * Advance the scan.  Call every ~10 ms with the newest reading.  Fills *act
 * with what the HAL must do now.  Returns true once the scan is complete and
 * the servo has been returned to straight ahead.
 */
bool scan_update(scan_t *s, uint32_t now_ms, const us_reading_t *reading,
                 scan_action_t *act);

/* ---- profile ----------------------------------------------------------- */

typedef struct {
    bool     valid;             /* false: the scan could not establish anything */
    bool     obstacle_found;
    bool     centre_clear;      /* straight ahead confirmed free */
    uint8_t  closest_angle_deg;
    uint16_t closest_mm;
    int16_t  centre_x_mm;       /* lateral position of the obstacle centre */
    uint16_t forward_mm;        /* distance to the nearest point, along y */
    int16_t  left_edge_x_mm;    /* the obstacle's extent, right positive */
    int16_t  right_edge_x_mm;
    uint16_t width_mm;
    uint16_t left_clearance_mm;     /* confirmed free space beside it; 0 = unknown/none */
    uint16_t right_clearance_mm;
} obstacle_profile_t;

void obstacle_profile_build(const obstacle_config_t *cfg, const scan_point_t *points,
                            uint8_t count, obstacle_profile_t *profile);

/* ---- bypass plan ------------------------------------------------------- */

typedef enum {
    PLAN_CONTINUE = 0,
    PLAN_BYPASS_LEFT,
    PLAN_BYPASS_RIGHT,
    PLAN_REVERSE,
    PLAN_STOP
} plan_action_t;

typedef enum {
    PLAN_REASON_OK = 0,
    PLAN_REASON_NO_OBSTACLE,
    PLAN_REASON_INVALID_SCAN,
    PLAN_REASON_NO_CLEARANCE,
    PLAN_REASON_DETOUR_TOO_LARGE,
    PLAN_REASON_TOO_CLOSE
} plan_reason_t;

typedef struct {
    bool    is_turn;            /* true: turn by value degrees (+ = left); false: move value mm */
    int16_t value;
} plan_step_t;

typedef struct {
    plan_action_t action;
    plan_reason_t reason;
    uint8_t       step_count;
    plan_step_t   steps[PLAN_MAX_STEPS];
    uint16_t      return_leg_mm;    /* expected travel back to the line */
} bypass_plan_t;

void obstacle_plan(const obstacle_config_t *cfg, const obstacle_profile_t *profile,
                   bypass_plan_t *plan);

/*
 * A bypass WITHOUT confirmed free space, for a controlled open arena only.  When the
 * sonar sees the obstacle across the whole sweep (the beam is too wide to find its
 * edges) no side can be shown to be clear, so obstacle_plan() stops.  This goes round
 * anyway: it takes the side whose outermost rays read farther (the obstacle seems to
 * end sooner there; a tie goes right), with the sideways distance taken from the
 * scanned width plus the corridor.  Returns false, leaving *plan untouched, when there
 * is no obstacle in the path, it is too close to turn, or the detour exceeds the
 * recovery limit on both sides.  The caller must keep an eye on the sonar while moving.
 */
bool obstacle_blind_plan(const obstacle_config_t *cfg, const scan_point_t *pts, uint8_t n,
                         const obstacle_profile_t *profile, bypass_plan_t *plan);

/* Table-based trigonometry in per-mille, angles in whole degrees 0..180. */
int16_t trig_sin_permille(int16_t deg);
int16_t trig_cos_permille(int16_t deg);

#endif /* OBSTACLE_H */
