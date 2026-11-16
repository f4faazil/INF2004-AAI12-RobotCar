/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * robot_config.c
 * Default settings for the portable modules.  See robot_config.h: anything
 * marked PLACEHOLDER must be measured or tuned on the car.
 */
#include "robot_config.h"

#include "imu.h"
#include "pid.h"
#include "ultrasonic.h"

/*
 * Letter -> command table, as decided by the team: A turns left, F turns
 * right, Z makes a U-turn.  A turn runs only when the car reaches a junction;
 * a junction with no stored instruction goes straight, so there is no letter
 * for straight.  A decoded letter that is not listed is rejected and reported,
 * never guessed at.
 */
#define ROBOT_BARCODE_MAP_COUNT (3U)
const nav_map_entry_t ROBOT_BARCODE_MAP[ROBOT_BARCODE_MAP_COUNT] = {
    { 'A', NAV_CMD_LEFT },
    { 'F', NAV_CMD_RIGHT },
    { 'Z', NAV_CMD_UTURN }
};
const uint8_t ROBOT_BARCODE_MAP_LEN = (uint8_t)(sizeof(ROBOT_BARCODE_MAP) /
                                                sizeof(ROBOT_BARCODE_MAP[0]));

/*
 * Motion.  Calibrated on the car with the standalone motor test (three 100 cm
 * rolls: left 118.63 counts/cm, right 117.67 counts/cm).  counts_per_rev is
 * fitted so that counts_per_rev / (pi x 65 mm) matches the mean of the two
 * wheels (11.77 counts/mm): the per-wheel revolution counts (2583 / 2552) are
 * older single-turn estimates and do not reconcile with a 65 mm tyre, so they
 * are NOT used here.  Still to measure: the real tyre diameter (calipers), the
 * track width, and the PWM dead zone of each wheel.
 *
 * Gains converted from the standalone test (35 RPM = 119 mm/s at D = 65 mm):
 *   Kp 0.0025 duty/RPM = 0.74 per-mille per mm/s -> 188/256
 *   Ki 0.0060 duty/(RPM s) = 1.76 per-mille per (mm/s s) -> 451/256
 *   feed-forward 0.20 (left) and 0.23 (right) of full duty at 119 mm/s:
 *   1.68 and 1.93 per-mille per mm/s.  These are interim values: they assume
 *   the PWM-to-speed line passes through zero.  Run the motor calibration
 *   (ROBOT_MOTOR_CAL, docs/MOTOR_CALIBRATION.md) and paste in the fitted
 *   feed-forward and the dead-zone offsets it prints.
 */
const motion_config_t ROBOT_MOTION_CFG = {
    .counts_per_rev = 2413U,            /* fitted: mean of 2422 (left) and 2403 (right) */
    .wheel_diameter_mm = 65U,           /* nominal: recheck the fitted tyre */
    .track_width_mm = 115U,             /* from the 360 turn tests: 150 -> ~450 deg, 120 -> ~370 deg overshoot; +/-10 deg by eye */
    .kp = 188,
    .ki = 451,
    .kd = 0,
    .ff_num = 134,                      /* left: 1.34 per-mille per mm/s (fit, 7 points) */
    .ff_den = 100,
    .ff_right_num = 131,                /* right: 1.31 */
    .ff_right_den = 100,
    .pwm_offset_left = 19,              /* fit intercept; below 30 mm/s the real curve is steeper */
    .pwm_offset_right = 16,
    .pwm_limit = 500,                   /* the standalone test capped PWM at 50 % */
    .balance_mm_s_per_mm = 2,
    .default_lease_ms = 50U,            /* Tribunal: 50 ms command lease */
    .max_speed_mm_s = 300U,
    .creep_speed_mm_s = 40U,
    .turn_speed_mm_s = 120U,
    .decel_per_s = 6U,
    .speed_window_ms = 20U,
    .stopped_speed_mm_s = 8U,
    .stopped_confirm_ms = 40U,
    .encoder_fault_ms = 300U,
    .encoder_fault_pwm = 300U
};

/* Sensor roles from the measured positions (robot_config.h). */
#define MM_ABS(mm)          (((mm) < 0) ? -(mm) : (mm))
#define IS_FAR(mm)          (MM_ABS(mm) >= ROBOT_FAR_SENSOR_MM)
#define POS_OF(mm)          ((int16_t)((((mm) * 1000) / ROBOT_TRACK_REF_MM)))
#define TRACK_BIT(ch, mm)   (IS_FAR(mm) ? 0 : (1 << (ch)))
#define LANE_BIT(ch, mm)    ((ROBOT_USE_BARCODE_SENSOR && IS_FAR(mm)) ? (1 << (ch)) : 0)

/*
 * Line following and barcodes.  junction_min_mm: the widest barcode bar is
 * about 9 mm and the crossing line 18 mm, so 13 mm separates them.  The steering
 * gain scales with the reference spacing (about 13 mm/s per mm of it at
 * 150 mm/s); PLACEHOLDER, to be tuned on the track.
 */
const line_nav_config_t ROBOT_LINE_NAV_CFG = {
    .lost_frames = 5U,                  /* 50 ms without a line */
    .found_frames = 3U,
    .junction_min_mm = (ROBOT_BARCODE_WIDE_MM + ROBOT_LINE_WIDTH_MM) / 2U,     /* 13 */
    .junction_merge_ms = 30U,
    .barcode_quiet_ms = 250U,
    .instruction_ttl_ms = 20000U,       /* PLACEHOLDER: stale instruction expires */
    .hump_suppress_max_ms = 1000U,      /* Tribunal: at most about 1 s */
    .hump_fresh_ms = 100U,
    .stale_ms = 50U,
    .steer_kp_mm_s = 13 * ROBOT_TRACK_REF_MM,
    .steer_td_ms = 30U,                 /* light derivative: the position is smoothed, see smooth_shift */
    .steer_limit_mm_s = 150,
    .map = ROBOT_BARCODE_MAP,
    .map_len = ROBOT_BARCODE_MAP_COUNT,
    .channel_pos_permille = {
        [LINE_CHANNEL_LEFT] = POS_OF(ROBOT_SENSOR_MM_LEFT),
        [LINE_CHANNEL_CENTRE] = POS_OF(ROBOT_SENSOR_MM_CENTRE),
        [LINE_CHANNEL_RIGHT] = POS_OF(ROBOT_SENSOR_MM_RIGHT)
    },
    .track_mask = (uint8_t)(TRACK_BIT(LINE_CHANNEL_LEFT, ROBOT_SENSOR_MM_LEFT) |
                            TRACK_BIT(LINE_CHANNEL_CENTRE, ROBOT_SENSOR_MM_CENTRE) |
                            TRACK_BIT(LINE_CHANNEL_RIGHT, ROBOT_SENSOR_MM_RIGHT)),
    .lane_mask = (uint8_t)(LANE_BIT(LINE_CHANNEL_LEFT, ROBOT_SENSOR_MM_LEFT) |
                           LANE_BIT(LINE_CHANNEL_RIGHT, ROBOT_SENSOR_MM_RIGHT)),
    .junction_mask = (uint8_t)(LANE_BIT(LINE_CHANNEL_LEFT, ROBOT_SENSOR_MM_LEFT) |
                               LANE_BIT(LINE_CHANNEL_RIGHT, ROBOT_SENSOR_MM_RIGHT)),
    .fixed_white = ROBOT_IR_COUNTS_WHITE,       /* DO sensors: nothing for a sweep to learn */
    .fixed_black = ROBOT_IR_COUNTS_BLACK,
    .fixed_mask = ROBOT_IR_FIXED_MASK,
    .smooth_shift = 2U                          /* digital sensors: gain 1/4 per 10 ms step, about 35 ms */
};

/*
 * Hump detection by roll: one wheel on the hump rolls the car, and the filtered
 * roll shows one smooth bump.  Measured driving at 120 mm/s over the 20 mm demo
 * hump (about 190 mm long): peak roll 278, 261, 278 (right wheel) and 238, 269,
 * 258, 254 (left wheel), mean 262 per-mille.  The lever 76 mm is fitted so that
 * mean reads 20 mm (the seven runs read 18.1 to 21.1 mm).  It is a fit to this
 * one hump: an 11 mm hump gave about 113 and would read 8.6 mm, so the roll is
 * not proportional to height.  Refit it if the board moves or the hump changes.
 */
const roll_hump_config_t ROBOT_ROLL_HUMP_CFG = {
    .enter_permille = 40,               /* flat running stays within about +-15 */
    .exit_permille = 25,
    .enter_ms = 60U,
    .exit_ms = 200U,                    /* a short dip at the foot of the ramp must not end it */
    .lever_mm = 76U,
    .min_length_mm = 60U,
    .max_length_mm = 400U,
    .min_height_mm = 6
};

/* The older pitch-based detector: pitch is biased by about 100 per-mille while
 * driving, so the robot no longer uses it. */
const hump_config_t ROBOT_HUMP_CFG = {
    .up_threshold_permille = 50,
    .down_threshold_permille = 50,
    .flat_band_permille = 25,
    .min_duration_ms = 60U,
    .max_duration_ms = 5000U,
    .min_height_mm = 4,
    .net_elevation_percent = 40U,
    .disturbance_mg = 300
};

/*
 * Scanning and bypass.  The servo is MEASURED (Buddy 5, on the car): 1500 us is
 * straight ahead, 1100 us is 32 degrees to the right, 1850 us is 28 degrees to
 * the left, so 12.5 us per degree on both sides.  Further left the horn hits the
 * chassis, so the sweep is about 60 degrees, not 180.  The pulse range below is
 * that line extended to 0 and 180 degrees (375 us at 0, 2625 us at 180); only the
 * software limits 58 to 118 degrees are ever commanded.  PLACEHOLDER: robot size
 * and margins.  The rays are 14 degrees apart (62, 76, 90, 104, 118): the
 * simulation shows a 100 mm object 60 mm off-centre hides between 30-degree rays.
 */
const obstacle_config_t ROBOT_OBSTACLE_CFG = {
    .servo = {
        .min_pulse_us = 375U,           /* 1500 us - 90 deg x 12.5 us/deg */
        .max_pulse_us = 2625U,          /* 1500 us + 90 deg x 12.5 us/deg */
        .trim_deg = 0,                  /* measured: 1500 us is straight ahead */
        .invert = false,
        .min_angle_deg = 58U,           /* 1100 us: 32 degrees right of centre */
        .max_angle_deg = 118U,          /* 1850 us: 28 degrees left, the chassis blocks more */
        .settle_base_ms = 60U,
        .settle_ms_per_deg = 4U
    },
    .coarse_min_deg = 62U,
    .coarse_max_deg = 118U,
    .coarse_step_deg = 14U,
    .fine_step_deg = 5U,
    .detect_range_mm = 600U,
    .samples_per_point = 3U,
    .measure_timeout_ms = 120U,
    .no_echo_is_clear = false,          /* Tribunal: no echo is not free space */
    .left_is_high_angle = true,         /* measured: a longer pulse points left */
    .robot_width_mm = 140U,             /* PLACEHOLDER */
    .robot_length_mm = 200U,            /* PLACEHOLDER */
    .margin_mm = 30U,
    .assumed_depth_mm = 150U,
    .min_turn_distance_mm = 120U,
    .recovery_limit_mm = 500U           /* Tribunal: 0.5 m */
};

const vehicle_config_t ROBOT_VEHICLE_CFG = {
    .cruise_mm_s = 150,                 /* course range is 0.10 - 0.20 m/s */
    .curve_mm_s = 90,
    .hump_mm_s = 110,
    .creep_mm_s = 60,
    .junction_advance_mm = 70,          /* PLACEHOLDER: sensor bar to axle, less crossing offset */
    .turn_deg = 90,
    .uturn_deg = 180,
    .cal_sweep_deg = { 25, -50, 25 },
    .search_sweep_deg = { 30, -60, 90, -120 },
    .search_sweep_count = 4U,
    .boot_ms = 500U,
    .stop_confirm_timeout_ms = 1000U,
    .scan_timeout_ms = 20000U,
    .reacquire_timeout_ms = 2000U,
    .reacquire_limit_mm = 300,
    .recovery_time_limit_ms = 5000U,    /* Tribunal: 5 s ... */
    .recovery_distance_limit_mm = 500,  /* ... or 0.5 m, whichever comes first */
    .obstacle_blank_ms = 2000U,
    .max_reverse_attempts = 1U,
    .found_cycles = 3U,
    .cmd_timeout_base_ms = 2000U,
    .cmd_timeout_per_mm_ms = 30U,
    .cmd_timeout_per_deg_ms = 30U,
    .allow_remote_stop = (ROBOT_ALLOW_REMOTE_STOP != 0)
};

const safety_limits_t ROBOT_SAFETY_LIMITS = {
    .motion_stall_ms = 50U,
    .imu_stale_ms = 200U,
    .ultrasonic_stale_ms = 500U
};
