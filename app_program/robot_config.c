/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * robot_config.c
 * Default settings for the portable modules.  See robot_config.h: anything
 * marked PLACEHOLDER must be measured or tuned on the car.
 */
#include "robot_config.h"

#include <stddef.h>

#include "imu.h"
#include "pid.h"
#include "ultrasonic.h"

/*
 * There is no built-in letter -> movement mapping.  The PC sends it over MQTT
 * (topic <base>/map, see barcode_map.h) and the Line task applies it; until one
 * arrives every decoded letter is rejected and the car goes straight at a junction.
 */

/*
 * Motion.  PLACEHOLDER: wheel diameter, track width and counts per revolution
 * must be measured (roll the car 1 m and count; spin a wheel 10 turns and
 * count), and the PID gains tuned from a speed-step response.
 */
const motion_config_t ROBOT_MOTION_CFG = {
    .counts_per_rev = 1440U,            /* PLACEHOLDER: as counted (4x) per wheel turn */
    .wheel_diameter_mm = 65U,           /* PLACEHOLDER */
    .track_width_mm = 150U,             /* PLACEHOLDER: wheel contact point to contact point */
    .kp = PID_GAIN_ONE,                 /* PLACEHOLDER: 1.0 PWM per-mille per mm/s of error */
    .ki = 2 * PID_GAIN_ONE,
    .kd = 0,
    .ff_num = 5,                        /* PLACEHOLDER: PWM = speed x 2.5 (about 400 mm/s at full) */
    .ff_den = 2,
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
    .barcode_quiet_ms = 600U,
    .instruction_ttl_ms = 20000U,       /* PLACEHOLDER: stale instruction expires */
    .hump_suppress_max_ms = 1000U,      /* Tribunal: at most about 1 s */
    .hump_fresh_ms = 100U,
    .stale_ms = 50U,
    .steer_kp_mm_s = 13 * ROBOT_TRACK_REF_MM,
    .steer_td_ms = 30U,                 /* light derivative: the position is smoothed, see smooth_shift */
    .steer_limit_mm_s = 150,
    .map = NULL,                        /* set at run time over MQTT: line_nav_set_map() */
    .map_len = 0U,
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
    .smooth_shift = 2U,                         /* digital sensors: gain 1/4 per 10 ms step, about 35 ms */
    .barcode_bars = 15U,                        /* "*X*" always has 15 bars: after them the next black is the junction */
    .barcode_min_bar = 24U                      /* quarter encoder counts: shorter is a glitch, not a bar */
};

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
 * Scanning and bypass.  PLACEHOLDER: servo pulse range/trim, robot size and
 * margins.  The coarse step is 15 degrees: the simulation shows a 100 mm
 * object 60 mm off-centre hides between 30-degree rays.
 */
const obstacle_config_t ROBOT_OBSTACLE_CFG = {
    .servo = {
        .min_pulse_us = 500U,
        .max_pulse_us = 2500U,
        .trim_deg = 0,                  /* PLACEHOLDER: horn alignment (bench: not aligned) */
        .invert = false,
        .min_angle_deg = 20U,
        .max_angle_deg = 160U,
        .settle_base_ms = 60U,
        .settle_ms_per_deg = 4U
    },
    .coarse_min_deg = 30U,
    .coarse_max_deg = 150U,
    .coarse_step_deg = 15U,
    .fine_step_deg = 5U,
    .detect_range_mm = 600U,
    .samples_per_point = 3U,
    .measure_timeout_ms = 120U,
    .no_echo_is_clear = false,          /* Tribunal: no echo is not free space */
    .left_is_high_angle = true,         /* PLACEHOLDER: depends on the horn fitting */
    .robot_width_mm = 140U,             /* PLACEHOLDER */
    .robot_length_mm = 200U,            /* PLACEHOLDER */
    .margin_mm = 30U,
    .assumed_depth_mm = 150U,
    .min_turn_distance_mm = 120U,
    .recovery_limit_mm = 500U           /* Tribunal: 0.5 m */
};

const vehicle_config_t ROBOT_VEHICLE_CFG = {
    .cruise_mm_s = 80,                  /* DEBUG: slow. Course range is 0.10 - 0.20 m/s; restore 150 for the mission */
    .curve_mm_s = 60,                   /* DEBUG: slow (was 90) */
    .hump_mm_s = 70,                    /* DEBUG: must stay below cruise (was 110) */
    .creep_mm_s = 55,
    .approach_mm_s = 55,                /* after a barcode, until its junction: 18 mm of arm lasts 0.3 s */
    .junction_advance_mm = 70,          /* PLACEHOLDER: sensor bar to axle, less crossing offset */
    .turn_deg = 90,
    .uturn_deg = -180,                  /* U-turn towards the right */
    .turn_spin_mm_s = 60,               /* slow enough to stop on an 18 mm line */
    .turn_spin_timeout_ms = 10000U,
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
