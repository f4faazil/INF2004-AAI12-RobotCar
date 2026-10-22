/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * robot_config.h
 * Every pin, dimension, gain and threshold of the robot firmware in one
 * place.  Values marked PLACEHOLDER are starting points that MUST be measured
 * or tuned on the car; they are not results.  The pure modules take their
 * settings from the structures built in robot_config.c, so the host tests
 * and the firmware share the same logic with different numbers.
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "imu.h"
#include "line_nav.h"
#include "motion.h"
#include "obstacle.h"
#include "safety.h"
#include "vehicle.h"

/*
 * 1 = build the robot application.  0 = build the lecturer's original
 * producer/consumer demo instead (app_main.c).  The two cannot coexist: both
 * provide usermain().
 */
#define ROBOT_FIRMWARE          (1)

/* Print a one-line status once a second on the console. */
#define ROBOT_DIAG_PRINT        (1)

/* ---- pins (Robo Pico GPIO numbers; Tribunal Version B/C map) ------------- */

/* Motors: M1 = right, M2 = left.  A drives forward, B backward. */
#define ROBOT_PIN_MOTOR_R_FWD   (8U)
#define ROBOT_PIN_MOTOR_R_REV   (9U)
#define ROBOT_PIN_MOTOR_L_FWD   (10U)
#define ROBOT_PIN_MOTOR_L_REV   (11U)
#define ROBOT_MOTOR_L_INVERT    (0)         /* swap if a wheel runs backwards */
#define ROBOT_MOTOR_R_INVERT    (0)
#define ROBOT_PWM_HZ            (20000U)    /* above hearing, below driver limit */

/* Scanning servo and ultrasonic sensor. */
#define ROBOT_PIN_SERVO         (12U)       /* PWM slice 6, channel A */
#define ROBOT_PIN_TRIG          (16U)
#define ROBOT_PIN_ECHO          (17U)       /* 5 V: needs the 1k + 2k divider */

/* IR sensors, read as ADC0..2 (Tribunal: right GP26, centre GP27, left GP28). */
#define ROBOT_ADC_CH_RIGHT      (0U)
#define ROBOT_ADC_CH_CENTRE     (1U)
#define ROBOT_ADC_CH_LEFT       (2U)

/* IMU on I2C1. */
#define ROBOT_PIN_I2C_SDA       (2U)
#define ROBOT_PIN_I2C_SCL       (3U)
#define ROBOT_I2C_HZ            (100000U)

/*
 * Wheel encoders.  The Tribunal plans GP0/GP1 for the left encoder, but GP0/GP1
 * are the UART console in the default (qualified) UART build.  With the USB
 * console they are free; with the UART console the left encoder falls back to
 * GP6/GP7 so it cannot corrupt the console.  Override with -D if the wiring
 * differs.
 */
#ifndef ROBOT_PIN_ENC_L_A
#if defined(TM_CONSOLE_USB_CDC) && (TM_CONSOLE_USB_CDC)
#define ROBOT_PIN_ENC_L_A       (0U)
#define ROBOT_PIN_ENC_L_B       (1U)
#else
#define ROBOT_PIN_ENC_L_A       (6U)
#define ROBOT_PIN_ENC_L_B       (7U)
#endif
#endif
#define ROBOT_PIN_ENC_R_A       (4U)
#define ROBOT_PIN_ENC_R_B       (5U)
#define ROBOT_ENC_L_INVERT      (0)
#define ROBOT_ENC_R_INVERT      (0)
/* 1 = only channel A is wired: direction comes from the commanded PWM sign. */
#define ROBOT_ENCODER_SINGLE_CHANNEL (0)

/* Robo Pico user buttons (active low). */
#define ROBOT_PIN_BTN_START     (20U)
#define ROBOT_PIN_BTN_STOP      (21U)

/*
 * IR sensor layout - MEASURE THIS ON THE CAR.  Lateral position of each IR
 * sensor from the car's centreline, in millimetres, right positive.  The roles
 * follow from the positions: a sensor within ROBOT_FAR_SENSOR_MM of the
 * centreline tracks the line; a sensor farther out reads barcodes and spots
 * junction branches.  The default is the layout the closed-loop simulation
 * (tests/host/test_follow_sim.c) shows to work: a close pair straddling the
 * 18 mm line plus one far sensor over the barcode strip (29-49 mm from the line
 * centre).  A symmetric bar wide enough to read a barcode cannot track the
 * line, and a tracking pair wider than about 16 mm can lose it between the
 * sensors, so both of those layouts need a hardware change.
 *
 * PLACEHOLDER: the positions below are a proposal, not the car's geometry.
 */
#define ROBOT_SENSOR_MM_LEFT    (-6)        /* IR on GP28 */
#define ROBOT_SENSOR_MM_CENTRE  (6)         /* IR on GP27 */
#define ROBOT_SENSOR_MM_RIGHT   (39)        /* IR on GP26 */
#define ROBOT_TRACK_REF_MM      (6)         /* spacing that maps to +/-1000 in the position */
#define ROBOT_FAR_SENSOR_MM     (25)

/* ---- timing ------------------------------------------------------------ */

#define ROBOT_ADC_FRAME_US      (1000U)     /* one three-channel frame per ms */
#define ROBOT_MOTION_PERIOD_MS  (5U)
#define ROBOT_CONTROL_PERIOD_MS (10U)
#define ROBOT_IMU_PERIOD_MS     (20U)
#define ROBOT_STATUS_PERIOD_MS  (100U)
#define ROBOT_HEARTBEAT_MS      (1000U)
#define ROBOT_US_PERIOD_MS      (70U)       /* HC-SR04 needs >= 60 ms */
#define ROBOT_US_ECHO_TIMEOUT_US (30000U)
#define ROBOT_IR_RING_FRAMES    (256U)

/*
 * Task priorities (1 = highest).  The control path must outrank the kernel's
 * own service tasks: the USB console task runs at 3 and the CYW43439 radio
 * task at 4 (it polls every 2 ms), so Motion, Line, Vehicle and Obstacle all
 * sit at 3 or better.  Telemetry is below both, as the Tribunal requires, but
 * low priority alone is not proof: measure control-task timing with Wi-Fi and
 * USB active (the `ovr=` and `us(...)` figures in the status line).
 */
#define ROBOT_PRI_MOTION        (1)
#define ROBOT_PRI_LINE          (2)
#define ROBOT_PRI_VEHICLE       (3)
#define ROBOT_PRI_OBSTACLE      (3)
#define ROBOT_PRI_IMU           (5)
#define ROBOT_PRI_NET           (10)
#define ROBOT_PRI_DIAG          (11)
#define ROBOT_STACK_BYTES       (2048)

/* ---- network ------------------------------------------------------------- */

/* Topic base; the laptop subscribes to <base>/# and publishes to <base>/cmd. */
#define ROBOT_MQTT_BASE         "aai12/robot"
#define ROBOT_MQTT_CLIENT_ID    "aai12-robot"
#define ROBOT_ALLOW_REMOTE_STOP (1)         /* team decision pending (Tribunal A1) */

/* ---- structures consumed by the portable modules ----------------------- */

extern const motion_config_t     ROBOT_MOTION_CFG;
extern const line_nav_config_t   ROBOT_LINE_NAV_CFG;
extern const hump_config_t       ROBOT_HUMP_CFG;
extern const obstacle_config_t   ROBOT_OBSTACLE_CFG;
extern const vehicle_config_t    ROBOT_VEHICLE_CFG;
extern const safety_limits_t     ROBOT_SAFETY_LIMITS;
extern const nav_map_entry_t     ROBOT_BARCODE_MAP[];
extern const uint8_t             ROBOT_BARCODE_MAP_LEN;

/* Servo trim and IMU mounting live with their structures in robot_config.c. */
#define ROBOT_IMU_FORWARD_AXIS  (IMU_AXIS_X)
#define ROBOT_IMU_FORWARD_SIGN  (1)
#define ROBOT_IMU_CAL_SAMPLES   (50U)
#define ROBOT_IMU_CAL_SPREAD_MG (80)

#endif /* ROBOT_CONFIG_H */
