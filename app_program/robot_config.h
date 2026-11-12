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

/*
 * 1 = build the motor calibration application (motor_cal_task.c) instead of the
 * robot: one task, driven by the START button, that measures the dead zone and
 * speed curve of each motor and checks distance and turn angle.  Needs
 * ROBOT_FIRMWARE 1 and the USB console.  Set back to 0 for the mission.
 */
#define ROBOT_MOTOR_CAL         (0)

/* Print a one-line status once a second on the console. */
#define ROBOT_DIAG_PRINT        (1)

/* ---- pins (Robo Pico GPIO numbers; Tribunal Version B/C map) ------------- */

/* Motors, per Buddy 2's wiring note: M1 = left, M2 = right.  A drives forward, B backward. */
#define ROBOT_PIN_MOTOR_L_FWD   (8U)
#define ROBOT_PIN_MOTOR_L_REV   (9U)
#define ROBOT_PIN_MOTOR_R_FWD   (10U)
#define ROBOT_PIN_MOTOR_R_REV   (11U)
#define ROBOT_MOTOR_L_INVERT    (0)         /* swap if a wheel runs backwards */
#define ROBOT_MOTOR_R_INVERT    (0)
#define ROBOT_PWM_HZ            (20000U)    /* above hearing, below driver limit */

/* Scanning servo and ultrasonic sensor. */
#define ROBOT_PIN_SERVO         (12U)       /* PWM slice 6, channel A */
#define ROBOT_PIN_TRIG          (4U)
#define ROBOT_PIN_ECHO          (5U)        /* 5 V: needs the 1k + 2k divider */

/*
 * IR sensors: the digital DO output of each module (AO is not connected), so
 * every sensor reads plain HIGH/LOW.  The module's trimmer sets the switching
 * point.  ROBOT_IR_DARK_LEVEL is the pin level over black (1 = DO high on
 * black); PLACEHOLDER until checked on the car.  The HAL turns each level into
 * one of the two counts below so line_nav sees the same 12-bit-style frames as
 * before: calibration then measures a span of BLACK - WHITE on every channel.
 */
#define ROBOT_PIN_IR_LEFT       (6U)
#define ROBOT_PIN_IR_CENTRE     (17U)
#define ROBOT_PIN_IR_RIGHT      (3U)
#define ROBOT_IR_DARK_LEVEL     (1U)
/*
 * 1 = the far sensor reads barcodes and spots junction branches; 0 = it is ignored
 * (line following only: no barcodes, no junctions).  Only the near pair steers.
 */
#define ROBOT_USE_BARCODE_SENSOR (0)
#define ROBOT_IR_COUNTS_WHITE   (150U)
#define ROBOT_IR_COUNTS_BLACK   (4000U)
/*
 * Sensors with fixed white/black readings (bit 0 left, 1 centre, 2 right), trusted
 * from start-up with no sweep: right for digital DO sensors.  0 = learn them all
 * from the calibration sweep, which fails for a sensor the car never moves across
 * the line with (and so cannot catch a dead one either way).
 */
#define ROBOT_IR_FIXED_MASK     (0x07U)

/*
 * 1 = driving needs a working ultrasonic sensor (a fault stops the car).  0 only
 * for testing the IR sensors on their own with the ultrasonic sensor absent: the
 * car then does NOT stop for obstacles.  Set it back to 1 for the real mission.
 */
#define ROBOT_REQUIRE_ULTRASONIC (0)

/* IMU on I2C1. */
#define ROBOT_PIN_I2C_SDA       (26U)
#define ROBOT_PIN_I2C_SCL       (27U)
#define ROBOT_I2C_HZ            (100000U)

/*
 * Wheel encoders, per Buddy 2's wiring note: two-channel quadrature.  Left on
 * GP7 (A) / GP28 (B), right on GP0 (A) / GP1 (B).  GP28 is also the VBAT-sense
 * pin: its back-of-board jumper must be open.  GP0/GP1 are the UART0 pins, so the
 * console must be USB (build with CONSOLE=usb_cdc).  Buddy 2's note gives the
 * count sign in their firmware's convention; this decoder reads A/B the other
 * way round, and the first run showed both signs reversed (the left wheel went
 * backward while the speed read positive), so the flags below are the reverse
 * of that note.  Verified by the first calibration turn: see docs.
 */
#define ROBOT_PIN_ENC_L_A       (7U)
#define ROBOT_PIN_ENC_L_B       (28U)
#define ROBOT_PIN_ENC_R_A       (0U)
#define ROBOT_PIN_ENC_R_B       (1U)
#define ROBOT_ENC_L_INVERT      (0)
#define ROBOT_ENC_R_INVERT      (1)
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
/* Line following uses the CENTRE and RIGHT sensors (a 12 mm pair straddling the centreline);
 * the LEFT sensor is the far one, unused while ROBOT_USE_BARCODE_SENSOR is 0. */
#define ROBOT_SENSOR_MM_LEFT    (-39)       /* IR on GP6: far, unused for now */
#define ROBOT_SENSOR_MM_CENTRE  (-6)        /* IR on GP17: tracks the line */
#define ROBOT_SENSOR_MM_RIGHT   (6)         /* IR on GP3: tracks the line */
#define ROBOT_TRACK_REF_MM      (6)         /* spacing that maps to +/-1000 in the position */
#define ROBOT_FAR_SENSOR_MM     (25)

/*
 * Track geometry from "Barcode & Line Specification.pdf".  The symbol is Code 39,
 * "*X*": 47 narrow units (3 characters x 15 + 2 gaps), so 141 mm is 3 mm per
 * narrow bar and 9 mm per wide bar.  A far sensor has to land on the strip:
 * ROBOT_BARCODE_NEAR_MM from the line centre, to the strip's far edge (the
 * strip height is not in the specification: measure it on the track).
 */
#define ROBOT_LINE_WIDTH_MM     (18U)
#define ROBOT_BARCODE_GAP_MM    (20U)       /* line edge to the barcode's near edge */
#define ROBOT_BARCODE_NARROW_MM (3U)
#define ROBOT_BARCODE_WIDE_MM   (9U)
#define ROBOT_BARCODE_NEAR_MM   ((ROBOT_LINE_WIDTH_MM / 2U) + ROBOT_BARCODE_GAP_MM)   /* 29 */

/* ---- timing ------------------------------------------------------------ */

#define ROBOT_ADC_FRAME_US      (1000U)     /* one three-channel IR frame per ms */
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
