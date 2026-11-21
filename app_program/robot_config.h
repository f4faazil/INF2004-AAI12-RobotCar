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

/* Debug aid: wait this long after boot before starting the robot tasks, printing the radio's state once a second, so the
 * USB console is up first and shows whether starting the robot is what breaks it.  0 = start at once (the mission). */
#define ROBOT_START_DELAY_MS    (0U)

/*
 * 1 = build the motor calibration application (motor_cal_task.c) instead of the
 * robot: one task, driven by the START button, that measures the dead zone and
 * speed curve of each motor and checks distance and turn angle.  Needs
 * ROBOT_FIRMWARE 1 and the USB console.  Set back to 0 for the mission.
 */
#define ROBOT_MOTOR_CAL         (0)

/*
 * 1 = build the IMU bring-up application (imu_test_task.c) instead of the robot:
 * checks the accelerometer, finds the forward axis and its sign, and runs the hump
 * detector over a 1 m drive.  Needs ROBOT_FIRMWARE 1 and the USB console.  Takes
 * priority over ROBOT_MOTOR_CAL.  Set back to 0 for the mission.
 */
#define ROBOT_IMU_TEST          (0)

/*
 * 1 = build the ultrasonic and servo bring-up application (obstacle_test_task.c)
 * instead of the robot: distance at known ranges, the no-echo handling, the servo
 * limits and the real scan, profile and bypass plan.  The motors are never driven.
 * Needs ROBOT_FIRMWARE 1 and the USB console.  Takes priority over ROBOT_IMU_TEST
 * and ROBOT_MOTOR_CAL.  Set back to 0 for the mission.
 */
#define ROBOT_OBSTACLE_TEST     (0)

/*
 * 1 = build the open-space obstacle demo (freeroam_task.c) instead of the robot: no
 * line.  The car drives straight, stops at an obstacle, scans, goes round it on the
 * open side, returns to its path and carries on.  The motors DRIVE: keep a hand near
 * the car and 2 m clear ahead.  Needs ROBOT_FIRMWARE 1 and the USB console.  Takes
 * priority over the other bring-up switches.  Set back to 0 for the mission.
 */
#define ROBOT_FREEROAM          (0)
#define ROBOT_IMU_TEST_SKIP_SETUP (0)   /* 1 = IMU test starts at the hump drive (axis and sign from below) */
#define ROBOT_IMU_TEST_HAND     (0)   /* 1 = no motors: push the car over the hump by hand, START ends the run */

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
#define ROBOT_USE_BARCODE_SENSOR (1)
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

/*
 * BENCH TESTING ONLY.  The letter -> movement mapping belongs to the PC and arrives over
 * MQTT (topic <base>/map); the car has no mapping of its own.  Until the Wi-Fi/MQTT link is
 * working, a mapping for bench tests can be loaded at start-up from this text, in the same
 * format as the MQTT message (see barcode_map.h), for example "A=L,Z=U".  An MQTT mapping
 * that arrives later replaces it.  Leave it "" (no mapping) for the real build.
 */
#define ROBOT_BENCH_MAP ""

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
/* Line following uses the CENTRE and RIGHT sensors (a 12 mm pair straddling the centreline).
 * The LEFT sensor is the far one: it reads the barcodes and spots junction branches on the
 * left (ROBOT_USE_BARCODE_SENSOR).  A barcode is executed at the next junction. */
#define ROBOT_SENSOR_MM_LEFT    (-39)       /* IR on GP6: far, barcodes and left branches */
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
#define ROBOT_SENSORS_PERIOD_MS (500U)      /* the per-sensor detail record */
#define ROBOT_US_PERIOD_MS      (70U)       /* HC-SR04 needs >= 60 ms */
#define ROBOT_US_ECHO_TIMEOUT_US (30000U)
/* Geometry measured by Buddy 5: the sensor's front face is 80 mm behind the car's front
 * edge, so a reading of D is only D - 80 mm of clearance for the bumper. */
#define ROBOT_SENSOR_TO_FRONT_MM (80U)
/* 1 = when the scan shows no open side (the box fills the whole sweep), go round anyway
 * toward the side whose outer rays read farther.  The sonar cannot see the sides then, so
 * only use it with nothing beside the track.  0 = stop and fault instead. */
#define ROBOT_OBSTACLE_BLIND    (1)

/*
 * 1 = measurement build for the obstacle-to-stop time (Tribunal 1, test 1: PWM zero within 150 ms).  The firmware prints one "[stop]" line per
 * obstacle stop with the time at each hand-over: the sonar's first reading inside the stop distance, the monitor declaring the obstacle, the
 * Vehicle Controller posting STOP, Motion setting the PWM to zero, and the wheels confirmed still.  0 (the default) compiles all of it out:
 * the mission firmware is byte-for-byte the same.
 */
#ifndef ROBOT_STOP_TIMING
#define ROBOT_STOP_TIMING       (0)
#endif
/* An obstacle ahead nearer than this stops the car and starts the scan.  The scan
 * can only confirm a path of the car's width from about 650 mm (the servo sweep is
 * only 28 degrees each side); see the note on ROBOT_OBSTACLE_CFG. */
#define ROBOT_OBSTACLE_SCAN_MM  (330U)
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
extern const roll_hump_config_t  ROBOT_ROLL_HUMP_CFG;
extern const obstacle_config_t   ROBOT_OBSTACLE_CFG;
extern const vehicle_config_t    ROBOT_VEHICLE_CFG;
extern const safety_limits_t     ROBOT_SAFETY_LIMITS;

/* Servo trim and IMU mounting live with their structures in robot_config.c. */
#define ROBOT_IMU_FORWARD_AXIS  (IMU_AXIS_X)
#define ROBOT_IMU_FORWARD_SIGN  (-1)         /* measured on the car: X reads +ve with the nose up, so pitch needs -1 */
#define ROBOT_IMU_CAL_SAMPLES   (50U)
#define ROBOT_IMU_CAL_SPREAD_MG (250)       /* floor vibration is about +-50 mg at rest */
#define ROBOT_IMU_FILTER_DIV    (8)         /* accelerometer low-pass: 1/8 per 20 ms sample */
/*
 * Provisional lift/overturn policy: at least 60 degrees from the calibrated
 * attitude (cosine <= 0.5) or at most 300 mg for 300 ms. Tune on the car;
 * an accelerometer cannot distinguish a level, stationary car from one
 * lifted evenly.
 */
#define ROBOT_LIFT_TILT_COS_PERMILLE (500U)
#define ROBOT_LIFT_FREEFALL_MG       (300U)
#define ROBOT_LIFT_HOLD_MS           (300U)

/*
 * After a lift the car re-arms by itself, so it can be tested again without a reboot, once it is back on the floor:
 * level to within 15 degrees (cosine 966), at 1 g within 120 mg, and steady (no axis moves more than 60 mg between 20 ms
 * samples) for 2 s in a row, and not sooner than 1 s after the lift was detected.  Re-arming only lets START work again.
 */
#define ROBOT_LIFT_REARM_COS_PERMILLE (966U)
#define ROBOT_LIFT_REARM_MAG_TOL_MG   (120U)
#define ROBOT_LIFT_REARM_STEP_MG      (60U)
#define ROBOT_LIFT_REARM_HOLD_MS      (2000U)
#define ROBOT_LIFT_MIN_LATCH_MS       (1000U)

#endif /* ROBOT_CONFIG_H */
