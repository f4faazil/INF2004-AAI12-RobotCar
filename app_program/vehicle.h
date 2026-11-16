/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * vehicle.h
 * The Vehicle Controller: mission state and coordination.  It is the only
 * module that sends movement commands to Motion Control; line following,
 * barcode navigation and obstacle avoidance ask it, never the wheels.
 *
 *   BOOT -> IDLE -> CALIBRATE -> FOLLOW <-> junction manoeuvres
 *                                  |  \-> LINE LOST -> SEARCH
 *                                  \----> OBSTACLE: stop, confirm stopped, scan,
 *                                         bypass, recover the line, realign
 *   any state --stop fault--> FAULT        operator stop -> STOPPED
 *
 * Rules from the Tribunal enforced here:
 *  - a decoded barcode instruction is stored by the line module and executed
 *    at the next JUNCTION, exactly once;
 *  - the obstacle scan starts only after Motion confirms the car has stopped;
 *  - an invalid scan or no safe route keeps the car stopped with a fault;
 *  - line recovery after a bypass is limited to 5 s or 0.5 m, whichever comes
 *    first;
 *  - STOP-class faults stop the car at once and latch until the operator
 *    clears them;
 *  - remote commands cannot actuate the car: START, CALIBRATE and
 *    CLEAR_FAULTS are refused while the mission runs, and no remote command
 *    drives the wheels at all.
 *
 * Pure C: step() takes a snapshot of every input and returns what to do.
 */
#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdbool.h>
#include <stdint.h>

#include "fault_mask.h"
#include "line_nav.h"
#include "motion.h"
#include "obstacle.h"

/* The order matches STATE_NAMES in tools/robot_telemetry.py and fits 4 bits. */
typedef enum {
    VC_BOOT = 0,
    VC_IDLE,
    VC_CALIBRATE,
    VC_FOLLOW,
    VC_JUNCTION_ADVANCE,
    VC_JUNCTION_TURN,
    VC_REACQUIRE,
    VC_SEARCH_LINE,
    VC_OBSTACLE_STOP,
    VC_OBSTACLE_SCAN,
    VC_BYPASS,
    VC_REVERSE,
    VC_RECOVER_LINE,
    VC_STOPPED,
    VC_FAULT,
    VC_DONE                 /* reserved: no mission-complete condition is defined yet */
} vehicle_state_t;

#define VC_REMOTE_STOP          (1U << 0)
#define VC_REMOTE_START         (1U << 1)
#define VC_REMOTE_CALIBRATE     (1U << 2)
#define VC_REMOTE_CLEAR_FAULTS  (1U << 3)

#define VC_ACT_LINE_REQUIRED    (1U << 0)   /* for the safety monitor */
#define VC_ACT_DRIVING          (1U << 1)
#define VC_ACT_OBSTACLE_WATCH   (1U << 2)

#define VC_MAX_EVENTS           (4U)

typedef struct {
    int16_t  cruise_mm_s;
    int16_t  curve_mm_s;                /* slowest line-following speed */
    int16_t  hump_mm_s;
    int16_t  creep_mm_s;                /* reacquiring / recovering a line */
    int16_t  junction_advance_mm;       /* sensor bar to wheel axle, less the crossing offset */
    int16_t  turn_deg;                  /* a left/right instruction */
    int16_t  uturn_deg;
    int16_t  cal_sweep_deg[3];
    int16_t  search_sweep_deg[4];
    uint8_t  search_sweep_count;
    uint32_t boot_ms;
    uint32_t stop_confirm_timeout_ms;   /* Motion must report stopped within this */
    uint32_t scan_timeout_ms;
    uint32_t reacquire_timeout_ms;
    int16_t  reacquire_limit_mm;
    uint32_t recovery_time_limit_ms;    /* 5 s */
    int16_t  recovery_distance_limit_mm;/* 0.5 m */
    uint32_t obstacle_blank_ms;         /* ignore a cleared obstacle for a while */
    uint8_t  max_reverse_attempts;
    uint8_t  found_cycles;              /* consecutive visible cycles = line found */
    uint32_t cmd_timeout_base_ms;       /* finite manoeuvre time allowance */
    uint32_t cmd_timeout_per_mm_ms;
    uint32_t cmd_timeout_per_deg_ms;
    bool     allow_remote_stop;         /* emergency STOP over MQTT: team decision pending */
} vehicle_config_t;

typedef struct {
    uint32_t         now_ms;
    bool             start_pressed;
    bool             stop_pressed;
    uint8_t          remote;            /* VC_REMOTE_* requests accepted this cycle */

    line_nav_out_t   line;
    bool             instruction_valid; /* an unexpired barcode instruction is waiting */
    nav_instruction_t instruction;
    bool             calibration_done;  /* answer to calibrate_end */
    bool             calibration_ok;

    motion_status_t  motion;
    bool             obstacle_present;  /* forward monitor */
    bool             scan_done;
    bypass_plan_t    plan;
    bool             hump_in_progress;
    fault_bits_t     faults;
} vehicle_in_t;

typedef struct {
    uint8_t  code;                      /* TEL_EVT_* */
    uint16_t arg;
} vehicle_event_t;

typedef struct {
    motion_cmd_t    cmd;                /* type NONE: nothing to send this cycle */
    bool            calibrate_begin;
    bool            calibrate_end;
    bool            consume_instruction;
    uint16_t        consume_id;
    bool            request_scan;
    fault_bits_t    raise_faults;
    fault_bits_t    clear_faults;       /* authorised by the operator */
    vehicle_state_t state;
    uint8_t         activity;           /* VC_ACT_* */
    uint8_t         event_count;
    vehicle_event_t events[VC_MAX_EVENTS];
} vehicle_out_t;

typedef struct {
    vehicle_config_t cfg;
    vehicle_state_t  state;
    vehicle_state_t  prev_reported;
    uint32_t         state_since_ms;
    bool             state_entry;       /* first cycle in this state */
    bool             calibrated;

    uint16_t         next_id;
    uint16_t         stop_id;           /* id of the last STOP sent */
    bool             started;
    motion_cmd_t     cmd;
    bool             cmd_active;
    uint32_t         cmd_started_ms;
    uint32_t         cmd_timeout_ms;

    int16_t          pending_turn_deg;
    uint16_t         last_executed_id;
    bool             have_executed;

    bypass_plan_t    plan;
    uint8_t          step_idx;
    uint8_t          cal_idx;
    uint8_t          search_idx;
    uint8_t          reverse_attempts;
    uint8_t          found_run;
    uint32_t         blank_until_ms;
    uint32_t         phase_since_ms;    /* a sub-phase timer inside a state */
    uint8_t          phase;
    int32_t          start_distance_mm;
    int16_t          realign_turn_deg;
    bool             stop_sent;
    bool             imu_lift_latched;
} vehicle_t;

void vehicle_init(vehicle_t *vc, const vehicle_config_t *cfg);

/* One control cycle (10 ms). */
void vehicle_step(vehicle_t *vc, const vehicle_in_t *in, vehicle_out_t *out);

/*
 * Decide what to do with a command from the network (a stateless policy so
 * the telemetry task can call it without touching controller state).
 * Returns the TEL_ACK_* result; on TEL_ACK_RECEIVED *request_bit is the
 * VC_REMOTE_* bit the caller hands to the controller with its next input.
 * Remote commands never move the wheels, and START, CALIBRATE and
 * CLEAR_FAULTS are refused while the mission is running.
 */
uint8_t vehicle_remote_policy(vehicle_state_t state, bool allow_remote_stop,
                              bool imu_lift_latched, uint8_t cmd, uint8_t *request_bit);

bool vehicle_is_running(vehicle_state_t state);

#endif /* VEHICLE_H */
