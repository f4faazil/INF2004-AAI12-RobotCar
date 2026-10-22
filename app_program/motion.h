/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * motion.h
 * Motion control (Buddy 2): turns authorised movement commands into left and
 * right motor PWM using encoder feedback.  This is the only module whose
 * output reaches the motors; every other subsystem asks the Vehicle
 * Controller, which sends commands here.
 *
 * Safety rules from the Tribunal, all enforced here:
 *  - a command carries an id and a lease; if the lease lapses the motors
 *    stop and that id can never restart;
 *  - STOP overrides everything, and a cancelled or finished id cannot be
 *    resumed by a late refresh;
 *  - repeated refreshes of the same id do not restart a manoeuvre;
 *  - commands older than the newest accepted id are rejected (wrap-aware).
 *
 * Pure C: the caller passes encoder snapshots and the time, and applies the
 * returned PWM.  Gains and geometry come from motion_config_t; the defaults
 * in robot_config.h are placeholders to be calibrated on the car.
 */
#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "encoder.h"
#include "pid.h"

#define MOTION_PWM_MAX          (1000)
#define MOTION_HISTORY_LEN      (16U)

typedef enum {
    MOTION_CMD_NONE = 0,
    MOTION_CMD_STOP,
    MOTION_CMD_DRIVE,       /* continuous: speed and optional differential */
    MOTION_CMD_MOVE,        /* finite: signed distance, straight */
    MOTION_CMD_TURN         /* finite: in-place rotation by angle */
} motion_cmd_type_t;

typedef struct {
    uint16_t          id;           /* newer commands have a larger id (wraps) */
    motion_cmd_type_t type;
    int16_t           speed_mm_s;   /* DRIVE: signed centre speed; MOVE/TURN: limit (0 = default) */
    int16_t           diff_mm_s;    /* DRIVE: right minus left; + veers left */
    int16_t           distance_mm;  /* MOVE: signed distance */
    int16_t           angle_deg;    /* TURN: + is left (counter-clockwise) */
    uint16_t          lease_ms;     /* 0 = default lease */
} motion_cmd_t;

typedef enum {
    MOTION_IDLE = 0,
    MOTION_RUNNING,
    MOTION_DONE,            /* a finite command completed */
    MOTION_EXPIRED,         /* the lease lapsed before it completed */
    MOTION_OVERRIDDEN       /* a safety stop cancelled it */
} motion_state_t;

#define MOTION_FLAG_STOPPED         (1U << 0)   /* wheels confirmed stationary */
#define MOTION_FLAG_DONE            (1U << 1)
#define MOTION_FLAG_EXPIRED         (1U << 2)
#define MOTION_FLAG_ENCODER_FAULT   (1U << 3)
#define MOTION_FLAG_OVERRIDE        (1U << 4)

typedef struct {
    uint16_t       cmd_id;          /* the command this status describes */
    motion_state_t state;
    uint8_t        flags;
    int16_t        left_mm_s;       /* measured wheel speeds */
    int16_t        right_mm_s;
    int32_t        distance_mm;     /* signed travel of the car centre since init */
    uint32_t       timestamp_ms;
} motion_status_t;

typedef struct {
    int16_t left_pwm;               /* signed per-mille, -1000..+1000 */
    int16_t right_pwm;
} motion_output_t;

typedef struct {
    uint32_t counts_per_rev;        /* encoder counts per wheel revolution (as counted) */
    uint32_t wheel_diameter_mm;
    uint32_t track_width_mm;        /* distance between the wheel contact points */
    int32_t  kp;                    /* wheel-speed PID gains, 1/256 units */
    int32_t  ki;
    int32_t  kd;
    int32_t  ff_num;                /* feed-forward: pwm = speed * num / den */
    int32_t  ff_den;
    int32_t  balance_mm_s_per_mm;   /* straight-line correction per mm of imbalance */
    uint32_t default_lease_ms;
    uint16_t max_speed_mm_s;
    uint16_t creep_speed_mm_s;      /* slowest speed used to finish a move */
    uint16_t turn_speed_mm_s;
    uint16_t decel_per_s;           /* approach speed = remaining mm * this */
    uint16_t speed_window_ms;       /* averaging window for the speed estimate */
    uint16_t stopped_speed_mm_s;
    uint16_t stopped_confirm_ms;
    uint16_t encoder_fault_ms;      /* PWM applied but no counts for this long */
    uint16_t encoder_fault_pwm;
} motion_config_t;

typedef struct {
    int32_t  left_counts;
    int32_t  right_counts;
    uint32_t time_us;
} motion_sample_t;

typedef struct {
    motion_config_t cfg;
    pid_ctrl_t      pid_left;
    pid_ctrl_t      pid_right;

    motion_cmd_t    cmd;
    bool            active;
    bool            have_id;            /* last_id is meaningful */
    uint16_t        last_id;            /* newest id accepted */
    bool            cancelled_valid;
    uint16_t        cancelled_id;       /* stopped/expired/done: may not resume */
    bool            override_stop;
    uint32_t        lease_deadline_ms;
    bool            base_pending;
    int32_t         base_left;
    int32_t         base_right;

    motion_sample_t history[MOTION_HISTORY_LEN];
    uint8_t         history_count;
    uint8_t         history_head;

    int32_t         speed_left;
    int32_t         speed_right;
    int32_t         origin_left;            /* counts at the first step */
    int32_t         origin_right;
    bool            origin_set;
    bool            have_step;
    uint32_t        last_step_ms;
    bool            stopped_timing;
    uint32_t        stopped_since_ms;
    int32_t         last_left_counts;
    int32_t         last_right_counts;
    bool            left_still_timing;
    bool            right_still_timing;
    uint32_t        left_still_since_ms;
    uint32_t        right_still_since_ms;
    bool            encoder_fault;

    uint32_t        rejected_stale;
    uint32_t        expired_count;

    motion_status_t status;
} motion_t;

void motion_init(motion_t *m, const motion_config_t *cfg);

/*
 * Submit a command (Vehicle Controller only).  Returns true when the command
 * was accepted or was a valid refresh of the active command.
 */
bool motion_submit(motion_t *m, const motion_cmd_t *cmd, uint32_t now_ms);

/* Safety override: while true, all outputs are zero and new commands are
 * rejected.  Clearing it does not resume the cancelled command. */
void motion_set_override(motion_t *m, bool stop);

/* One control step (every 5 ms).  Reads the encoder snapshots, runs the
 * speed loops and fills *out with the PWM to apply. */
void motion_step(motion_t *m,
                 const encoder_snapshot_t *left,
                 const encoder_snapshot_t *right,
                 uint32_t now_ms,
                 uint32_t now_us,
                 motion_output_t *out);

const motion_status_t *motion_status(const motion_t *m);

/* Conversions, exposed for the other modules and the tests. */
int32_t motion_counts_to_um(const motion_config_t *cfg, int32_t counts);
int32_t motion_um_to_counts(const motion_config_t *cfg, int32_t um);

#endif /* MOTION_H */
