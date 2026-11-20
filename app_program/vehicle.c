/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * vehicle.c
 * The Vehicle Controller.  See vehicle.h.
 */
#include <stddef.h>

#include "telemetry.h"
#include "vehicle.h"

#define PERMILLE            (1000)
#define POSITION_MAX        (1000)
#define CAL_LEGS            (3U)
#define CAL_RESULT_WAIT_MS  (1000U)
#define FOUND_FAULT_MS      (1000U)

/* RECOVER_LINE phases */
#define REC_CREEP           (0U)
#define REC_ALIGN           (1U)
#define REC_ADVANCE         (2U)
#define REC_STOPPING        (3U)        /* stopped for something in the way: wait for the wheels */
#define REC_HOLD            (4U)        /* waiting for the way to clear */

/* BYPASS phases */
#define BYP_RUN             (0U)
#define BYP_STOPPING        (1U)
#define BYP_HOLD            (2U)

#define BYPASS_CLEAR_HYST_MM    (40U)   /* the way is clear this far beyond the stop distance ... */
#define BYPASS_CLEAR_MS         (300U)  /* ... for this long */
#define BYPASS_MIN_LEFT_MM      (10)    /* less than this left of a leg: it is done */
#define SIDE_SEEN_N             (3U)    /* readings in a row that see the box before it counts as seen */
#define SIDE_LOST_N             (3U)    /* readings in a row that miss it before it counts as passed */
#define SIDE_MIN_CUT_MM         (20)    /* the shortest leg left after the box is passed */
#define RECOVER_EARLY_MM        (100)   /* a line this much nearer than the planned return is not the one to rejoin */

/* SEARCH_LINE phases */
#define SEARCH_STOPPING     (0U)
#define SEARCH_SWEEPING     (1U)

/* CALIBRATE phases */
#define CAL_SWEEPING        (0U)
#define CAL_WAITING         (1U)

bool vehicle_is_running(vehicle_state_t state)
{
    return (state >= VC_CALIBRATE) && (state <= VC_RECOVER_LINE);
}

uint8_t vehicle_remote_policy(vehicle_state_t state, bool allow_remote_stop, uint8_t cmd,
                              uint8_t *request_bit)
{
    const bool running = vehicle_is_running(state);

    *request_bit = 0U;
    switch (cmd) {
    case TEL_CMD_STOP:
        if (running && !allow_remote_stop) {
            return TEL_ACK_REJECTED_RUNNING;
        }
        *request_bit = VC_REMOTE_STOP;
        return TEL_ACK_RECEIVED;

    case TEL_CMD_START:
    case TEL_CMD_CALIBRATE:
    case TEL_CMD_CLEAR_FAULTS:
        if (running) {
            return TEL_ACK_REJECTED_RUNNING;    /* never change the mission mid-run */
        }
        *request_bit = (cmd == TEL_CMD_START) ? VC_REMOTE_START
                     : ((cmd == TEL_CMD_CALIBRATE) ? VC_REMOTE_CALIBRATE
                                                   : VC_REMOTE_CLEAR_FAULTS);
        return TEL_ACK_RECEIVED;

    case TEL_CMD_REQUEST_STATUS:
        return TEL_ACK_COMPLETED;               /* the status goes out regardless */

    default:
        return TEL_ACK_INVALID;
    }
}

void vehicle_init(vehicle_t *vc, const vehicle_config_t *cfg)
{
    vc->cfg = *cfg;
    vc->state = VC_BOOT;
    vc->prev_reported = VC_BOOT;
    vc->state_since_ms = 0U;
    vc->state_entry = true;
    vc->calibrated = false;
    vc->next_id = 0U;
    vc->stop_id = 0U;
    vc->started = false;
    vc->cmd.id = 0U;
    vc->cmd.type = MOTION_CMD_NONE;
    vc->cmd_active = false;
    vc->cmd_started_ms = 0U;
    vc->cmd_timeout_ms = 0U;
    vc->pending_turn_deg = 0;
    vc->last_executed_id = 0U;
    vc->have_executed = false;
    vc->step_idx = 0U;
    vc->cal_idx = 0U;
    vc->search_idx = 0U;
    vc->reverse_attempts = 0U;
    vc->found_run = 0U;
    vc->blank_until_ms = 0U;
    vc->phase_since_ms = 0U;
    vc->phase = 0U;
    vc->start_distance_mm = 0;
    vc->realign_turn_deg = 0;
    vc->stop_sent = false;
    vc->hold_since_ms = 0U;
    vc->clear_since_ms = 0U;
    vc->step_start_mm = 0;
    vc->leg_mm = 0;
    vc->leg_origin_mm = 0;
    vc->leg_total_mm = 0;
    vc->leg_extended_mm = 0;
    vc->side_seq_seen = 0U;
    vc->side_hits = 0U;
    vc->side_miss = 0U;
    vc->side_seen = false;
    vc->side_cut_done = false;
    vc->side_last_mm = 0U;
    vc->side_miss_mm = 0;
}

/* ---- helpers --------------------------------------------------------- */

static int32_t abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static void add_event(vehicle_out_t *out, uint8_t code, uint16_t arg)
{
    if (out->event_count < VC_MAX_EVENTS) {
        out->events[out->event_count].code = code;
        out->events[out->event_count].arg = arg;
        out->event_count++;
    }
}

static uint16_t new_id(vehicle_t *vc)
{
    vc->next_id++;
    return vc->next_id;
}

static void send_stop(vehicle_t *vc, vehicle_out_t *out)
{
    out->cmd.id = new_id(vc);
    out->cmd.type = MOTION_CMD_STOP;
    out->cmd.speed_mm_s = 0;
    out->cmd.diff_mm_s = 0;
    out->cmd.distance_mm = 0;
    out->cmd.angle_deg = 0;
    out->cmd.lease_ms = 0U;
    vc->stop_id = out->cmd.id;
    vc->cmd_active = false;
    vc->stop_sent = true;
}

static bool stop_confirmed(const vehicle_t *vc, const vehicle_in_t *in)
{
    return (in->motion.cmd_id == vc->stop_id) &&
           ((in->motion.flags & MOTION_FLAG_STOPPED) != 0U);
}

static void start_cmd(vehicle_t *vc, vehicle_out_t *out, uint32_t now, motion_cmd_type_t type,
                      int16_t speed, int16_t diff, int16_t distance, int16_t angle)
{
    vc->cmd.id = new_id(vc);
    vc->cmd.type = type;
    vc->cmd.speed_mm_s = speed;
    vc->cmd.diff_mm_s = diff;
    vc->cmd.distance_mm = distance;
    vc->cmd.angle_deg = angle;
    vc->cmd.lease_ms = 0U;
    vc->cmd_active = true;
    vc->cmd_started_ms = now;
    if (type == MOTION_CMD_MOVE) {
        vc->cmd_timeout_ms = vc->cfg.cmd_timeout_base_ms +
                             (vc->cfg.cmd_timeout_per_mm_ms * (uint32_t)abs32(distance));
    } else if (type == MOTION_CMD_TURN) {
        vc->cmd_timeout_ms = vc->cfg.cmd_timeout_base_ms +
                             (vc->cfg.cmd_timeout_per_deg_ms * (uint32_t)abs32(angle));
    } else {
        vc->cmd_timeout_ms = 0U;                /* continuous: no completion */
    }
    out->cmd = vc->cmd;
}

static void refresh_cmd(const vehicle_t *vc, vehicle_out_t *out)
{
    if (vc->cmd_active) {
        out->cmd = vc->cmd;
    }
}

static bool cmd_done(const vehicle_t *vc, const vehicle_in_t *in)
{
    return vc->cmd_active && (in->motion.cmd_id == vc->cmd.id) &&
           ((in->motion.flags & MOTION_FLAG_DONE) != 0U);
}

static bool cmd_timed_out(const vehicle_t *vc, uint32_t now)
{
    return vc->cmd_active && ((now - vc->cmd_started_ms) > vc->cmd_timeout_ms);
}

static void enter(vehicle_t *vc, vehicle_out_t *out, vehicle_state_t state, uint32_t now)
{
    vc->state = state;
    vc->state_since_ms = now;
    vc->phase = 0U;
    vc->found_run = 0U;
    add_event(out, TEL_EVT_STATE, (uint16_t)state);
}

static void enter_fault(vehicle_t *vc, vehicle_out_t *out, uint32_t now, fault_bits_t raise)
{
    out->raise_faults = (fault_bits_t)(out->raise_faults | raise);
    if (raise != 0U) {
        add_event(out, TEL_EVT_FAULT, raise);
    }
    send_stop(vc, out);
    enter(vc, out, VC_FAULT, now);
}

static void enter_idle(vehicle_t *vc, vehicle_out_t *out, uint32_t now)
{
    send_stop(vc, out);
    enter(vc, out, VC_IDLE, now);
}

static void enter_stopped(vehicle_t *vc, vehicle_out_t *out, uint32_t now)
{
    send_stop(vc, out);
    enter(vc, out, VC_STOPPED, now);
}

static void enter_follow(vehicle_t *vc, vehicle_out_t *out, uint32_t now)
{
    start_cmd(vc, out, now, MOTION_CMD_DRIVE, vc->cfg.cruise_mm_s, 0, 0, 0);
    enter(vc, out, VC_FOLLOW, now);
}

static void enter_calibrate(vehicle_t *vc, vehicle_out_t *out, uint32_t now)
{
    out->calibrate_begin = true;
    vc->cal_idx = 0U;
    start_cmd(vc, out, now, MOTION_CMD_TURN, 0, 0, 0, vc->cfg.cal_sweep_deg[0]);
    enter(vc, out, VC_CALIBRATE, now);
}

static void enter_obstacle_stop(vehicle_t *vc, vehicle_out_t *out, uint32_t now,
                                const vehicle_in_t *in)
{
    send_stop(vc, out);
    add_event(out, TEL_EVT_OBSTACLE, 0U);
    (void)in;
    enter(vc, out, VC_OBSTACLE_STOP, now);
}

static void enter_scan(vehicle_t *vc, vehicle_out_t *out, uint32_t now)
{
    out->request_scan = true;
    enter(vc, out, VC_OBSTACLE_SCAN, now);
}

static void enter_search(vehicle_t *vc, vehicle_out_t *out, uint32_t now)
{
    send_stop(vc, out);
    add_event(out, TEL_EVT_LINE_LOST, 0U);
    vc->search_idx = 0U;
    enter(vc, out, VC_SEARCH_LINE, now);
}

/* Something is closer than the stop distance (plus extra) straight ahead of the car. */
static bool path_blocked(const vehicle_t *vc, const vehicle_in_t *in, uint16_t extra_mm)
{
    return (vc->cfg.bypass_stop_mm > 0U) && in->obstacle_mm_valid &&
           (in->obstacle_mm < (uint16_t)(vc->cfg.bypass_stop_mm + extra_mm));
}

/* True once the way has stayed clear long enough to move again. */
static bool way_cleared(vehicle_t *vc, const vehicle_in_t *in, uint32_t now)
{
    if (path_blocked(vc, in, BYPASS_CLEAR_HYST_MM)) {
        vc->clear_since_ms = 0U;
        return false;
    }
    if (vc->clear_since_ms == 0U) {
        vc->clear_since_ms = now;
    }
    return (now - vc->clear_since_ms) >= BYPASS_CLEAR_MS;
}

/* The straight leg that runs beside the box: the last one before the turn toward the line. */
static bool is_side_leg(const vehicle_t *vc)
{
    return (vc->cfg.side_aim_deg > 0) && (vc->plan.step_count >= 3U) &&
           ((vc->step_idx + 2U) == vc->plan.step_count) &&
           !vc->plan.steps[vc->step_idx].is_turn;
}

/* Count the raw sonar readings that see the box's side, and note where they stop. */
static void side_observe(vehicle_t *vc, const vehicle_in_t *in)
{
    if (in->side_seq == vc->side_seq_seen) {
        return;                                 /* no new reading */
    }
    vc->side_seq_seen = in->side_seq;
    if (in->side_echo && (in->side_mm >= vc->cfg.side_min_mm) &&
        (in->side_mm <= vc->cfg.side_max_mm)) {
        if (vc->side_hits < UINT8_MAX) {
            vc->side_hits++;
        }
        vc->side_miss = 0U;
        vc->side_last_mm = in->side_mm;
        if (vc->side_hits >= SIDE_SEEN_N) {
            vc->side_seen = true;
        }
    } else {
        if (vc->side_miss == 0U) {
            vc->side_miss_mm = in->motion.distance_mm;      /* it was last seen about here */
        }
        if (vc->side_miss < UINT8_MAX) {
            vc->side_miss++;
        }
        vc->side_hits = 0U;
    }
}

/* The box was seen and now is not: the leg need only cover what the rear needs. */
static void side_cut_leg(vehicle_t *vc, vehicle_out_t *out, uint32_t now, const vehicle_in_t *in)
{
    const int32_t ahead = ((int32_t)vc->side_last_mm * (int32_t)vc->cfg.side_loss_ahead_pct) / 100;
    const int32_t total = abs32(vc->side_miss_mm - vc->leg_origin_mm) + ahead +
                          (int32_t)vc->cfg.side_tail_mm;

    vc->side_cut_done = true;
    if (total < (int32_t)vc->leg_total_mm) {
        const int32_t travelled = abs32(in->motion.distance_mm - vc->leg_origin_mm);
        int32_t left = total - travelled;

        if (left < SIDE_MIN_CUT_MM) {
            left = SIDE_MIN_CUT_MM;
        }
        vc->leg_total_mm = (int16_t)total;
        vc->leg_mm = (int16_t)left;
        vc->step_start_mm = in->motion.distance_mm;
        start_cmd(vc, out, now, MOTION_CMD_MOVE, vc->cfg.curve_mm_s, 0, vc->leg_mm, 0);
        add_event(out, TEL_EVT_OBSTACLE, 2U);   /* 2: the box is passed, the leg is cut short */
    }
}

static void stop_for_obstacle(vehicle_t *vc, vehicle_out_t *out, uint32_t now)
{
    send_stop(vc, out);
    add_event(out, TEL_EVT_OBSTACLE, 1U);       /* 1: met during a bypass */
    vc->hold_since_ms = now;
    vc->clear_since_ms = 0U;
}

static void start_bypass_step(vehicle_t *vc, vehicle_out_t *out, uint32_t now,
                              const vehicle_in_t *in)
{
    const plan_step_t *step = &vc->plan.steps[vc->step_idx];

    vc->found_run = 0U;
    vc->leg_mm = step->value;
    vc->step_start_mm = in->motion.distance_mm;
    vc->leg_origin_mm = in->motion.distance_mm;
    vc->leg_total_mm = step->value;
    vc->leg_extended_mm = 0;
    vc->side_seq_seen = in->side_seq;
    vc->side_hits = 0U;
    vc->side_miss = 0U;
    vc->side_seen = false;
    vc->side_cut_done = false;
    if (step->is_turn) {
        start_cmd(vc, out, now, MOTION_CMD_TURN, 0, 0, 0, step->value);
    } else {
        start_cmd(vc, out, now, MOTION_CMD_MOVE, vc->cfg.curve_mm_s, 0, step->value, 0);
    }
}

static void enter_recover(vehicle_t *vc, vehicle_out_t *out, uint32_t now,
                          const vehicle_in_t *in)
{
    const plan_step_t *last = &vc->plan.steps[vc->plan.step_count - 1U];

    /* The last bypass turn faced the car toward the line; once the line is
     * found, undo that turn to face along it again. */
    vc->realign_turn_deg = (vc->plan.step_count > 0U && last->is_turn)
                           ? (int16_t)(-last->value) : 0;
    vc->start_distance_mm = in->motion.distance_mm;
    start_cmd(vc, out, now, MOTION_CMD_DRIVE, vc->cfg.recover_mm_s, 0, 0, 0);
    enter(vc, out, VC_RECOVER_LINE, now);
    vc->phase = REC_CREEP;
    vc->phase_since_ms = now;
}

/* A visible, not-lost line for found_cycles cycles in a row. */
static bool line_found(vehicle_t *vc, const vehicle_in_t *in)
{
    if (in->line.position_valid && !in->line.line_lost) {
        if (vc->found_run < UINT8_MAX) {
            vc->found_run++;
        }
    } else {
        vc->found_run = 0U;
    }
    return vc->found_run >= vc->cfg.found_cycles;
}

/* ---- following -------------------------------------------------------- */

static void follow_drive(vehicle_t *vc, vehicle_out_t *out, const vehicle_in_t *in)
{
    int32_t speed = vc->cfg.cruise_mm_s;
    int32_t diff = in->line.steer_diff_mm_s;

    if (in->line.position_valid) {
        int32_t err = abs32(in->line.position);

        if (err > POSITION_MAX) {
            err = POSITION_MAX;
        }
        speed = (int32_t)vc->cfg.cruise_mm_s -
                (((int32_t)vc->cfg.cruise_mm_s - (int32_t)vc->cfg.curve_mm_s) * err) / PERMILLE;
    } else {
        speed = vc->cfg.curve_mm_s;             /* no position: slow and straight */
        diff = 0;
    }
    if (in->hump_in_progress && (speed > vc->cfg.hump_mm_s)) {
        speed = vc->cfg.hump_mm_s;
    }
    vc->cmd.speed_mm_s = (int16_t)speed;
    vc->cmd.diff_mm_s = (int16_t)diff;
    out->cmd = vc->cmd;
}

static void handle_junction(vehicle_t *vc, vehicle_out_t *out, const vehicle_in_t *in,
                            uint32_t now)
{
    vc->pending_turn_deg = 0;
    if (in->instruction_valid &&
        !(vc->have_executed && (in->instruction.id == vc->last_executed_id))) {
        const nav_instruction_t *ins = &in->instruction;

        out->consume_instruction = true;
        out->consume_id = ins->id;
        vc->last_executed_id = ins->id;
        vc->have_executed = true;
        add_event(out, TEL_EVT_NAV_EXEC, (uint16_t)ins->command);
        if (ins->command == NAV_CMD_LEFT) {
            vc->pending_turn_deg = vc->cfg.turn_deg;
        } else if (ins->command == NAV_CMD_RIGHT) {
            vc->pending_turn_deg = (int16_t)(-vc->cfg.turn_deg);
        } else if (ins->command == NAV_CMD_UTURN) {
            vc->pending_turn_deg = vc->cfg.uturn_deg;
        } else {
            vc->pending_turn_deg = 0;           /* STRAIGHT */
        }
    }
    add_event(out, TEL_EVT_JUNCTION, in->line.junction_mask);
    start_cmd(vc, out, now, MOTION_CMD_MOVE, vc->cfg.curve_mm_s, 0,
              vc->cfg.junction_advance_mm, 0);
    enter(vc, out, VC_JUNCTION_ADVANCE, now);
}

/* ---- the step ---------------------------------------------------------- */

static void run_state(vehicle_t *vc, const vehicle_in_t *in, vehicle_out_t *out, uint32_t now)
{
    const bool start = in->start_pressed || ((in->remote & VC_REMOTE_START) != 0U);
    const bool calibrate_req = (in->remote & VC_REMOTE_CALIBRATE) != 0U;
    const bool clear_req = (in->remote & VC_REMOTE_CLEAR_FAULTS) != 0U;

    switch (vc->state) {
    case VC_BOOT:
        if ((now - vc->state_since_ms) >= vc->cfg.boot_ms) {
            enter_idle(vc, out, now);
        }
        break;

    case VC_IDLE:
        if (clear_req) {
            out->clear_faults = in->faults;
        }
        if (start || calibrate_req) {
            out->clear_faults = (fault_bits_t)(out->clear_faults | (in->faults & FAULT_STOP_MASK));
            enter_calibrate(vc, out, now);
        }
        break;

    case VC_STOPPED:
        if (clear_req) {
            out->clear_faults = in->faults;
        }
        if (calibrate_req || (start && !vc->calibrated)) {
            enter_calibrate(vc, out, now);
        } else if (start) {
            enter_follow(vc, out, now);
        } else {
            /* waiting for the operator */
        }
        break;

    case VC_FAULT:
        if (start || clear_req) {
            out->clear_faults = clear_req ? in->faults : (in->faults & FAULT_STOP_MASK);
            add_event(out, TEL_EVT_FAULT_CLEAR, out->clear_faults);
            enter_idle(vc, out, now);
        }
        break;

    case VC_CALIBRATE:
        if (vc->phase == CAL_SWEEPING) {
            if (cmd_done(vc, in)) {
                vc->cal_idx++;
                if (vc->cal_idx < CAL_LEGS) {
                    start_cmd(vc, out, now, MOTION_CMD_TURN, 0, 0, 0,
                              vc->cfg.cal_sweep_deg[vc->cal_idx]);
                } else {
                    vc->cmd_active = false;
                    out->calibrate_end = true;
                    vc->phase = CAL_WAITING;
                    vc->phase_since_ms = now;
                }
            } else if (cmd_timed_out(vc, now)) {
                enter_fault(vc, out, now, FAULT_MOTION);
            } else {
                refresh_cmd(vc, out);
            }
        } else if (in->calibration_done) {
            if (in->calibration_ok) {
                vc->calibrated = true;
                enter_follow(vc, out, now);
            } else {
                enter_fault(vc, out, now, FAULT_LINE_CAL);
            }
        } else if ((now - vc->phase_since_ms) > CAL_RESULT_WAIT_MS) {
            enter_fault(vc, out, now, FAULT_LINE_CAL);
        } else {
            /* waiting for the line module's verdict */
        }
        break;

    case VC_FOLLOW:
        if (in->line.stale) {
            enter_fault(vc, out, now, FAULT_LINE_STALE);
        } else if (in->obstacle_present && ((int32_t)(now - vc->blank_until_ms) >= 0)) {
            enter_obstacle_stop(vc, out, now, in);
        } else if ((in->line.events & NAV_EVENT_JUNCTION) != 0U) {
            handle_junction(vc, out, in, now);
        } else if (in->line.line_lost && !in->line.position_valid) {
            enter_search(vc, out, now);
        } else {
            follow_drive(vc, out, in);
        }
        break;

    case VC_JUNCTION_ADVANCE:
        if (cmd_done(vc, in)) {
            if (vc->pending_turn_deg != 0) {
                start_cmd(vc, out, now, MOTION_CMD_TURN, 0, 0, 0, vc->pending_turn_deg);
                enter(vc, out, VC_JUNCTION_TURN, now);
            } else {
                enter_follow(vc, out, now);
            }
        } else if (cmd_timed_out(vc, now)) {
            enter_fault(vc, out, now, FAULT_MOTION);
        } else {
            refresh_cmd(vc, out);
        }
        break;

    case VC_JUNCTION_TURN:
        if (cmd_done(vc, in)) {
            vc->start_distance_mm = in->motion.distance_mm;
            start_cmd(vc, out, now, MOTION_CMD_DRIVE, vc->cfg.creep_mm_s, 0, 0, 0);
            enter(vc, out, VC_REACQUIRE, now);
        } else if (cmd_timed_out(vc, now)) {
            enter_fault(vc, out, now, FAULT_MOTION);
        } else {
            refresh_cmd(vc, out);
        }
        break;

    case VC_REACQUIRE:
        if (line_found(vc, in)) {
            enter_follow(vc, out, now);
        } else if (((now - vc->state_since_ms) >= vc->cfg.reacquire_timeout_ms) ||
                   (abs32(in->motion.distance_mm - vc->start_distance_mm) >=
                    vc->cfg.reacquire_limit_mm)) {
            enter_search(vc, out, now);
        } else {
            refresh_cmd(vc, out);
        }
        break;

    case VC_SEARCH_LINE:
        if (vc->phase == SEARCH_STOPPING) {
            if (stop_confirmed(vc, in)) {
                vc->phase = SEARCH_SWEEPING;
                start_cmd(vc, out, now, MOTION_CMD_TURN, 0, 0, 0,
                          vc->cfg.search_sweep_deg[0]);
            } else if ((now - vc->state_since_ms) > vc->cfg.stop_confirm_timeout_ms) {
                enter_fault(vc, out, now, FAULT_MOTION);
            } else {
                /* waiting for the wheels to stop */
            }
        } else if (line_found(vc, in)) {
            add_event(out, TEL_EVT_LINE_FOUND, 0U);
            enter_follow(vc, out, now);
        } else if (cmd_done(vc, in)) {
            vc->search_idx++;
            if (vc->search_idx < vc->cfg.search_sweep_count) {
                start_cmd(vc, out, now, MOTION_CMD_TURN, 0, 0, 0,
                          vc->cfg.search_sweep_deg[vc->search_idx]);
            } else {
                enter_fault(vc, out, now, FAULT_LINE_LOST);
            }
        } else if (cmd_timed_out(vc, now)) {
            enter_fault(vc, out, now, FAULT_MOTION);
        } else {
            refresh_cmd(vc, out);
        }
        break;

    case VC_OBSTACLE_STOP:
        if (stop_confirmed(vc, in)) {
            enter_scan(vc, out, now);
        } else if ((now - vc->state_since_ms) > vc->cfg.stop_confirm_timeout_ms) {
            enter_fault(vc, out, now, FAULT_MOTION);
        } else {
            /* the car has not stopped moving yet: no scanning */
        }
        break;

    case VC_OBSTACLE_SCAN:
        if (!in->scan_done) {
            if ((now - vc->state_since_ms) > vc->cfg.scan_timeout_ms) {
                enter_fault(vc, out, now, FAULT_ULTRASONIC);
            }
            break;
        }
        add_event(out, TEL_EVT_PLAN, (uint16_t)in->plan.action);
        if (in->plan.action == PLAN_CONTINUE) {
            vc->blank_until_ms = now + vc->cfg.obstacle_blank_ms;
            enter_follow(vc, out, now);
        } else if ((in->plan.action == PLAN_BYPASS_LEFT) ||
                   (in->plan.action == PLAN_BYPASS_RIGHT)) {
            vc->plan = in->plan;
            vc->step_idx = 0U;
            start_bypass_step(vc, out, now, in);
            enter(vc, out, VC_BYPASS, now);
        } else if (in->plan.action == PLAN_REVERSE) {
            if (vc->reverse_attempts >= vc->cfg.max_reverse_attempts) {
                enter_fault(vc, out, now, FAULT_NO_SAFE_PATH);
            } else {
                vc->reverse_attempts++;
                start_cmd(vc, out, now, MOTION_CMD_MOVE, vc->cfg.curve_mm_s, 0,
                          in->plan.steps[0].value, 0);
                enter(vc, out, VC_REVERSE, now);
            }
        } else {
            enter_fault(vc, out, now,
                        (in->plan.reason == PLAN_REASON_INVALID_SCAN) ? FAULT_ULTRASONIC
                                                                     : FAULT_NO_SAFE_PATH);
        }
        break;

    case VC_REVERSE:
        if (cmd_done(vc, in)) {
            enter_scan(vc, out, now);
        } else if (cmd_timed_out(vc, now)) {
            enter_fault(vc, out, now, FAULT_MOTION);
        } else {
            refresh_cmd(vc, out);
        }
        break;

    case VC_BYPASS:
        if (vc->phase == BYP_STOPPING) {
            if (stop_confirmed(vc, in)) {
                const int32_t travelled = abs32(in->motion.distance_mm - vc->step_start_mm);

                vc->leg_mm = (int16_t)(vc->leg_mm - travelled);       /* what is left of the leg */
                vc->phase = BYP_HOLD;
            } else if ((now - vc->hold_since_ms) > vc->cfg.stop_confirm_timeout_ms) {
                enter_fault(vc, out, now, FAULT_MOTION);
            } else {
                /* the wheels are still stopping */
            }
        } else if (vc->phase == BYP_HOLD) {
            if (way_cleared(vc, in, now)) {
                vc->phase = BYP_RUN;
                if (vc->leg_mm > BYPASS_MIN_LEFT_MM) {
                    vc->step_start_mm = in->motion.distance_mm;
                    start_cmd(vc, out, now, MOTION_CMD_MOVE, vc->cfg.curve_mm_s, 0, vc->leg_mm, 0);
                } else {
                    vc->step_idx++;                 /* the leg was all but done */
                    if (vc->step_idx < vc->plan.step_count) {
                        start_bypass_step(vc, out, now, in);
                    } else {
                        enter_recover(vc, out, now, in);
                    }
                }
            } else if ((now - vc->hold_since_ms) > vc->cfg.bypass_hold_ms) {
                enter_fault(vc, out, now, FAULT_NO_SAFE_PATH);
            } else {
                /* still blocked: stay put */
            }
        } else if (cmd_done(vc, in)) {
            if (is_side_leg(vc) && vc->side_seen && (vc->side_miss == 0U) &&
                (vc->leg_extended_mm < (int16_t)vc->cfg.side_extend_max_mm)) {
                /* the box is still in view at the end of the leg: it is deeper than assumed */
                vc->leg_extended_mm = (int16_t)(vc->leg_extended_mm + (int16_t)vc->cfg.side_extend_mm);
                vc->leg_mm = (int16_t)vc->cfg.side_extend_mm;
                vc->step_start_mm = in->motion.distance_mm;
                start_cmd(vc, out, now, MOTION_CMD_MOVE, vc->cfg.curve_mm_s, 0, vc->leg_mm, 0);
                add_event(out, TEL_EVT_OBSTACLE, 3U);   /* 3: still beside the box, going on */
            } else {
                vc->step_idx++;
                if (vc->step_idx < vc->plan.step_count) {
                    start_bypass_step(vc, out, now, in);
                } else {
                    enter_recover(vc, out, now, in);
                }
            }
        } else if (cmd_timed_out(vc, now)) {
            enter_fault(vc, out, now, FAULT_MOTION);
        } else {
            const bool straight = !vc->plan.steps[vc->step_idx].is_turn;
            const bool settled = (now - vc->cmd_started_ms) >= vc->cfg.bypass_blank_ms;

            if (straight && settled && is_side_leg(vc)) {
                side_observe(vc, in);
            }
            if (straight && settled && path_blocked(vc, in, 0U)) {
                stop_for_obstacle(vc, out, now);
                vc->phase = BYP_STOPPING;
            } else if (straight && settled && is_side_leg(vc) && vc->side_seen &&
                       !vc->side_cut_done && (vc->side_miss >= SIDE_LOST_N)) {
                side_cut_leg(vc, out, now, in);
            } else {
                refresh_cmd(vc, out);
            }
        }
        break;

    case VC_RECOVER_LINE:
        if (vc->phase == REC_CREEP) {
            /* The line to rejoin is about return_leg_mm away.  A line seen much sooner is a line
             * crossing the way (a junction beside the box), not it: drive on over it. */
            const int32_t travelled = abs32(in->motion.distance_mm - vc->start_distance_mm);
            const bool may_accept = travelled >= ((int32_t)vc->plan.return_leg_mm - RECOVER_EARLY_MM);

            if (may_accept && line_found(vc, in)) {
                if ((vc->realign_turn_deg != 0) && (vc->cfg.recover_advance_mm > 0)) {
                    /* The sensors are ahead of the axle: turning here would swing the line off to
                     * the side.  Drive on until the axle is over it, then turn. */
                    start_cmd(vc, out, now, MOTION_CMD_MOVE, vc->cfg.creep_mm_s, 0, vc->cfg.recover_advance_mm, 0);
                    vc->phase = REC_ADVANCE;
                } else if (vc->realign_turn_deg != 0) {
                    start_cmd(vc, out, now, MOTION_CMD_TURN, 0, 0, 0, vc->realign_turn_deg);
                    vc->phase = REC_ALIGN;
                } else {
                    add_event(out, TEL_EVT_RECOVERED, 0U);
                    vc->reverse_attempts = 0U;
                    enter_follow(vc, out, now);
                }
            } else if ((now - vc->phase_since_ms) >= vc->cfg.recovery_time_limit_ms) {
                enter_fault(vc, out, now, FAULT_RECOVERY_LIMIT);
            } else if (abs32(in->motion.distance_mm - vc->start_distance_mm) >=
                       vc->cfg.recovery_distance_limit_mm) {
                enter_fault(vc, out, now, FAULT_RECOVERY_LIMIT);
            } else if (path_blocked(vc, in, 0U) &&
                       ((now - vc->cmd_started_ms) >= vc->cfg.bypass_blank_ms)) {
                stop_for_obstacle(vc, out, now);
                vc->phase = REC_STOPPING;
            } else {
                refresh_cmd(vc, out);
            }
        } else if (vc->phase == REC_STOPPING) {
            if (stop_confirmed(vc, in)) {
                vc->phase = REC_HOLD;
            } else if ((now - vc->hold_since_ms) > vc->cfg.stop_confirm_timeout_ms) {
                enter_fault(vc, out, now, FAULT_MOTION);
            } else {
                /* the wheels are still stopping */
            }
        } else if (vc->phase == REC_HOLD) {
            if (way_cleared(vc, in, now)) {
                /* the time spent waiting does not count against the 5 s recovery limit */
                vc->phase_since_ms += (now - vc->hold_since_ms);
                start_cmd(vc, out, now, MOTION_CMD_DRIVE, vc->cfg.recover_mm_s, 0, 0, 0);
                vc->phase = REC_CREEP;
            } else if ((now - vc->hold_since_ms) > vc->cfg.bypass_hold_ms) {
                enter_fault(vc, out, now, FAULT_NO_SAFE_PATH);
            } else {
                /* still blocked: stay put */
            }
        } else if (vc->phase == REC_ADVANCE) {
            if (cmd_done(vc, in)) {
                start_cmd(vc, out, now, MOTION_CMD_TURN, 0, 0, 0, vc->realign_turn_deg);
                vc->phase = REC_ALIGN;
            } else if (cmd_timed_out(vc, now)) {
                enter_fault(vc, out, now, FAULT_MOTION);
            } else {
                refresh_cmd(vc, out);
            }
        } else if (cmd_done(vc, in)) {
            add_event(out, TEL_EVT_RECOVERED, 0U);
            vc->reverse_attempts = 0U;
            enter_follow(vc, out, now);
        } else if (cmd_timed_out(vc, now)) {
            enter_fault(vc, out, now, FAULT_MOTION);
        } else {
            refresh_cmd(vc, out);
        }
        break;

    default:
        break;
    }
}

void vehicle_step(vehicle_t *vc, const vehicle_in_t *in, vehicle_out_t *out)
{
    const uint32_t now = in->now_ms;
    const bool fault_stop = (in->faults & FAULT_STOP_MASK) != 0U;
    const bool stop_req = in->stop_pressed ||
                          (((in->remote & VC_REMOTE_STOP) != 0U) && vc->cfg.allow_remote_stop);

    out->cmd.type = MOTION_CMD_NONE;
    out->cmd.id = 0U;
    out->cmd.speed_mm_s = 0;
    out->cmd.diff_mm_s = 0;
    out->cmd.distance_mm = 0;
    out->cmd.angle_deg = 0;
    out->cmd.lease_ms = 0U;
    out->calibrate_begin = false;
    out->calibrate_end = false;
    out->consume_instruction = false;
    out->consume_id = 0U;
    out->request_scan = false;
    out->raise_faults = 0U;
    out->clear_faults = 0U;
    out->event_count = 0U;

    if (!vc->started) {
        vc->started = true;
        vc->state_since_ms = now;
    }

    if (vehicle_is_running(vc->state)) {
        if (fault_stop) {
            enter_fault(vc, out, now, 0U);          /* someone else raised it */
        } else if (stop_req) {
            enter_stopped(vc, out, now);
        } else {
            run_state(vc, in, out, now);
        }
    } else {
        run_state(vc, in, out, now);
    }

    out->state = vc->state;
    out->sonar_aim = 0;
    if ((vc->state == VC_BYPASS) && (vc->cfg.side_aim_deg > 0) && (vc->plan.step_count >= 3U) &&
        (vc->step_idx + 3U >= vc->plan.step_count) && (vc->step_idx + 2U <= vc->plan.step_count)) {
        /* from the turn before the leg beside the box until that leg ends: look at the box */
        out->sonar_aim = (vc->plan.action == PLAN_BYPASS_LEFT) ? (int8_t)-1 : (int8_t)1;
    }
    out->activity = 0U;
    if ((vc->state == VC_FOLLOW) || (vc->state == VC_JUNCTION_ADVANCE) ||
        (vc->state == VC_JUNCTION_TURN) || (vc->state == VC_REACQUIRE) ||
        (vc->state == VC_SEARCH_LINE) || (vc->state == VC_RECOVER_LINE)) {
        out->activity |= VC_ACT_LINE_REQUIRED;
    }
    if (vc->cmd_active) {
        out->activity |= VC_ACT_DRIVING;
    }
    if ((vc->state == VC_FOLLOW) || (vc->state == VC_REACQUIRE)) {
        out->activity |= VC_ACT_OBSTACLE_WATCH;
    }
}
