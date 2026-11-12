/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * motion.c
 * Motion control.  See motion.h.
 */
#include <stddef.h>

#include "motion.h"

#define PI_MICRO            (3141593LL)     /* pi x 10^6 */
#define UM_PER_MM           (1000)
#define US_PER_MS           (1000)
#define FULL_TURN_DEG       (360LL)
#define PID_AUTHORITY_PWM   (300LL)         /* most the integral may add */
#define MS_PER_S            (1000LL)
#define BALANCE_LIMIT_DIV   (4)             /* correction <= max speed / 4 */

/* ---- helpers --------------------------------------------------------- */

static int32_t clamp32(int32_t value, int32_t low, int32_t high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static int32_t abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* True when 'id' is the same as or newer than 'reference' (mod 2^16). */
static bool id_not_older(uint16_t id, uint16_t reference)
{
    return (int16_t)(uint16_t)(id - reference) >= 0;
}

int32_t motion_counts_to_um(const motion_config_t *cfg, int32_t counts)
{
    int64_t num = (int64_t)counts * (int64_t)cfg->wheel_diameter_mm * PI_MICRO;
    int64_t den = (int64_t)cfg->counts_per_rev * UM_PER_MM;

    return (int32_t)(num / den);
}

int32_t motion_um_to_counts(const motion_config_t *cfg, int32_t um)
{
    int64_t num = (int64_t)um * (int64_t)cfg->counts_per_rev * UM_PER_MM;
    int64_t den = (int64_t)cfg->wheel_diameter_mm * PI_MICRO;
    int64_t half = den / 2;

    return (int32_t)((num >= 0) ? ((num + half) / den) : ((num - half) / den));
}

/* ---- lifecycle ------------------------------------------------------- */

void motion_init(motion_t *m, const motion_config_t *cfg)
{
    int32_t limit = 0;

    m->cfg = *cfg;
    if (cfg->ki != 0) {
        limit = (int32_t)((PID_AUTHORITY_PWM * MS_PER_S * PID_GAIN_ONE) / cfg->ki);
    }
    pid_init(&m->pid_left, cfg->kp, cfg->ki, cfg->kd,
             -MOTION_PWM_MAX, MOTION_PWM_MAX, limit);
    pid_init(&m->pid_right, cfg->kp, cfg->ki, cfg->kd,
             -MOTION_PWM_MAX, MOTION_PWM_MAX, limit);

    m->active = false;
    m->have_id = false;
    m->last_id = 0U;
    m->cancelled_valid = false;
    m->cancelled_id = 0U;
    m->override_stop = false;
    m->lease_deadline_ms = 0U;
    m->base_pending = false;
    m->base_left = 0;
    m->base_right = 0;
    m->history_count = 0U;
    m->history_head = 0U;
    m->speed_left = 0;
    m->speed_right = 0;
    m->origin_left = 0;
    m->origin_right = 0;
    m->origin_set = false;
    m->have_step = false;
    m->last_step_ms = 0U;
    m->stopped_timing = false;
    m->stopped_since_ms = 0U;
    m->last_left_counts = 0;
    m->last_right_counts = 0;
    m->left_still_timing = false;
    m->right_still_timing = false;
    m->left_still_since_ms = 0U;
    m->right_still_since_ms = 0U;
    m->encoder_fault = false;
    m->rejected_stale = 0U;
    m->expired_count = 0U;

    m->cmd.id = 0U;
    m->cmd.type = MOTION_CMD_NONE;
    m->cmd.speed_mm_s = 0;
    m->cmd.diff_mm_s = 0;
    m->cmd.distance_mm = 0;
    m->cmd.angle_deg = 0;
    m->cmd.lease_ms = 0U;

    m->status.cmd_id = 0U;
    m->status.state = MOTION_IDLE;
    m->status.flags = 0U;
    m->status.left_mm_s = 0;
    m->status.right_mm_s = 0;
    m->status.distance_mm = 0;
    m->status.timestamp_ms = 0U;
}

const motion_status_t *motion_status(const motion_t *m)
{
    return &m->status;
}

/* ---- command intake -------------------------------------------------- */

static void end_command(motion_t *m, motion_state_t state)
{
    m->active = false;
    m->cancelled_valid = true;
    m->cancelled_id = m->cmd.id;
    m->status.state = state;
    pid_reset(&m->pid_left);
    pid_reset(&m->pid_right);
}

static uint32_t lease_for(const motion_t *m, const motion_cmd_t *cmd)
{
    return (cmd->lease_ms != 0U) ? cmd->lease_ms : m->cfg.default_lease_ms;
}

bool motion_submit(motion_t *m, const motion_cmd_t *cmd, uint32_t now_ms)
{
    if (cmd->type == MOTION_CMD_NONE) {
        return false;
    }

    if (cmd->type == MOTION_CMD_STOP) {
        if (m->active) {
            end_command(m, MOTION_IDLE);
        }
        m->cmd = *cmd;                  /* status.cmd_id now names the STOP */
        m->status.state = MOTION_IDLE;
        m->have_id = true;
        m->last_id = cmd->id;
        return true;
    }

    if (m->override_stop) {
        return false;
    }
    if (m->have_id && !id_not_older(cmd->id, m->last_id)) {
        m->rejected_stale++;
        return false;
    }
    if (m->cancelled_valid && (cmd->id == m->cancelled_id)) {
        return false;
    }

    if (m->active && (cmd->id == m->cmd.id)) {
        /* A refresh: keep the lease alive.  Only a continuous DRIVE takes
         * new targets; a finite manoeuvre is never restarted. */
        if (m->cmd.type == MOTION_CMD_DRIVE && cmd->type == MOTION_CMD_DRIVE) {
            m->cmd.speed_mm_s = cmd->speed_mm_s;
            m->cmd.diff_mm_s = cmd->diff_mm_s;
        }
        m->lease_deadline_ms = now_ms + lease_for(m, cmd);
        return true;
    }

    m->cmd = *cmd;
    m->active = true;
    m->have_id = true;
    m->last_id = cmd->id;
    m->base_pending = true;
    m->lease_deadline_ms = now_ms + lease_for(m, cmd);
    m->status.state = MOTION_RUNNING;
    pid_reset(&m->pid_left);
    pid_reset(&m->pid_right);
    return true;
}

void motion_set_override(motion_t *m, bool stop)
{
    m->override_stop = stop;
}

/* ---- estimates ------------------------------------------------------- */

static void push_history(motion_t *m, int32_t left, int32_t right, uint32_t now_us)
{
    motion_sample_t *slot = &m->history[m->history_head];

    slot->left_counts = left;
    slot->right_counts = right;
    slot->time_us = now_us;
    m->history_head = (uint8_t)((m->history_head + 1U) % MOTION_HISTORY_LEN);
    if (m->history_count < MOTION_HISTORY_LEN) {
        m->history_count++;
    }
}

/* Speed over (at least) the averaging window, in mm/s. */
static void estimate_speed(motion_t *m, int32_t left, int32_t right, uint32_t now_us)
{
    uint32_t window_us = (uint32_t)m->cfg.speed_window_ms * (uint32_t)US_PER_MS;
    const motion_sample_t *ref = NULL;
    uint8_t i;

    for (i = 1U; i < m->history_count; i++) {
        uint8_t idx = (uint8_t)((m->history_head + MOTION_HISTORY_LEN - 1U - i) %
                                MOTION_HISTORY_LEN);
        const motion_sample_t *s = &m->history[idx];

        ref = s;
        if ((uint32_t)(now_us - s->time_us) >= window_us) {
            break;
        }
    }
    if (ref == NULL) {
        return;
    }

    {
        uint32_t dt_us = now_us - ref->time_us;
        int32_t  dl_um = motion_counts_to_um(&m->cfg, left - ref->left_counts);
        int32_t  dr_um = motion_counts_to_um(&m->cfg, right - ref->right_counts);

        if (dt_us != 0U) {
            m->speed_left = (int32_t)(((int64_t)dl_um * US_PER_MS) / (int64_t)dt_us);
            m->speed_right = (int32_t)(((int64_t)dr_um * US_PER_MS) / (int64_t)dt_us);
        }
    }
}

/* Has a wheel been driven hard without its encoder counting? */
static bool wheel_stuck(int32_t counts, int32_t *last_counts, bool *timing,
                        uint32_t *since, int32_t pwm, uint32_t now_ms,
                        const motion_config_t *cfg)
{
    bool driven = abs32(pwm) >= (int32_t)cfg->encoder_fault_pwm;

    if ((counts != *last_counts) || !driven) {
        *last_counts = counts;
        *timing = false;
        return false;
    }
    if (!*timing) {
        *timing = true;
        *since = now_ms;
        return false;
    }
    return (now_ms - *since) >= cfg->encoder_fault_ms;
}

/* ---- manoeuvre targets ----------------------------------------------- */

/* Approach speed for a finite manoeuvre with 'remaining_um' to go. */
static int32_t approach_speed(const motion_t *m, int32_t remaining_um, int32_t limit)
{
    int32_t remaining_mm = remaining_um / UM_PER_MM;
    int32_t v = remaining_mm * (int32_t)m->cfg.decel_per_s;

    if (v < (int32_t)m->cfg.creep_speed_mm_s) {
        v = (int32_t)m->cfg.creep_speed_mm_s;
    }
    return (v > limit) ? limit : v;
}

/* Fills the wheel speed targets for the active command; returns true when a
 * finite manoeuvre has just completed. */
static bool compute_targets(motion_t *m, int32_t left, int32_t right,
                            int32_t *target_left, int32_t *target_right)
{
    const int32_t max_speed = (int32_t)m->cfg.max_speed_mm_s;
    int32_t limit;

    *target_left = 0;
    *target_right = 0;

    switch (m->cmd.type) {
    case MOTION_CMD_DRIVE: {
        int32_t speed = clamp32(m->cmd.speed_mm_s, -max_speed, max_speed);
        int32_t half_diff = m->cmd.diff_mm_s / 2;

        *target_left = speed - half_diff;
        *target_right = speed + half_diff;
        return false;
    }

    case MOTION_CMD_MOVE: {
        int32_t goal = motion_um_to_counts(&m->cfg,
                                           (int32_t)m->cmd.distance_mm * UM_PER_MM);
        int32_t progress = ((left - m->base_left) + (right - m->base_right)) / 2;
        int32_t remaining = goal - progress;
        int32_t dir = (goal >= 0) ? 1 : -1;
        int32_t v;
        int32_t imbalance_um;
        int32_t correction;

        if ((dir * remaining) <= 0) {
            return true;
        }
        limit = (m->cmd.speed_mm_s != 0) ? abs32(m->cmd.speed_mm_s) : max_speed;
        limit = (limit > max_speed) ? max_speed : limit;
        v = approach_speed(m, motion_counts_to_um(&m->cfg, abs32(remaining)), limit);

        imbalance_um = motion_counts_to_um(&m->cfg,
                           (left - m->base_left) - (right - m->base_right));
        correction = clamp32((m->cfg.balance_mm_s_per_mm * imbalance_um) / UM_PER_MM,
                             -(max_speed / BALANCE_LIMIT_DIV),
                             (max_speed / BALANCE_LIMIT_DIV));
        *target_left = (dir * v) - correction;
        *target_right = (dir * v) + correction;
        return false;
    }

    case MOTION_CMD_TURN: {
        int64_t arc_um = ((int64_t)m->cmd.angle_deg * (int64_t)m->cfg.track_width_mm *
                          PI_MICRO) / (FULL_TURN_DEG * 1000LL);
        int32_t goal = motion_um_to_counts(&m->cfg, (int32_t)arc_um);
        int32_t progress = ((right - m->base_right) - (left - m->base_left)) / 2;
        int32_t remaining = goal - progress;
        int32_t dir = (goal >= 0) ? 1 : -1;
        int32_t v;

        if ((dir * remaining) <= 0) {
            return true;
        }
        limit = (int32_t)m->cfg.turn_speed_mm_s;
        v = approach_speed(m, motion_counts_to_um(&m->cfg, abs32(remaining)), limit);
        *target_left = -dir * v;
        *target_right = dir * v;
        return false;
    }

    default:
        return false;
    }
}

static int32_t wheel_pwm(pid_ctrl_t *pid, int32_t target, int32_t measured,
                         int32_t ff_num, int32_t ff_den, int32_t offset,
                         const motion_config_t *cfg, uint32_t dt_ms)
{
    const int32_t limit = (cfg->pwm_limit > 0) ? (int32_t)cfg->pwm_limit : MOTION_PWM_MAX;
    int32_t pwm;

    if (target == 0) {
        pid_reset(pid);
        return 0;
    }
    pwm = (target * ff_num) / ff_den;
    pwm += (target > 0) ? offset : -offset;
    pwm += pid_update(pid, target, measured, dt_ms);
    return clamp32(pwm, -limit, limit);
}

/* ---- the control step ------------------------------------------------ */

void motion_step(motion_t *m,
                 const encoder_snapshot_t *left,
                 const encoder_snapshot_t *right,
                 uint32_t now_ms,
                 uint32_t now_us,
                 motion_output_t *out)
{
    int32_t target_left = 0;
    int32_t target_right = 0;
    int32_t pwm_left = 0;
    int32_t pwm_right = 0;
    uint32_t dt_ms = m->have_step ? (now_ms - m->last_step_ms) : 0U;
    bool finished = false;

    if (!m->origin_set) {
        m->origin_left = left->count;
        m->origin_right = right->count;
        m->origin_set = true;
    }
    estimate_speed(m, left->count, right->count, now_us);
    push_history(m, left->count, right->count, now_us);
    m->have_step = true;
    m->last_step_ms = now_ms;

    if (m->override_stop) {
        if (m->active) {
            end_command(m, MOTION_OVERRIDDEN);
        }
    } else if (m->active) {
        if ((int32_t)(now_ms - m->lease_deadline_ms) >= 0) {
            m->expired_count++;
            end_command(m, MOTION_EXPIRED);
        } else {
            if (m->base_pending) {
                m->base_left = left->count;
                m->base_right = right->count;
                m->base_pending = false;
            }
            finished = compute_targets(m, left->count, right->count,
                                       &target_left, &target_right);
            if (finished) {
                target_left = 0;
                target_right = 0;
                end_command(m, MOTION_DONE);
            }
        }
    }

    if (m->active && (dt_ms != 0U)) {
        const bool own_ff = (m->cfg.ff_right_den != 0);

        pwm_left = wheel_pwm(&m->pid_left, target_left, m->speed_left,
                             m->cfg.ff_num, m->cfg.ff_den, m->cfg.pwm_offset_left,
                             &m->cfg, dt_ms);
        pwm_right = wheel_pwm(&m->pid_right, target_right, m->speed_right,
                              own_ff ? m->cfg.ff_right_num : m->cfg.ff_num,
                              own_ff ? m->cfg.ff_right_den : m->cfg.ff_den,
                              m->cfg.pwm_offset_right, &m->cfg, dt_ms);
    }
    out->left_pwm = (int16_t)pwm_left;
    out->right_pwm = (int16_t)pwm_right;

    /* Health: PWM applied but an encoder silent. */
    {
        bool stuck_l = wheel_stuck(left->count, &m->last_left_counts,
                                   &m->left_still_timing, &m->left_still_since_ms,
                                   pwm_left, now_ms, &m->cfg);
        bool stuck_r = wheel_stuck(right->count, &m->last_right_counts,
                                   &m->right_still_timing, &m->right_still_since_ms,
                                   pwm_right, now_ms, &m->cfg);

        m->encoder_fault = stuck_l || stuck_r;
    }

    /* Stopped confirmation: both wheels below the threshold for a while. */
    if ((abs32(m->speed_left) < (int32_t)m->cfg.stopped_speed_mm_s) &&
        (abs32(m->speed_right) < (int32_t)m->cfg.stopped_speed_mm_s)) {
        if (!m->stopped_timing) {
            m->stopped_timing = true;
            m->stopped_since_ms = now_ms;
        }
    } else {
        m->stopped_timing = false;
    }

    m->status.cmd_id = m->cmd.id;
    m->status.timestamp_ms = now_ms;
    m->status.left_mm_s = (int16_t)clamp32(m->speed_left, INT16_MIN, INT16_MAX);
    m->status.right_mm_s = (int16_t)clamp32(m->speed_right, INT16_MIN, INT16_MAX);
    m->status.distance_mm =
        (motion_counts_to_um(&m->cfg, left->count - m->origin_left) +
         motion_counts_to_um(&m->cfg, right->count - m->origin_right)) /
        (2 * UM_PER_MM);

    m->status.flags = 0U;
    if (m->stopped_timing &&
        ((now_ms - m->stopped_since_ms) >= m->cfg.stopped_confirm_ms)) {
        m->status.flags |= MOTION_FLAG_STOPPED;
    }
    if (m->status.state == MOTION_DONE) {
        m->status.flags |= MOTION_FLAG_DONE;
    }
    if (m->status.state == MOTION_EXPIRED) {
        m->status.flags |= MOTION_FLAG_EXPIRED;
    }
    if (m->encoder_fault) {
        m->status.flags |= MOTION_FLAG_ENCODER_FAULT;
    }
    if (m->override_stop) {
        m->status.flags |= MOTION_FLAG_OVERRIDE;
    }
}
