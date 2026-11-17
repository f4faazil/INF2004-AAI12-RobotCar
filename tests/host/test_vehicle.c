/*
 * test_vehicle.c
 * Scenario tests for the Vehicle Controller.  Motion Control is replaced by a
 * small fake that obeys the real contract (finite commands complete after a
 * delay and report DONE for their own id; STOP reports STOPPED a little
 * later; DRIVE keeps running), so the tests check the controller's command
 * sequences, its waiting rules and its safety behaviour, not just its state
 * names.
 */
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "telemetry.h"
#include "test_port.h"
#include "test_support.h"
#include "vehicle.h"

#define LOG_MAX     (128)
#define EV_MAX      (256)

typedef struct {
    vehicle_t       vc;
    vehicle_in_t    in;
    vehicle_out_t   out;
    uint32_t        t;

    uint16_t        m_id;
    motion_cmd_type_t m_type;
    bool            m_running;
    bool            m_done;
    bool            m_stopped;
    uint32_t        m_finish_at;
    uint32_t        m_stop_at;
    int32_t         m_distance_um;
    int16_t         m_speed;
    uint32_t        finite_ms;
    uint32_t        stop_ms;
    bool            never_complete;
    bool            never_stop;
    int             status_lag;             /* cycles the status report trails reality */
    motion_status_t hist[16];
    int             hist_n;

    motion_cmd_t    log[LOG_MAX];
    int             log_n;
    int             cal_begin;
    int             cal_end;
    bool            cal_ok;
    int             scans;
    int             consumes;
    uint16_t        last_consume;
    vehicle_event_t ev[EV_MAX];
    int             ev_n;
    fault_bits_t    raised;
} h_t;

static vehicle_config_t make_cfg(void)
{
    vehicle_config_t c;

    memset(&c, 0, sizeof(c));
    c.cruise_mm_s = 150;
    c.curve_mm_s = 80;
    c.hump_mm_s = 100;
    c.creep_mm_s = 60;
    c.recover_mm_s = 60;
    c.junction_advance_mm = 70;
    c.turn_deg = 90;
    c.uturn_deg = 180;
    c.cal_sweep_deg[0] = 25;
    c.cal_sweep_deg[1] = -50;
    c.cal_sweep_deg[2] = 25;
    c.search_sweep_deg[0] = 30;
    c.search_sweep_deg[1] = -60;
    c.search_sweep_deg[2] = 90;
    c.search_sweep_deg[3] = -120;
    c.search_sweep_count = 4U;
    c.boot_ms = 20U;
    c.stop_confirm_timeout_ms = 1000U;
    c.scan_timeout_ms = 20000U;
    c.reacquire_timeout_ms = 2000U;
    c.reacquire_limit_mm = 300;
    c.recovery_time_limit_ms = 5000U;
    c.recovery_distance_limit_mm = 500;
    c.obstacle_blank_ms = 2000U;
    c.max_reverse_attempts = 1U;
    c.found_cycles = 3U;
    c.cmd_timeout_base_ms = 2000U;
    c.cmd_timeout_per_mm_ms = 30U;
    c.cmd_timeout_per_deg_ms = 30U;
    c.allow_remote_stop = true;
    return c;
}

static void line_ok(h_t *h, int16_t pos, int16_t steer)
{
    h->in.line.position_valid = true;
    h->in.line.position = pos;
    h->in.line.steer_diff_mm_s = steer;
    h->in.line.line_lost = false;
    h->in.line.stale = false;
}

static void line_gone(h_t *h)
{
    h->in.line.position_valid = false;
    h->in.line.line_lost = true;
}

static void h_init(h_t *h)
{
    vehicle_config_t cfg = make_cfg();

    memset(h, 0, sizeof(*h));
    vehicle_init(&h->vc, &cfg);
    h->t = 1000U;
    h->finite_ms = 200U;
    h->stop_ms = 50U;
    h->m_stopped = true;
    h->cal_ok = true;
    line_ok(h, 0, 0);
}

static void apply_command(h_t *h, const motion_cmd_t *c)
{
    if (c->type == MOTION_CMD_NONE) {
        return;
    }
    if (c->type == MOTION_CMD_STOP) {
        h->m_id = c->id;
        h->m_type = MOTION_CMD_STOP;
        h->m_running = false;
        h->m_done = false;
        h->m_stopped = false;
        h->m_stop_at = h->t + h->stop_ms;
        if (h->log_n < LOG_MAX) { h->log[h->log_n++] = *c; }
        return;
    }
    if (c->id != h->m_id) {
        h->m_id = c->id;
        h->m_type = c->type;
        h->m_running = true;
        h->m_done = false;
        h->m_stopped = false;
        h->m_finish_at = h->t + h->finite_ms;
        h->m_speed = (c->type == MOTION_CMD_DRIVE) ? c->speed_mm_s : 0;
        if (h->log_n < LOG_MAX) { h->log[h->log_n++] = *c; }
    } else if (c->type == MOTION_CMD_DRIVE) {
        h->m_speed = c->speed_mm_s;             /* a refresh updates a DRIVE */
    } else {
        /* a refresh of a finite command: no restart */
    }
}

static void h_cycle(h_t *h)
{
    h->t += 10U;
    h->in.now_ms = h->t;

    if (!h->m_stopped && (h->m_type == MOTION_CMD_STOP) && !h->never_stop &&
        (h->t >= h->m_stop_at)) {
        h->m_stopped = true;
    }
    if (h->m_running && (h->m_type != MOTION_CMD_DRIVE) && !h->never_complete &&
        (h->t >= h->m_finish_at)) {
        h->m_running = false;
        h->m_done = true;
        h->m_stopped = true;
    }
    if (h->m_running && (h->m_type == MOTION_CMD_DRIVE)) {
        h->m_distance_um += (int32_t)h->m_speed * 10;      /* mm/s x 10 ms = um */
    }
    {
        motion_status_t now_status;
        int slot;

        now_status.cmd_id = h->m_id;
        now_status.flags = (uint8_t)((h->m_stopped ? MOTION_FLAG_STOPPED : 0U) |
                                     (h->m_done ? MOTION_FLAG_DONE : 0U));
        now_status.state = h->m_done ? MOTION_DONE : (h->m_running ? MOTION_RUNNING : MOTION_IDLE);
        now_status.distance_mm = h->m_distance_um / 1000;
        now_status.left_mm_s = 0;
        now_status.right_mm_s = 0;
        now_status.timestamp_ms = h->t;
        h->hist[h->hist_n % 16] = now_status;
        slot = h->hist_n - h->status_lag;
        h->in.motion = h->hist[((slot < 0) ? 0 : slot) % 16];
        h->hist_n++;
    }

    vehicle_step(&h->vc, &h->in, &h->out);

    apply_command(h, &h->out.cmd);
    if (h->out.calibrate_begin) {
        h->cal_begin++;
        h->in.calibration_done = false;
    }
    if (h->out.calibrate_end) {
        h->cal_end++;
        h->in.calibration_done = true;
        h->in.calibration_ok = h->cal_ok;
    }
    if (h->out.consume_instruction) {
        h->consumes++;
        h->last_consume = h->out.consume_id;
        h->in.instruction_valid = false;
    }
    if (h->out.request_scan) {
        h->scans++;
        h->in.scan_done = false;
    }
    h->in.faults = (fault_bits_t)((h->in.faults | h->out.raise_faults) &
                                  (fault_bits_t)(~h->out.clear_faults));
    h->raised = (fault_bits_t)(h->raised | h->out.raise_faults);
    {
        uint8_t i;

        for (i = 0U; i < h->out.event_count; i++) {
            if (h->ev_n < EV_MAX) { h->ev[h->ev_n++] = h->out.events[i]; }
        }
    }
    h->in.line.events = 0U;
    h->in.remote = 0U;
    h->in.start_pressed = false;
    h->in.stop_pressed = false;
}

static void cycles(h_t *h, int n)
{
    int i;

    for (i = 0; i < n; i++) {
        h_cycle(h);
    }
}

/* Cycle until 'state' is reached or 'max' cycles pass. */
static bool until(h_t *h, vehicle_state_t state, int max)
{
    int i;

    for (i = 0; i < max; i++) {
        if (h->out.state == state) {
            return true;
        }
        h_cycle(h);
    }
    return h->out.state == state;
}

static void to_follow(h_t *h)
{
    cycles(h, 6);
    h->in.start_pressed = true;
    h_cycle(h);
    (void)until(h, VC_FOLLOW, 400);
}

static int count_type(const h_t *h, motion_cmd_type_t type)
{
    int i;
    int n = 0;

    for (i = 0; i < h->log_n; i++) {
        if (h->log[i].type == type) { n++; }
    }
    return n;
}

static bool saw_event(const h_t *h, uint8_t code, uint16_t arg)
{
    int i;

    for (i = 0; i < h->ev_n; i++) {
        if ((h->ev[i].code == code) && (h->ev[i].arg == arg)) { return true; }
    }
    return false;
}

static void junction(h_t *h, uint8_t mask)
{
    h->in.line.events = NAV_EVENT_JUNCTION;
    h->in.line.junction_mask = mask;
}

static nav_instruction_t instr(char letter, nav_command_t cmd, uint16_t id)
{
    nav_instruction_t i;

    i.letter = letter;
    i.command = cmd;
    i.id = id;
    i.timestamp_ms = 0U;
    return i;
}

/* ---- boot, calibration, start ------------------------------------------ */

/* The car boots into IDLE and sends a STOP so the wheels are known to be still
 * until the operator starts it. */
static void test_boots_into_idle_and_commands_stop(void)
{
    h_t h;

    h_init(&h);
    cycles(&h, 6);
    CHECK_INT(VC_IDLE, h.out.state);
    CHECK(h.log_n >= 1);
    CHECK_INT(MOTION_CMD_STOP, h.log[0].type);
    CHECK_INT(0, h.out.activity);
}

/* START runs a calibration sweep (+25, -50, +25 degrees) over the line, then
 * ends calibration and, if it worked, starts following.  Each sweep leg is a
 * finite command that is refreshed until it completes.  Catches: calibrating
 * while still moving, or following without calibration. */
static void test_start_sweeps_calibrates_then_follows(void)
{
    h_t h;

    h_init(&h);
    cycles(&h, 6);
    h.in.start_pressed = true;
    h_cycle(&h);
    CHECK_INT(VC_CALIBRATE, h.out.state);
    CHECK(until(&h, VC_FOLLOW, 400));

    CHECK_INT(1, h.cal_begin);
    CHECK_INT(1, h.cal_end);
    CHECK_INT(MOTION_CMD_TURN, h.log[1].type);
    CHECK_INT(25, h.log[1].angle_deg);
    CHECK_INT(-50, h.log[2].angle_deg);
    CHECK_INT(25, h.log[3].angle_deg);
    CHECK_INT(MOTION_CMD_DRIVE, h.log[4].type);
    CHECK((h.out.activity & VC_ACT_LINE_REQUIRED) != 0U);
    CHECK((h.out.activity & VC_ACT_DRIVING) != 0U);
    CHECK((h.out.activity & VC_ACT_OBSTACLE_WATCH) != 0U);
}

/* A failed calibration is a stop-class fault: the car must not follow a line
 * it cannot see. */
static void test_calibration_failure_stops_with_a_fault(void)
{
    h_t h;

    h_init(&h);
    h.cal_ok = false;
    cycles(&h, 6);
    h.in.start_pressed = true;
    h_cycle(&h);
    CHECK(until(&h, VC_FAULT, 400));
    CHECK((h.raised & FAULT_LINE_CAL) != 0U);
    CHECK_INT(MOTION_CMD_STOP, h.log[h.log_n - 1].type);
    CHECK_INT(0, count_type(&h, MOTION_CMD_DRIVE));
}

/* ---- following ---------------------------------------------------------- */

/* While following, one DRIVE command (one id) is refreshed with the steering
 * request; the speed drops with the line error and on a hump.  Catches: a new
 * id every cycle (which would restart the motion layer) or no slowing in a
 * curve. */
static void test_follow_refreshes_one_drive_command_and_slows_in_curves(void)
{
    h_t h;
    int drives_before;

    h_init(&h);
    to_follow(&h);
    drives_before = count_type(&h, MOTION_CMD_DRIVE);

    line_ok(&h, 0, 20);
    cycles(&h, 20);
    CHECK_INT(MOTION_CMD_DRIVE, h.out.cmd.type);
    CHECK_INT(150, h.out.cmd.speed_mm_s);
    CHECK_INT(20, h.out.cmd.diff_mm_s);

    line_ok(&h, 1000, -150);
    cycles(&h, 2);
    CHECK_INT(80, h.out.cmd.speed_mm_s);                /* full error: slowest */
    line_ok(&h, 500, -60);
    cycles(&h, 2);
    CHECK(h.out.cmd.speed_mm_s < 150);
    CHECK(h.out.cmd.speed_mm_s > 80);

    h.in.hump_in_progress = true;
    line_ok(&h, 0, 0);
    cycles(&h, 2);
    CHECK_INT(100, h.out.cmd.speed_mm_s);               /* hump speed limit */
    CHECK_INT(drives_before, count_type(&h, MOTION_CMD_DRIVE));
}

/* A momentary loss of the line position (not yet "lost") slows the car and
 * holds it straight instead of steering on nothing. */
static void test_follow_without_a_position_goes_slow_and_straight(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.in.line.position_valid = false;
    h.in.line.line_lost = false;
    h.in.line.steer_diff_mm_s = 99;
    cycles(&h, 3);
    CHECK_INT(80, h.out.cmd.speed_mm_s);
    CHECK_INT(0, h.out.cmd.diff_mm_s);
}

/* Stale line data while following stops the car at once (Tribunal: stale
 * steering data means a sensor-data fault and STOP). */
static void test_stale_line_data_stops_the_car(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.in.line.stale = true;
    cycles(&h, 2);
    CHECK_INT(MOTION_CMD_STOP, h.log[h.log_n - 1].type);
    CHECK(h.out.state != VC_FOLLOW);
}

/* ---- junctions and barcode instructions ------------------------------ */

/* A LEFT instruction executes at the junction: advance to put the wheels over
 * it, turn +90, reacquire the line, resume.  It is consumed exactly once, and
 * a stale copy of it showing up again does not re-execute.  Catches: a turn
 * at the barcode instead of the junction, a wrong direction, and duplicate
 * execution (Tribunal rules). */
static void test_left_instruction_executes_once_at_the_junction(void)
{
    h_t h;
    int turns_before;

    h_init(&h);
    to_follow(&h);
    turns_before = count_type(&h, MOTION_CMD_TURN);
    h.in.instruction_valid = true;
    h.in.instruction = instr('A', NAV_CMD_LEFT, 3U);

    cycles(&h, 5);                                       /* barcode seen, no junction yet */
    CHECK_INT(VC_FOLLOW, h.out.state);
    CHECK_INT(turns_before, count_type(&h, MOTION_CMD_TURN));
    CHECK_INT(0, h.consumes);

    junction(&h, NAV_JUNCTION_LEFT | NAV_JUNCTION_RIGHT);
    h_cycle(&h);
    CHECK_INT(VC_JUNCTION_ADVANCE, h.out.state);
    CHECK_INT(1, h.consumes);
    CHECK_INT(3, h.last_consume);
    CHECK_INT(MOTION_CMD_MOVE, h.log[h.log_n - 1].type);
    CHECK_INT(70, h.log[h.log_n - 1].distance_mm);

    CHECK(until(&h, VC_JUNCTION_TURN, 100));
    CHECK_INT(MOTION_CMD_TURN, h.log[h.log_n - 1].type);
    CHECK_INT(90, h.log[h.log_n - 1].angle_deg);

    line_gone(&h);
    CHECK(until(&h, VC_REACQUIRE, 100));
    CHECK_INT(MOTION_CMD_DRIVE, h.log[h.log_n - 1].type);
    CHECK_INT(60, h.log[h.log_n - 1].speed_mm_s);        /* creep */

    line_ok(&h, 0, 0);
    CHECK(until(&h, VC_FOLLOW, 20));
    CHECK(saw_event(&h, TEL_EVT_NAV_EXEC, NAV_CMD_LEFT));

    /* the same instruction reappears (the consume was slow): not run again */
    h.in.instruction_valid = true;
    h.in.instruction = instr('A', NAV_CMD_LEFT, 3U);
    junction(&h, NAV_JUNCTION_LEFT);
    h_cycle(&h);
    CHECK_INT(VC_JUNCTION_ADVANCE, h.out.state);  /* crosses the junction blind... */
    CHECK(until(&h, VC_FOLLOW, 100));          /* ...and carries on */
    CHECK_INT(1, count_type(&h, MOTION_CMD_TURN) - turns_before);
    CHECK_INT(1, h.consumes);
}

/* RIGHT turns -90; UTURN turns 180. */
static void test_right_and_uturn_instructions_turn_the_right_angle(void)
{
    static const struct { nav_command_t cmd; int16_t deg; } CASES[] = {
        { NAV_CMD_RIGHT, -90 }, { NAV_CMD_UTURN, 180 }
    };
    size_t i;

    for (i = 0U; i < 2U; i++) {
        h_t h;

        h_init(&h);
        to_follow(&h);
        h.in.instruction_valid = true;
        h.in.instruction = instr('X', CASES[i].cmd, 1U);
        junction(&h, NAV_JUNCTION_LEFT | NAV_JUNCTION_RIGHT);
        h_cycle(&h);
        CHECK(until(&h, VC_JUNCTION_TURN, 100));
        CHECK_INT(MOTION_CMD_TURN, h.log[h.log_n - 1].type);
        CHECK_INT(CASES[i].deg, h.log[h.log_n - 1].angle_deg);
    }
}

/* STRAIGHT is consumed and the car crosses the junction blind (advance only,
 * no turn) so the branches cannot pull the steering. */
static void test_straight_instruction_crosses_without_turning(void)
{
    h_t h;
    int turns_before;

    h_init(&h);
    to_follow(&h);
    turns_before = count_type(&h, MOTION_CMD_TURN);
    h.in.instruction_valid = true;
    h.in.instruction = instr('W', NAV_CMD_STRAIGHT, 5U);
    junction(&h, NAV_JUNCTION_LEFT | NAV_JUNCTION_RIGHT);
    h_cycle(&h);
    CHECK_INT(1, h.consumes);
    CHECK(until(&h, VC_FOLLOW, 100));
    CHECK_INT(turns_before, count_type(&h, MOTION_CMD_TURN));
    CHECK_INT(1, count_type(&h, MOTION_CMD_MOVE));
}

/* A junction with no instruction: cross it straight, consume nothing. */
static void test_junction_without_an_instruction_goes_straight(void)
{
    h_t h;
    int turns_before;

    h_init(&h);
    to_follow(&h);
    turns_before = count_type(&h, MOTION_CMD_TURN);    /* the calibration sweep */
    junction(&h, NAV_JUNCTION_RIGHT);
    h_cycle(&h);
    CHECK_INT(0, h.consumes);
    CHECK(until(&h, VC_FOLLOW, 100));
    CHECK_INT(turns_before, count_type(&h, MOTION_CMD_TURN));
}

/* ---- line lost ---------------------------------------------------------- */

/* Line lost: stop, then search by sweeping left and right; if the line is
 * seen again following resumes. */
static void test_lost_line_is_searched_for_and_resumed(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    line_gone(&h);
    h.in.line.events = NAV_EVENT_LINE_LOST;
    h_cycle(&h);
    CHECK_INT(VC_SEARCH_LINE, h.out.state);
    CHECK_INT(MOTION_CMD_STOP, h.log[h.log_n - 1].type);

    cycles(&h, 12);                                      /* stopped, first sweep leg */
    CHECK_INT(MOTION_CMD_TURN, h.log[h.log_n - 1].type);
    CHECK_INT(30, h.log[h.log_n - 1].angle_deg);

    line_ok(&h, 100, -10);
    CHECK(until(&h, VC_FOLLOW, 10));
    CHECK_INT(MOTION_CMD_DRIVE, h.log[h.log_n - 1].type);
}

/* If the whole sweep finds nothing, that is a stop-class fault. */
static void test_line_never_found_is_a_fault(void)
{
    h_t h;
    int base;

    h_init(&h);
    to_follow(&h);
    line_gone(&h);
    h.in.line.events = NAV_EVENT_LINE_LOST;
    base = h.log_n;
    h_cycle(&h);
    CHECK(until(&h, VC_FAULT, 400));
    CHECK((h.raised & FAULT_LINE_LOST) != 0U);
    CHECK_INT(MOTION_CMD_STOP, h.log[base].type);
    CHECK_INT(30, h.log[base + 1].angle_deg);
    CHECK_INT(-60, h.log[base + 2].angle_deg);
    CHECK_INT(90, h.log[base + 3].angle_deg);
    CHECK_INT(-120, h.log[base + 4].angle_deg);
}

/* ---- obstacles ---------------------------------------------------------- */

static bypass_plan_t left_plan(void)
{
    bypass_plan_t p;

    memset(&p, 0, sizeof(p));
    p.action = PLAN_BYPASS_LEFT;
    p.reason = PLAN_REASON_OK;
    p.step_count = 5U;
    p.steps[0].is_turn = true;   p.steps[0].value = 90;
    p.steps[1].is_turn = false;  p.steps[1].value = 140;
    p.steps[2].is_turn = true;   p.steps[2].value = -90;
    p.steps[3].is_turn = false;  p.steps[3].value = 680;
    p.steps[4].is_turn = true;   p.steps[4].value = -90;
    p.return_leg_mm = 140U;
    return p;
}

/* The scan waits for Motion to CONFIRM the car has stopped.  Catches: scanning
 * while the car is still coasting (Tribunal: sending a stop request does not
 * establish that the vehicle has stopped). */
static void test_scan_starts_only_after_the_stop_is_confirmed(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.stop_ms = 300U;
    h.in.obstacle_present = true;
    h_cycle(&h);
    CHECK_INT(VC_OBSTACLE_STOP, h.out.state);
    CHECK_INT(MOTION_CMD_STOP, h.log[h.log_n - 1].type);
    cycles(&h, 20);                                      /* 200 ms: still coasting */
    CHECK_INT(0, h.scans);
    CHECK_INT(VC_OBSTACLE_STOP, h.out.state);
    cycles(&h, 15);
    CHECK_INT(1, h.scans);
    CHECK_INT(VC_OBSTACLE_SCAN, h.out.state);
}

/* The motion status can trail reality by a cycle or two.  A STOPPED flag left
 * over from an earlier, finished command must not be taken as confirmation
 * that the NEW stop has worked.  Catches: starting the scan while the car is
 * still moving because the status was out of date. */
static void test_a_stale_stopped_flag_does_not_confirm_a_new_stop(void)
{
    h_t h;

    h_init(&h);
    h.status_lag = 4;
    to_follow(&h);                  /* the last thing Motion finished was a TURN: DONE + STOPPED */
    h.stop_ms = 200U;
    h.in.obstacle_present = true;
    h_cycle(&h);
    CHECK_INT(VC_OBSTACLE_STOP, h.out.state);
    cycles(&h, 3);                  /* the report still shows the old, stopped state */
    CHECK_INT(0, h.scans);
    cycles(&h, 40);                 /* now the real stop has been reported */
    CHECK_INT(1, h.scans);
}

/* A car that never reports stopped is a motion fault, not an endless wait. */
static void test_unconfirmed_stop_is_a_motion_fault(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.never_stop = true;
    h.in.obstacle_present = true;
    cycles(&h, 120);
    CHECK_INT(VC_FAULT, h.out.state);
    CHECK((h.raised & FAULT_MOTION) != 0U);
    CHECK_INT(0, h.scans);
}

/* A full left bypass: the plan's five steps are sent in order as separate
 * commands, then the car creeps to find the line, turns back to the line's
 * heading and resumes following.  Catches: steps out of order or skipped,
 * resuming before the line is found. */
static void test_left_bypass_runs_the_plan_then_recovers_the_line(void)
{
    h_t h;
    int first;

    h_init(&h);
    to_follow(&h);
    h.in.obstacle_present = true;
    CHECK(until(&h, VC_OBSTACLE_SCAN, 100));
    h.in.obstacle_present = false;
    line_gone(&h);
    h.in.scan_done = true;
    h.in.plan = left_plan();
    h_cycle(&h);
    CHECK_INT(VC_BYPASS, h.out.state);

    first = h.log_n;
    CHECK(until(&h, VC_RECOVER_LINE, 400));
    CHECK_INT(5, h.log_n - first);       /* four more steps, then the creep drive */
    CHECK_INT(MOTION_CMD_TURN, h.log[first - 1].type);
    CHECK_INT(90, h.log[first - 1].angle_deg);
    CHECK_INT(MOTION_CMD_MOVE, h.log[first].type);
    CHECK_INT(140, h.log[first].distance_mm);
    CHECK_INT(-90, h.log[first + 1].angle_deg);
    CHECK_INT(680, h.log[first + 2].distance_mm);
    CHECK_INT(-90, h.log[first + 3].angle_deg);

    cycles(&h, 5);                                       /* creeping */
    CHECK_INT(MOTION_CMD_DRIVE, h.log[h.log_n - 1].type);
    CHECK_INT(60, h.log[h.log_n - 1].speed_mm_s);

    line_ok(&h, 0, 0);
    CHECK(until(&h, VC_FOLLOW, 200));
    /* the realign turn undoes the last bypass turn: back to the line's heading */
    {
        int k;
        bool realigned = false;

        for (k = first + 4; k < h.log_n; k++) {
            if ((h.log[k].type == MOTION_CMD_TURN) && (h.log[k].angle_deg == 90)) {
                realigned = true;
            }
        }
        CHECK(realigned);
    }
    CHECK(saw_event(&h, TEL_EVT_RECOVERED, 0U));
}

/* The IR sensors sit ahead of the wheel axle, so on finding the line the car must drive on
 * by recover_advance_mm and only then turn back to the line's heading.  Turning at once
 * swings the line off to the side and the car loses it (seen on the track).  Catches: the
 * advance missing, or the turn before it. */
static void test_recovery_drives_on_before_turning_back(void)
{
    h_t h;
    int k;
    int move_k = -1;

    h_init(&h);
    h.vc.cfg.recover_advance_mm = 100;
    to_follow(&h);
    h.in.obstacle_present = true;
    CHECK(until(&h, VC_OBSTACLE_SCAN, 100));
    h.in.obstacle_present = false;
    line_gone(&h);
    h.in.scan_done = true;
    h.in.plan = left_plan();
    h_cycle(&h);
    CHECK(until(&h, VC_RECOVER_LINE, 400));
    cycles(&h, 5);                                       /* creeping */
    line_ok(&h, 0, 0);
    CHECK(until(&h, VC_FOLLOW, 300));
    for (k = h.log_n - 1; k >= 0; k--) {
        if ((h.log[k].type == MOTION_CMD_MOVE) && (h.log[k].distance_mm == 100)) {
            move_k = k;
            break;
        }
    }
    CHECK(move_k >= 0);
    CHECK((move_k + 1) < h.log_n);
    CHECK_INT(MOTION_CMD_TURN, h.log[move_k + 1].type);
    CHECK_INT(90, h.log[move_k + 1].angle_deg);
    CHECK(saw_event(&h, TEL_EVT_RECOVERED, 0U));
}

/* ---- awareness during the bypass --------------------------------------- */

/* A bypass test rig: the sonar watch is on, and each finite command takes 3 s so the legs
 * last longer than the sonar blanking period. */
static void bypass_rig(h_t *h)
{
    h_init(h);
    h->vc.cfg.bypass_stop_mm = 100U;
    h->vc.cfg.bypass_blank_ms = 500U;
    h->vc.cfg.bypass_hold_ms = 4000U;
    to_follow(h);
    h->finite_ms = 3000U;                                  /* legs outlast the sonar blanking */
    h->in.obstacle_present = true;
    (void)until(h, VC_OBSTACLE_SCAN, 100);
    h->in.obstacle_present = false;
    line_gone(h);
    h->in.scan_done = true;
    h->in.plan = left_plan();
    h_cycle(h);
}

/* Cycle until the car has started a MOVE of 'mm'. */
static bool until_move(h_t *h, int mm, int max)
{
    int i;

    for (i = 0; i < max; i++) {
        const motion_cmd_t *last = &h->log[h->log_n - 1];

        if ((last->type == MOTION_CMD_MOVE) && (last->distance_mm == mm)) {
            return true;
        }
        h_cycle(h);
    }
    return false;
}

static void sonar(h_t *h, uint16_t mm)
{
    h->in.obstacle_mm_valid = true;
    h->in.obstacle_mm = mm;
}

/* The sonar used to be ignored once the bypass began: a box (or a hand) in the way of the
 * sideways leg was driven into.  Now the car stops, waits for the way to clear, and drives
 * only what is left of the leg.  Catches: no stop; a restart of the whole leg. */
static void test_bypass_stops_for_something_in_the_way_then_finishes_the_leg(void)
{
    h_t h;
    int stops;

    bypass_rig(&h);
    CHECK(until_move(&h, 140, 800));
    cycles(&h, 60);                                         /* the leg is under way ... */
    h.m_distance_um = 60000;                                /* ... 60 mm along */
    cycles(&h, 2);
    stops = count_type(&h, MOTION_CMD_STOP);
    sonar(&h, 300U);
    cycles(&h, 5);
    CHECK_INT(stops, count_type(&h, MOTION_CMD_STOP));      /* clear: it carries on */

    sonar(&h, 80U);
    cycles(&h, 3);
    CHECK_INT(stops + 1, count_type(&h, MOTION_CMD_STOP));
    CHECK_INT(VC_BYPASS, h.out.state);
    cycles(&h, 100);
    CHECK_INT(MOTION_CMD_STOP, h.log[h.log_n - 1].type);    /* still held */

    sonar(&h, 300U);
    cycles(&h, 20);
    CHECK_INT(MOTION_CMD_STOP, h.log[h.log_n - 1].type);    /* it must stay clear a while */
    cycles(&h, 40);
    CHECK_INT(MOTION_CMD_MOVE, h.log[h.log_n - 1].type);
    CHECK(abs(h.log[h.log_n - 1].distance_mm - 80) <= 3);   /* 140 - 60, to the odometer's rounding */
    CHECK_INT(VC_BYPASS, h.out.state);
}

/* The sonar sees the box itself while the car turns, and its median holds old readings just
 * after: neither may stop the bypass.  Catches: stopping on the first turn. */
static void test_bypass_ignores_the_sonar_while_turning_and_just_after(void)
{
    h_t h;
    int stops;

    bypass_rig(&h);
    stops = count_type(&h, MOTION_CMD_STOP);
    sonar(&h, 60U);                                         /* the box, right in front */
    CHECK(until_move(&h, 140, 800));
    cycles(&h, 40);                                         /* 400 ms into the leg: blanked */
    CHECK_INT(stops, count_type(&h, MOTION_CMD_STOP));
    cycles(&h, 30);                                         /* past the blanking: now it counts */
    CHECK_INT(stops + 1, count_type(&h, MOTION_CMD_STOP));
}

/* A way that never clears is a fault, not a wait forever. */
static void test_bypass_blocked_too_long_is_a_fault(void)
{
    h_t h;

    bypass_rig(&h);
    CHECK(until_move(&h, 140, 800));
    cycles(&h, 60);
    sonar(&h, 50U);
    CHECK(until(&h, VC_FAULT, 700));
    CHECK((h.raised & FAULT_NO_SAFE_PATH) != 0U);
}

/* The last straight leg runs back toward the track's line.  If the sensors see the line
 * during it, the car follows it at once rather than driving on past.  The first sideways
 * leg starts on the line, so seeing it there must not end the bypass.  Catches: leaving the
 * bypass on the first leg; ignoring the line until the leg ends. */
static void test_bypass_follows_the_line_if_it_is_seen_on_the_last_straight_leg(void)
{
    h_t h;

    bypass_rig(&h);
    CHECK(until_move(&h, 140, 800));
    cycles(&h, 60);
    line_ok(&h, 0, 0);                                      /* still on the line's edge: ignore */
    cycles(&h, 60);
    CHECK_INT(VC_BYPASS, h.out.state);

    line_gone(&h);
    CHECK(until_move(&h, 680, 800));
    cycles(&h, 60);
    CHECK_INT(VC_BYPASS, h.out.state);
    line_ok(&h, 0, 0);
    cycles(&h, 6);
    CHECK_INT(VC_FOLLOW, h.out.state);
    CHECK(saw_event(&h, TEL_EVT_RECOVERED, 0U));
}

/* ---- looking at the box while passing it -------------------------------- */

static void side_rig(h_t *h)
{
    bypass_rig(h);
    h->vc.cfg.side_aim_deg = 25;
    h->vc.cfg.side_min_mm = 50U;
    h->vc.cfg.side_max_mm = 700U;
    h->vc.cfg.side_loss_ahead_pct = 60U;
    h->vc.cfg.side_tail_mm = 235U;
    h->vc.cfg.side_extend_mm = 100U;
    h->vc.cfg.side_extend_max_mm = 300U;
}

/* One raw sonar reading (70 ms apart): an echo at 'mm', or none when mm is 0. */
static void side_reading(h_t *h, uint16_t mm)
{
    h->in.side_seq++;
    h->in.side_echo = (mm != 0U);
    h->in.side_mm = mm;
    h->in.obstacle_mm_valid = (mm != 0U);              /* the median follows the same readings */
    h->in.obstacle_mm = mm;
    cycles(h, 7);
}

/* The sonar looks at the box from the turn before the leg beside it until that leg ends, and
 * is straight ahead the rest of the time.  A left bypass leaves the box on the right.
 * Catches: looking at it the wrong way; not looking; staying tilted afterwards. */
static void test_sonar_tilts_toward_the_box_beside_it_only(void)
{
    h_t h;

    side_rig(&h);
    CHECK_INT(0, h.out.sonar_aim);                          /* turning away: ahead */
    CHECK(until_move(&h, 140, 800));
    CHECK_INT(0, h.out.sonar_aim);                          /* sideways leg: ahead */
    cycles(&h, 400);                                        /* the leg and the next turn start */
    CHECK_INT(-1, h.out.sonar_aim);                         /* turning back: look at the box (right) */
    CHECK(until_move(&h, 680, 800));
    CHECK_INT(-1, h.out.sonar_aim);
    CHECK(until(&h, VC_RECOVER_LINE, 1500));
    CHECK_INT(0, h.out.sonar_aim);                          /* going back to the line: ahead */
}

/* The box is seen along the leg and then not: the leg is cut to what the rear needs.  Here the
 * last range was 200 mm, lost at 300 mm of travel: 300 + 200 x 60% + 235 = 655 mm in all, so
 * 355 mm are left from the 300 mm travelled.  Catches: no cut; the planned length kept. */
static void test_leg_beside_the_box_ends_when_the_box_is_passed(void)
{
    h_t h;
    int i;

    side_rig(&h);
    CHECK(until_move(&h, 680, 1500));
    cycles(&h, 55);                                         /* past the blanking */
    for (i = 0; i < 5; i++) {
        side_reading(&h, 200U);                             /* the box's side */
    }
    h.m_distance_um = 300000;                               /* 300 mm along ... */
    cycles(&h, 2);
    for (i = 0; i < 4; i++) {
        side_reading(&h, 0U);                               /* ... the echo is gone */
    }
    CHECK_INT(MOTION_CMD_MOVE, h.log[h.log_n - 1].type);
    CHECK(abs(h.log[h.log_n - 1].distance_mm - 355) <= 4);
    CHECK(saw_event(&h, TEL_EVT_OBSTACLE, 2U));
}

/* A box that was never seen (an echo that does not come back from its side) leaves the planned
 * leg alone: the sonar may only shorten the leg when it saw the box first.  Catches: cutting
 * the leg because of a missing echo. */
static void test_leg_is_not_cut_if_the_box_was_never_seen(void)
{
    h_t h;
    int i;
    int moves;

    side_rig(&h);
    CHECK(until_move(&h, 680, 1500));
    cycles(&h, 55);
    moves = count_type(&h, MOTION_CMD_MOVE);
    for (i = 0; i < 8; i++) {
        side_reading(&h, 0U);
    }
    CHECK_INT(moves, count_type(&h, MOTION_CMD_MOVE));
}

/* A box deeper than assumed is still in view when the planned leg ends: the car goes on a
 * little at a time, not past the limit, and turns only once the box is no longer seen.
 * Catches: turning toward a box that is still beside the car; no limit. */
static void test_leg_goes_on_while_the_box_is_still_beside_the_car(void)
{
    h_t h;
    int i;
    int extra = 0;

    side_rig(&h);
    CHECK(until_move(&h, 680, 1500));
    cycles(&h, 55);
    for (i = 0; i < 4; i++) {
        side_reading(&h, 300U);
    }
    for (i = 0; i < 400; i++) {                             /* the leg ends, the box is still there */
        side_reading(&h, 300U);
        if ((h.log[h.log_n - 1].type == MOTION_CMD_MOVE) && (h.log[h.log_n - 1].distance_mm == 100)) {
            extra++;
            h.log[h.log_n - 1].distance_mm = 0;             /* count each new command once */
        }
        if (h.out.state != VC_BYPASS) {
            break;
        }
    }
    CHECK_INT(3, extra);                                    /* 300 mm in 100 mm steps, then on */
    CHECK(saw_event(&h, TEL_EVT_OBSTACLE, 3U));
}

/* On the drive back to the line the sonar still watches, and the time spent waiting does not
 * use up the 5 s recovery limit.  Catches: driving on into something; a fault from a pause. */
static void test_recovery_drive_waits_for_the_way_and_keeps_its_time_limit(void)
{
    h_t h;
    int stops;

    bypass_rig(&h);
    h.finite_ms = 100U;                                     /* quick legs: we want the recovery */
    CHECK(until(&h, VC_RECOVER_LINE, 800));
    cycles(&h, 60);
    stops = count_type(&h, MOTION_CMD_STOP);
    sonar(&h, 70U);
    cycles(&h, 3);
    CHECK_INT(stops + 1, count_type(&h, MOTION_CMD_STOP));
    cycles(&h, 300);                                        /* 3 s waiting: over the 5 s with the drive */
    CHECK_INT(VC_RECOVER_LINE, h.out.state);

    sonar(&h, 400U);
    cycles(&h, 60);
    CHECK_INT(MOTION_CMD_DRIVE, h.log[h.log_n - 1].type);
    cycles(&h, 250);                                        /* driving again, within its own 5 s */
    CHECK_INT(VC_RECOVER_LINE, h.out.state);
    line_ok(&h, 0, 0);
    CHECK(until(&h, VC_FOLLOW, 300));
}

/* The recovery is bounded: no line within 5 s is a fault. */
static void test_recovery_time_limit(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.in.obstacle_present = true;
    CHECK(until(&h, VC_OBSTACLE_SCAN, 100));
    h.in.obstacle_present = false;
    line_gone(&h);
    h.in.scan_done = true;
    h.in.plan = left_plan();
    CHECK(until(&h, VC_RECOVER_LINE, 400));
    CHECK(until(&h, VC_FAULT, 700));
    CHECK((h.raised & FAULT_RECOVERY_LIMIT) != 0U);
}

/* ...and by distance: 0.5 m of creeping without finding the line is a fault,
 * whichever comes first. */
static void test_recovery_distance_limit(void)
{
    h_t h;
    uint32_t t_recover = 0U;

    h_init(&h);
    h.vc.cfg.recover_mm_s = 300;                         /* 3 mm per cycle */
    to_follow(&h);
    h.in.obstacle_present = true;
    CHECK(until(&h, VC_OBSTACLE_SCAN, 100));
    h.in.obstacle_present = false;
    line_gone(&h);
    h.in.scan_done = true;
    h.in.plan = left_plan();
    CHECK(until(&h, VC_RECOVER_LINE, 400));
    t_recover = h.t;
    CHECK(until(&h, VC_FAULT, 700));
    CHECK((h.raised & FAULT_RECOVERY_LIMIT) != 0U);
    CHECK((h.t - t_recover) < 4000U);                    /* long before the 5 s limit */
}

/* No safe path: stay stopped with a fault. */
static void test_no_safe_path_keeps_the_car_stopped(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.in.obstacle_present = true;
    CHECK(until(&h, VC_OBSTACLE_SCAN, 100));
    h.in.scan_done = true;
    memset(&h.in.plan, 0, sizeof(h.in.plan));
    h.in.plan.action = PLAN_STOP;
    h.in.plan.reason = PLAN_REASON_NO_CLEARANCE;
    cycles(&h, 3);
    CHECK_INT(VC_FAULT, h.out.state);
    CHECK((h.raised & FAULT_NO_SAFE_PATH) != 0U);
    CHECK_INT(MOTION_CMD_STOP, h.log[h.log_n - 1].type);
}

/* An invalid scan (the sensor could not see) is a sensor fault, not
 * "no safe path". */
static void test_invalid_scan_is_an_ultrasonic_fault(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.in.obstacle_present = true;
    CHECK(until(&h, VC_OBSTACLE_SCAN, 100));
    h.in.scan_done = true;
    memset(&h.in.plan, 0, sizeof(h.in.plan));
    h.in.plan.action = PLAN_STOP;
    h.in.plan.reason = PLAN_REASON_INVALID_SCAN;
    cycles(&h, 3);
    CHECK_INT(VC_FAULT, h.out.state);
    CHECK((h.raised & FAULT_ULTRASONIC) != 0U);
}

/* Too close to turn: reverse, scan again, once.  A second REVERSE is refused. */
static void test_reverse_then_rescan_once(void)
{
    h_t h;
    bypass_plan_t rev;

    h_init(&h);
    to_follow(&h);
    h.in.obstacle_present = true;
    CHECK(until(&h, VC_OBSTACLE_SCAN, 100));
    memset(&rev, 0, sizeof(rev));
    rev.action = PLAN_REVERSE;
    rev.reason = PLAN_REASON_TOO_CLOSE;
    rev.step_count = 1U;
    rev.steps[0].is_turn = false;
    rev.steps[0].value = -70;

    h.in.scan_done = true;
    h.in.plan = rev;
    h_cycle(&h);
    CHECK_INT(VC_REVERSE, h.out.state);
    CHECK_INT(MOTION_CMD_MOVE, h.log[h.log_n - 1].type);
    CHECK_INT(-70, h.log[h.log_n - 1].distance_mm);
    CHECK(until(&h, VC_OBSTACLE_SCAN, 100));
    CHECK_INT(2, h.scans);

    h.in.scan_done = true;
    h.in.plan = rev;
    cycles(&h, 3);
    CHECK_INT(VC_FAULT, h.out.state);
    CHECK((h.raised & FAULT_NO_SAFE_PATH) != 0U);
}

/* The scan cannot take forever. */
static void test_scan_timeout_is_a_fault(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.in.obstacle_present = true;
    CHECK(until(&h, VC_OBSTACLE_SCAN, 100));
    cycles(&h, 2100);
    CHECK_INT(VC_FAULT, h.out.state);
    CHECK((h.raised & FAULT_ULTRASONIC) != 0U);
}

/* "Not in the way" resumes following, and the obstacle monitor is ignored for
 * a while so the same object does not immediately trigger another scan. */
static void test_continue_plan_resumes_and_blanks_the_monitor(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.in.obstacle_present = true;
    CHECK(until(&h, VC_OBSTACLE_SCAN, 100));
    h.in.scan_done = true;
    memset(&h.in.plan, 0, sizeof(h.in.plan));
    h.in.plan.action = PLAN_CONTINUE;
    h.in.plan.reason = PLAN_REASON_NO_OBSTACLE;
    cycles(&h, 3);
    CHECK_INT(VC_FOLLOW, h.out.state);
    cycles(&h, 100);                                     /* obstacle flag still set */
    CHECK_INT(VC_FOLLOW, h.out.state);
    CHECK_INT(1, h.scans);
    cycles(&h, 150);                                     /* blanking over: it can trigger again */
    CHECK_INT(2, h.scans);
}

/* ---- faults, stop, remote -------------------------------------------- */

/* A stop-class fault raised by anyone stops the car immediately in any running
 * state, latches, and no drive command follows.  Catches: continuing to drive
 * after a safety fault. */
static void test_stop_fault_stops_the_car_in_any_running_state(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    cycles(&h, 5);
    h.in.faults = FAULT_ENCODER;
    h_cycle(&h);
    CHECK_INT(VC_FAULT, h.out.state);
    CHECK_INT(MOTION_CMD_STOP, h.log[h.log_n - 1].type);
    {
        int drives = count_type(&h, MOTION_CMD_DRIVE);

        cycles(&h, 50);
        CHECK_INT(drives, count_type(&h, MOTION_CMD_DRIVE));
        CHECK_INT(VC_FAULT, h.out.state);
    }
}

/* An informational fault does not stop the car. */
static void test_informational_fault_does_not_stop(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.in.faults = (fault_bits_t)(FAULT_IMU | FAULT_NET_DOWN | FAULT_TELEMETRY_DROP);
    cycles(&h, 20);
    CHECK_INT(VC_FOLLOW, h.out.state);
}

/* A stop fault during a scan stops the scan wait and faults. */
static void test_fault_during_a_scan_wins(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.in.obstacle_present = true;
    CHECK(until(&h, VC_OBSTACLE_SCAN, 100));
    h.in.faults = FAULT_MOTION;
    cycles(&h, 2);
    CHECK_INT(VC_FAULT, h.out.state);
}

/* FAULT latches until the operator presses START, which clears the faults and
 * returns to IDLE (not straight to driving). */
static void test_fault_needs_the_operator_to_clear_it(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.in.faults = FAULT_LINE_STALE;
    cycles(&h, 3);
    CHECK_INT(VC_FAULT, h.out.state);
    h.in.faults = 0U;                                    /* the cause went away... */
    cycles(&h, 30);
    CHECK_INT(VC_FAULT, h.out.state);                    /* ...but it stays latched */

    h.in.faults = FAULT_LINE_STALE;
    h.in.start_pressed = true;
    h_cycle(&h);
    CHECK_INT(VC_IDLE, h.out.state);
    CHECK((h.out.clear_faults & FAULT_LINE_STALE) != 0U);
}

/* The operator STOP button stops the car; START resumes following without
 * running calibration again. */
static void test_operator_stop_and_resume(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.in.stop_pressed = true;
    h_cycle(&h);
    CHECK_INT(VC_STOPPED, h.out.state);
    CHECK_INT(MOTION_CMD_STOP, h.log[h.log_n - 1].type);
    cycles(&h, 10);
    CHECK_INT(VC_STOPPED, h.out.state);

    h.in.start_pressed = true;
    h_cycle(&h);
    CHECK_INT(VC_FOLLOW, h.out.state);
    CHECK_INT(1, h.cal_begin);                           /* no second calibration */
}

/* A remote STOP works when allowed, and is ignored when the team disables it. */
static void test_remote_stop_obeys_the_policy_flag(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.in.remote = VC_REMOTE_STOP;
    h_cycle(&h);
    CHECK_INT(VC_STOPPED, h.out.state);

    h_init(&h);
    h.vc.cfg.allow_remote_stop = false;
    to_follow(&h);
    h.in.remote = VC_REMOTE_STOP;
    h_cycle(&h);
    CHECK_INT(VC_FOLLOW, h.out.state);
}

/* The remote command policy: nothing but a stop may touch a running mission;
 * there is no remote command that drives the wheels at all. */
static void test_remote_policy(void)
{
    uint8_t bit = 0U;

    CHECK_INT(TEL_ACK_REJECTED_RUNNING,
              vehicle_remote_policy(VC_FOLLOW, true, TEL_CMD_START, &bit));
    CHECK_INT(TEL_ACK_REJECTED_RUNNING,
              vehicle_remote_policy(VC_FOLLOW, true, TEL_CMD_CALIBRATE, &bit));
    CHECK_INT(TEL_ACK_REJECTED_RUNNING,
              vehicle_remote_policy(VC_OBSTACLE_SCAN, true, TEL_CMD_CLEAR_FAULTS, &bit));

    CHECK_INT(TEL_ACK_RECEIVED, vehicle_remote_policy(VC_FOLLOW, true, TEL_CMD_STOP, &bit));
    CHECK_INT(VC_REMOTE_STOP, bit);
    CHECK_INT(TEL_ACK_REJECTED_RUNNING,
              vehicle_remote_policy(VC_FOLLOW, false, TEL_CMD_STOP, &bit));

    CHECK_INT(TEL_ACK_RECEIVED, vehicle_remote_policy(VC_IDLE, true, TEL_CMD_START, &bit));
    CHECK_INT(VC_REMOTE_START, bit);
    CHECK_INT(TEL_ACK_RECEIVED, vehicle_remote_policy(VC_STOPPED, true, TEL_CMD_CALIBRATE, &bit));
    CHECK_INT(VC_REMOTE_CALIBRATE, bit);
    CHECK_INT(TEL_ACK_RECEIVED, vehicle_remote_policy(VC_FAULT, true, TEL_CMD_CLEAR_FAULTS, &bit));
    CHECK_INT(VC_REMOTE_CLEAR_FAULTS, bit);

    CHECK_INT(TEL_ACK_COMPLETED,
              vehicle_remote_policy(VC_FOLLOW, true, TEL_CMD_REQUEST_STATUS, &bit));
    CHECK_INT(TEL_ACK_INVALID, vehicle_remote_policy(VC_IDLE, true, 15U, &bit));
}

/* A manoeuvre that Motion never completes is a motion fault, not an endless
 * wait. */
static void test_motion_command_timeout_is_a_fault(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    h.in.instruction_valid = true;
    h.in.instruction = instr('A', NAV_CMD_LEFT, 1U);
    junction(&h, NAV_JUNCTION_LEFT);
    h_cycle(&h);
    h.never_complete = true;
    cycles(&h, 700);
    CHECK_INT(VC_FAULT, h.out.state);
    CHECK((h.raised & FAULT_MOTION) != 0U);
}

/* Every new motion command carries a newer id than the one before, so the
 * motion layer's stale-command rule can never reject a legitimate command. */
static void test_command_ids_only_increase(void)
{
    h_t h;
    int i;

    h_init(&h);
    to_follow(&h);
    h.in.instruction_valid = true;
    h.in.instruction = instr('A', NAV_CMD_LEFT, 1U);
    junction(&h, NAV_JUNCTION_LEFT);
    h_cycle(&h);
    line_gone(&h);
    cycles(&h, 120);
    line_ok(&h, 0, 0);
    cycles(&h, 20);
    CHECK(h.log_n > 5);
    for (i = 1; i < h.log_n; i++) {
        CHECK((int16_t)(h.log[i].id - h.log[i - 1].id) > 0);
    }
}

/* State changes are reported as events for telemetry. */
static void test_state_changes_produce_events(void)
{
    h_t h;

    h_init(&h);
    to_follow(&h);
    CHECK(saw_event(&h, TEL_EVT_STATE, VC_IDLE));
    CHECK(saw_event(&h, TEL_EVT_STATE, VC_CALIBRATE));
    CHECK(saw_event(&h, TEL_EVT_STATE, VC_FOLLOW));
}

int main(void)
{
    RUN(test_boots_into_idle_and_commands_stop);
    RUN(test_start_sweeps_calibrates_then_follows);
    RUN(test_calibration_failure_stops_with_a_fault);
    RUN(test_follow_refreshes_one_drive_command_and_slows_in_curves);
    RUN(test_follow_without_a_position_goes_slow_and_straight);
    RUN(test_stale_line_data_stops_the_car);
    RUN(test_left_instruction_executes_once_at_the_junction);
    RUN(test_right_and_uturn_instructions_turn_the_right_angle);
    RUN(test_straight_instruction_crosses_without_turning);
    RUN(test_junction_without_an_instruction_goes_straight);
    RUN(test_lost_line_is_searched_for_and_resumed);
    RUN(test_line_never_found_is_a_fault);
    RUN(test_scan_starts_only_after_the_stop_is_confirmed);
    RUN(test_a_stale_stopped_flag_does_not_confirm_a_new_stop);
    RUN(test_unconfirmed_stop_is_a_motion_fault);
    RUN(test_left_bypass_runs_the_plan_then_recovers_the_line);
    RUN(test_recovery_drives_on_before_turning_back);
    RUN(test_bypass_stops_for_something_in_the_way_then_finishes_the_leg);
    RUN(test_bypass_ignores_the_sonar_while_turning_and_just_after);
    RUN(test_bypass_blocked_too_long_is_a_fault);
    RUN(test_bypass_follows_the_line_if_it_is_seen_on_the_last_straight_leg);
    RUN(test_sonar_tilts_toward_the_box_beside_it_only);
    RUN(test_leg_beside_the_box_ends_when_the_box_is_passed);
    RUN(test_leg_is_not_cut_if_the_box_was_never_seen);
    RUN(test_leg_goes_on_while_the_box_is_still_beside_the_car);
    RUN(test_recovery_drive_waits_for_the_way_and_keeps_its_time_limit);
    RUN(test_recovery_time_limit);
    RUN(test_recovery_distance_limit);
    RUN(test_no_safe_path_keeps_the_car_stopped);
    RUN(test_invalid_scan_is_an_ultrasonic_fault);
    RUN(test_reverse_then_rescan_once);
    RUN(test_scan_timeout_is_a_fault);
    RUN(test_continue_plan_resumes_and_blanks_the_monitor);
    RUN(test_stop_fault_stops_the_car_in_any_running_state);
    RUN(test_informational_fault_does_not_stop);
    RUN(test_fault_during_a_scan_wins);
    RUN(test_fault_needs_the_operator_to_clear_it);
    RUN(test_operator_stop_and_resume);
    RUN(test_remote_stop_obeys_the_policy_flag);
    RUN(test_remote_policy);
    RUN(test_motion_command_timeout_is_a_fault);
    RUN(test_command_ids_only_increase);
    RUN(test_state_changes_produce_events);
    return TEST_REPORT();
}
