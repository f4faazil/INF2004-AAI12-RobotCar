/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * robot_tasks.c
 * The robot's kernel tasks.  Each task owns one subsystem and talks to the
 * others only through the snapshots and mailboxes below; each exchange is a
 * short copy inside a critical section, so no task ever waits for another.
 *
 *   priority  task      period   owns
 *   --------  --------  -------  ------------------------------------------
 *      1      Motion    5 ms     motor PWM (the only writer), encoders
 *      2      Line      10 ms    IR ring, calibration, barcode, line events
 *      3      Vehicle   10 ms    mission state; the only sender of movement
 *      3      Obstacle  10 ms    servo, HC-SR04, scan and bypass planning
 *      5      IMU       20 ms    I2C1, tilt and hump height
 *     10      Net       20 ms    WiFi/MQTT, telemetry publishing (lowest)
 *     11      Diag      1 s      one status line on the console
 *
 * The kernel's own USB console task runs at priority 3 and the CYW43439 radio
 * task at 4, so the control path (1-3) outranks both.
 *
 * If the Vehicle task stops refreshing its commands, Motion's 50 ms lease
 * expires and the wheels stop on their own; a stop-class fault also zeroes
 * the PWM from inside the Motion task without waiting for the Vehicle task.
 *
 * NOT YET RUN ON THE CAR: it builds cleanly, the logic it calls is tested on
 * the host, but task timing, stack sizes and priorities are unmeasured.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "fault_mask.h"
#include "hal_rp2040.h"
#include "imu.h"
#include "line_nav.h"
#include "motion.h"
#include "mqtt_client.h"
#include "obstacle.h"
#include "robot_config.h"
#include "robot_net.h"
#include "robot_port.h"
#include "robot_tasks.h"
#include "safety.h"
#include "telemetry.h"
#include "ultrasonic.h"
#include "vehicle.h"

/* Forward-obstacle detection range inputs.  PLACEHOLDER: measure the stopping
 * distance on the car (Tribunal: physical stopping distance is not PWM-zero). */
#define ROBOT_STOP_RESPONSE_MS      (120U)
#define ROBOT_BRAKING_MM            (40U)
/* margin: the 80 mm between the sensor and the bumper plus 60 mm of clearance */
#define ROBOT_OBSTACLE_MARGIN_MM    (ROBOT_SENSOR_TO_FRONT_MM + 60U)
#define ROBOT_OBSTACLE_HYST_MM      (30U)
#define ROBOT_OBSTACLE_CONFIRM      (2U)
#define ROBOT_US_INVALID_LIMIT      (8U)
#define ROBOT_POLL_MS               (10U)
#define ROBOT_BUTTON_DEBOUNCE       (3U)
#define ROBOT_CMD_RING              (8U)
#define ROBOT_MOTION_FRESH_MS       (100U)
#define ROBOT_JUNCTION_HOLD_MS      (500U)
#define ROBOT_IMU_FAIL_LIMIT        (3U)
#define ROBOT_IMU_PROBE_MS          (1000U)
#define ROBOT_NET_POP_PER_LOOP      (4U)
#define ROBOT_NET_DOWN_MS           (3000U)

#define LSM303D_ADDR                (0x1DU)
#define LSM303D_WHO_AM_I_REG        (0x0FU)
#define LSM303D_WHO_AM_I_VALUE      (0x49U)
#define LSM303DLHC_ACCEL_ADDR       (0x19U)
#define ACCEL_CTRL1_REG             (0x20U)
#define ACCEL_CTRL1_100HZ_XYZ       (0x57U)     /* LSM303DLHC: 100 Hz, normal, XYZ */
#define LSM303D_CTRL1_100HZ_XYZ_BDU (0x6FU)     /* LSM303D: 100 Hz, XYZ, block data update */
#define LSM303D_CTRL2_AAF50_2G      (0xC0U)     /* LSM303D: 50 Hz anti-alias filter (less vibration), +/-2 g */
#define LSM303D_CTRL2_REG           (0x21U)
#define LSM303DLHC_CTRL4_REG        (0x23U)
#define LSM303DLHC_CTRL4_HR_2G      (0x08U)
#define ACCEL_OUT_X_L_AUTOINC       (0x28U | 0x80U)

/* ---- state shared between tasks ------------------------------------------ */

typedef struct {
    line_nav_out_t    out;
    bool              instruction_valid;
    nav_instruction_t instruction;
    bool              calibration_done;
    bool              calibration_ok;
    bool              rejected_recent;
} line_share_t;

typedef struct {
    bool     begin;
    bool     end;
    bool     consume;
    uint16_t consume_id;
} line_request_t;

typedef struct {
    motion_status_t status;
    bool            encoder_fault;
} motion_share_t;

typedef struct {
    bool               obstacle;
    bool               sensor_fault;
    bool               distance_valid;
    uint16_t           distance_mm;
    uint32_t           updated_ms;
    bool               scan_request;
    bool               scan_done;
    bypass_plan_t      plan;
    obstacle_profile_t profile;
} obstacle_share_t;

typedef struct {
    bool     in_progress;
    uint16_t packed;
    uint8_t  highest_code;
    bool     read_failed;
    uint32_t updated_ms;
} imu_share_t;

static ID g_flg;
static fault_mask_t g_faults;
static tel_queue_t g_tel_q;

static line_share_t g_line_share;
static line_request_t g_line_req;
static motion_share_t g_motion_share;
static obstacle_share_t g_obstacle_share;
static imu_share_t g_imu_share;

static motion_cmd_t g_cmd_ring[ROBOT_CMD_RING];
static uint32_t g_cmd_head;
static uint32_t g_cmd_tail;
static uint32_t g_cmd_overwritten;

static volatile uint8_t g_vstate = (uint8_t)VC_BOOT;
static volatile uint8_t g_remote_bits;
static volatile uint8_t g_remote_req[4];        /* req_id + 1 per VC_REMOTE_* bit; 0 = none */
static uint8_t g_event_seq;

static volatile uint32_t g_max_motion_us;
static volatile uint32_t g_max_line_us;
static volatile uint32_t g_max_vehicle_us;
static volatile uint32_t g_ir_frames;
static volatile uint32_t g_mqtt_state;
static volatile uint32_t g_mqtt_reconnects;

/* ---- small helpers ------------------------------------------------------------- */

static uint16_t ts10_now(void)
{
    return (uint16_t)(hal_time_ms() / 10U);
}

static void note_max(volatile uint32_t *max, uint32_t value)
{
    if (value > *max) {
        *max = value;
    }
}

/* Non-blocking telemetry submit: a full queue drops the new record and sets an
 * informational fault; it never waits. */
static void tel_submit(const uint8_t *data, uint16_t len)
{
    if ((len == 0U) || !tel_queue_push(&g_tel_q, data, len, hal_time_ms())) {
        fault_set(&g_faults, FAULT_TELEMETRY_DROP);
    }
}

static void tel_push_event(uint8_t code, uint16_t arg)
{
    uint8_t buf[TEL_MAX_MESSAGE];
    tel_event_t e;
    robot_irq_state_t state = robot_irq_save();

    e.seq = g_event_seq;
    g_event_seq++;
    robot_irq_restore(state);
    e.ts10 = ts10_now();
    e.code = code;
    e.arg = arg;
    tel_submit(buf, tel_encode_event(&e, buf, sizeof(buf)));
}

static void tel_push_ack(uint8_t req_id, uint8_t result)
{
    uint8_t buf[TEL_MAX_MESSAGE];
    tel_ack_t a;

    a.req_id = req_id;
    a.result = result;
    tel_submit(buf, tel_encode_ack(&a, buf, sizeof(buf)));
}

/* Command mailbox: Vehicle -> Motion, a small FIFO so a STOP is never lost. */
static void cmd_post(const motion_cmd_t *cmd)
{
    robot_irq_state_t state = robot_irq_save();

    if ((g_cmd_head - g_cmd_tail) >= ROBOT_CMD_RING) {
        g_cmd_overwritten++;
        g_cmd_head--;                           /* replace the newest entry */
    }
    g_cmd_ring[g_cmd_head % ROBOT_CMD_RING] = *cmd;
    g_cmd_head++;
    robot_irq_restore(state);
}

static bool cmd_take(motion_cmd_t *cmd)
{
    robot_irq_state_t state = robot_irq_save();
    bool have = g_cmd_tail != g_cmd_head;

    if (have) {
        *cmd = g_cmd_ring[g_cmd_tail % ROBOT_CMD_RING];
        g_cmd_tail++;
    }
    robot_irq_restore(state);
    return have;
}

static void motion_share_get(motion_share_t *dst)
{
    robot_irq_state_t state = robot_irq_save();

    *dst = g_motion_share;
    robot_irq_restore(state);
}

static int16_t centre_speed(const motion_share_t *m)
{
    return (int16_t)(((int32_t)m->status.left_mm_s + (int32_t)m->status.right_mm_s) / 2);
}

static void wait_tick(uint32_t bit)
{
    UINT pattern;

    (void)tk_wai_flg(g_flg, bit, TWF_ORW | TWF_BITCLR, &pattern, TMO_FEVR);
    hal_tick_ack(bit);
}

/* ---- Motion task -------------------------------------------------------------- */

static void motion_task(INT stacd, void *exinf)
{
    static motion_t motion;
    encoder_snapshot_t left;
    encoder_snapshot_t right;
    motion_output_t out;
    motion_cmd_t cmd;

    (void)stacd;
    (void)exinf;
    motion_init(&motion, &ROBOT_MOTION_CFG);
    hal_motor_set(0, 0);

    for (;;) {
        uint32_t start;

        wait_tick(HAL_TICK_MOTION);
        start = hal_time_us();

        /* A stop-class fault stops the wheels here, without waiting for the
         * Vehicle task. */
        motion_set_override(&motion, fault_stop_required(&g_faults));
        while (cmd_take(&cmd)) {
            (void)motion_submit(&motion, &cmd, hal_time_ms());
        }
        hal_encoders_snapshot(&left, &right);
        motion_step(&motion, &left, &right, hal_time_ms(), hal_time_us(), &out);
        hal_motor_set(out.left_pwm, out.right_pwm);

        {
            robot_irq_state_t state = robot_irq_save();

            g_motion_share.status = *motion_status(&motion);
            g_motion_share.encoder_fault =
                (motion_status(&motion)->flags & MOTION_FLAG_ENCODER_FAULT) != 0U;
            robot_irq_restore(state);
        }
        note_max(&g_max_motion_us, hal_time_us() - start);
    }
}

/* ---- Line task ---------------------------------------------------------------- */

static void line_task(INT stacd, void *exinf)
{
    static line_nav_t nav;
    static uint32_t overflows_seen;
    static uint32_t rejected_until_ms;
    line_nav_out_t out;
    line_request_t req;
    imu_share_t imu;
    motion_share_t motion;
    hal_ir_frame_t frame;

    (void)stacd;
    (void)exinf;
    line_nav_init(&nav, &ROBOT_LINE_NAV_CFG);

    for (;;) {
        uint32_t start;
        uint32_t seen;

        wait_tick(HAL_TICK_CONTROL);
        start = hal_time_us();

        {
            robot_irq_state_t state = robot_irq_save();

            req = g_line_req;
            g_line_req.begin = false;
            g_line_req.end = false;
            g_line_req.consume = false;
            imu = g_imu_share;
            robot_irq_restore(state);
        }
        if (req.begin) {
            line_nav_calibrate_begin(&nav);
            robot_irq_state_t state = robot_irq_save();

            g_line_share.calibration_done = false;
            robot_irq_restore(state);
        }
        if (req.end) {
            bool ok = line_nav_calibrate_end(&nav);
            robot_irq_state_t state = robot_irq_save();

            g_line_share.calibration_done = true;
            g_line_share.calibration_ok = ok;
            robot_irq_restore(state);
        }

        seen = hal_ir_overflows();
        if (seen != overflows_seen) {
            overflows_seen = seen;
            line_nav_note_overflow(&nav);       /* the barcode attempt in progress is void */
        }
        while (hal_ir_pop(&frame)) {
            line_nav_push_frame(&nav, frame.adc, frame.ms);
            g_ir_frames++;
        }

        line_nav_set_hump(&nav, imu.in_progress, imu.updated_ms);
        motion_share_get(&motion);
        line_nav_step(&nav, hal_time_ms(), centre_speed(&motion), &out);

        if ((out.events & NAV_EVENT_OVERFLOW) != 0U) {
            fault_set(&g_faults, FAULT_LINE_OVERFLOW);
        }
        if ((out.events & NAV_EVENT_BARCODE_REJECTED) != 0U) {
            fault_set(&g_faults, FAULT_BARCODE_REJECT);
            rejected_until_ms = hal_time_ms() + 1000U;
        }
        if ((out.events & NAV_EVENT_BARCODE) != 0U) {
            nav_instruction_t ins;

            if (line_nav_peek_instruction(&nav, hal_time_ms(), &ins)) {
                tel_push_event(TEL_EVT_BARCODE, (uint16_t)(uint8_t)ins.letter);
            }
        }

        if (req.consume) {
            nav_instruction_t pending;

            if (line_nav_peek_instruction(&nav, hal_time_ms(), &pending) &&
                (pending.id == req.consume_id)) {
                (void)line_nav_take_instruction(&nav, hal_time_ms(), &pending);
            }
        }

        {
            robot_irq_state_t state = robot_irq_save();

            g_line_share.out = out;
            g_line_share.instruction_valid =
                line_nav_peek_instruction(&nav, hal_time_ms(), &g_line_share.instruction);
            g_line_share.rejected_recent = (int32_t)(rejected_until_ms - hal_time_ms()) > 0;
            robot_irq_restore(state);
        }
        note_max(&g_max_line_us, hal_time_us() - start);
    }
}

/* ---- Vehicle task ------------------------------------------------------------- */

typedef struct {
    uint8_t count;
    bool    state;
} debounce_t;

/* Returns true on the cycle the debounced button goes from released to pressed. */
static bool debounce_edge(debounce_t *d, bool raw)
{
    bool pressed_edge = false;

    if (raw == d->state) {
        d->count = 0U;
    } else if (++d->count >= ROBOT_BUTTON_DEBOUNCE) {
        d->state = raw;
        d->count = 0U;
        pressed_edge = raw;
    } else {
        /* still settling */
    }
    return pressed_edge;
}

static uint8_t line_state_code(const line_nav_out_t *o)
{
    if (o->stale) {
        return 3U;
    }
    if (o->suppressing_lost) {
        return 2U;
    }
    return o->line_lost ? 1U : 0U;
}

static void push_status(const vehicle_in_t *in, const line_share_t *line,
                        const obstacle_share_t *obs, const imu_share_t *imu,
                        vehicle_state_t state, uint8_t junction_mask)
{
    uint8_t buf[TEL_MAX_MESSAGE];
    tel_status_t s;

    s.ts10 = ts10_now();
    s.state = (uint8_t)state;
    s.nav_cmd = line->instruction_valid ? (uint8_t)line->instruction.command : 0U;
    s.line_state = line_state_code(&line->out);
    s.junction_state = junction_mask;
    s.barcode_state = line->instruction_valid ? 1U : (line->rejected_recent ? 2U : 0U);
    s.line_position = line->out.position;
    s.left_mm_s = in->motion.left_mm_s;
    s.right_mm_s = in->motion.right_mm_s;
    s.distance_mm = in->motion.distance_mm;
    s.faults = in->faults;
    s.obstacle_valid = obs->distance_valid;
    s.obstacle_cm = (uint16_t)(obs->distance_mm / 10U);
    s.hump_packed = imu->packed;
    s.highest_hump_mm = imu->highest_code;
    s.saturated = 0U;
    tel_submit(buf, tel_encode_status(&s, buf, sizeof(buf)));
}

static void vehicle_task(INT stacd, void *exinf)
{
    static vehicle_t vc;
    static debounce_t start_btn;
    static debounce_t stop_btn;
    static uint32_t last_status_ms;
    static uint32_t last_junction_ms;
    static uint8_t last_junction_mask;
    static vehicle_in_t in;
    static vehicle_out_t out;
    line_share_t line;
    obstacle_share_t obs;
    imu_share_t imu;
    motion_share_t motion;
    safety_in_t health;

    (void)stacd;
    (void)exinf;
    vehicle_init(&vc, &ROBOT_VEHICLE_CFG);

    for (;;) {
        uint32_t start;
        uint32_t now;
        uint8_t remote;
        uint8_t i;

        wait_tick(HAL_TICK_CONTROL);
        start = hal_time_us();
        now = hal_time_ms();

        {
            robot_irq_state_t state = robot_irq_save();

            line = g_line_share;
            obs = g_obstacle_share;
            imu = g_imu_share;
            motion = g_motion_share;
            remote = g_remote_bits;
            g_remote_bits = 0U;
            robot_irq_restore(state);
        }

        in.now_ms = now;
        in.start_pressed = debounce_edge(&start_btn, hal_button_start());
        in.stop_pressed = debounce_edge(&stop_btn, hal_button_stop());
        in.remote = remote;
        in.line = line.out;
        in.instruction_valid = line.instruction_valid;
        in.instruction = line.instruction;
        in.calibration_done = line.calibration_done;
        in.calibration_ok = line.calibration_ok;
        in.motion = motion.status;
        in.obstacle_present = obs.obstacle;
        in.scan_done = obs.scan_done;
        in.plan = obs.plan;
        in.hump_in_progress = imu.in_progress;
        in.faults = fault_read(&g_faults);

        vehicle_step(&vc, &in, &out);

        if (out.cmd.type != MOTION_CMD_NONE) {
            cmd_post(&out.cmd);
        }
        if (out.calibrate_begin || out.calibrate_end || out.consume_instruction) {
            robot_irq_state_t state = robot_irq_save();

            g_line_req.begin = g_line_req.begin || out.calibrate_begin;
            g_line_req.end = g_line_req.end || out.calibrate_end;
            if (out.consume_instruction) {
                g_line_req.consume = true;
                g_line_req.consume_id = out.consume_id;
            }
            robot_irq_restore(state);
        }
        if (out.request_scan) {
            robot_irq_state_t state = robot_irq_save();

            g_obstacle_share.scan_request = true;
            g_obstacle_share.scan_done = false;
            robot_irq_restore(state);
        }
        if (out.raise_faults != 0U) {
            fault_set(&g_faults, out.raise_faults);
        }
        if (out.clear_faults != 0U) {
            fault_clear(&g_faults, out.clear_faults);
        }
        for (i = 0U; i < out.event_count; i++) {
            tel_push_event(out.events[i].code, out.events[i].arg);
            if (out.events[i].code == TEL_EVT_JUNCTION) {
                last_junction_mask = (uint8_t)out.events[i].arg;
                last_junction_ms = now;
            }
        }
        g_vstate = (uint8_t)out.state;

        /* Acknowledge handled remote requests (receipt was acknowledged by the
         * Net task; this says the controller has now processed it). */
        for (i = 0U; i < 4U; i++) {
            if (((remote >> i) & 1U) != 0U) {
                uint8_t req = g_remote_req[i];

                if (req != 0U) {
                    tel_push_ack((uint8_t)(req - 1U), TEL_ACK_COMPLETED);
                    g_remote_req[i] = 0U;
                }
            }
        }

        /* Safety monitor: turn producer health into fault bits. */
        health.line_required = (out.activity & VC_ACT_LINE_REQUIRED) != 0U;
        health.driving = (out.activity & VC_ACT_DRIVING) != 0U;
        health.obstacle_watch_required = ROBOT_REQUIRE_ULTRASONIC && ((out.activity & VC_ACT_OBSTACLE_WATCH) != 0U);
        health.line_stale = line.out.stale;
        health.encoder_fault = motion.encoder_fault;
        health.motion_age_ms = now - motion.status.timestamp_ms;
        health.imu_read_failed = imu.read_failed;
        health.imu_age_ms = now - imu.updated_ms;
        health.ultrasonic_fault = obs.sensor_fault;
        health.ultrasonic_age_ms = now - obs.updated_ms;
        health.adc_overflow = false;               /* raised by the Line task */
        health.barcode_rejected = false;
        {
            fault_bits_t raise = safety_evaluate(&health, &ROBOT_SAFETY_LIMITS);

            if (raise != 0U) {
                fault_set(&g_faults, raise);
            }
        }

        if ((now - last_status_ms) >= ROBOT_STATUS_PERIOD_MS) {
            last_status_ms = now;
            push_status(&in, &line, &obs, &imu, out.state,
                        ((now - last_junction_ms) < ROBOT_JUNCTION_HOLD_MS) ? last_junction_mask : 0U);
        }
        note_max(&g_max_vehicle_us, hal_time_us() - start);
    }
}

/* ---- Obstacle task ------------------------------------------------------------- */

static void publish_obstacle_state(const us_monitor_t *mon, uint32_t now)
{
    uint16_t mm = 0U;
    bool valid = us_monitor_distance(mon, &mm);
    robot_irq_state_t state = robot_irq_save();

    g_obstacle_share.obstacle = mon->obstacle;
    g_obstacle_share.sensor_fault = us_monitor_sensor_fault(mon);
    g_obstacle_share.distance_valid = valid;
    g_obstacle_share.distance_mm = mm;
    g_obstacle_share.updated_ms = now;
    robot_irq_restore(state);
}

static void push_obstacle_message(const obstacle_profile_t *p, const bypass_plan_t *plan)
{
    uint8_t buf[TEL_MAX_MESSAGE];
    tel_obstacle_t o;

    o.ts10 = ts10_now();
    o.valid = p->valid && p->obstacle_found;
    o.closest_cm = (uint16_t)(p->closest_mm / 10U);
    o.width_cm = (uint16_t)(p->width_mm / 10U);
    o.centre_cm = (int16_t)(p->centre_x_mm / 10);
    o.left_clear_cm = (uint16_t)(p->left_clearance_mm / 10U);
    o.right_clear_cm = (uint16_t)(p->right_clearance_mm / 10U);
    o.action = (uint8_t)plan->action;
    o.reason = (uint8_t)plan->reason;
    tel_submit(buf, tel_encode_obstacle(&o, buf, sizeof(buf)));
}

static void obstacle_task(INT stacd, void *exinf)
{
    static us_monitor_t mon;
    static scan_t scan;
    static us_reading_t reading;
    bool measuring = false;
    bool scanning = false;
    uint32_t last_start_ms = 0U;
    motion_share_t motion;

    (void)stacd;
    (void)exinf;
    us_monitor_init(&mon, (uint8_t)ROBOT_OBSTACLE_CONFIRM, (uint8_t)ROBOT_US_INVALID_LIMIT);
    reading.status = US_ERROR;
    reading.seq = 1U;

    for (;;) {
        uint32_t now;

        (void)tk_dly_tsk(ROBOT_POLL_MS);
        now = hal_time_ms();

        if (measuring) {
            hal_echo_t echo = hal_us_poll();

            if (echo.finished) {
                uint16_t mm = 0U;

                reading.status = us_convert(echo.echo_us, echo.complete, &mm);
                reading.distance_mm = (reading.status == US_VALID) ? mm : 0U;
                reading.timestamp_ms = now;
                reading.seq++;
                measuring = false;
                if (!scanning) {
                    uint16_t threshold;

                    motion_share_get(&motion);
                    threshold = us_stop_threshold_mm(centre_speed(&motion), ROBOT_STOP_RESPONSE_MS,
                                                     ROBOT_BRAKING_MM, ROBOT_OBSTACLE_MARGIN_MM);
                    if (threshold < ROBOT_OBSTACLE_SCAN_MM) {
                        threshold = ROBOT_OBSTACLE_SCAN_MM;     /* stop early enough for the scan to work */
                    }
                    (void)us_monitor_update(&mon, &reading, threshold, ROBOT_OBSTACLE_HYST_MM,
                                            (uint16_t)(threshold / 2U));
                    publish_obstacle_state(&mon, now);
                }
            }
        }

        if (!scanning && !measuring) {
            bool requested;
            robot_irq_state_t state = robot_irq_save();

            requested = g_obstacle_share.scan_request;
            g_obstacle_share.scan_request = false;
            robot_irq_restore(state);
            if (requested) {
                scan_start(&scan, &ROBOT_OBSTACLE_CFG, now);
                scanning = true;
            }
        }

        if (scanning) {
            scan_action_t act;
            bool done = scan_update(&scan, now, &reading, &act);

            if (act.servo_move) {
                hal_servo_set_pulse_us(servo_pulse_us(&ROBOT_OBSTACLE_CFG.servo, act.servo_angle));
            }
            if (act.request_measure && !measuring) {
                hal_us_trigger();
                measuring = true;
            }
            if (done) {
                static obstacle_profile_t profile;
                static bypass_plan_t plan;
                robot_irq_state_t state;

                obstacle_profile_build(&ROBOT_OBSTACLE_CFG, scan.points, scan.point_count, &profile);
                obstacle_plan(&ROBOT_OBSTACLE_CFG, &profile, &plan);
                state = robot_irq_save();
                g_obstacle_share.profile = profile;
                g_obstacle_share.plan = plan;
                g_obstacle_share.scan_done = true;
                robot_irq_restore(state);
                push_obstacle_message(&profile, &plan);
                scanning = false;
            }
        } else if (!measuring && ((now - last_start_ms) >= ROBOT_US_PERIOD_MS)) {
            hal_us_trigger();
            measuring = true;
            last_start_ms = now;
        } else {
            /* waiting for the next monitoring cycle */
        }
    }
}

/* ---- IMU task ------------------------------------------------------------------ */

bool robot_imu_probe(imu_variant_t *variant, uint8_t *addr)
{
    uint8_t id = 0U;

    if (hal_i2c_read(LSM303D_ADDR, LSM303D_WHO_AM_I_REG, &id, 1U) &&
        (id == LSM303D_WHO_AM_I_VALUE)) {
        *variant = IMU_VARIANT_LSM303D;
        *addr = LSM303D_ADDR;
        /* BDU keeps the six output bytes of one sample together, so a read that
         * lands as the chip updates cannot mix two samples into a false tilt. */
        return hal_i2c_write(LSM303D_ADDR, ACCEL_CTRL1_REG, LSM303D_CTRL1_100HZ_XYZ_BDU) &&
               hal_i2c_write(LSM303D_ADDR, LSM303D_CTRL2_REG, LSM303D_CTRL2_AAF50_2G);
    }
    if (hal_i2c_read(LSM303DLHC_ACCEL_ADDR, ACCEL_CTRL1_REG, &id, 1U)) {
        *variant = IMU_VARIANT_LSM303DLHC;
        *addr = LSM303DLHC_ACCEL_ADDR;
        return hal_i2c_write(LSM303DLHC_ACCEL_ADDR, ACCEL_CTRL1_REG, ACCEL_CTRL1_100HZ_XYZ) &&
               hal_i2c_write(LSM303DLHC_ACCEL_ADDR, LSM303DLHC_CTRL4_REG,
                             LSM303DLHC_CTRL4_HR_2G);
    }
    return false;
}

static void publish_imu(const roll_hump_t *h, bool in_progress, bool failed, uint32_t now)
{
    int32_t best = roll_hump_highest_valid_mm(h);
    robot_irq_state_t state = robot_irq_save();

    g_imu_share.in_progress = in_progress;
    g_imu_share.packed = hump_event_pack(h->state, roll_hump_last_height_code(h),
                                         roll_hump_last_validity(h), failed);
    g_imu_share.highest_code = (best < 0) ? (uint8_t)TEL_HIGHEST_NONE
                             : ((best > (int32_t)HUMP_HEIGHT_MAX_MM) ? (uint8_t)TEL_HIGHEST_NONE
                                                                     : (uint8_t)best);
    g_imu_share.read_failed = failed;
    g_imu_share.updated_ms = now;
    robot_irq_restore(state);
}

static void imu_task(INT stacd, void *exinf)
{
    static imu_cal_t cal;
    static roll_hump_t hump;
    int32_t filt[3] = { 0, 0, 0 };
    bool filt_init = false;
    imu_variant_t variant = IMU_VARIANT_LSM303D;
    uint8_t addr = LSM303D_ADDR;
    bool present = false;
    uint8_t failures = 0U;
    uint32_t last_probe_ms = 0U;
    motion_share_t motion;

    (void)stacd;
    (void)exinf;
    roll_hump_init(&hump, &ROBOT_ROLL_HUMP_CFG);
    imu_cal_init(&cal, ROBOT_IMU_FORWARD_AXIS, ROBOT_IMU_FORWARD_SIGN, ROBOT_IMU_CAL_SAMPLES,
                 ROBOT_IMU_CAL_SPREAD_MG);

    for (;;) {
        uint8_t raw[6];
        uint32_t now;
        bool ok;

        wait_tick(HAL_TICK_IMU);
        now = hal_time_ms();

        if (!present) {
            if ((now - last_probe_ms) >= ROBOT_IMU_PROBE_MS) {
                last_probe_ms = now;
                present = robot_imu_probe(&variant, &addr);
            }
            publish_imu(&hump, false, true, now);
            continue;
        }

        ok = hal_i2c_read(addr, (uint8_t)ACCEL_OUT_X_L_AUTOINC, raw, 6U);
        if (!ok) {
            failures++;
            if (failures >= ROBOT_IMU_FAIL_LIMIT) {
                present = false;                /* degrade to "hump N/A" and re-probe */
            }
            publish_imu(&hump, false, true, now);
            continue;
        }
        failures = 0U;

        {
            imu_vec3_t g;

            imu_decode_accel(variant, raw, &g);
            if (cal.state != IMU_CAL_DONE) {
                if (imu_cal_feed(&cal, &g) == IMU_CAL_FAILED_MOVING) {
                    imu_cal_init(&cal, ROBOT_IMU_FORWARD_AXIS, ROBOT_IMU_FORWARD_SIGN,
                                 ROBOT_IMU_CAL_SAMPLES, ROBOT_IMU_CAL_SPREAD_MG);
                }
                filt_init = false;
                publish_imu(&hump, false, false, now);
                continue;
            }
            motion_share_get(&motion);
            {
                roll_hump_input_t hin;
                hump_event_t ev;
                imu_vec3_t gf;

                /*
                 * Driving shakes the sensor by several times a 1 cm hump, so the vector
                 * is low-pass filtered (8 samples = 160 ms) before the roll is taken.
                 */
                if (!filt_init) {
                    filt[0] = (int32_t)g.x * 16;
                    filt[1] = (int32_t)g.y * 16;
                    filt[2] = (int32_t)g.z * 16;
                    filt_init = true;
                } else {
                    filt[0] += (((int32_t)g.x * 16) - filt[0]) / ROBOT_IMU_FILTER_DIV;
                    filt[1] += (((int32_t)g.y * 16) - filt[1]) / ROBOT_IMU_FILTER_DIV;
                    filt[2] += (((int32_t)g.z * 16) - filt[2]) / ROBOT_IMU_FILTER_DIV;
                }
                gf.x = (int16_t)(filt[0] / 16);
                gf.y = (int16_t)(filt[1] / 16);
                gf.z = (int16_t)(filt[2] / 16);
                hin.roll_permille = imu_roll_permille(&cal, &gf);
                hin.distance_mm = motion.status.distance_mm;
                hin.distance_valid = ((now - motion.status.timestamp_ms) <= ROBOT_MOTION_FRESH_MS) &&
                                     !motion.encoder_fault;
                hin.imu_ok = true;
                hin.now_ms = now;
                ev = roll_hump_update(&hump, &hin);
                publish_imu(&hump, roll_hump_in_progress(&hump), false, now);
                if (ev == HUMP_EVENT_FINISHED) {
                    uint8_t buf[TEL_MAX_MESSAGE];
                    tel_hump_t th;

                    th.ts10 = ts10_now();
                    th.hump_packed = hump.last_event_packed;
                    th.highest_hump_mm = g_imu_share.highest_code;
                    tel_submit(buf, tel_encode_hump(&th, buf, sizeof(buf)));
                    tel_push_event(TEL_EVT_HUMP, hump.last_event_packed);
                }
            }
        }
    }
}

/* ---- Net task ------------------------------------------------------------------- */

#define TOPIC_TELEMETRY     ROBOT_MQTT_BASE "/telemetry"
#define TOPIC_EVENT         ROBOT_MQTT_BASE "/event"
#define TOPIC_ACK           ROBOT_MQTT_BASE "/ack"
#define TOPIC_STATUS        ROBOT_MQTT_BASE "/status"
#define TOPIC_CMD           ROBOT_MQTT_BASE "/cmd"

/* An incoming command: apply the policy, latch the request for the Vehicle
 * Controller, and acknowledge receipt (completion is acknowledged later). */
static void on_command(void *ctx, const char *topic, const uint8_t *payload, uint16_t len)
{
    tel_command_t c;
    uint8_t bit = 0U;
    uint8_t result;

    (void)ctx;
    (void)topic;
    if (!tel_decode_command(payload, len, &c)) {
        return;
    }
    result = vehicle_remote_policy((vehicle_state_t)g_vstate, ROBOT_ALLOW_REMOTE_STOP != 0,
                                   c.cmd, &bit);
    if ((result == TEL_ACK_RECEIVED) && (bit != 0U)) {
        robot_irq_state_t state = robot_irq_save();
        uint8_t index = (bit == VC_REMOTE_STOP) ? 0U : ((bit == VC_REMOTE_START) ? 1U
                      : ((bit == VC_REMOTE_CALIBRATE) ? 2U : 3U));

        g_remote_bits |= bit;
        g_remote_req[index] = (uint8_t)(c.req_id + 1U);
        robot_irq_restore(state);
    }
    tel_push_ack(c.req_id, result);
}

static const char *topic_for(uint8_t type)
{
    if (type == TEL_TYPE_EVENT) {
        return TOPIC_EVENT;
    }
    if (type == TEL_TYPE_ACK) {
        return TOPIC_ACK;
    }
    return TOPIC_TELEMETRY;
}

static void net_task(INT stacd, void *exinf)
{
    static mqtt_client_t client;
    static uint32_t last_heartbeat_ms;
    static uint32_t down_since_ms;
    mqtt_config_t cfg;
    mqtt_transport_t net = robot_net_transport();

    (void)stacd;
    (void)exinf;
    cfg.client_id = ROBOT_MQTT_CLIENT_ID;
    cfg.username = NULL;
    cfg.password = NULL;
    cfg.status_topic = TOPIC_STATUS;
    cfg.command_topic = TOPIC_CMD;
    cfg.keepalive_s = 30U;
    cfg.connect_timeout_ms = 5000U;
    cfg.backoff_initial_ms = 1000U;
    cfg.backoff_max_ms = 8000U;
    cfg.on_message = on_command;
    cfg.cb_ctx = NULL;
    mqtt_client_init(&client, &cfg, &net);

    for (;;) {
        uint32_t now;
        uint8_t i;
        bool connected;

        (void)tk_dly_tsk(20U);
        now = hal_time_ms();
        mqtt_client_set_link(&client, robot_net_link_up());
        mqtt_client_step(&client, now);
        connected = mqtt_client_state(&client) == MQTT_CONNECTED;
        g_mqtt_state = (uint32_t)mqtt_client_state(&client);
        g_mqtt_reconnects = client.reconnects;

        if (connected) {
            down_since_ms = now;
            fault_clear(&g_faults, FAULT_NET_DOWN);
        } else if ((now - down_since_ms) >= ROBOT_NET_DOWN_MS) {
            fault_set(&g_faults, FAULT_NET_DOWN);
        } else {
            /* briefly down: not worth reporting yet */
        }

        /* Publish only while connected, so records age out in the queue
         * instead of being sent late; the queue's freshness rule discards the
         * stale ones when service returns. */
        for (i = 0U; connected && (i < ROBOT_NET_POP_PER_LOOP); i++) {
            uint8_t buf[TEL_RECORD_MAX];
            uint16_t len = 0U;

            if (!tel_queue_pop(&g_tel_q, buf, &len, now)) {
                break;
            }
            (void)mqtt_client_publish(&client, topic_for(buf[0]), buf, len, false);
        }
        if (connected && (tel_queue_depth(&g_tel_q) < (TEL_QUEUE_DEPTH / 4U))) {
            fault_clear(&g_faults, FAULT_TELEMETRY_DROP);
        }

        if (connected && ((now - last_heartbeat_ms) >= ROBOT_HEARTBEAT_MS)) {
            uint8_t buf[TEL_MAX_MESSAGE];
            tel_heartbeat_t hb;

            last_heartbeat_ms = now;
            hb.uptime_s = now / 1000U;
            hb.dropped = (uint16_t)g_tel_q.dropped_full;
            hb.reconnects = (client.reconnects > 255U) ? 255U : (uint8_t)client.reconnects;
            hb.queue_depth = tel_queue_depth(&g_tel_q);
            hb.net_state = (uint8_t)mqtt_client_state(&client);
            (void)mqtt_client_publish(&client, TOPIC_TELEMETRY, buf,
                                      tel_encode_heartbeat(&hb, buf, sizeof(buf)), false);
        }
    }
}

/* ---- Diagnostics task --------------------------------------------------------------- */

#if ROBOT_DIAG_PRINT
static void diag_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    for (;;) {
        line_share_t line;
        motion_share_t motion;
        encoder_snapshot_t enc_l;
        encoder_snapshot_t enc_r;
        robot_irq_state_t state;
        uint32_t ir_edges[3];
        uint32_t ir_do;

        (void)tk_dly_tsk(1000);
        state = robot_irq_save();
        line = g_line_share;
        motion = g_motion_share;
        robot_irq_restore(state);
        hal_encoders_snapshot(&enc_l, &enc_r);
        ir_do = hal_ir_levels();
        hal_ir_edges(ir_edges);
        tm_printf((UB *)"[robot] st=%d f=%04x pos=%d v=%d/%d d=%dmm enc=%d/%d bad=%u/%u ir=%u "
                        "do=%u%u%u edges=%u/%u/%u vm=%x bc=%c ovf=%u drop=%u net=%u us(m/l/v/isr)=%u/%u/%u/%u ovr=%u/%u\n",
                  (INT)g_vstate, (UINT)fault_read(&g_faults), (INT)line.out.position,
                  (INT)motion.status.left_mm_s, (INT)motion.status.right_mm_s,
                  (INT)motion.status.distance_mm, (INT)enc_l.count, (INT)enc_r.count,
                  (UINT)enc_l.illegal, (UINT)enc_r.illegal, (UINT)g_ir_frames,
                  (UINT)(ir_do & 1U), (UINT)((ir_do >> 1) & 1U), (UINT)((ir_do >> 2) & 1U),
                  (UINT)ir_edges[0], (UINT)ir_edges[1], (UINT)ir_edges[2],
                  (UINT)line.out.valid_mask,
                  (INT)(line.instruction_valid ? line.instruction.letter : '-'),
                  (UINT)hal_ir_overflows(), (UINT)g_tel_q.dropped_full, (UINT)g_mqtt_state,
                  (UINT)g_max_motion_us, (UINT)g_max_line_us, (UINT)g_max_vehicle_us,
                  (UINT)hal_adc_max_isr_us(), (UINT)hal_tick_overruns(HAL_TICK_MOTION),
                  (UINT)hal_tick_overruns(HAL_TICK_CONTROL));
    }
}
#endif

/* ---- start-up ------------------------------------------------------------------------ */

static ER make_task(void (*entry)(INT, void *), PRI pri, INT stack)
{
    T_CTSK ct;
    ID id;

    ct.exinf = NULL;
    ct.tskatr = TA_HLNG | TA_RNG3;
    ct.task = entry;
    ct.itskpri = pri;
    ct.stksz = stack;
#if TK_SUPPORT_SMP
    /* Peripheral interrupts are owned by processor 1 (RP2040 core 0). */
    ct.tskatr |= TA_ASSPRC;
    ct.assprc = TP_PRC1;
#endif
    id = tk_cre_tsk(&ct);
    if (id < E_OK) {
        return id;
    }
    return tk_sta_tsk(id, 0);
}

ER robot_start(void)
{
    T_CFLG cflg;
    ER err;

    hal_init();
    fault_mask_init(&g_faults);
    tel_queue_init(&g_tel_q);
    g_obstacle_share.updated_ms = 0U;
    g_imu_share.highest_code = (uint8_t)TEL_HIGHEST_NONE;

    cflg.flgatr = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0U;
    g_flg = tk_cre_flg(&cflg);
    if (g_flg < E_OK) {
        return g_flg;
    }

    err = make_task(motion_task, ROBOT_PRI_MOTION, ROBOT_STACK_BYTES);
    if (err < E_OK) { return err; }
    err = make_task(line_task, ROBOT_PRI_LINE, ROBOT_STACK_BYTES);
    if (err < E_OK) { return err; }
    err = make_task(vehicle_task, ROBOT_PRI_VEHICLE, ROBOT_STACK_BYTES);
    if (err < E_OK) { return err; }
    err = make_task(obstacle_task, ROBOT_PRI_OBSTACLE, ROBOT_STACK_BYTES);
    if (err < E_OK) { return err; }
    err = make_task(imu_task, ROBOT_PRI_IMU, ROBOT_STACK_BYTES);
    if (err < E_OK) { return err; }
    err = make_task(net_task, ROBOT_PRI_NET, ROBOT_STACK_BYTES);
    if (err < E_OK) { return err; }
#if ROBOT_DIAG_PRINT
    err = make_task(diag_task, ROBOT_PRI_DIAG, ROBOT_STACK_BYTES * 2);
    if (err < E_OK) { return err; }
#endif

    err = hal_gpio_irq_start();
    if (err < E_OK) {
        return err;
    }
    return hal_tick_start(g_flg);
}
