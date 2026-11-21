/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * telemetry.h
 * Compact binary messages and the bounded telemetry queue (Buddy 1).
 *
 * Minimum-bit change (Tribunal Version B/C), as implemented here:
 *  - every field has a stated range and uses only the bits that range needs
 *    (see the TEL_*_BITS constants: they are the data representation table);
 *  - messages are bit-packed MSB first with explicit masks and shifts and a
 *    1-byte message-type header.  C bit-fields are NOT used on the wire,
 *    because their layout is compiler-dependent;
 *  - a value that does not fit is saturated AND flagged in the message's
 *    "saturated" field, so a clipped reading is never mistaken for a real one;
 *  - invalid commands are rejected, not clipped;
 *  - internal calculations (PID, IMU height sum, raw ADC) stay full width and
 *    are only packed when stored for sending.
 *
 * The queue is bounded (16 records of up to 128 bytes), producers never block
 * (a full queue drops the NEW record and counts it), and records older than
 * one second are discarded rather than published late.
 *
 * Pure C.  tools/robot_telemetry.py is the matching decoder for the laptop.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

/* ---- bit writer / reader --------------------------------------------- */

typedef struct {
    uint8_t  *buf;
    uint16_t  cap_bits;
    uint16_t  pos;
    bool      overflow;
} bitw_t;

typedef struct {
    const uint8_t *buf;
    uint16_t       len_bits;
    uint16_t       pos;
    bool           underflow;
} bitr_t;

void     bitw_init(bitw_t *w, uint8_t *buf, uint16_t cap_bytes);
void     bitw_put(bitw_t *w, uint32_t value, uint8_t width);        /* masks to width */
void     bitw_put_signed(bitw_t *w, int32_t value, uint8_t width);  /* two's complement */
uint16_t bitw_bytes(const bitw_t *w);                               /* rounded up */

void     bitr_init(bitr_t *r, const uint8_t *buf, uint16_t len_bytes);
uint32_t bitr_get(bitr_t *r, uint8_t width);
int32_t  bitr_get_signed(bitr_t *r, uint8_t width);

/* ---- message types ----------------------------------------------------- */

#define TEL_TYPE_STATUS     (0x01U)
#define TEL_TYPE_EVENT      (0x02U)
#define TEL_TYPE_HUMP       (0x03U)
#define TEL_TYPE_OBSTACLE   (0x04U)
#define TEL_TYPE_HEARTBEAT  (0x05U)
#define TEL_TYPE_SENSORS    (0x06U)     /* the per-sensor detail record */
#define TEL_TYPE_COMMAND    (0x10U)     /* laptop -> robot */
#define TEL_TYPE_ACK        (0x11U)     /* robot -> laptop */

/* Field widths: the data representation table. */
#define TEL_TYPE_BITS       (8U)
#define TEL_TS_BITS         (16U)       /* 10 ms units; wraps every 655 s, and a
                                           record older than 1 s is never sent */
#define TEL_STATE_BITS      (4U)        /* vehicle state 0..15 */
#define TEL_NAVCMD_BITS     (3U)        /* none/left/right/straight/u-turn */
#define TEL_LINE_BITS       (2U)        /* tracking / lost / masked by hump / stale */
#define TEL_JUNCTION_BITS   (2U)        /* none / left / right / both */
#define TEL_BARCODE_BITS    (2U)        /* none / pending / rejected / expired */
#define TEL_POSITION_BITS   (11U)       /* -1000..+1000 fits +/-1023 */
#define TEL_SPEED_BITS      (10U)       /* +/-511 mm/s (course max ~300) */
#define TEL_DISTANCE_BITS   (16U)       /* +/-32767 mm: reverse and a whole run */
#define TEL_FAULT_BITS      (16U)
#define TEL_OBSTACLE_BITS   (9U)        /* 0..510 cm at 1 cm; 511 is reserved */
#define TEL_HUMP_BITS       (13U)       /* imu.h: 3 + 7 + 2 + 1 */
#define TEL_HIGHEST_BITS    (7U)        /* mm 0..126; 127 = none/out of range */
#define TEL_SAT_BITS        (4U)

#define TEL_SAT_POSITION    (1U << 0)
#define TEL_SAT_SPEED       (1U << 1)
#define TEL_SAT_DISTANCE    (1U << 2)
#define TEL_SAT_OBSTACLE    (1U << 3)

#define TEL_HIGHEST_NONE    (127U)
#define TEL_OBSTACLE_MAX_CM (510U)

typedef struct {
    uint16_t ts10;                  /* time in 10 ms units */
    uint8_t  state;                 /* vehicle_state_t */
    uint8_t  nav_cmd;               /* nav_command_t of the pending instruction */
    uint8_t  line_state;
    uint8_t  junction_state;
    uint8_t  barcode_state;
    int16_t  line_position;
    int16_t  left_mm_s;
    int16_t  right_mm_s;
    int32_t  distance_mm;
    uint16_t faults;
    bool     obstacle_valid;
    uint16_t obstacle_cm;           /* meaningful when obstacle_valid */
    uint16_t hump_packed;
    uint8_t  highest_hump_mm;
    uint8_t  saturated;             /* set by encode, read by decode */
} tel_status_t;

#define TEL_EVT_STATE       (1U)    /* arg: new vehicle state */
#define TEL_EVT_BARCODE     (2U)    /* arg: letter */
#define TEL_EVT_JUNCTION    (3U)    /* arg: junction mask */
#define TEL_EVT_NAV_EXEC    (4U)    /* arg: nav_command_t executed */
#define TEL_EVT_HUMP        (5U)    /* arg: packed hump event */
#define TEL_EVT_OBSTACLE    (6U)    /* arg: distance mm */
#define TEL_EVT_PLAN        (7U)    /* arg: plan_action_t */
#define TEL_EVT_FAULT       (8U)    /* arg: fault bits raised */
#define TEL_EVT_FAULT_CLEAR (9U)    /* arg: fault bits cleared */
#define TEL_EVT_LINE_LOST   (10U)
#define TEL_EVT_LINE_FOUND  (11U)
#define TEL_EVT_RECOVERED   (12U)   /* line reacquired after a bypass */
#define TEL_EVT_DONE        (13U)
#define TEL_EVT_BITS        (5U)
#define TEL_EVT_ARG_BITS    (16U)
#define TEL_EVT_SEQ_BITS    (8U)

typedef struct {
    uint16_t ts10;
    uint8_t  code;
    uint16_t arg;
    uint8_t  seq;                   /* lets the laptop spot lost events */
} tel_event_t;

typedef struct {
    uint16_t ts10;
    uint16_t hump_packed;
    uint8_t  highest_hump_mm;
} tel_hump_t;

#define TEL_CM_BITS         (9U)

typedef struct {
    uint16_t ts10;
    bool     valid;
    uint16_t closest_cm;
    uint16_t width_cm;
    int16_t  centre_cm;             /* signed, right positive */
    uint16_t left_clear_cm;
    uint16_t right_clear_cm;
    uint8_t  action;                /* plan_action_t */
    uint8_t  reason;                /* plan_reason_t */
} tel_obstacle_t;

/*
 * The per-sensor detail record: everything the dashboard shows beyond the main status record, each field
 * with its range and the bits that range needs (the data representation table):
 *   line / IR      ir_levels 3, valid_mask 3, pair 1, armed 1
 *   barcode        bars_now 4 (0..15), bars_done 8, decoded 8, rejected 8, junctions 8, executed 8,
 *                  map_entries 5 (0..26), last_letter 5 (0 none, 1..26 = A..Z)
 *   motors         motion_state 2, stopped 1, encoder_fault 1, enc_bad_left 8, enc_bad_right 8
 *   IMU            roll_permille 11 signed (sin of the sideways tilt x 1000), imu_ok 1, lift 1, hump_active 1
 *   ultrasonic     us_mm 13 (0..8190, 8191 reserved), us_valid 1, us_obstacle 1, us_fault 1, servo_aim 2
 *   network        net_state 3 (mqtt_lwip.c state), link_up 1
 *   counters 8 bits wide saturate at 255 and set `saturated`.
 * 143 bits: 18 bytes.
 */
#define TEL_IR_BITS         (3U)
#define TEL_BARS_BITS       (4U)
#define TEL_COUNT_BITS      (8U)
#define TEL_MAP_BITS        (5U)
#define TEL_LETTER_BITS     (5U)
#define TEL_ROLL_BITS       (11U)
#define TEL_US_MM_BITS      (13U)
#define TEL_NETSTATE_BITS   (3U)
#define TEL_COUNT_MAX       (255U)
#define TEL_US_MM_MAX       (8190U)
#define TEL_AIM_AHEAD       (0U)
#define TEL_AIM_LEFT        (1U)
#define TEL_AIM_RIGHT       (2U)

typedef struct {
    uint16_t ts10;
    uint8_t  ir_levels;             /* bit0 left, bit1 centre, bit2 right; 1 = black */
    uint8_t  ir_valid_mask;
    bool     pair_on_line;
    bool     junction_armed;
    uint8_t  bars_now;
    uint32_t bars_done;
    uint32_t decoded;
    uint32_t rejected;
    uint32_t junctions;
    uint32_t executed;
    uint8_t  map_entries;
    uint8_t  last_letter;
    uint8_t  motion_state;          /* motion_state_t: idle / running / done */
    bool     motion_stopped;
    bool     encoder_fault;
    uint32_t enc_bad_left;
    uint32_t enc_bad_right;
    int16_t  roll_permille;
    bool     imu_ok;                /* the accelerometer answers and the rest attitude is calibrated */
    bool     lift_latched;
    bool     hump_active;
    uint16_t us_mm;
    bool     us_valid;
    bool     us_obstacle;
    bool     us_fault;
    uint8_t  servo_aim;
    uint8_t  net_state;
    bool     link_up;
    uint8_t  saturated;             /* set by encode: a counter or distance was clipped */
} tel_sensors_t;

typedef struct {
    uint32_t uptime_s;
    uint16_t dropped;               /* telemetry records dropped (queue full) */
    uint8_t  reconnects;            /* saturates at 255 */
    uint8_t  queue_depth;
    uint8_t  net_state;             /* mqtt_state_t */
} tel_heartbeat_t;

#define TEL_CMD_STOP        (1U)
#define TEL_CMD_START       (2U)
#define TEL_CMD_CALIBRATE   (3U)
#define TEL_CMD_CLEAR_FAULTS (4U)
#define TEL_CMD_REQUEST_STATUS (5U)
#define TEL_CMD_BITS        (4U)
#define TEL_REQ_BITS        (8U)
#define TEL_PARAM_BITS      (16U)

typedef struct {
    uint8_t  req_id;
    uint8_t  cmd;
    uint16_t param;
} tel_command_t;

/* An ACK distinguishes receipt from completion (Tribunal). */
#define TEL_ACK_RECEIVED         (0U)
#define TEL_ACK_COMPLETED        (1U)
#define TEL_ACK_REJECTED_RUNNING (2U)   /* not permitted while the mission runs */
#define TEL_ACK_INVALID          (3U)
#define TEL_ACK_BITS             (3U)

typedef struct {
    uint8_t req_id;
    uint8_t result;
} tel_ack_t;

/* Maximum encoded sizes, in bytes. */
#define TEL_MAX_MESSAGE     (32U)

/*
 * Encoders return the number of bytes written, or 0 if the buffer is too
 * small.  Out-of-range fields are saturated and flagged; status->saturated
 * is updated.  Decoders return false for a wrong type or a short message.
 */
uint16_t tel_encode_status(tel_status_t *s, uint8_t *out, uint16_t cap);
bool     tel_decode_status(const uint8_t *in, uint16_t len, tel_status_t *s);

uint16_t tel_encode_event(const tel_event_t *e, uint8_t *out, uint16_t cap);
bool     tel_decode_event(const uint8_t *in, uint16_t len, tel_event_t *e);

uint16_t tel_encode_hump(const tel_hump_t *h, uint8_t *out, uint16_t cap);
bool     tel_decode_hump(const uint8_t *in, uint16_t len, tel_hump_t *h);

uint16_t tel_encode_obstacle(const tel_obstacle_t *o, uint8_t *out, uint16_t cap);
bool     tel_decode_obstacle(const uint8_t *in, uint16_t len, tel_obstacle_t *o);

uint16_t tel_encode_sensors(tel_sensors_t *s, uint8_t *out, uint16_t cap);
bool     tel_decode_sensors(const uint8_t *in, uint16_t len, tel_sensors_t *s);

uint16_t tel_encode_heartbeat(const tel_heartbeat_t *h, uint8_t *out, uint16_t cap);
bool     tel_decode_heartbeat(const uint8_t *in, uint16_t len, tel_heartbeat_t *h);

uint16_t tel_encode_command(const tel_command_t *c, uint8_t *out, uint16_t cap);
bool     tel_decode_command(const uint8_t *in, uint16_t len, tel_command_t *c);

uint16_t tel_encode_ack(const tel_ack_t *a, uint8_t *out, uint16_t cap);
bool     tel_decode_ack(const uint8_t *in, uint16_t len, tel_ack_t *a);

/* ---- the bounded queue ----------------------------------------------- */

#define TEL_QUEUE_DEPTH     (16U)
/* The largest record is the per-sensor record at 18 bytes (see the data representation table); TEL_MAX_MESSAGE (32) is the encoders' own
 * limit, so a queue slot needs no more. It was 128, which made the queue 2192 bytes for records that average under 12. */
#define TEL_RECORD_MAX      (TEL_MAX_MESSAGE)
#define TEL_MAX_AGE_MS      (1000U)

typedef struct {
    uint32_t ts_ms;
    uint8_t  len;
    uint8_t  data[TEL_RECORD_MAX];
} tel_record_t;

typedef struct {
    tel_record_t rec[TEL_QUEUE_DEPTH];
    uint8_t      head;              /* next to pop */
    uint8_t      count;
    uint32_t     pushed;
    uint32_t     dropped_full;
    uint32_t     discarded_stale;
} tel_queue_t;

void tel_queue_init(tel_queue_t *q);

/* Non-blocking, callable from tasks.  A full queue drops the NEW record. */
bool tel_queue_push(tel_queue_t *q, const uint8_t *data, uint16_t len, uint32_t now_ms);

/*
 * Pop the oldest record that is still fresh.  Records older than
 * TEL_MAX_AGE_MS are discarded (and counted) instead of returned.
 */
bool tel_queue_pop(tel_queue_t *q, uint8_t *data, uint16_t *len, uint32_t now_ms);

uint8_t tel_queue_depth(const tel_queue_t *q);

#endif /* TELEMETRY_H */
