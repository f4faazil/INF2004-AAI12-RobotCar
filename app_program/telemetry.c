/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * telemetry.c
 * Bit-packed telemetry messages and the bounded queue.  See telemetry.h.
 */
#include "robot_port.h"
#include "telemetry.h"

#define BITS_PER_BYTE       (8U)

/* Representable ranges of the signed and unsigned fields. */
#define POSITION_LIMIT      (1023)
#define SPEED_LIMIT         (511)
#define DISTANCE_LIMIT      (32767)
#define CENTRE_CM_LIMIT     (255)
#define CM_MAX              (511)

/* Message sizes in bits, for the short-message check in the decoders. */
#define STATUS_BITS         (TEL_TYPE_BITS + TEL_TS_BITS + TEL_STATE_BITS + TEL_NAVCMD_BITS + \
                             TEL_LINE_BITS + TEL_JUNCTION_BITS + TEL_BARCODE_BITS + \
                             TEL_POSITION_BITS + (2U * TEL_SPEED_BITS) + TEL_DISTANCE_BITS + \
                             TEL_FAULT_BITS + 1U + TEL_OBSTACLE_BITS + TEL_HUMP_BITS + \
                             TEL_HIGHEST_BITS + TEL_SAT_BITS)
#define EVENT_BITS          (TEL_TYPE_BITS + TEL_TS_BITS + TEL_EVT_BITS + TEL_EVT_ARG_BITS + \
                             TEL_EVT_SEQ_BITS)
#define HUMP_MSG_BITS       (TEL_TYPE_BITS + TEL_TS_BITS + TEL_HUMP_BITS + TEL_HIGHEST_BITS)
#define OBSTACLE_MSG_BITS   (TEL_TYPE_BITS + TEL_TS_BITS + 1U + (5U * TEL_CM_BITS) + 3U + 3U)
#define HEARTBEAT_BITS      (TEL_TYPE_BITS + 32U + 16U + 8U + 5U + 3U)
#define COMMAND_BITS        (TEL_TYPE_BITS + TEL_REQ_BITS + TEL_CMD_BITS + TEL_PARAM_BITS)
#define ACK_BITS            (TEL_TYPE_BITS + TEL_REQ_BITS + TEL_ACK_BITS)

static uint16_t bytes_for(uint16_t bits)
{
    return (uint16_t)((bits + BITS_PER_BYTE - 1U) / BITS_PER_BYTE);
}

/* ---- bit writer / reader --------------------------------------------- */

void bitw_init(bitw_t *w, uint8_t *buf, uint16_t cap_bytes)
{
    uint16_t i;

    w->buf = buf;
    w->cap_bits = (uint16_t)(cap_bytes * BITS_PER_BYTE);
    w->pos = 0U;
    w->overflow = false;
    for (i = 0U; i < cap_bytes; i++) {
        buf[i] = 0U;
    }
}

void bitw_put(bitw_t *w, uint32_t value, uint8_t width)
{
    uint8_t i;

    if (((uint32_t)w->pos + width) > w->cap_bits) {
        w->overflow = true;
        return;
    }
    for (i = 0U; i < width; i++) {
        uint8_t shift = (uint8_t)(width - 1U - i);

        if (((value >> shift) & 1U) != 0U) {
            w->buf[w->pos / BITS_PER_BYTE] |=
                (uint8_t)(0x80U >> (w->pos % BITS_PER_BYTE));
        }
        w->pos++;
    }
}

void bitw_put_signed(bitw_t *w, int32_t value, uint8_t width)
{
    bitw_put(w, (uint32_t)value, width);        /* two's complement, masked by width */
}

uint16_t bitw_bytes(const bitw_t *w)
{
    return bytes_for(w->pos);
}

void bitr_init(bitr_t *r, const uint8_t *buf, uint16_t len_bytes)
{
    r->buf = buf;
    r->len_bits = (uint16_t)(len_bytes * BITS_PER_BYTE);
    r->pos = 0U;
    r->underflow = false;
}

uint32_t bitr_get(bitr_t *r, uint8_t width)
{
    uint32_t value = 0U;
    uint8_t i;

    if (((uint32_t)r->pos + width) > r->len_bits) {
        r->underflow = true;
        return 0U;
    }
    for (i = 0U; i < width; i++) {
        uint8_t bit = (uint8_t)((r->buf[r->pos / BITS_PER_BYTE] >>
                                 (7U - (r->pos % BITS_PER_BYTE))) & 1U);

        value = (value << 1) | bit;
        r->pos++;
    }
    return value;
}

int32_t bitr_get_signed(bitr_t *r, uint8_t width)
{
    uint32_t v = bitr_get(r, width);
    uint32_t sign = (uint32_t)1U << (width - 1U);

    if ((v & sign) != 0U) {
        return (int32_t)(v - ((uint32_t)1U << width));
    }
    return (int32_t)v;
}

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/* Common prologue: header check and minimum length. */
static bool open_reader(bitr_t *r, const uint8_t *in, uint16_t len, uint8_t type,
                        uint16_t min_bits)
{
    if (len < bytes_for(min_bits)) {
        return false;
    }
    bitr_init(r, in, len);
    return bitr_get(r, TEL_TYPE_BITS) == type;
}

/* ---- status ----------------------------------------------------------- */

uint16_t tel_encode_status(tel_status_t *s, uint8_t *out, uint16_t cap)
{
    bitw_t w;
    int32_t pos = clamp_i32(s->line_position, -POSITION_LIMIT, POSITION_LIMIT);
    int32_t left = clamp_i32(s->left_mm_s, -SPEED_LIMIT, SPEED_LIMIT);
    int32_t right = clamp_i32(s->right_mm_s, -SPEED_LIMIT, SPEED_LIMIT);
    int32_t dist = clamp_i32(s->distance_mm, -DISTANCE_LIMIT, DISTANCE_LIMIT);
    uint32_t cm = s->obstacle_valid ? s->obstacle_cm : 0U;
    uint8_t sat = 0U;

    if (pos != s->line_position) {
        sat |= TEL_SAT_POSITION;
    }
    if ((left != s->left_mm_s) || (right != s->right_mm_s)) {
        sat |= TEL_SAT_SPEED;
    }
    if (dist != s->distance_mm) {
        sat |= TEL_SAT_DISTANCE;
    }
    if (cm > TEL_OBSTACLE_MAX_CM) {
        cm = TEL_OBSTACLE_MAX_CM;
        sat |= TEL_SAT_OBSTACLE;
    }
    s->saturated = sat;

    bitw_init(&w, out, cap);
    bitw_put(&w, TEL_TYPE_STATUS, TEL_TYPE_BITS);
    bitw_put(&w, s->ts10, TEL_TS_BITS);
    bitw_put(&w, s->state, TEL_STATE_BITS);
    bitw_put(&w, s->nav_cmd, TEL_NAVCMD_BITS);
    bitw_put(&w, s->line_state, TEL_LINE_BITS);
    bitw_put(&w, s->junction_state, TEL_JUNCTION_BITS);
    bitw_put(&w, s->barcode_state, TEL_BARCODE_BITS);
    bitw_put_signed(&w, pos, TEL_POSITION_BITS);
    bitw_put_signed(&w, left, TEL_SPEED_BITS);
    bitw_put_signed(&w, right, TEL_SPEED_BITS);
    bitw_put_signed(&w, dist, TEL_DISTANCE_BITS);
    bitw_put(&w, s->faults, TEL_FAULT_BITS);
    bitw_put(&w, s->obstacle_valid ? 1U : 0U, 1U);
    bitw_put(&w, cm, TEL_OBSTACLE_BITS);
    bitw_put(&w, s->hump_packed, TEL_HUMP_BITS);
    bitw_put(&w, s->highest_hump_mm, TEL_HIGHEST_BITS);
    bitw_put(&w, sat, TEL_SAT_BITS);
    return w.overflow ? 0U : bitw_bytes(&w);
}

bool tel_decode_status(const uint8_t *in, uint16_t len, tel_status_t *s)
{
    bitr_t r;

    if (!open_reader(&r, in, len, TEL_TYPE_STATUS, STATUS_BITS)) {
        return false;
    }
    s->ts10 = (uint16_t)bitr_get(&r, TEL_TS_BITS);
    s->state = (uint8_t)bitr_get(&r, TEL_STATE_BITS);
    s->nav_cmd = (uint8_t)bitr_get(&r, TEL_NAVCMD_BITS);
    s->line_state = (uint8_t)bitr_get(&r, TEL_LINE_BITS);
    s->junction_state = (uint8_t)bitr_get(&r, TEL_JUNCTION_BITS);
    s->barcode_state = (uint8_t)bitr_get(&r, TEL_BARCODE_BITS);
    s->line_position = (int16_t)bitr_get_signed(&r, TEL_POSITION_BITS);
    s->left_mm_s = (int16_t)bitr_get_signed(&r, TEL_SPEED_BITS);
    s->right_mm_s = (int16_t)bitr_get_signed(&r, TEL_SPEED_BITS);
    s->distance_mm = bitr_get_signed(&r, TEL_DISTANCE_BITS);
    s->faults = (uint16_t)bitr_get(&r, TEL_FAULT_BITS);
    s->obstacle_valid = bitr_get(&r, 1U) != 0U;
    s->obstacle_cm = (uint16_t)bitr_get(&r, TEL_OBSTACLE_BITS);
    s->hump_packed = (uint16_t)bitr_get(&r, TEL_HUMP_BITS);
    s->highest_hump_mm = (uint8_t)bitr_get(&r, TEL_HIGHEST_BITS);
    s->saturated = (uint8_t)bitr_get(&r, TEL_SAT_BITS);
    return !r.underflow;
}

/* ---- the smaller messages ---------------------------------------------- */

uint16_t tel_encode_event(const tel_event_t *e, uint8_t *out, uint16_t cap)
{
    bitw_t w;

    bitw_init(&w, out, cap);
    bitw_put(&w, TEL_TYPE_EVENT, TEL_TYPE_BITS);
    bitw_put(&w, e->ts10, TEL_TS_BITS);
    bitw_put(&w, e->code, TEL_EVT_BITS);
    bitw_put(&w, e->arg, TEL_EVT_ARG_BITS);
    bitw_put(&w, e->seq, TEL_EVT_SEQ_BITS);
    return w.overflow ? 0U : bitw_bytes(&w);
}

bool tel_decode_event(const uint8_t *in, uint16_t len, tel_event_t *e)
{
    bitr_t r;

    if (!open_reader(&r, in, len, TEL_TYPE_EVENT, EVENT_BITS)) {
        return false;
    }
    e->ts10 = (uint16_t)bitr_get(&r, TEL_TS_BITS);
    e->code = (uint8_t)bitr_get(&r, TEL_EVT_BITS);
    e->arg = (uint16_t)bitr_get(&r, TEL_EVT_ARG_BITS);
    e->seq = (uint8_t)bitr_get(&r, TEL_EVT_SEQ_BITS);
    return !r.underflow;
}

uint16_t tel_encode_hump(const tel_hump_t *h, uint8_t *out, uint16_t cap)
{
    bitw_t w;

    bitw_init(&w, out, cap);
    bitw_put(&w, TEL_TYPE_HUMP, TEL_TYPE_BITS);
    bitw_put(&w, h->ts10, TEL_TS_BITS);
    bitw_put(&w, h->hump_packed, TEL_HUMP_BITS);
    bitw_put(&w, h->highest_hump_mm, TEL_HIGHEST_BITS);
    return w.overflow ? 0U : bitw_bytes(&w);
}

bool tel_decode_hump(const uint8_t *in, uint16_t len, tel_hump_t *h)
{
    bitr_t r;

    if (!open_reader(&r, in, len, TEL_TYPE_HUMP, HUMP_MSG_BITS)) {
        return false;
    }
    h->ts10 = (uint16_t)bitr_get(&r, TEL_TS_BITS);
    h->hump_packed = (uint16_t)bitr_get(&r, TEL_HUMP_BITS);
    h->highest_hump_mm = (uint8_t)bitr_get(&r, TEL_HIGHEST_BITS);
    return !r.underflow;
}

uint16_t tel_encode_obstacle(const tel_obstacle_t *o, uint8_t *out, uint16_t cap)
{
    bitw_t w;

    bitw_init(&w, out, cap);
    bitw_put(&w, TEL_TYPE_OBSTACLE, TEL_TYPE_BITS);
    bitw_put(&w, o->ts10, TEL_TS_BITS);
    bitw_put(&w, o->valid ? 1U : 0U, 1U);
    bitw_put(&w, (uint32_t)clamp_i32(o->closest_cm, 0, CM_MAX), TEL_CM_BITS);
    bitw_put(&w, (uint32_t)clamp_i32(o->width_cm, 0, CM_MAX), TEL_CM_BITS);
    bitw_put_signed(&w, clamp_i32(o->centre_cm, -CENTRE_CM_LIMIT, CENTRE_CM_LIMIT), TEL_CM_BITS);
    bitw_put(&w, (uint32_t)clamp_i32(o->left_clear_cm, 0, CM_MAX), TEL_CM_BITS);
    bitw_put(&w, (uint32_t)clamp_i32(o->right_clear_cm, 0, CM_MAX), TEL_CM_BITS);
    bitw_put(&w, o->action, 3U);
    bitw_put(&w, o->reason, 3U);
    return w.overflow ? 0U : bitw_bytes(&w);
}

bool tel_decode_obstacle(const uint8_t *in, uint16_t len, tel_obstacle_t *o)
{
    bitr_t r;

    if (!open_reader(&r, in, len, TEL_TYPE_OBSTACLE, OBSTACLE_MSG_BITS)) {
        return false;
    }
    o->ts10 = (uint16_t)bitr_get(&r, TEL_TS_BITS);
    o->valid = bitr_get(&r, 1U) != 0U;
    o->closest_cm = (uint16_t)bitr_get(&r, TEL_CM_BITS);
    o->width_cm = (uint16_t)bitr_get(&r, TEL_CM_BITS);
    o->centre_cm = (int16_t)bitr_get_signed(&r, TEL_CM_BITS);
    o->left_clear_cm = (uint16_t)bitr_get(&r, TEL_CM_BITS);
    o->right_clear_cm = (uint16_t)bitr_get(&r, TEL_CM_BITS);
    o->action = (uint8_t)bitr_get(&r, 3U);
    o->reason = (uint8_t)bitr_get(&r, 3U);
    return !r.underflow;
}

uint16_t tel_encode_heartbeat(const tel_heartbeat_t *h, uint8_t *out, uint16_t cap)
{
    bitw_t w;

    bitw_init(&w, out, cap);
    bitw_put(&w, TEL_TYPE_HEARTBEAT, TEL_TYPE_BITS);
    bitw_put(&w, h->uptime_s, 32U);
    bitw_put(&w, h->dropped, 16U);
    bitw_put(&w, h->reconnects, 8U);
    bitw_put(&w, h->queue_depth, 5U);
    bitw_put(&w, h->net_state, 3U);
    return w.overflow ? 0U : bitw_bytes(&w);
}

bool tel_decode_heartbeat(const uint8_t *in, uint16_t len, tel_heartbeat_t *h)
{
    bitr_t r;

    if (!open_reader(&r, in, len, TEL_TYPE_HEARTBEAT, HEARTBEAT_BITS)) {
        return false;
    }
    h->uptime_s = bitr_get(&r, 32U);
    h->dropped = (uint16_t)bitr_get(&r, 16U);
    h->reconnects = (uint8_t)bitr_get(&r, 8U);
    h->queue_depth = (uint8_t)bitr_get(&r, 5U);
    h->net_state = (uint8_t)bitr_get(&r, 3U);
    return !r.underflow;
}

uint16_t tel_encode_command(const tel_command_t *c, uint8_t *out, uint16_t cap)
{
    bitw_t w;

    bitw_init(&w, out, cap);
    bitw_put(&w, TEL_TYPE_COMMAND, TEL_TYPE_BITS);
    bitw_put(&w, c->req_id, TEL_REQ_BITS);
    bitw_put(&w, c->cmd, TEL_CMD_BITS);
    bitw_put(&w, c->param, TEL_PARAM_BITS);
    return w.overflow ? 0U : bitw_bytes(&w);
}

bool tel_decode_command(const uint8_t *in, uint16_t len, tel_command_t *c)
{
    bitr_t r;

    if (!open_reader(&r, in, len, TEL_TYPE_COMMAND, COMMAND_BITS)) {
        return false;
    }
    c->req_id = (uint8_t)bitr_get(&r, TEL_REQ_BITS);
    c->cmd = (uint8_t)bitr_get(&r, TEL_CMD_BITS);
    c->param = (uint16_t)bitr_get(&r, TEL_PARAM_BITS);
    return !r.underflow;
}

uint16_t tel_encode_ack(const tel_ack_t *a, uint8_t *out, uint16_t cap)
{
    bitw_t w;

    bitw_init(&w, out, cap);
    bitw_put(&w, TEL_TYPE_ACK, TEL_TYPE_BITS);
    bitw_put(&w, a->req_id, TEL_REQ_BITS);
    bitw_put(&w, a->result, TEL_ACK_BITS);
    return w.overflow ? 0U : bitw_bytes(&w);
}

bool tel_decode_ack(const uint8_t *in, uint16_t len, tel_ack_t *a)
{
    bitr_t r;

    if (!open_reader(&r, in, len, TEL_TYPE_ACK, ACK_BITS)) {
        return false;
    }
    a->req_id = (uint8_t)bitr_get(&r, TEL_REQ_BITS);
    a->result = (uint8_t)bitr_get(&r, TEL_ACK_BITS);
    return !r.underflow;
}

/* ---- the queue --------------------------------------------------------- */

void tel_queue_init(tel_queue_t *q)
{
    q->head = 0U;
    q->count = 0U;
    q->pushed = 0U;
    q->dropped_full = 0U;
    q->discarded_stale = 0U;
}

bool tel_queue_push(tel_queue_t *q, const uint8_t *data, uint16_t len, uint32_t now_ms)
{
    robot_irq_state_t state;
    bool ok = false;

    if (len > TEL_RECORD_MAX) {
        return false;
    }
    state = robot_irq_save();
    if (q->count >= TEL_QUEUE_DEPTH) {
        q->dropped_full++;                      /* drop the NEW record */
    } else {
        tel_record_t *slot = &q->rec[(q->head + q->count) % TEL_QUEUE_DEPTH];
        uint16_t i;

        for (i = 0U; i < len; i++) {
            slot->data[i] = data[i];
        }
        slot->len = (uint8_t)len;
        slot->ts_ms = now_ms;
        q->count++;
        q->pushed++;
        ok = true;
    }
    robot_irq_restore(state);
    return ok;
}

bool tel_queue_pop(tel_queue_t *q, uint8_t *data, uint16_t *len, uint32_t now_ms)
{
    robot_irq_state_t state = robot_irq_save();
    bool ok = false;

    while (q->count > 0U) {
        const tel_record_t *slot = &q->rec[q->head];

        q->head = (uint8_t)((q->head + 1U) % TEL_QUEUE_DEPTH);
        q->count--;
        if ((now_ms - slot->ts_ms) > TEL_MAX_AGE_MS) {
            q->discarded_stale++;               /* too old to be worth sending */
            continue;
        }
        {
            uint16_t i;

            for (i = 0U; i < slot->len; i++) {
                data[i] = slot->data[i];
            }
            *len = slot->len;
        }
        ok = true;
        break;
    }
    robot_irq_restore(state);
    return ok;
}

uint8_t tel_queue_depth(const tel_queue_t *q)
{
    return q->count;
}
