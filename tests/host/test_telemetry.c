/*
 * test_telemetry.c
 * Host tests for the bit-packed telemetry formats and the bounded queue.
 *
 * The byte vectors below were produced by the independent Python reference
 * implementation (tools/robot_telemetry.py), not by the C code under test, so
 * a layout or byte-order mistake on either side shows up as a mismatch.  They
 * stand in for the Tribunal's "known test pattern decoded on the laptop".
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "telemetry.h"
#include "test_port.h"
#include "test_support.h"

static void check_bytes(const uint8_t *expected, size_t n, const uint8_t *actual, uint16_t got)
{
    CHECK_INT((long)n, got);
    CHECK(memcmp(expected, actual, n) == 0);
}

/* ---- bit writer / reader ---------------------------------------------- */

/* Fields are MSB first, can straddle byte boundaries, and read back exactly.
 * Catches: wrong bit order or byte order (big-endian on the wire). */
static void test_bit_round_trip_across_byte_boundaries(void)
{
    uint8_t buf[8];
    bitw_t w;
    bitr_t r;

    bitw_init(&w, buf, sizeof(buf));
    bitw_put(&w, 0x5U, 3);
    bitw_put(&w, 0x1FFU, 9);
    bitw_put(&w, 0x0U, 1);
    bitw_put(&w, 0xABCDU, 16);
    bitw_put_signed(&w, -3, 5);
    CHECK_INT(5, bitw_bytes(&w));               /* 34 bits */
    CHECK(!w.overflow);

    bitr_init(&r, buf, 5);
    CHECK_INT(0x5, bitr_get(&r, 3));
    CHECK_INT(0x1FF, bitr_get(&r, 9));
    CHECK_INT(0, bitr_get(&r, 1));
    CHECK_INT(0xABCD, bitr_get(&r, 16));
    CHECK_INT(-3, bitr_get_signed(&r, 5));
    CHECK(!r.underflow);
}

/* A value wider than its field is masked to the field, never spilling into
 * its neighbour. */
static void test_oversize_value_is_masked_to_its_width(void)
{
    uint8_t buf[2] = { 0U, 0U };
    bitw_t w;

    bitw_init(&w, buf, sizeof(buf));
    bitw_put(&w, 0xFFFFU, 4);
    bitw_put(&w, 0U, 4);
    CHECK_INT(0xF0, buf[0]);
}

/* Writing past the buffer sets the overflow flag instead of corrupting memory;
 * reading past the end sets underflow. */
static void test_buffer_limits_are_flagged(void)
{
    uint8_t buf[1];
    bitw_t w;
    bitr_t r;

    bitw_init(&w, buf, sizeof(buf));
    bitw_put(&w, 0xFFU, 8);
    CHECK(!w.overflow);
    bitw_put(&w, 1U, 1);
    CHECK(w.overflow);

    bitr_init(&r, buf, 1);
    (void)bitr_get(&r, 8);
    CHECK(!r.underflow);
    (void)bitr_get(&r, 1);
    CHECK(r.underflow);
}

/* ---- message vectors -------------------------------------------------- */

static tel_status_t sample_status(void)
{
    tel_status_t s;

    memset(&s, 0, sizeof(s));
    s.ts10 = 0x0102U;
    s.state = 5U;
    s.nav_cmd = 1U;
    s.line_position = -1;
    s.left_mm_s = -3;
    s.right_mm_s = 7;
    s.distance_mm = -2;
    s.faults = 0x8001U;
    s.obstacle_valid = true;
    s.obstacle_cm = 200U;
    s.hump_packed = 2349U;
    s.highest_hump_mm = 40U;
    return s;
}

/* The status message equals the independently computed bytes, and is 17 bytes
 * (134 bits) against a much larger in-memory struct.  Catches: any field
 * width, order or signedness error. */
static void test_status_matches_the_reference_vector(void)
{
    static const uint8_t EXPECTED[] = { 0x01, 0x01, 0x02, 0x52, 0x07, 0xff, 0xff, 0x40,
                                        0x7f, 0xff, 0xe8, 0x00, 0x1b, 0x21, 0x25, 0xaa, 0x00 };
    uint8_t out[TEL_MAX_MESSAGE];
    tel_status_t s = sample_status();
    tel_status_t d;
    uint16_t n = tel_encode_status(&s, out, sizeof(out));

    check_bytes(EXPECTED, sizeof(EXPECTED), out, n);
    CHECK(sizeof(tel_status_t) > n);            /* packing really shrinks it */

    CHECK(tel_decode_status(out, n, &d));
    CHECK_INT(0x0102, d.ts10);
    CHECK_INT(5, d.state);
    CHECK_INT(1, d.nav_cmd);
    CHECK_INT(-1, d.line_position);
    CHECK_INT(-3, d.left_mm_s);
    CHECK_INT(7, d.right_mm_s);
    CHECK_INT(-2, d.distance_mm);
    CHECK_INT(0x8001, d.faults);
    CHECK(d.obstacle_valid);
    CHECK_INT(200, d.obstacle_cm);
    CHECK_INT(2349, d.hump_packed);
    CHECK_INT(40, d.highest_hump_mm);
    CHECK_INT(0, d.saturated);
}

/* Fields that do not fit are saturated and FLAGGED, never wrapped and never
 * silently clipped (Tribunal Version C).  Catches: a clipped reading that
 * looks like a real one. */
static void test_out_of_range_fields_saturate_and_flag(void)
{
    static const uint8_t EXPECTED[] = { 0x01, 0x01, 0x02, 0x52, 0x03, 0xff, 0x7f, 0xe0,
                                        0x17, 0xff, 0xf8, 0x00, 0x1f, 0xf9, 0x25, 0xaa, 0x3c };
    uint8_t out[TEL_MAX_MESSAGE];
    tel_status_t s = sample_status();
    tel_status_t d;
    uint16_t n;

    s.line_position = 1500;
    s.left_mm_s = 900;
    s.right_mm_s = -900;
    s.distance_mm = 40000;
    s.obstacle_cm = 700U;
    n = tel_encode_status(&s, out, sizeof(out));
    check_bytes(EXPECTED, sizeof(EXPECTED), out, n);

    CHECK(tel_decode_status(out, n, &d));
    CHECK_INT(1023, d.line_position);
    CHECK_INT(511, d.left_mm_s);
    CHECK_INT(-511, d.right_mm_s);
    CHECK_INT(32767, d.distance_mm);
    CHECK_INT(510, d.obstacle_cm);
    CHECK_INT(TEL_SAT_POSITION | TEL_SAT_SPEED | TEL_SAT_DISTANCE | TEL_SAT_OBSTACLE,
              d.saturated);
    CHECK_INT(d.saturated, s.saturated);        /* the encoder reports it too */
}

/* A negative value at the exact limit is NOT flagged: -1023 is representable. */
static void test_values_at_the_limit_are_not_flagged(void)
{
    uint8_t out[TEL_MAX_MESSAGE];
    tel_status_t s = sample_status();
    tel_status_t d;
    uint16_t n;

    s.line_position = -1000;
    s.left_mm_s = -511;
    s.right_mm_s = 511;
    s.distance_mm = -32767;
    s.obstacle_cm = 510U;
    n = tel_encode_status(&s, out, sizeof(out));
    CHECK(tel_decode_status(out, n, &d));
    CHECK_INT(-1000, d.line_position);
    CHECK_INT(-32767, d.distance_mm);
    CHECK_INT(0, d.saturated);
}

/* An invalid ultrasonic reading is carried as invalid, not as a distance. */
static void test_invalid_obstacle_reading_is_not_a_distance(void)
{
    uint8_t out[TEL_MAX_MESSAGE];
    tel_status_t s = sample_status();
    tel_status_t d;
    uint16_t n;

    s.obstacle_valid = false;
    s.obstacle_cm = 300U;           /* must not leak onto the wire */
    n = tel_encode_status(&s, out, sizeof(out));
    CHECK(tel_decode_status(out, n, &d));
    CHECK(!d.obstacle_valid);
    CHECK_INT(0, d.obstacle_cm);
}

static void test_event_matches_the_reference_vector(void)
{
    static const uint8_t EXPECTED[] = { 0x02, 0x12, 0x34, 0x10, 0x02, 0x08, 0x38 };
    uint8_t out[TEL_MAX_MESSAGE];
    tel_event_t e = { 0x1234U, TEL_EVT_BARCODE, (uint16_t)'A', 7U };
    tel_event_t d;
    uint16_t n = tel_encode_event(&e, out, sizeof(out));

    check_bytes(EXPECTED, sizeof(EXPECTED), out, n);
    CHECK(tel_decode_event(out, n, &d));
    CHECK_INT(0x1234, d.ts10);
    CHECK_INT(TEL_EVT_BARCODE, d.code);
    CHECK_INT('A', d.arg);
    CHECK_INT(7, d.seq);
}

static void test_hump_matches_the_reference_vector(void)
{
    static const uint8_t EXPECTED[] = { 0x03, 0x00, 0xff, 0x49, 0x6a, 0x80 };
    uint8_t out[TEL_MAX_MESSAGE];
    tel_hump_t h = { 0x00FFU, 2349U, 40U };
    tel_hump_t d;
    uint16_t n = tel_encode_hump(&h, out, sizeof(out));

    check_bytes(EXPECTED, sizeof(EXPECTED), out, n);
    CHECK(tel_decode_hump(out, n, &d));
    CHECK_INT(2349, d.hump_packed);
    CHECK_INT(40, d.highest_hump_mm);
}

static void test_obstacle_matches_the_reference_vector(void)
{
    static const uint8_t EXPECTED[] = { 0x04, 0x00, 0x64, 0x87, 0x81, 0x5f, 0xc1, 0x80, 0x1c, 0x80 };
    uint8_t out[TEL_MAX_MESSAGE];
    tel_obstacle_t o = { 100U, true, 30U, 10U, -4, 48U, 7U, 1U, 0U };
    tel_obstacle_t d;
    uint16_t n = tel_encode_obstacle(&o, out, sizeof(out));

    check_bytes(EXPECTED, sizeof(EXPECTED), out, n);
    CHECK(tel_decode_obstacle(out, n, &d));
    CHECK(d.valid);
    CHECK_INT(30, d.closest_cm);
    CHECK_INT(10, d.width_cm);
    CHECK_INT(-4, d.centre_cm);
    CHECK_INT(48, d.left_clear_cm);
    CHECK_INT(7, d.right_clear_cm);
    CHECK_INT(1, d.action);
}

static tel_sensors_t sample_sensors(void)
{
    tel_sensors_t s;

    s.ts10 = 12345U;
    s.ir_levels = 5U;               /* left and right black */
    s.ir_valid_mask = 7U;
    s.pair_on_line = true;
    s.junction_armed = false;
    s.bars_now = 9U;
    s.bars_done = 3U;
    s.decoded = 2U;
    s.rejected = 1U;
    s.junctions = 4U;
    s.executed = 2U;
    s.map_entries = 26U;
    s.last_letter = 26U;            /* Z */
    s.motion_state = 1U;
    s.motion_stopped = false;
    s.encoder_fault = false;
    s.enc_bad_left = 12U;
    s.enc_bad_right = 200U;
    s.roll_permille = -137;
    s.imu_ok = true;
    s.lift_latched = false;
    s.hump_active = true;
    s.us_mm = 406U;
    s.us_valid = true;
    s.us_obstacle = false;
    s.us_fault = false;
    s.servo_aim = TEL_AIM_RIGHT;
    s.net_state = 3U;
    s.link_up = true;
    s.saturated = 0U;
    return s;
}

/* Every field of the per-sensor record survives an encode and a decode, and the record is 18 bytes. */
static void test_sensors_round_trip_and_size(void)
{
    uint8_t out[TEL_MAX_MESSAGE];
    tel_sensors_t s = sample_sensors();
    tel_sensors_t d;
    uint16_t n = tel_encode_sensors(&s, out, sizeof(out));

    CHECK_INT(18, n);
    CHECK_INT(TEL_TYPE_SENSORS, out[0]);
    CHECK(tel_decode_sensors(out, n, &d));
    CHECK_INT(12345, d.ts10);
    CHECK_INT(5, d.ir_levels);
    CHECK_INT(7, d.ir_valid_mask);
    CHECK(d.pair_on_line);
    CHECK(!d.junction_armed);
    CHECK_INT(9, d.bars_now);
    CHECK_INT(3, (int)d.bars_done);
    CHECK_INT(2, (int)d.decoded);
    CHECK_INT(1, (int)d.rejected);
    CHECK_INT(4, (int)d.junctions);
    CHECK_INT(2, (int)d.executed);
    CHECK_INT(26, d.map_entries);
    CHECK_INT(26, d.last_letter);
    CHECK_INT(1, d.motion_state);
    CHECK(!d.motion_stopped);
    CHECK(!d.encoder_fault);
    CHECK_INT(12, (int)d.enc_bad_left);
    CHECK_INT(200, (int)d.enc_bad_right);
    CHECK_INT(-137, d.roll_permille);
    CHECK(d.imu_ok);
    CHECK(!d.lift_latched);
    CHECK(d.hump_active);
    CHECK_INT(406, d.us_mm);
    CHECK(d.us_valid);
    CHECK(!d.us_obstacle);
    CHECK(!d.us_fault);
    CHECK_INT(TEL_AIM_RIGHT, d.servo_aim);
    CHECK_INT(3, d.net_state);
    CHECK(d.link_up);
    CHECK_INT(0, d.saturated);
}

/* A counter past 255 or a distance past 8190 mm is clipped and the record says so; the roll is clipped to
 * +/-1000.  Catches: a wrapped counter that looks like a small number. */
static void test_sensors_clip_and_flag_out_of_range_values(void)
{
    uint8_t out[TEL_MAX_MESSAGE];
    tel_sensors_t s = sample_sensors();
    tel_sensors_t d;
    uint16_t n;

    s.decoded = 300U;
    s.roll_permille = 1500;
    s.us_mm = 9000U;
    n = tel_encode_sensors(&s, out, sizeof(out));
    CHECK(n > 0U);
    CHECK_INT(1, s.saturated);
    CHECK(tel_decode_sensors(out, n, &d));
    CHECK_INT(255, (int)d.decoded);
    CHECK_INT(1000, d.roll_permille);
    CHECK_INT(8190, d.us_mm);
    CHECK_INT(1, d.saturated);
}

/* A short record or another type is refused. */
static void test_sensors_wrong_type_and_short_records_are_rejected(void)
{
    uint8_t out[TEL_MAX_MESSAGE];
    tel_sensors_t s = sample_sensors();
    tel_sensors_t d;
    uint16_t n = tel_encode_sensors(&s, out, sizeof(out));

    CHECK(!tel_decode_sensors(out, (uint16_t)(n - 1U), &d));
    out[0] = TEL_TYPE_STATUS;
    CHECK(!tel_decode_sensors(out, n, &d));
}

static void test_heartbeat_matches_the_reference_vector(void)
{
    static const uint8_t EXPECTED[] = { 0x05, 0x00, 0x00, 0x0e, 0x4d, 0x00, 0x03, 0x02, 0x2a };
    uint8_t out[TEL_MAX_MESSAGE];
    tel_heartbeat_t h = { 3661U, 3U, 2U, 5U, 2U };
    tel_heartbeat_t d;
    uint16_t n = tel_encode_heartbeat(&h, out, sizeof(out));

    check_bytes(EXPECTED, sizeof(EXPECTED), out, n);
    CHECK(tel_decode_heartbeat(out, n, &d));
    CHECK_INT(3661, d.uptime_s);
    CHECK_INT(3, d.dropped);
    CHECK_INT(2, d.reconnects);
    CHECK_INT(5, d.queue_depth);
    CHECK_INT(2, d.net_state);
}

/* The laptop -> robot command and the robot's ack. */
static void test_command_and_ack_round_trip(void)
{
    static const uint8_t EXPECTED[] = { 0x10, 0x09, 0x21, 0x23, 0x40 };
    uint8_t out[TEL_MAX_MESSAGE];
    tel_command_t c = { 9U, TEL_CMD_START, 0x1234U };
    tel_command_t dc;
    tel_ack_t a = { 9U, TEL_ACK_REJECTED_RUNNING };
    tel_ack_t da;
    uint16_t n = tel_encode_command(&c, out, sizeof(out));

    check_bytes(EXPECTED, sizeof(EXPECTED), out, n);
    CHECK(tel_decode_command(out, n, &dc));
    CHECK_INT(9, dc.req_id);
    CHECK_INT(TEL_CMD_START, dc.cmd);
    CHECK_INT(0x1234, dc.param);

    n = tel_encode_ack(&a, out, sizeof(out));
    CHECK_INT(3, n);
    CHECK(tel_decode_ack(out, n, &da));
    CHECK_INT(9, da.req_id);
    CHECK_INT(TEL_ACK_REJECTED_RUNNING, da.result);
}

/* A message of the wrong type, or too short, is rejected - never decoded into
 * garbage (Tribunal: malformed payload tests). */
static void test_wrong_type_and_short_messages_are_rejected(void)
{
    uint8_t out[TEL_MAX_MESSAGE];
    tel_status_t s = sample_status();
    tel_status_t d;
    tel_event_t e;
    tel_command_t c;
    uint16_t n = tel_encode_status(&s, out, sizeof(out));

    CHECK(!tel_decode_event(out, n, &e));               /* wrong type */
    CHECK(!tel_decode_status(out, (uint16_t)(n - 1U), &d));   /* truncated */
    CHECK(!tel_decode_status(out, 0U, &d));
    CHECK(!tel_decode_command(out, n, &c));
}

/* A buffer that is too small yields 0 bytes, not a truncated message. */
static void test_too_small_output_buffer_encodes_nothing(void)
{
    uint8_t out[8];
    tel_status_t s = sample_status();

    CHECK_INT(0, tel_encode_status(&s, out, sizeof(out)));
}

/* ---- the queue -------------------------------------------------------- */

static uint8_t rec(uint8_t v, uint8_t *buf)
{
    buf[0] = v;
    buf[1] = (uint8_t)(v + 1U);
    return 2U;
}

/* FIFO order, with the bounded depth and wrap-around.  Catches: index errors
 * at the ring's end. */
static void test_queue_is_fifo_and_wraps(void)
{
    tel_queue_t q;
    uint8_t b[TEL_RECORD_MAX];
    uint16_t len = 0U;
    unsigned i;

    tel_queue_init(&q);
    for (i = 0U; i < 40U; i++) {
        uint8_t n = rec((uint8_t)i, b);

        CHECK(tel_queue_push(&q, b, n, 1000U));
        CHECK(tel_queue_pop(&q, b, &len, 1000U));
        CHECK_INT(2, len);
        CHECK_INT(i, b[0]);
    }
    CHECK_INT(0, tel_queue_depth(&q));
}

/* A full queue drops the NEW record and counts it; old records are kept and
 * the producer never blocks.  Catches: a full buffer blocking a control task
 * or silently overwriting (Tribunal: drop new, count). */
static void test_full_queue_drops_new_records_and_counts_them(void)
{
    tel_queue_t q;
    uint8_t b[TEL_RECORD_MAX];
    uint16_t len = 0U;
    unsigned i;

    tel_queue_init(&q);
    for (i = 0U; i < TEL_QUEUE_DEPTH; i++) {
        CHECK(tel_queue_push(&q, b, rec((uint8_t)i, b), 1000U));
    }
    CHECK(!tel_queue_push(&q, b, rec(99U, b), 1000U));
    CHECK(!tel_queue_push(&q, b, rec(98U, b), 1000U));
    CHECK_INT(2, q.dropped_full);
    CHECK_INT(TEL_QUEUE_DEPTH, tel_queue_depth(&q));

    CHECK(tel_queue_pop(&q, b, &len, 1000U));
    CHECK_INT(0, b[0]);                         /* the oldest survived */
}

/* Records older than one second are discarded, not published late. */
static void test_stale_records_are_discarded_not_published(void)
{
    tel_queue_t q;
    uint8_t b[TEL_RECORD_MAX];
    uint16_t len = 0U;

    tel_queue_init(&q);
    CHECK(tel_queue_push(&q, b, rec(1U, b), 1000U));
    CHECK(tel_queue_push(&q, b, rec(2U, b), 1900U));
    CHECK(tel_queue_pop(&q, b, &len, 2100U));   /* the first is 1100 ms old */
    CHECK_INT(2, b[0]);
    CHECK_INT(1, q.discarded_stale);
    CHECK(!tel_queue_pop(&q, b, &len, 2100U));  /* nothing left */
}

/* A record exactly at the age limit is still sent; one ms older is not. */
static void test_freshness_boundary(void)
{
    tel_queue_t q;
    uint8_t b[TEL_RECORD_MAX];
    uint16_t len = 0U;

    tel_queue_init(&q);
    CHECK(tel_queue_push(&q, b, rec(1U, b), 0U));
    CHECK(tel_queue_pop(&q, b, &len, TEL_MAX_AGE_MS));
    CHECK(tel_queue_push(&q, b, rec(2U, b), 0U));
    CHECK(!tel_queue_pop(&q, b, &len, TEL_MAX_AGE_MS + 1U));
}

/* Oversize records are refused rather than truncated. */
static void test_oversize_record_is_refused(void)
{
    tel_queue_t q;
    uint8_t big[TEL_RECORD_MAX + 1U];

    memset(big, 0, sizeof(big));
    tel_queue_init(&q);
    CHECK(!tel_queue_push(&q, big, (uint16_t)sizeof(big), 0U));
    CHECK_INT(0, tel_queue_depth(&q));
    CHECK(tel_queue_push(&q, big, TEL_RECORD_MAX, 0U));
}

/* Queue operations are inside balanced critical sections. */
static void test_queue_updates_are_protected(void)
{
    tel_queue_t q;
    uint8_t b[TEL_RECORD_MAX];
    uint16_t len = 0U;

    tel_queue_init(&q);
    g_port_sections = 0;
    (void)tel_queue_push(&q, b, rec(1U, b), 0U);
    (void)tel_queue_pop(&q, b, &len, 0U);
    CHECK(g_port_sections >= 2);
    CHECK_INT(0, g_port_depth);
}

int main(void)
{
    RUN(test_bit_round_trip_across_byte_boundaries);
    RUN(test_oversize_value_is_masked_to_its_width);
    RUN(test_buffer_limits_are_flagged);
    RUN(test_status_matches_the_reference_vector);
    RUN(test_out_of_range_fields_saturate_and_flag);
    RUN(test_values_at_the_limit_are_not_flagged);
    RUN(test_invalid_obstacle_reading_is_not_a_distance);
    RUN(test_event_matches_the_reference_vector);
    RUN(test_hump_matches_the_reference_vector);
    RUN(test_obstacle_matches_the_reference_vector);
    RUN(test_sensors_round_trip_and_size);
    RUN(test_sensors_clip_and_flag_out_of_range_values);
    RUN(test_sensors_wrong_type_and_short_records_are_rejected);
    RUN(test_heartbeat_matches_the_reference_vector);
    RUN(test_command_and_ack_round_trip);
    RUN(test_wrong_type_and_short_messages_are_rejected);
    RUN(test_too_small_output_buffer_encodes_nothing);
    RUN(test_queue_is_fifo_and_wraps);
    RUN(test_full_queue_drops_new_records_and_counts_them);
    RUN(test_stale_records_are_discarded_not_published);
    RUN(test_freshness_boundary);
    RUN(test_oversize_record_is_refused);
    RUN(test_queue_updates_are_protected);
    return TEST_REPORT();
}
