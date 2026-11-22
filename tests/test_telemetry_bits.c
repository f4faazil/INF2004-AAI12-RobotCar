#include "../app_program/telemetry_bits.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void test_hump_candidate_layout(void)
{
    /* Example ONLY, from live Tribunal suggestion, not a final schema.
       type=0x04, status=5(3b), height=100(7b), validity=2(2b), fault=1(1b). */
    uint8_t bytes[3];
    telemetry_bit_writer_t w;
    telemetry_bit_reader_t r;
    uint32_t field = 0U;
    const uint8_t expected[] = {0x04U, 0x25U, 0x1BU};

    assert(telemetry_bits_writer_init(&w, bytes, sizeof bytes));
    assert(telemetry_bits_write(&w, 0x04U, 8U));
    assert(telemetry_bits_write(&w, 5U, 3U));
    assert(telemetry_bits_write(&w, 100U, 7U));
    assert(telemetry_bits_write(&w, 2U, 2U));
    assert(telemetry_bits_write(&w, 1U, 1U));
    assert(telemetry_bits_written_bytes(&w) == 3U);
    assert(memcmp(bytes, expected, sizeof expected) == 0);

    assert(telemetry_bits_reader_init(&r, bytes, sizeof bytes));
    assert(telemetry_bits_read(&r, 8U, &field) && field == 4U);
    assert(telemetry_bits_read(&r, 3U, &field) && field == 5U);
    assert(telemetry_bits_read(&r, 7U, &field) && field == 100U);
    assert(telemetry_bits_read(&r, 2U, &field) && field == 2U);
    assert(telemetry_bits_read(&r, 1U, &field) && field == 1U);
}

static void test_reject_out_of_range_without_partial_write(void)
{
    uint8_t bytes[2];
    telemetry_bit_writer_t w;
    assert(telemetry_bits_writer_init(&w, bytes, sizeof bytes));
    assert(telemetry_bits_write(&w, 7U, 3U));
    const size_t old_position = w.used_bits;
    const uint8_t snapshot[2] = {bytes[0], bytes[1]};
    assert(!telemetry_bits_write(&w, 8U, 3U));
    assert(!telemetry_bits_write(&w, 1U, 0U));
    assert(!telemetry_bits_write(&w, 1U, 33U));
    assert(w.used_bits == old_position);
    assert(memcmp(bytes, snapshot, sizeof bytes) == 0);
}

static void test_reject_capacity_overflow_and_truncation(void)
{
    uint8_t bytes[1];
    telemetry_bit_writer_t w;
    telemetry_bit_reader_t r;
    uint32_t unchanged = 77U;

    assert(telemetry_bits_writer_init(&w, bytes, sizeof bytes));
    assert(telemetry_bits_write(&w, 255U, 8U));
    assert(!telemetry_bits_write(&w, 1U, 1U));
    assert(w.used_bits == 8U && bytes[0] == 255U);

    assert(telemetry_bits_reader_init(&r, bytes, sizeof bytes));
    assert(telemetry_bits_read(&r, 8U, &unchanged));
    unchanged = 77U;
    assert(!telemetry_bits_read(&r, 1U, &unchanged));
    assert(unchanged == 77U && r.used_bits == 8U);
}

static void test_all_32_bits_and_boundaries(void)
{
    uint8_t bytes[5];
    telemetry_bit_writer_t w;
    telemetry_bit_reader_t r;
    uint32_t field;

    assert(telemetry_bits_writer_init(&w, bytes, sizeof bytes));
    assert(telemetry_bits_write(&w, 1U, 1U));
    assert(telemetry_bits_write(&w, UINT32_MAX, 32U));
    assert(telemetry_bits_written_bytes(&w) == 5U);
    assert(telemetry_bits_reader_init(&r, bytes, sizeof bytes));
    assert(telemetry_bits_read(&r, 1U, &field) && field == 1U);
    assert(telemetry_bits_read(&r, 32U, &field) && field == UINT32_MAX);
}


/*
 * PROVISIONAL example schema only. Version C requires a 13-bit hump record
 * and reserves height=127 as the out-of-range marker. The IDs below are
 * test-only choices: type 0x04, validity=0 means valid, validity=3 means
 * out-of-range. None of these numeric codes are approved by the team yet.
 *
 * Wire format for this example:
 *  byte 0: 8-bit type
 *  next 13 bits: status(3), height(7), validity(2), IMU fault(1)
 *  top three bits of byte 2 must be zero padding.
 */
static bool candidate_hump_frame_ok(const uint8_t *bytes, size_t length)
{
    telemetry_bit_reader_t r;
    uint32_t type, status, height, validity, fault;

    if (bytes == NULL || length != 3U || (bytes[2] & 0xE0U) != 0U) {
        return false;
    }
    if (!telemetry_bits_reader_init(&r, bytes, length)) {
        return false;
    }
    if (!telemetry_bits_read(&r, 8U, &type) ||
        !telemetry_bits_read(&r, 3U, &status) ||
        !telemetry_bits_read(&r, 7U, &height) ||
        !telemetry_bits_read(&r, 2U, &validity) ||
        !telemetry_bits_read(&r, 1U, &fault)) {
        return false;
    }
    (void)status;
    (void)fault;
    if (type != 0x04U) {
        return false;
    }
    /* Code 127 may never be presented as an ordinary valid 127 mm value. */
    if (height == 127U && validity == 0U) {
        return false;
    }
    return true;
}

static void test_version_c_hump_sentinel(void)
{
    uint8_t bytes[3];
    telemetry_bit_writer_t w;

    /* Maximum representable genuine value is 126 mm in this proposal. */
    assert(telemetry_bits_writer_init(&w, bytes, sizeof bytes));
    assert(telemetry_bits_write(&w, 0x04U, 8U));
    assert(telemetry_bits_write(&w, 1U, 3U));
    assert(telemetry_bits_write(&w, 126U, 7U));
    assert(telemetry_bits_write(&w, 0U, 2U));
    assert(telemetry_bits_write(&w, 0U, 1U));
    assert(telemetry_bits_written_bytes(&w) == 3U);
    assert(candidate_hump_frame_ok(bytes, sizeof bytes));

    /* Height=127 carries a mandatory not-valid indicator. */
    assert(telemetry_bits_writer_init(&w, bytes, sizeof bytes));
    assert(telemetry_bits_write(&w, 0x04U, 8U));
    assert(telemetry_bits_write(&w, 1U, 3U));
    assert(telemetry_bits_write(&w, 127U, 7U));
    assert(telemetry_bits_write(&w, 3U, 2U));
    assert(telemetry_bits_write(&w, 0U, 1U));
    assert(candidate_hump_frame_ok(bytes, sizeof bytes));

    /* Same sentinel but falsely labelled valid must be rejected. */
    assert(telemetry_bits_writer_init(&w, bytes, sizeof bytes));
    assert(telemetry_bits_write(&w, 0x04U, 8U));
    assert(telemetry_bits_write(&w, 1U, 3U));
    assert(telemetry_bits_write(&w, 127U, 7U));
    assert(telemetry_bits_write(&w, 0U, 2U));
    assert(telemetry_bits_write(&w, 0U, 1U));
    assert(!candidate_hump_frame_ok(bytes, sizeof bytes));
    assert(!telemetry_bits_write(&w, 128U, 7U));
}

static void test_version_c_malformed_frames(void)
{
    /* Published example: type/status/height/validity/fault, then padding. */
    uint8_t bytes[3] = {0x04U, 0x25U, 0x1BU};
    assert(candidate_hump_frame_ok(bytes, sizeof bytes));
    assert(!candidate_hump_frame_ok(bytes, 2U));
    assert(!candidate_hump_frame_ok(NULL, 3U));
    bytes[0] = 0x07U;          /* Wrong type */
    assert(!candidate_hump_frame_ok(bytes, sizeof bytes));
    bytes[0] = 0x04U;
    bytes[2] |= 0x80U;         /* Reserved padding must stay zero */
    assert(!candidate_hump_frame_ok(bytes, sizeof bytes));
}

static void test_null_arguments_and_zero_width(void)
{
    uint8_t bytes[2];
    telemetry_bit_writer_t w;
    telemetry_bit_reader_t r;
    uint32_t out = 42U;
    assert(!telemetry_bits_writer_init(NULL, bytes, sizeof bytes));
    assert(!telemetry_bits_writer_init(&w, NULL, sizeof bytes));
    assert(telemetry_bits_writer_init(&w, bytes, sizeof bytes));
    assert(!telemetry_bits_write(NULL, 1U, 1U));
    assert(!telemetry_bits_write(&w, 1U, 0U));
    assert(!telemetry_bits_reader_init(NULL, bytes, sizeof bytes));
    assert(!telemetry_bits_reader_init(&r, NULL, sizeof bytes));
    assert(telemetry_bits_reader_init(&r, bytes, sizeof bytes));
    assert(!telemetry_bits_read(&r, 1U, NULL));
    assert(!telemetry_bits_read(&r, 0U, &out));
    assert(out == 42U);
}

int main(void)
{
    test_hump_candidate_layout();
    test_reject_out_of_range_without_partial_write();
    test_reject_capacity_overflow_and_truncation();
    test_all_32_bits_and_boundaries();
    test_version_c_hump_sentinel();
    test_version_c_malformed_frames();
    test_null_arguments_and_zero_width();
    puts("PASS: 7 host-side telemetry bit-packing/protocol-example tests");
    return 0;
}
