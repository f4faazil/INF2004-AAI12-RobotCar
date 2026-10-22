/*
 * test_imu.c
 * Host tests for accelerometer decoding, calibration, tilt and hump
 * detection.  The hump scenarios are SIMULATIONS: a car drives at constant
 * speed over a hump of known shape, the tilt is sampled every 20 ms with
 * noise and the distance is quantised to 1 mm like the encoder feedback.
 * They show the method recovers a known height; real accuracy has to be
 * measured on the known 20 mm and 40 mm humps.
 */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "imu.h"
#include "test_support.h"

#define PI_D        (3.14159265358979)
#define STEP_MS     (20U)

/* ---- helpers --------------------------------------------------------- */

static uint32_t g_rng = 12345U;

static double rand_pm1(void)
{
    g_rng = (g_rng * 1664525U) + 1013904223U;
    return ((double)(g_rng >> 8) / 8388608.0) - 1.0;
}

static void put16(uint8_t *dst, int16_t v)
{
    dst[0] = (uint8_t)((uint16_t)v & 0xFFU);
    dst[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFFU);
}

static imu_vec3_t vec(int16_t x, int16_t y, int16_t z)
{
    imu_vec3_t v = { x, y, z };
    return v;
}

/* Reading of a level sensor (X forward, Z up) pitched nose-up by 'deg'. */
static imu_vec3_t nose_up(double deg)
{
    double a = deg * PI_D / 180.0;

    return vec((int16_t)lround(-1000.0 * sin(a)), 0, (int16_t)lround(1000.0 * cos(a)));
}

static hump_config_t hump_config(void)
{
    hump_config_t c;

    c.up_threshold_permille = 50;
    c.down_threshold_permille = 50;
    c.flat_band_permille = 25;
    c.min_duration_ms = 60U;
    c.max_duration_ms = 5000U;
    c.min_height_mm = 4;
    c.net_elevation_percent = 40U;
    c.disturbance_mg = 300;
    return c;
}

/* ---- decoding -------------------------------------------------------- */

/* LSM303D: 0.061 mg per count at +/-2 g, so 16393 counts is 1 g.  Catches: a
 * wrong scale factor (every height would be wrong) or byte order. */
static void test_lsm303d_decode(void)
{
    uint8_t raw[6];
    imu_vec3_t g;

    put16(&raw[0], 0);
    put16(&raw[2], 8197);       /* 500 mg */
    put16(&raw[4], 16393);      /* 1000 mg */
    imu_decode_accel(IMU_VARIANT_LSM303D, raw, &g);
    CHECK_INT(0, g.x);
    CHECK(abs(g.y - 500) <= 1);
    CHECK(abs(g.z - 1000) <= 1);

    put16(&raw[4], -16393);
    imu_decode_accel(IMU_VARIANT_LSM303D, raw, &g);
    CHECK(abs(g.z + 1000) <= 1);
}

/* LSM303DLHC: 12-bit left-justified in 16, so 1 mg per count after >> 4.
 * Catches: forgetting the shift, or a logical (unsigned) shift of negatives. */
static void test_lsm303dlhc_decode(void)
{
    uint8_t raw[6];
    imu_vec3_t g;

    put16(&raw[0], (int16_t)(1000 * 16));
    put16(&raw[2], (int16_t)(-1000 * 16));
    put16(&raw[4], (int16_t)(250 * 16));
    imu_decode_accel(IMU_VARIANT_LSM303DLHC, raw, &g);
    CHECK_INT(1000, g.x);
    CHECK_INT(-1000, g.y);
    CHECK_INT(250, g.z);
}

/* ---- calibration and tilt ------------------------------------------- */

static void calibrate(imu_cal_t *cal, imu_vec3_t base, uint8_t axis, int8_t sign)
{
    int i;

    imu_cal_init(cal, axis, sign, 50U, 60);
    for (i = 0; i < 50; i++) {
        imu_vec3_t s = vec((int16_t)(base.x + (i % 3) - 1), (int16_t)(base.y - (i % 2)),
                           (int16_t)(base.z + (i % 5) - 2));
        (void)imu_cal_feed(cal, &s);
    }
}

/* A still sensor calibrates after the required samples.  Catches: calibration
 * never completing, or completing early. */
static void test_calibration_completes_when_still(void)
{
    imu_cal_t cal;
    imu_vec3_t s = vec(0, 0, 1000);
    int i;

    imu_cal_init(&cal, IMU_AXIS_X, 1, 50U, 60);
    for (i = 0; i < 49; i++) {
        CHECK_INT(IMU_CAL_COLLECTING, imu_cal_feed(&cal, &s));
    }
    CHECK_INT(IMU_CAL_DONE, imu_cal_feed(&cal, &s));
}

/* A moving car must not be accepted as the rest baseline.  Catches: baking
 * the vehicle's acceleration into the zero (every later pitch would be off). */
static void test_calibration_refuses_a_moving_sensor(void)
{
    imu_cal_t cal;
    int i;
    imu_cal_state_t st = IMU_CAL_COLLECTING;

    imu_cal_init(&cal, IMU_AXIS_X, 1, 50U, 60);
    for (i = 0; i < 50; i++) {
        imu_vec3_t s = vec((int16_t)((i % 2) * 400), 0, 1000);
        st = imu_cal_feed(&cal, &s);
    }
    CHECK_INT(IMU_CAL_FAILED_MOVING, st);
    CHECK_INT(0, imu_pitch_permille(&cal, &(imu_vec3_t){ -500, 0, 866 }));
}

/* sin(pitch) for known tilts, nose up positive.  Catches: wrong sign, wrong
 * geometry, or a degrees/radians mix-up. */
static void test_pitch_for_known_tilts(void)
{
    imu_cal_t cal;
    imu_vec3_t t;

    calibrate(&cal, vec(0, 0, 1000), IMU_AXIS_X, 1);
    CHECK_INT(IMU_CAL_DONE, cal.state);
    CHECK(abs(imu_pitch_permille(&cal, &(imu_vec3_t){ 0, 0, 1000 })) <= 2);

    t = nose_up(5.0);
    CHECK(abs(imu_pitch_permille(&cal, &t) - 87) <= 3);
    t = nose_up(10.0);
    CHECK(abs(imu_pitch_permille(&cal, &t) - 174) <= 3);
    t = nose_up(20.0);
    CHECK(abs(imu_pitch_permille(&cal, &t) - 342) <= 3);
    t = nose_up(-8.0);
    CHECK(abs(imu_pitch_permille(&cal, &t) + 139) <= 3);
}

/* The bench found the sensor reading Z = -0.83 g flat (mounted face down or
 * axis reversed) plus an offset.  The tilt must still come out right.
 * Catches: assuming Z is up. */
static void test_pitch_with_an_upside_down_sensor(void)
{
    imu_cal_t cal;
    imu_vec3_t t;

    calibrate(&cal, vec(0, 0, -830), IMU_AXIS_X, 1);
    t = vec((int16_t)lround(-830.0 * sin(10.0 * PI_D / 180.0)), 0,
            (int16_t)lround(-830.0 * cos(10.0 * PI_D / 180.0)));
    CHECK(abs(imu_pitch_permille(&cal, &t) - 174) <= 4);
}

/* The forward axis and its sign are configuration: Y reversed here.
 * Catches: a hard-coded X axis. */
static void test_pitch_with_a_reversed_y_forward_axis(void)
{
    imu_cal_t cal;
    imu_vec3_t t;

    calibrate(&cal, vec(0, 0, 1000), IMU_AXIS_Y, -1);
    /* nose up 10 degrees: the reaction gravity moves toward -forward = +Y */
    t = vec(0, (int16_t)lround(1000.0 * sin(10.0 * PI_D / 180.0)),
            (int16_t)lround(1000.0 * cos(10.0 * PI_D / 180.0)));
    CHECK(abs(imu_pitch_permille(&cal, &t) - 174) <= 3);
}

/* A baseline offset (the bench saw about +0.15 g) cancels: rest reads zero
 * pitch and the magnitude baseline is the measured one. */
static void test_baseline_offset_cancels(void)
{
    imu_cal_t cal;

    calibrate(&cal, vec(30, -20, 1150), IMU_AXIS_X, 1);
    CHECK(abs(imu_pitch_permille(&cal, &(imu_vec3_t){ 30, -20, 1150 })) <= 3);
    CHECK(abs(imu_disturbance_mg(&cal, &(imu_vec3_t){ 30, -20, 1150 })) <= 3);
}

/* A hard knock shows as a disturbance. */
static void test_disturbance_reports_magnitude_change(void)
{
    imu_cal_t cal;
    imu_vec3_t knock = vec(600, 0, 1000);

    calibrate(&cal, vec(0, 0, 1000), IMU_AXIS_X, 1);
    CHECK(abs(imu_disturbance_mg(&cal, &knock) - 166) <= 4);
}

/* ---- packed event ---------------------------------------------------- */

/* Field layout: status[12:10] height[9:3] validity[2:1] fault[0].  Catches:
 * overlapping fields or a layout the laptop would decode differently. */
static void test_hump_event_pack_layout(void)
{
    hump_state_t st;
    uint8_t h;
    hump_validity_t v;
    bool f;

    CHECK_INT(2349, hump_event_pack(HUMP_ON_HUMP, 37U, HUMP_LOW_CONFIDENCE, true));
    CHECK_INT(0x1FFF, hump_event_pack((hump_state_t)7, 127U, (hump_validity_t)3, true));
    CHECK_INT(0, hump_event_pack(HUMP_FLAT, 0U, HUMP_VALID, false));

    hump_event_unpack(2349U, &st, &h, &v, &f);
    CHECK_INT(HUMP_ON_HUMP, st);
    CHECK_INT(37, h);
    CHECK_INT(HUMP_LOW_CONFIDENCE, v);
    CHECK(f);
}

/* Out-of-range input is masked, not allowed to corrupt a neighbouring field. */
static void test_hump_event_pack_masks_oversize_fields(void)
{
    uint16_t p = hump_event_pack(HUMP_FLAT, 200U, HUMP_VALID, false);

    CHECK((p >> 13) == 0U);                     /* never more than 13 bits */
}

/* ---- hump detection ------------------------------------------------- */

typedef struct {
    double lead_mm;
    double up_mm;           /* surface length of the rising ramp */
    double plateau_mm;
    double down_mm;
    double height_mm;
    double down_height_mm;  /* height descended (equals height_mm for a hump) */
    double tail_mm;
} shape_t;

typedef struct {
    int started;
    int finished;
    int saw_rising;
    int saw_on_hump;
    int saw_descending;
    int finished_validity;
    int finished_code;
} run_result_t;

/* Drive the shape at speed_mm_s.  flags: bit0 = distance invalid for a stretch
 * mid-hump, bit1 = IMU fault mid-hump, bit2 = a disturbance mid-hump. */
static run_result_t drive_shape(hump_detector_t *h, const shape_t *s,
                                double speed_mm_s, double noise_permille,
                                unsigned flags)
{
    run_result_t r = { 0, 0, 0, 0, 0, 0, 0 };
    double total = s->lead_mm + s->up_mm + s->plateau_mm + s->down_mm + s->tail_mm;
    double pos = 0.0;
    uint32_t now = 100000U;
    unsigned step = 0U;
    double t0 = s->lead_mm + (0.4 * s->up_mm);
    double t1 = s->lead_mm + (0.6 * s->up_mm);

    while (pos < total) {
        double slope = 0.0;
        hump_input_t in;
        hump_event_t ev;

        if ((pos >= s->lead_mm) && (pos < s->lead_mm + s->up_mm)) {
            slope = s->height_mm / s->up_mm;
        } else if ((pos >= s->lead_mm + s->up_mm + s->plateau_mm) &&
                   (pos < s->lead_mm + s->up_mm + s->plateau_mm + s->down_mm)) {
            slope = -s->down_height_mm / s->down_mm;
        }
        in.pitch_permille = (int16_t)lround((slope * 1000.0) + (rand_pm1() * noise_permille));
        in.disturbance_mg = 10;
        in.distance_mm = (int32_t)lround(pos);
        in.distance_valid = true;
        in.imu_ok = true;
        in.now_ms = now;
        if (((flags & 1U) != 0U) && (pos > t0) && (pos < t1)) {
            in.distance_valid = false;
        }
        if (((flags & 2U) != 0U) && (pos > t0) && (pos < t1)) {
            in.imu_ok = false;
        }
        if (((flags & 4U) != 0U) && (pos > t0) && (pos < t1)) {
            in.disturbance_mg = 500;
        }

        ev = hump_update(h, &in);
        if (ev == HUMP_EVENT_STARTED) { r.started++; }
        if (ev == HUMP_EVENT_FINISHED) {
            r.finished++;
            r.finished_validity = (int)hump_last_validity(h);
            r.finished_code = (int)hump_last_height_code(h);
        }
        if (h->state == HUMP_RISING) { r.saw_rising = 1; }
        if (h->state == HUMP_ON_HUMP) { r.saw_on_hump = 1; }
        if (h->state == HUMP_DESCENDING) { r.saw_descending = 1; }

        pos += speed_mm_s * ((double)STEP_MS / 1000.0);
        now += STEP_MS;
        step++;
    }
    (void)step;
    return r;
}

static shape_t simple_hump(double height_mm)
{
    shape_t s = { 200.0, 150.0, 0.0, 150.0, height_mm, height_mm, 250.0 };
    return s;
}

/* A 20 mm and a 40 mm hump are measured within the Tribunal's +/-10 mm target
 * at 0.10, 0.15 and 0.20 m/s, with sensor noise.  Catches: a wrong
 * accumulation (distance x sin(pitch)), speed dependence, or noise
 * sensitivity.  SIMULATION. */
static void test_known_humps_are_measured_within_target(void)
{
    static const double HEIGHTS[] = { 20.0, 40.0 };
    static const double SPEEDS[] = { 100.0, 150.0, 200.0 };
    size_t hi;
    size_t si;

    for (hi = 0U; hi < 2U; hi++) {
        for (si = 0U; si < 3U; si++) {
            hump_detector_t h;
            hump_config_t cfg = hump_config();
            shape_t s = simple_hump(HEIGHTS[hi]);
            run_result_t r;

            hump_init(&h, &cfg);
            r = drive_shape(&h, &s, SPEEDS[si], 10.0, 0U);
            CHECK_INT(1, r.started);
            CHECK_INT(1, r.finished);
            CHECK_INT(HUMP_VALID, r.finished_validity);
            CHECK(fabs((double)r.finished_code - HEIGHTS[hi]) <= 5.0);
            CHECK(fabs((double)hump_highest_valid_mm(&h) - HEIGHTS[hi]) <= 5.0);
        }
    }
}

/* The state machine passes through RISING, ON_HUMP (a plateau) and
 * DESCENDING and ends FLAT. */
static void test_plateau_hump_visits_every_state(void)
{
    hump_detector_t h;
    hump_config_t cfg = hump_config();
    shape_t s = { 200.0, 150.0, 120.0, 150.0, 30.0, 30.0, 250.0 };
    run_result_t r;

    hump_init(&h, &cfg);
    r = drive_shape(&h, &s, 120.0, 8.0, 0U);
    CHECK(r.saw_rising);
    CHECK(r.saw_on_hump);
    CHECK(r.saw_descending);
    CHECK_INT(HUMP_FLAT, h.state);
    CHECK(!hump_in_progress(&h));
    CHECK(fabs((double)r.finished_code - 30.0) <= 5.0);
}

/* A one-sample spike is a bump or noise, not a hump (the min-duration filter).
 * Catches: false hump events from vibration. */
static void test_single_sample_spike_is_not_a_hump(void)
{
    hump_detector_t h;
    hump_config_t cfg = hump_config();
    hump_input_t in = { 0, 10, 0, true, true, 1000U };
    int i;
    int events = 0;

    hump_init(&h, &cfg);
    for (i = 0; i < 40; i++) {
        in.pitch_permille = (i == 10) ? 400 : 0;
        in.distance_mm = i * 3;
        in.now_ms += STEP_MS;
        if (hump_update(&h, &in) != HUMP_EVENT_NONE) {
            events++;
        }
    }
    CHECK_INT(0, events);
    CHECK_INT(HUMP_FLAT, h.state);
}

/* No encoder distance: the height is UNAVAILABLE and the highest valid
 * result is untouched (Tribunal rule).  Catches: inventing a height from
 * missing data. */
static void test_missing_distance_makes_height_unavailable(void)
{
    hump_detector_t h;
    hump_config_t cfg = hump_config();
    shape_t good = simple_hump(20.0);
    shape_t big = simple_hump(60.0);
    run_result_t r;

    int32_t before;

    hump_init(&h, &cfg);
    (void)drive_shape(&h, &good, 150.0, 5.0, 0U);
    before = hump_highest_valid_mm(&h);
    CHECK(abs((int)before - 20) <= 5);

    r = drive_shape(&h, &big, 150.0, 5.0, 1U);
    CHECK_INT(HUMP_UNAVAILABLE, r.finished_validity);
    CHECK_INT(before, hump_highest_valid_mm(&h)); /* not overwritten by ~60 */
}

/* An IMU read failure mid-hump: UNAVAILABLE and the fault bit is packed. */
static void test_imu_fault_is_reported_and_packed(void)
{
    hump_detector_t h;
    hump_config_t cfg = hump_config();
    shape_t s = simple_hump(30.0);
    run_result_t r;
    hump_state_t st;
    uint8_t code;
    hump_validity_t v;
    bool fault;

    hump_init(&h, &cfg);
    r = drive_shape(&h, &s, 150.0, 5.0, 2U);
    CHECK_INT(HUMP_UNAVAILABLE, r.finished_validity);
    hump_event_unpack(h.last_event_packed, &st, &code, &v, &fault);
    CHECK(fault);
    CHECK_INT(HUMP_UNAVAILABLE, v);
}

/* An acceleration disturbance (wheel slip, hard braking) lowers confidence,
 * and a low-confidence height never becomes the highest valid one. */
static void test_disturbance_gives_low_confidence(void)
{
    hump_detector_t h;
    hump_config_t cfg = hump_config();
    shape_t s = simple_hump(30.0);
    run_result_t r;

    hump_init(&h, &cfg);
    r = drive_shape(&h, &s, 150.0, 5.0, 4U);
    CHECK_INT(HUMP_LOW_CONFIDENCE, r.finished_validity);
    CHECK_INT(-1, hump_highest_valid_mm(&h));
}

/* The highest valid hump is retained across several. */
static void test_highest_valid_is_retained(void)
{
    hump_detector_t h;
    hump_config_t cfg = hump_config();
    shape_t a = simple_hump(20.0);
    shape_t b = simple_hump(40.0);

    hump_init(&h, &cfg);
    (void)drive_shape(&h, &a, 150.0, 5.0, 0U);
    CHECK(abs((int)hump_highest_valid_mm(&h) - 20) <= 5);
    (void)drive_shape(&h, &b, 150.0, 5.0, 0U);
    CHECK(abs((int)hump_highest_valid_mm(&h) - 40) <= 5);
    (void)drive_shape(&h, &a, 150.0, 5.0, 0U);
    CHECK(abs((int)hump_highest_valid_mm(&h) - 40) <= 5);
}

/* A ramp that goes up but does not come back down (a step or a slope) is not
 * a clean hump: low confidence.  Catches: reporting a step as a hump height. */
static void test_unreturned_elevation_is_low_confidence(void)
{
    hump_detector_t h;
    hump_config_t cfg = hump_config();
    shape_t s = { 200.0, 150.0, 0.0, 150.0, 40.0, 14.0, 250.0 };   /* up 40, down 14 */
    run_result_t r;

    hump_init(&h, &cfg);
    r = drive_shape(&h, &s, 150.0, 5.0, 0U);
    CHECK_INT(1, r.finished);
    CHECK_INT(HUMP_LOW_CONFIDENCE, r.finished_validity);
    CHECK_INT(-1, hump_highest_valid_mm(&h));
}

/* Heights beyond the 7-bit field report the reserved out-of-range code 127
 * instead of a wrong number (Tribunal: saturate and flag). */
static void test_out_of_range_height_uses_code_127(void)
{
    hump_detector_t h;
    hump_config_t cfg = hump_config();
    shape_t s = { 200.0, 200.0, 0.0, 200.0, 140.0, 140.0, 250.0 };
    run_result_t r;

    hump_init(&h, &cfg);
    r = drive_shape(&h, &s, 150.0, 5.0, 0U);
    CHECK_INT(HUMP_HEIGHT_OUT_OF_RANGE, r.finished_code);
    CHECK_INT(-1, hump_highest_valid_mm(&h));
}

/* A very small rise (2 mm) is below the detection slope and is never a
 * valid height.  Catches: surface texture being reported as a hump. */
static void test_tiny_rise_never_becomes_a_valid_height(void)
{
    hump_detector_t h;
    hump_config_t cfg = hump_config();
    shape_t s = { 200.0, 60.0, 0.0, 60.0, 2.0, 2.0, 250.0 };
    run_result_t r;

    hump_init(&h, &cfg);
    r = drive_shape(&h, &s, 100.0, 4.0, 0U);
    CHECK_INT(-1, hump_highest_valid_mm(&h));
    (void)r;
}

/* ---- motion classification ------------------------------------------ */

static void test_motion_classification(void)
{
    imu_motion_input_t in = { 0, 0, 10, 0, HUMP_FLAT };

    CHECK_INT(IMU_MOTION_STATIONARY, imu_classify_motion(&in));

    in.speed_mm_s = 150;
    CHECK_INT(IMU_MOTION_CRUISING, imu_classify_motion(&in));

    in.accel_mm_s2 = 400;
    CHECK_INT(IMU_MOTION_ACCELERATING, imu_classify_motion(&in));
    in.accel_mm_s2 = 0;

    in.turn_rate_dps = 90;
    CHECK_INT(IMU_MOTION_TURNING, imu_classify_motion(&in));
    in.turn_rate_dps = 0;

    in.hump = HUMP_RISING;
    CHECK_INT(IMU_MOTION_CLIMBING, imu_classify_motion(&in));
    in.hump = HUMP_DESCENDING;
    CHECK_INT(IMU_MOTION_DESCENDING, imu_classify_motion(&in));
    in.hump = HUMP_FLAT;

    in.disturbance_mg = 900;
    CHECK_INT(IMU_MOTION_IMPACT, imu_classify_motion(&in));    /* beats the rest */
}

int main(void)
{
    RUN(test_lsm303d_decode);
    RUN(test_lsm303dlhc_decode);
    RUN(test_calibration_completes_when_still);
    RUN(test_calibration_refuses_a_moving_sensor);
    RUN(test_pitch_for_known_tilts);
    RUN(test_pitch_with_an_upside_down_sensor);
    RUN(test_pitch_with_a_reversed_y_forward_axis);
    RUN(test_baseline_offset_cancels);
    RUN(test_disturbance_reports_magnitude_change);
    RUN(test_hump_event_pack_layout);
    RUN(test_hump_event_pack_masks_oversize_fields);
    RUN(test_known_humps_are_measured_within_target);
    RUN(test_plateau_hump_visits_every_state);
    RUN(test_single_sample_spike_is_not_a_hump);
    RUN(test_missing_distance_makes_height_unavailable);
    RUN(test_imu_fault_is_reported_and_packed);
    RUN(test_disturbance_gives_low_confidence);
    RUN(test_highest_valid_is_retained);
    RUN(test_unreturned_elevation_is_low_confidence);
    RUN(test_out_of_range_height_uses_code_127);
    RUN(test_tiny_rise_never_becomes_a_valid_height);
    RUN(test_motion_classification);
    return TEST_REPORT();
}
