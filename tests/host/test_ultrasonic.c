/*
 * test_ultrasonic.c
 * Host tests for echo conversion, the median/monitor logic and the servo
 * angle mapping.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "servo.h"
#include "test_support.h"
#include "ultrasonic.h"

static us_reading_t reading(us_status_t st, uint16_t mm, uint32_t seq)
{
    us_reading_t r;

    r.status = st;
    r.distance_mm = mm;
    r.timestamp_ms = seq * 60U;
    r.seq = seq;
    return r;
}

/* ---- conversion ------------------------------------------------------ */

/* 343 m/s round trip: 5831 us is one metre.  Catches: forgetting to halve the
 * round trip, or a wrong speed of sound. */
static void test_echo_converts_to_distance(void)
{
    uint16_t mm = 0U;

    CHECK_INT(US_VALID, us_convert(5831U, true, &mm));
    CHECK(mm >= 999U && mm <= 1001U);

    CHECK_INT(US_VALID, us_convert(1000U, true, &mm));
    CHECK_INT(171, mm);                         /* 1000 us * 0.1715 mm/us */
}

/* A timeout is NO_ECHO, not a big distance.  Catches: reading a missing echo
 * as free space (Tribunal rule). */
static void test_timeout_is_no_echo_not_free_space(void)
{
    uint16_t mm = 777U;

    CHECK_INT(US_NO_ECHO, us_convert(38000U, false, &mm));
    CHECK_INT(777, mm);                         /* output untouched */
}

/* Inside the blind zone (< 2 cm) the reading is not trusted; beyond 4 m it is
 * out of range. */
static void test_blind_zone_and_range_limits(void)
{
    uint16_t mm = 0U;

    CHECK_INT(US_TOO_CLOSE, us_convert(100U, true, &mm));       /* ~17 mm */
    CHECK_INT(US_VALID, us_convert(120U, true, &mm));           /* ~20 mm: edge */
    CHECK_INT(US_OUT_OF_RANGE, us_convert(24500U, true, &mm));  /* ~4.2 m */
}

/* ---- monitor --------------------------------------------------------- */

/* Median of three rejects a single wild reading.  Catches: one echo off a
 * stray object clearing or setting the obstacle flag. */
static void test_median_rejects_a_single_outlier(void)
{
    us_monitor_t m;
    uint16_t d = 0U;
    uint32_t i = 0U;

    us_monitor_init(&m, 1U, 5U);
    (void)us_monitor_update(&m, &(us_reading_t){ US_VALID, 300U, 0U, ++i }, 200U, 20U, 0U);
    (void)us_monitor_update(&m, &(us_reading_t){ US_VALID, 310U, 0U, ++i }, 200U, 20U, 0U);
    (void)us_monitor_update(&m, &(us_reading_t){ US_VALID, 90U, 0U, ++i }, 200U, 20U, 0U);
    CHECK(us_monitor_distance(&m, &d));
    CHECK_INT(300, d);
}

/* An obstacle is declared only after the confirmation count of consecutive
 * readings below the threshold, and cleared with hysteresis.  Catches:
 * chatter around the threshold and one-shot false stops. */
static void test_obstacle_needs_confirmation_and_has_hysteresis(void)
{
    us_monitor_t m;
    uint32_t i = 0U;
    bool obs;

    us_monitor_init(&m, 2U, 5U);
    obs = us_monitor_update(&m, &(us_reading_t){ US_VALID, 150U, 0U, ++i }, 200U, 30U, 0U);
    CHECK(!obs);                                /* first below: not yet */
    obs = us_monitor_update(&m, &(us_reading_t){ US_VALID, 150U, 0U, ++i }, 200U, 30U, 0U);
    CHECK(obs);                                 /* second: confirmed */

    obs = us_monitor_update(&m, &(us_reading_t){ US_VALID, 215U, 0U, ++i }, 200U, 30U, 0U);
    obs = us_monitor_update(&m, &(us_reading_t){ US_VALID, 215U, 0U, ++i }, 200U, 30U, 0U);
    obs = us_monitor_update(&m, &(us_reading_t){ US_VALID, 215U, 0U, ++i }, 200U, 30U, 0U);
    CHECK(obs);                                 /* 215 < 200 + 30: still held */

    obs = us_monitor_update(&m, &(us_reading_t){ US_VALID, 400U, 0U, ++i }, 200U, 30U, 0U);
    obs = us_monitor_update(&m, &(us_reading_t){ US_VALID, 400U, 0U, ++i }, 200U, 30U, 0U);
    obs = us_monitor_update(&m, &(us_reading_t){ US_VALID, 400U, 0U, ++i }, 200U, 30U, 0U);
    CHECK(!obs);                                /* well clear: released */
}

/* A single valid reading closer than the urgent distance declares the obstacle
 * at once - no median, no confirmation - so the car does not spend two 70 ms
 * measurement cycles deciding about something very near.  Catches: the
 * confirmation delay eating the 150 ms obstacle-to-stop budget (Tribunal). */
static void test_a_very_close_reading_declares_an_obstacle_immediately(void)
{
    us_monitor_t m;
    uint32_t i = 0U;
    bool obs;

    us_monitor_init(&m, 3U, 5U);
    obs = us_monitor_update(&m, &(us_reading_t){ US_VALID, 300U, 0U, ++i }, 200U, 20U, 80U);
    obs = us_monitor_update(&m, &(us_reading_t){ US_VALID, 300U, 0U, ++i }, 200U, 20U, 80U);
    CHECK(!obs);
    obs = us_monitor_update(&m, &(us_reading_t){ US_VALID, 60U, 0U, ++i }, 200U, 20U, 80U);
    CHECK(obs);                                 /* median is still 300, but 60 < 80 */
}

/* Between the urgent and the normal distance the usual confirmation applies,
 * and an invalid reading is never urgent. */
static void test_urgent_path_ignores_readings_that_are_not_urgent_or_not_valid(void)
{
    us_monitor_t m;
    uint32_t i = 0U;
    bool obs;

    us_monitor_init(&m, 2U, 5U);
    obs = us_monitor_update(&m, &(us_reading_t){ US_VALID, 150U, 0U, ++i }, 200U, 20U, 80U);
    CHECK(!obs);                                /* below threshold, above urgent: wait */
    us_monitor_init(&m, 2U, 5U);
    obs = us_monitor_update(&m, &(us_reading_t){ US_NO_ECHO, 0U, 0U, ++i }, 200U, 20U, 80U);
    obs = us_monitor_update(&m, &(us_reading_t){ US_TOO_CLOSE, 0U, 0U, ++i }, 200U, 20U, 80U);
    CHECK(!obs);                                /* no distance, so no urgent decision */
}

/* Invalid readings neither declare nor clear an obstacle; a held obstacle
 * stays held through a no-echo.  Catches: no echo clearing the stop. */
static void test_invalid_reading_does_not_clear_an_obstacle(void)
{
    us_monitor_t m;
    uint32_t i = 0U;
    bool obs = false;
    int k;

    us_monitor_init(&m, 1U, 20U);
    obs = us_monitor_update(&m, &(us_reading_t){ US_VALID, 100U, 0U, ++i }, 200U, 20U, 0U);
    CHECK(obs);
    for (k = 0; k < 6; k++) {
        obs = us_monitor_update(&m, &(us_reading_t){ US_NO_ECHO, 0U, 0U, ++i }, 200U, 20U, 0U);
    }
    CHECK(obs);
}

/* A run of invalid readings is a sensor fault; one valid reading resets it. */
static void test_persistent_invalid_readings_are_a_sensor_fault(void)
{
    us_monitor_t m;
    uint32_t i = 0U;
    int k;

    us_monitor_init(&m, 1U, 5U);
    for (k = 0; k < 4; k++) {
        (void)us_monitor_update(&m, &(us_reading_t){ US_NO_ECHO, 0U, 0U, ++i }, 200U, 20U, 0U);
    }
    CHECK(!us_monitor_sensor_fault(&m));
    (void)us_monitor_update(&m, &(us_reading_t){ US_NO_ECHO, 0U, 0U, ++i }, 200U, 20U, 0U);
    CHECK(us_monitor_sensor_fault(&m));

    (void)us_monitor_update(&m, &(us_reading_t){ US_VALID, 500U, 0U, ++i }, 200U, 20U, 0U);
    CHECK(!us_monitor_sensor_fault(&m));
}

/* With no valid reading yet, no distance is offered. */
static void test_no_distance_before_a_valid_reading(void)
{
    us_monitor_t m;
    uint16_t d = 0U;

    us_monitor_init(&m, 1U, 5U);
    CHECK(!us_monitor_distance(&m, &d));
}

/* The same reading seen twice (same seq) is one measurement.  Catches: a
 * polling task re-feeding the old value and satisfying the confirmation count
 * with a single measurement. */
static void test_a_repeated_sequence_number_is_ignored(void)
{
    us_monitor_t m;
    bool obs;
    us_reading_t r = reading(US_VALID, 100U, 1U);

    us_monitor_init(&m, 2U, 5U);
    obs = us_monitor_update(&m, &r, 200U, 20U, 0U);
    obs = us_monitor_update(&m, &r, 200U, 20U, 0U);
    CHECK(!obs);
    r.seq = 2U;
    obs = us_monitor_update(&m, &r, 200U, 20U, 0U);
    CHECK(obs);
}

/* Faster means a longer detection range; units are consistent. */
static void test_stop_threshold_grows_with_speed(void)
{
    /* 200 mm/s x 100 ms = 20 mm, + 40 braking + 60 margin = 120 */
    CHECK_INT(120, us_stop_threshold_mm(200, 100U, 40U, 60U));
    CHECK_INT(100, us_stop_threshold_mm(0, 100U, 40U, 60U));
    CHECK_INT(120, us_stop_threshold_mm(-200, 100U, 40U, 60U));   /* sign ignored */
    CHECK(us_stop_threshold_mm(300, 150U, 40U, 60U) >
          us_stop_threshold_mm(100, 150U, 40U, 60U));
    CHECK(us_stop_threshold_mm(32000, 5000U, 4000U, 4000U) <= US_MAX_RANGE_MM);
}

/* ---- servo ----------------------------------------------------------- */

static servo_cfg_t servo_defaults(void)
{
    servo_cfg_t c;

    c.min_pulse_us = 500U;
    c.max_pulse_us = 2500U;
    c.trim_deg = 0;
    c.invert = false;
    c.min_angle_deg = 20U;
    c.max_angle_deg = 160U;
    c.settle_base_ms = 60U;
    c.settle_ms_per_deg = 4U;
    return c;
}

/* 90 degrees is mid-pulse; the sweep is linear. */
static void test_servo_pulse_mapping(void)
{
    servo_cfg_t c = servo_defaults();

    CHECK_INT(1500, servo_pulse_us(&c, 90));
    CHECK_INT(500 + (30 * 2000) / 180, servo_pulse_us(&c, 30));
    CHECK(servo_pulse_us(&c, 150) > servo_pulse_us(&c, 120));
}

/* Out-of-range requests are clamped to the safe sweep. */
static void test_servo_angle_is_clamped(void)
{
    servo_cfg_t c = servo_defaults();

    CHECK_INT(20, servo_clamp_angle(&c, -50));
    CHECK_INT(160, servo_clamp_angle(&c, 400));
    CHECK_INT(servo_pulse_us(&c, 160), servo_pulse_us(&c, 179));
}

/* Trim shifts every position (horn alignment); invert mirrors the sweep.
 * Catches: the misaligned horn the bench found being unfixable in code. */
static void test_servo_trim_and_invert(void)
{
    servo_cfg_t c = servo_defaults();
    uint16_t straight = servo_pulse_us(&c, 90);

    c.trim_deg = 9;
    CHECK(servo_pulse_us(&c, 90) > straight);
    CHECK_INT(1500 + (9 * 2000) / 180, servo_pulse_us(&c, 90));

    c.trim_deg = 0;
    c.invert = true;
    CHECK_INT(straight, servo_pulse_us(&c, 90));
    CHECK(servo_pulse_us(&c, 30) > servo_pulse_us(&c, 150));
}

/* A bigger move needs longer to settle; no move still waits the base time
 * (the measurement must not start the instant the pulse changes). */
static void test_servo_settle_time(void)
{
    servo_cfg_t c = servo_defaults();

    CHECK_INT(60, servo_settle_ms(&c, 90, 90));
    CHECK_INT(60 + (4 * 60), servo_settle_ms(&c, 90, 150));
    CHECK_INT(servo_settle_ms(&c, 150, 90), servo_settle_ms(&c, 90, 150));
}

int main(void)
{
    RUN(test_echo_converts_to_distance);
    RUN(test_timeout_is_no_echo_not_free_space);
    RUN(test_blind_zone_and_range_limits);
    RUN(test_median_rejects_a_single_outlier);
    RUN(test_obstacle_needs_confirmation_and_has_hysteresis);
    RUN(test_a_very_close_reading_declares_an_obstacle_immediately);
    RUN(test_urgent_path_ignores_readings_that_are_not_urgent_or_not_valid);
    RUN(test_invalid_reading_does_not_clear_an_obstacle);
    RUN(test_persistent_invalid_readings_are_a_sensor_fault);
    RUN(test_no_distance_before_a_valid_reading);
    RUN(test_a_repeated_sequence_number_is_ignored);
    RUN(test_stop_threshold_grows_with_speed);
    RUN(test_servo_pulse_mapping);
    RUN(test_servo_angle_is_clamped);
    RUN(test_servo_trim_and_invert);
    RUN(test_servo_settle_time);
    return TEST_REPORT();
}
