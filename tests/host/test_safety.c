/*
 * test_safety.c
 * Host tests for the safety monitor's fault rules.
 */
#include <stdint.h>

#include "safety.h"
#include "test_port.h"
#include "test_support.h"

static safety_limits_t limits(void)
{
    safety_limits_t l = { 50U, 200U, 500U };
    return l;
}

static safety_in_t healthy_following(void)
{
    safety_in_t s = { true, true, true, false, false, 5U, false, 20U, false, 70U, false, false };
    return s;
}

/* Everything alive and well raises nothing. */
static void test_healthy_car_raises_no_faults(void)
{
    safety_limits_t l = limits();
    safety_in_t s = healthy_following();

    CHECK_INT(0, safety_evaluate(&s, &l));
}

/* Stale line data while following requests a stop (Tribunal: stale steering
 * data means a sensor-data fault and STOP), but not while the car is idle.
 * Catches: steering on old data, and false alarms while parked. */
static void test_stale_line_data_matters_only_while_following(void)
{
    safety_limits_t l = limits();
    safety_in_t s = healthy_following();

    s.line_stale = true;
    CHECK_INT(FAULT_LINE_STALE, safety_evaluate(&s, &l));
    s.line_required = false;
    CHECK_INT(0, safety_evaluate(&s, &l));
}

/* A silent encoder only matters when driving. */
static void test_encoder_fault_matters_only_while_driving(void)
{
    safety_limits_t l = limits();
    safety_in_t s = healthy_following();

    s.encoder_fault = true;
    CHECK_INT(FAULT_ENCODER, safety_evaluate(&s, &l));
    s.driving = false;
    CHECK_INT(0, safety_evaluate(&s, &l));
}

/* A stalled motion task (no step for longer than the limit) is a fault while
 * driving; right at the limit it is not. */
static void test_motion_task_stall(void)
{
    safety_limits_t l = limits();
    safety_in_t s = healthy_following();

    s.motion_age_ms = 50U;
    CHECK_INT(0, safety_evaluate(&s, &l));
    s.motion_age_ms = 51U;
    CHECK_INT(FAULT_MOTION, safety_evaluate(&s, &l));
}

/* IMU trouble is reported but is not a stop-class fault (the IMU degrades to
 * "hump N/A").  Catches: an IMU failure stopping the car. */
static void test_imu_failure_is_reported_but_never_stops_the_car(void)
{
    safety_limits_t l = limits();
    safety_in_t s = healthy_following();
    fault_mask_t m;

    s.imu_read_failed = true;
    CHECK_INT(FAULT_IMU, safety_evaluate(&s, &l));
    s.imu_read_failed = false;
    s.imu_age_ms = 300U;
    CHECK_INT(FAULT_IMU, safety_evaluate(&s, &l));

    fault_mask_init(&m);
    fault_set(&m, FAULT_IMU);
    CHECK(!fault_stop_required(&m));
}

/* If the ultrasonic sensor cannot see while the car is moving, that is a
 * stop-class fault (it is the only obstacle protection). */
static void test_blind_obstacle_sensor_while_moving(void)
{
    safety_limits_t l = limits();
    safety_in_t s = healthy_following();

    s.ultrasonic_fault = true;
    CHECK_INT(FAULT_ULTRASONIC, safety_evaluate(&s, &l));
    s.ultrasonic_fault = false;
    s.ultrasonic_age_ms = 600U;
    CHECK_INT(FAULT_ULTRASONIC, safety_evaluate(&s, &l));
    s.obstacle_watch_required = false;
    CHECK_INT(0, safety_evaluate(&s, &l));
}

/* Overflow and rejected-barcode events are informational. */
static void test_one_shot_events_map_to_informational_faults(void)
{
    safety_limits_t l = limits();
    safety_in_t s = healthy_following();
    fault_mask_t m;

    s.adc_overflow = true;
    s.barcode_rejected = true;
    fault_mask_init(&m);
    fault_set(&m, safety_evaluate(&s, &l));
    CHECK_INT(FAULT_LINE_OVERFLOW | FAULT_BARCODE_REJECT, fault_read(&m));
    CHECK(!fault_stop_required(&m));
}

/* Several problems at once are all reported. */
static void test_multiple_faults_combine(void)
{
    safety_limits_t l = limits();
    safety_in_t s = healthy_following();

    s.line_stale = true;
    s.encoder_fault = true;
    s.ultrasonic_fault = true;
    CHECK_INT(FAULT_LINE_STALE | FAULT_ENCODER | FAULT_ULTRASONIC, safety_evaluate(&s, &l));
}

int main(void)
{
    RUN(test_healthy_car_raises_no_faults);
    RUN(test_stale_line_data_matters_only_while_following);
    RUN(test_encoder_fault_matters_only_while_driving);
    RUN(test_motion_task_stall);
    RUN(test_imu_failure_is_reported_but_never_stops_the_car);
    RUN(test_blind_obstacle_sensor_while_moving);
    RUN(test_one_shot_events_map_to_informational_faults);
    RUN(test_multiple_faults_combine);
    return TEST_REPORT();
}
