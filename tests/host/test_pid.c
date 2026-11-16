/*
 * test_pid.c
 * Host tests for the integer PID.  Gains are in 1/256 units, so kp = 256 is
 * 1.0.  Expected outputs are worked out by hand in the comments.
 */
#include <stdint.h>

#include "pid.h"
#include "test_support.h"

/* kp = 2.0, error 50 -> output 100.  Catches: wrong gain scaling or sign. */
static void test_proportional_term(void)
{
    pid_ctrl_t p;

    pid_init(&p, 2 * PID_GAIN_ONE, 0, 0, -1000, 1000, 100000);
    CHECK_INT(100, pid_update(&p, 150, 100, 5U));
    CHECK_INT(-100, pid_update(&p, 50, 100, 5U));
}

/* Output saturates at the limits.  Catches: unbounded PWM request. */
static void test_output_is_clamped(void)
{
    pid_ctrl_t p;

    pid_init(&p, 4 * PID_GAIN_ONE, 0, 0, -1000, 1000, 100000);
    CHECK_INT(1000, pid_update(&p, 900, 0, 5U));
    CHECK_INT(-1000, pid_update(&p, -900, 0, 5U));
}

/* ki = 1.0 per second, error 100 held for 1000 ms: integral = 100 * 1000 /
 * 1000 = 100 -> output 100.  Catches: integral not accumulating, or using
 * the wrong time base. */
static void test_integral_accumulates_over_time(void)
{
    pid_ctrl_t p;
    int32_t out = 0;
    int i;

    pid_init(&p, 0, PID_GAIN_ONE, 0, -1000, 1000, 1000000);
    for (i = 0; i < 100; i++) {
        out = pid_update(&p, 100, 0, 10U);      /* 100 x 10 ms = 1 s */
    }
    CHECK_INT(100, out);
}

/* The integral clamp stops windup: after a long saturated stretch, the
 * integral term never exceeds the limit.  Catches: windup that makes the
 * wheel overshoot and keep driving after the setpoint is reached. */
static void test_integral_is_clamped(void)
{
    pid_ctrl_t p;
    int32_t out = 0;
    int i;

    pid_init(&p, 0, PID_GAIN_ONE, 0, -1000, 1000, 50000);   /* limit 50 */
    for (i = 0; i < 2000; i++) {
        out = pid_update(&p, 1000, 0, 10U);
    }
    CHECK_INT(50, out);
    /* error reverses: output must fall promptly, not unwind a huge integral */
    for (i = 0; i < 100; i++) {
        out = pid_update(&p, 0, 1000, 10U);
    }
    CHECK(out <= 0);
}

/* Derivative acts on the measurement: a rising measurement lowers the
 * output.  kd = 1.0, measurement rises 10 in 10 ms (1000 units/s) -> -1000
 * units/s * kd.  Catches: derivative on error (setpoint kick) or wrong sign. */
static void test_derivative_damps_rising_measurement(void)
{
    pid_ctrl_t p;
    int32_t out;

    pid_init(&p, 0, 0, PID_GAIN_ONE, -5000, 5000, 100000);
    (void)pid_update(&p, 100, 0, 10U);          /* primes previous measurement */
    out = pid_update(&p, 100, 10, 10U);
    CHECK_INT(-1000, out);
}

/* A setpoint step with the measurement unchanged produces no derivative
 * output.  Catches: derivative kick on setpoint changes. */
static void test_setpoint_step_causes_no_derivative_kick(void)
{
    pid_ctrl_t p;

    pid_init(&p, 0, 0, 4 * PID_GAIN_ONE, -5000, 5000, 100000);
    (void)pid_update(&p, 0, 50, 10U);
    CHECK_INT(0, pid_update(&p, 900, 50, 10U));
}

/* Reset forgets the integral and previous measurement.  Catches: a stopped
 * wheel restarting with the old integral (a lurch). */
static void test_reset_clears_history(void)
{
    pid_ctrl_t p;
    int i;

    pid_init(&p, 0, PID_GAIN_ONE, 0, -1000, 1000, 1000000);
    for (i = 0; i < 50; i++) {
        (void)pid_update(&p, 100, 0, 10U);
    }
    pid_reset(&p);
    CHECK_INT(0, p.integral);
    CHECK_INT(0, pid_update(&p, 0, 0, 10U));
}

/* dt of zero must not divide by zero; it returns the proportional output
 * only. */
static void test_zero_dt_is_safe(void)
{
    pid_ctrl_t p;

    pid_init(&p, PID_GAIN_ONE, PID_GAIN_ONE, PID_GAIN_ONE, -1000, 1000, 100000);
    CHECK_INT(10, pid_update(&p, 10, 0, 0U));
}

int main(void)
{
    RUN(test_proportional_term);
    RUN(test_output_is_clamped);
    RUN(test_integral_accumulates_over_time);
    RUN(test_integral_is_clamped);
    RUN(test_derivative_damps_rising_measurement);
    RUN(test_setpoint_step_causes_no_derivative_kick);
    RUN(test_reset_clears_history);
    RUN(test_zero_dt_is_safe);
    return TEST_REPORT();
}
