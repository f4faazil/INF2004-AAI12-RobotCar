/*
 * test_pipeline.c
 * End-to-end SIMULATION of a barcode pass: synthetic ADC trace -> run
 * extractor -> Code 39 decoder.  Not car evidence (see sim_trace.h).
 */
#include <stddef.h>
#include <stdint.h>

#include "sim_trace.h"
#include "test_support.h"

#define TRIALS          (200U)
#define SPEED_MIN_MM_S  (100.0)     /* course speed range 0.10 - 0.20 m/s */
#define SPEED_MAX_MM_S  (200.0)

typedef struct {
    unsigned correct;
    unsigned missed;
    unsigned wrong;
} tally_t;

/* Run TRIALS passes cycling through the A/F/Z fixtures, speeds spread over the
 * course range, at the given sample period. */
static tally_t run_trials(double period_us)
{
    static const size_t LARGE_FIXTURES[] = { 0U, 3U, 6U };   /* A, F, Z (large) */
    const sim_optics_t optics = { 2.1, 60.0 };
    uint32_t rng = 20261022U;
    tally_t t = { 0U, 0U, 0U };
    unsigned n;

    for (n = 0U; n < TRIALS; n++) {
        double speed = SPEED_MIN_MM_S +
                       (sim_rand01(&rng) * (SPEED_MAX_MM_S - SPEED_MIN_MM_S));

        switch (sim_run_trial(LARGE_FIXTURES[n % 3U], speed, period_us, 0.0, &optics, &rng)) {
        case SIM_CORRECT: t.correct++; break;
        case SIM_MISSED:  t.missed++;  break;
        default:          t.wrong++;   break;
        }
    }
    return t;
}

/* The team's target: at least 95 correct per 100 and zero incorrect, across
 * 0.10-0.20 m/s.  With 1 ms frames the simulated chain meets it.  Catches: a
 * broken threshold, extractor or decoder that only works on clean idealised
 * runs (blurred edges, noise, random phase, speed spread). */
static void test_one_ms_sampling_meets_the_decode_target_in_simulation(void)
{
    tally_t t = run_trials(1000.0);

    CHECK_INT(0, t.wrong);
    CHECK(t.correct >= (TRIALS * 95U) / 100U);
}

/* The same at 2 ms - a cheaper option if 1 ms proves costly on the Pico. */
static void test_two_ms_sampling_meets_the_decode_target_in_simulation(void)
{
    tally_t t = run_trials(2000.0);

    CHECK_INT(0, t.wrong);
    CHECK(t.correct >= (TRIALS * 95U) / 100U);
}

int main(void)
{
    RUN(test_one_ms_sampling_meets_the_decode_target_in_simulation);
    RUN(test_two_ms_sampling_meets_the_decode_target_in_simulation);
    return TEST_REPORT();
}
