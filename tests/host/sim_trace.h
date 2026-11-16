/*
 * sim_trace.h
 * Synthetic ADC traces for a barcode passing under one IR sensor, and a
 * helper that runs a trace through the real run extractor + decoder.
 *
 * THIS IS SIMULATION, NOT CAR EVIDENCE.  It models: the printed symbol's
 * bar/space widths (measured from the lecturer's PDFs), a total symbol length
 * of 141 mm (the spec's "approx length of barcode", assumed to be the whole
 * "*X*" symbol), a box-shaped optical spot, the bench black/white ADC levels,
 * and uniform ADC noise.  It does not model height changes, lighting, wheel
 * slip or the sensor's real response curve.
 */
#ifndef SIM_TRACE_H
#define SIM_TRACE_H

#include <stddef.h>
#include <stdint.h>

#include "barcode.h"
#include "barcode_fixtures.h"
#include "line_sensor.h"

#define SIM_SYMBOL_MM       (141.0)
#define SIM_LEAD_MM         (30.0)    /* white before and after the symbol */
#define SIM_ADC_WHITE       (150.0)   /* bench readings, centre sensor */
#define SIM_ADC_BLACK       (4000.0)

typedef struct {
    double spot_mm;         /* optical spot diameter (TCRT5000: 2.1 mm) */
    double noise_counts;    /* +/- uniform ADC noise */
} sim_optics_t;

typedef enum { SIM_MISSED = 0, SIM_CORRECT, SIM_WRONG } sim_outcome_t;

/* Small deterministic generator (LCG) so every run is repeatable. */
static double sim_rand01(uint32_t *state)
{
    *state = (*state * 1664525U) + 1013904223U;
    return (double)(*state >> 8) / 16777216.0;
}

/* Fraction of the spot window [c - spot/2, c + spot/2] that lies on a bar. */
static double sim_black_fraction(const double *edge_mm, double centre_mm,
                                 double spot_mm)
{
    double lo = centre_mm - (spot_mm / 2.0);
    double hi = centre_mm + (spot_mm / 2.0);
    double black = 0.0;
    size_t i;

    for (i = 0U; i < BARCODE_ELEMENT_COUNT; i += 2U) {      /* even = bar */
        double a = (edge_mm[i] > lo) ? edge_mm[i] : lo;
        double b = (edge_mm[i + 1U] < hi) ? edge_mm[i + 1U] : hi;

        if (b > a) {
            black += (b - a);
        }
    }
    return black / spot_mm;
}

/*
 * One pass of fixture 'f' under the sensor at speed_mm_s, sampled every
 * period_us.  jitter stretches each individual period by a random factor in
 * [1, 1 + jitter] (0 = a perfectly steady sampler; 1.0 = each wake-up lands
 * 10-20 ms after the previous one for a 10 ms request, as a timed wait that
 * lasts "timeout + one tick" would).  The extractor still counts samples, so
 * jitter distorts the measured widths.  Returns whether the decoder produced exactly the right letter,
 * nothing, or anything else (a wrong or repeated decode).
 */
static sim_outcome_t sim_run_trial(size_t f, double speed_mm_s, double period_us,
                                   double jitter,
                                   const sim_optics_t *optics, uint32_t *rng)
{
    double edge_mm[BARCODE_ELEMENT_COUNT + 1U];
    double total_px = 0.0;
    double acc_px = 0.0;
    size_t i;
    double step_mm = speed_mm_s * period_us / 1.0e6;
    double x_mm;
    double end_mm = SIM_LEAD_MM + SIM_SYMBOL_MM + SIM_LEAD_MM;
    line_cal_t cal;
    run_extractor_t ex;
    barcode_decoder_t dec;
    int decodes = 0;
    char decoded = '\0';
    const uint16_t both[LINE_CHANNEL_COUNT] = { 0U, 0U, 0U };
    uint16_t lo_frame[LINE_CHANNEL_COUNT];
    uint16_t hi_frame[LINE_CHANNEL_COUNT];

    for (i = 0U; i < BARCODE_ELEMENT_COUNT; i++) {
        total_px += BARCODE_FIXTURES[f].runs[i];
    }
    edge_mm[0] = SIM_LEAD_MM;
    for (i = 0U; i < BARCODE_ELEMENT_COUNT; i++) {
        acc_px += BARCODE_FIXTURES[f].runs[i];
        edge_mm[i + 1U] = SIM_LEAD_MM + (acc_px / total_px) * SIM_SYMBOL_MM;
    }

    (void)both;
    for (i = 0U; i < LINE_CHANNEL_COUNT; i++) {
        lo_frame[i] = (uint16_t)SIM_ADC_WHITE;
        hi_frame[i] = (uint16_t)SIM_ADC_BLACK;
    }
    line_cal_init(&cal);
    line_cal_feed(&cal, lo_frame);
    line_cal_feed(&cal, hi_frame);
    run_extractor_init(&ex);
    barcode_init(&dec);

    /* random starting phase within one sample step */
    x_mm = -sim_rand01(rng) * step_mm;
    for (; x_mm < end_mm; x_mm += step_mm * (1.0 + (jitter * sim_rand01(rng)))) {
        double frac = sim_black_fraction(edge_mm, x_mm, optics->spot_mm);
        double adc = SIM_ADC_WHITE + (frac * (SIM_ADC_BLACK - SIM_ADC_WHITE))
                   + ((sim_rand01(rng) * 2.0 - 1.0) * optics->noise_counts);
        uint16_t counts;
        uint16_t blackness;
        bool is_bar = false;
        uint16_t ticks = 0U;

        if (adc < 0.0) {
            adc = 0.0;
        }
        if (adc > 4095.0) {
            adc = 4095.0;
        }
        counts = (uint16_t)adc;
        blackness = line_blackness(&cal, LINE_CHANNEL_CENTRE, counts);

        if (run_extractor_push(&ex, blackness, &is_bar, &ticks)) {
            char c = '\0';

            if (barcode_push_run(&dec, is_bar, ticks, &c) == BARCODE_DECODED) {
                decodes++;
                decoded = c;
            }
        }
    }

    if (decodes == 0) {
        return SIM_MISSED;
    }
    if ((decodes == 1) && (decoded == BARCODE_FIXTURES[f].expected)) {
        return SIM_CORRECT;
    }
    return SIM_WRONG;
}

#endif /* SIM_TRACE_H */
