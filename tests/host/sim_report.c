/*
 * sim_report.c
 * Prints, per sample period and car speed, how often the simulated chain
 * (synthetic ADC trace -> run extractor -> Code 39 decoder) decodes the
 * lecturer's barcodes correctly, misses them, or decodes the wrong letter.
 * SIMULATION ONLY - see sim_trace.h for what is and is not modelled.
 */
#include <stdio.h>

#include "sim_trace.h"

#define TRIALS_PER_CELL (300U)

int main(void)
{
    /* steady samplers, then samplers whose every wake-up is late by 0-100% */
    static const double PERIODS_US[] = { 1000.0, 2000.0, 5000.0, 10000.0,
                                         15000.0, 20000.0, 10000.0, 2000.0 };
    static const double JITTER[]     = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0 };
    static const double SPEEDS_MM_S[] = { 100.0, 150.0, 200.0 };
    static const size_t FIXTURES[] = { 0U, 3U, 6U };    /* A, F, Z at large scale */
    const sim_optics_t optics = { 2.1, 60.0 };
    size_t p;
    size_t s;

    printf("Simulated barcode decode, %u passes per cell (3 mm narrow bar, 2.1 mm spot)\n",
           TRIALS_PER_CELL);
    printf("%-8s %-8s %-8s %-10s %-9s %-8s %-6s\n",
           "period", "jitter", "speed", "samples/", "correct", "missed", "wrong");
    printf("%-8s %-8s %-8s %-10s\n", "", "", "(mm/s)", "narrow bar");

    for (p = 0U; p < (sizeof(PERIODS_US) / sizeof(PERIODS_US[0])); p++) {
        for (s = 0U; s < (sizeof(SPEEDS_MM_S) / sizeof(SPEEDS_MM_S[0])); s++) {
            uint32_t rng = 7U + (uint32_t)(p * 31U) + (uint32_t)s;
            unsigned correct = 0U;
            unsigned missed = 0U;
            unsigned wrong = 0U;
            unsigned n;
            double per_bar = 3.0 / (SPEEDS_MM_S[s] * PERIODS_US[p] / 1.0e6);

            for (n = 0U; n < TRIALS_PER_CELL; n++) {
                switch (sim_run_trial(FIXTURES[n % 3U], SPEEDS_MM_S[s],
                                      PERIODS_US[p], JITTER[p], &optics, &rng)) {
                case SIM_CORRECT: correct++; break;
                case SIM_MISSED:  missed++;  break;
                default:          wrong++;   break;
                }
            }
            printf("%-4.0f ms   %-8s %-8.0f %-10.1f %5.1f%%    %5.1f%%   %5.1f%%\n",
                   PERIODS_US[p] / 1000.0,
                   (JITTER[p] > 0.0) ? "0-100%" : "none",
                   SPEEDS_MM_S[s], per_bar,
                   100.0 * correct / TRIALS_PER_CELL,
                   100.0 * missed / TRIALS_PER_CELL,
                   100.0 * wrong / TRIALS_PER_CELL);
        }
    }
    return 0;
}
