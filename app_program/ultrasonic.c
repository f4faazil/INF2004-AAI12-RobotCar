/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ultrasonic.c
 * HC-SR04 measurement handling.  See ultrasonic.h.
 */
#include "ultrasonic.h"

/* Round-trip echo time to one-way distance: 343 m/s = 0.343 mm/us, halved. */
#define SOUND_MM_PER_MS         (343U)
#define ROUND_TRIP_DIV_US       (2000U)
#define MS_PER_S                (1000)

us_status_t us_convert(uint32_t echo_us, bool echo_complete, uint16_t *distance_mm)
{
    uint32_t mm;

    if (!echo_complete) {
        return US_NO_ECHO;
    }
    mm = (echo_us * SOUND_MM_PER_MS) / ROUND_TRIP_DIV_US;
    if (mm < US_MIN_RANGE_MM) {
        return US_TOO_CLOSE;
    }
    if (mm > US_MAX_RANGE_MM) {
        return US_OUT_OF_RANGE;
    }
    *distance_mm = (uint16_t)mm;
    return US_VALID;
}

void us_monitor_init(us_monitor_t *m, uint8_t confirm_count, uint8_t invalid_limit)
{
    uint8_t i;

    for (i = 0U; i < US_MEDIAN_LEN; i++) {
        m->window[i] = 0U;
    }
    m->count = 0U;
    m->head = 0U;
    m->invalid_run = 0U;
    m->invalid_limit = invalid_limit;
    m->confirm_count = (confirm_count == 0U) ? 1U : confirm_count;
    m->below_run = 0U;
    m->above_run = 0U;
    m->obstacle = false;
    m->last_median_mm = 0U;
    m->have_median = false;
    m->last_seq = 0U;
    m->have_seq = false;
}

static uint16_t median_of_window(const us_monitor_t *m)
{
    uint16_t v[US_MEDIAN_LEN];
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < m->count; i++) {
        v[i] = m->window[i];
    }
    for (i = 1U; i < m->count; i++) {           /* insertion sort, n <= 3 */
        uint16_t key = v[i];

        for (j = i; (j > 0U) && (v[j - 1U] > key); j--) {
            v[j] = v[j - 1U];
        }
        v[j] = key;
    }
    return v[m->count / 2U];
}

bool us_monitor_update(us_monitor_t *m, const us_reading_t *reading,
                       uint16_t threshold_mm, uint16_t hysteresis_mm,
                       uint16_t urgent_mm)
{
    uint32_t release_mm;
    uint16_t median;

    if (m->have_seq && (reading->seq == m->last_seq)) {
        return m->obstacle;
    }
    m->last_seq = reading->seq;
    m->have_seq = true;

    if (reading->status != US_VALID) {
        if (m->invalid_run < UINT8_MAX) {
            m->invalid_run++;
        }
        if (m->invalid_run >= m->invalid_limit) {
            m->count = 0U;                      /* stale values must not linger */
            m->have_median = false;
        }
        return m->obstacle;                     /* neither declares nor clears */
    }
    m->invalid_run = 0U;

    if (reading->distance_mm < urgent_mm) {
        m->obstacle = true;                     /* very close: no time to confirm */
        m->below_run = m->confirm_count;
        m->above_run = 0U;
    }

    m->window[m->head] = reading->distance_mm;
    m->head = (uint8_t)((m->head + 1U) % US_MEDIAN_LEN);
    if (m->count < US_MEDIAN_LEN) {
        m->count++;
    }
    median = median_of_window(m);
    m->last_median_mm = median;
    m->have_median = true;

    release_mm = (uint32_t)threshold_mm + (uint32_t)hysteresis_mm;
    if (median < threshold_mm) {
        m->above_run = 0U;
        if (m->below_run < UINT8_MAX) {
            m->below_run++;
        }
        if (m->below_run >= m->confirm_count) {
            m->obstacle = true;
        }
    } else if ((uint32_t)median > release_mm) {
        m->below_run = 0U;
        if (m->above_run < UINT8_MAX) {
            m->above_run++;
        }
        if (m->above_run >= m->confirm_count) {
            m->obstacle = false;
        }
    } else {
        m->below_run = 0U;                      /* in the hysteresis band: hold */
        m->above_run = 0U;
    }
    return m->obstacle;
}

bool us_monitor_sensor_fault(const us_monitor_t *m)
{
    return m->invalid_run >= m->invalid_limit;
}

bool us_monitor_distance(const us_monitor_t *m, uint16_t *distance_mm)
{
    if (!m->have_median) {
        return false;
    }
    *distance_mm = m->last_median_mm;
    return true;
}

uint16_t us_stop_threshold_mm(int16_t speed_mm_s, uint16_t response_ms,
                              uint16_t braking_mm, uint16_t margin_mm)
{
    int32_t speed = (speed_mm_s < 0) ? -(int32_t)speed_mm_s : (int32_t)speed_mm_s;
    int32_t mm = ((speed * (int32_t)response_ms) / MS_PER_S) +
                 (int32_t)braking_mm + (int32_t)margin_mm;

    return (uint16_t)((mm > (int32_t)US_MAX_RANGE_MM) ? (int32_t)US_MAX_RANGE_MM : mm);
}
