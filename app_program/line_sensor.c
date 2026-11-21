/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * line_sensor.c
 * Line-sensor processing.  See line_sensor.h.
 */
#include "line_sensor.h"

/*
 * A channel is trusted only if calibration saw at least this much ADC span
 * (of 4095 counts).  The bench readings span about 3600-3900; a sensor stuck
 * at ~4000 on both surfaces spans nothing.
 */
#define MIN_CONTRAST_COUNTS     (800U)

/* The line is "visible" when the channels in the mask add up to this much. */
#define POSITION_MIN_TOTAL      (300U)

/* Schmitt-trigger levels for bar/space, in per-mille blackness. */
#define BAR_ON_LEVEL            (600U)
#define BAR_OFF_LEVEL           (400U)

/* Channel positions across the sensor bar, in sensor positions x 1000. */
static const int32_t CHANNEL_POSITION[LINE_CHANNEL_COUNT] = { -1000, 0, 1000 };

/* ---- calibration ---------------------------------------------------- */

void line_cal_init(line_cal_t *cal)
{
    uint8_t ch;

    for (ch = 0U; ch < LINE_CHANNEL_COUNT; ch++) {
        cal->min[ch] = UINT16_MAX;
        cal->max[ch] = 0U;
    }
}

void line_cal_feed(line_cal_t *cal, const uint16_t adc[LINE_CHANNEL_COUNT])
{
    uint8_t ch;

    for (ch = 0U; ch < LINE_CHANNEL_COUNT; ch++) {
        if (adc[ch] < cal->min[ch]) {
            cal->min[ch] = adc[ch];
        }
        if (adc[ch] > cal->max[ch]) {
            cal->max[ch] = adc[ch];
        }
    }
}

uint8_t line_cal_valid_mask(const line_cal_t *cal)
{
    uint8_t mask = 0U;
    uint8_t ch;

    for (ch = 0U; ch < LINE_CHANNEL_COUNT; ch++) {
        if ((cal->max[ch] > cal->min[ch]) &&
            ((uint32_t)(cal->max[ch] - cal->min[ch]) >= MIN_CONTRAST_COUNTS)) {
            mask = (uint8_t)(mask | (1U << ch));
        }
    }
    return mask;
}

uint16_t line_blackness(const line_cal_t *cal, uint8_t channel, uint16_t adc)
{
    uint32_t span;

    if (cal->max[channel] <= cal->min[channel]) {
        return 0U;
    }
    if (adc <= cal->min[channel]) {
        return 0U;
    }
    if (adc >= cal->max[channel]) {
        return (uint16_t)LINE_BLACKNESS_MAX;
    }
    span = (uint32_t)(cal->max[channel] - cal->min[channel]);
    return (uint16_t)(((uint32_t)(adc - cal->min[channel]) * LINE_BLACKNESS_MAX) / span);
}

/* ---- line position -------------------------------------------------- */

bool line_position_weighted(const uint16_t blackness[LINE_CHANNEL_COUNT],
                            uint8_t channel_mask,
                            const int16_t weight[LINE_CHANNEL_COUNT],
                            int16_t *position)
{
    int32_t  weighted = 0;
    uint32_t total = 0U;
    uint8_t  ch;

    for (ch = 0U; ch < LINE_CHANNEL_COUNT; ch++) {
        if (((channel_mask >> ch) & 1U) != 0U) {
            total += blackness[ch];
            weighted += (int32_t)blackness[ch] * weight[ch];
        }
    }
    if (total < POSITION_MIN_TOTAL) {
        return false;
    }
    *position = (int16_t)(weighted / (int32_t)total);
    return true;
}

bool line_position(const uint16_t blackness[LINE_CHANNEL_COUNT],
                   uint8_t channel_mask,
                   int16_t *position)
{
    int16_t weight[LINE_CHANNEL_COUNT];
    uint8_t ch;

    for (ch = 0U; ch < LINE_CHANNEL_COUNT; ch++) {
        weight[ch] = (int16_t)CHANNEL_POSITION[ch];
    }
    return line_position_weighted(blackness, channel_mask, weight, position);
}

/* ---- line lost / found ---------------------------------------------- */

void line_tracker_init(line_tracker_t *tracker,
                       uint8_t lost_frames,
                       uint8_t found_frames)
{
    tracker->lost_frames = lost_frames;
    tracker->found_frames = found_frames;
    tracker->miss_count = 0U;
    tracker->hit_count = 0U;
    tracker->lost = false;
}

line_event_t line_tracker_update(line_tracker_t *tracker, bool line_visible)
{
    if (tracker->lost) {
        if (!line_visible) {
            tracker->hit_count = 0U;
            return LINE_EVENT_NONE;
        }
        tracker->hit_count++;
        if (tracker->hit_count >= tracker->found_frames) {
            tracker->lost = false;
            tracker->hit_count = 0U;
            tracker->miss_count = 0U;
            return LINE_EVENT_FOUND;
        }
        return LINE_EVENT_NONE;
    }

    if (line_visible) {
        tracker->miss_count = 0U;
        return LINE_EVENT_NONE;
    }
    tracker->miss_count++;
    if (tracker->miss_count >= tracker->lost_frames) {
        tracker->lost = true;
        tracker->miss_count = 0U;
        tracker->hit_count = 0U;
        return LINE_EVENT_LOST;
    }
    return LINE_EVENT_NONE;
}

/* ---- sample stream -> bar/space widths ------------------------------ */

void run_extractor_init(run_extractor_t *extractor)
{
    extractor->started = false;
    extractor->in_bar = false;
    extractor->first_run = true;
    extractor->ticks = 0U;
}

bool run_extractor_push(run_extractor_t *extractor,
                        uint16_t blackness,
                        bool *run_is_bar,
                        uint16_t *run_ticks)
{
    return run_extractor_push_w(extractor, blackness, 1U, run_is_bar, run_ticks);
}

bool run_extractor_push_w(run_extractor_t *extractor,
                          uint16_t blackness,
                          uint16_t weight,
                          bool *run_is_bar,
                          uint16_t *run_ticks)
{
    bool next_in_bar;
    bool emitted = false;

    if (!extractor->started) {
        extractor->started = true;
        extractor->in_bar = (blackness >= BAR_ON_LEVEL);
        extractor->first_run = true;
        extractor->ticks = weight;
        return false;
    }

    next_in_bar = extractor->in_bar;
    if (extractor->in_bar) {
        if (blackness <= BAR_OFF_LEVEL) {
            next_in_bar = false;
        }
    } else {
        if (blackness >= BAR_ON_LEVEL) {
            next_in_bar = true;
        }
    }

    if (next_in_bar != extractor->in_bar) {
        if (!extractor->first_run) {
            *run_is_bar = extractor->in_bar;
            *run_ticks = extractor->ticks;
            emitted = true;
        }
        extractor->first_run = false;
        extractor->in_bar = next_in_bar;
        extractor->ticks = weight;
    } else if (((uint32_t)extractor->ticks + weight) < UINT16_MAX) {
        extractor->ticks = (uint16_t)(extractor->ticks + weight);
    } else {
        extractor->ticks = UINT16_MAX;
    }
    return emitted;
}
