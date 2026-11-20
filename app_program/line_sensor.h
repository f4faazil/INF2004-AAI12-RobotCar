/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * line_sensor.h
 * Line-sensor processing for the three analogue IR channels (Buddy 3):
 * calibration, black/white conversion, line position, line-lost detection
 * and conversion of a sample stream into bar/space widths for the barcode
 * decoder.
 *
 * Pure C, no kernel or hardware dependencies: the caller supplies raw
 * 12-bit ADC readings.  The bench readings taken on the car rise on black
 * (white ~150, black ~4000), so "blackness" here means higher ADC = darker.
 */
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_CHANNEL_COUNT      (3U)
#define LINE_CHANNEL_LEFT       (0U)
#define LINE_CHANNEL_CENTRE     (1U)
#define LINE_CHANNEL_RIGHT      (2U)
#define LINE_CHANNEL_ALL_MASK   (0x07U)

/* Blackness is a per-mille value: 0 = white surface, 1000 = black. */
#define LINE_BLACKNESS_MAX      (1000U)

/* ---- calibration ---------------------------------------------------- */

typedef struct {
    uint16_t min[LINE_CHANNEL_COUNT];   /* lowest reading seen = white */
    uint16_t max[LINE_CHANNEL_COUNT];   /* highest reading seen = black */
} line_cal_t;

void line_cal_init(line_cal_t *cal);

/* Feed one frame while the sensors are swept over both black and white. */
void line_cal_feed(line_cal_t *cal, const uint16_t adc[LINE_CHANNEL_COUNT]);

/* Bit n set when channel n saw enough contrast to be trusted. */
uint8_t line_cal_valid_mask(const line_cal_t *cal);

uint16_t line_blackness(const line_cal_t *cal, uint8_t channel, uint16_t adc);

/* ---- line position -------------------------------------------------- */

/*
 * Weighted position of the line across the channels in channel_mask:
 * -1000 = under the left sensor, 0 = centre, +1000 = under the right sensor
 * (in sensor positions, not millimetres).  Returns false, leaving position
 * untouched, when too little black is visible to name a position.
 */
bool line_position(const uint16_t blackness[LINE_CHANNEL_COUNT],
                   uint8_t channel_mask,
                   int16_t *position);

/*
 * The same with the lateral position of each channel given explicitly, in
 * the same sensor-position units (1000 = the reference spacing).  This lets
 * an asymmetric layout work: for instance two close sensors that straddle the
 * line, plus a far sensor that reads a barcode and is left out of the mask.
 */
bool line_position_weighted(const uint16_t blackness[LINE_CHANNEL_COUNT],
                            uint8_t channel_mask,
                            const int16_t weight[LINE_CHANNEL_COUNT],
                            int16_t *position);

/* ---- line lost / found ---------------------------------------------- */

typedef enum {
    LINE_EVENT_NONE = 0,
    LINE_EVENT_LOST,
    LINE_EVENT_FOUND
} line_event_t;

typedef struct {
    uint8_t lost_frames;    /* consecutive frames without a line to call it lost */
    uint8_t found_frames;   /* consecutive frames with a line to call it found */
    uint8_t miss_count;
    uint8_t hit_count;
    bool    lost;
} line_tracker_t;

void line_tracker_init(line_tracker_t *tracker,
                       uint8_t lost_frames,
                       uint8_t found_frames);

/* Call once per steering frame; returns an event only when the state flips. */
line_event_t line_tracker_update(line_tracker_t *tracker, bool line_visible);

/* ---- sample stream -> bar/space widths ------------------------------ */

typedef struct {
    bool     started;
    bool     in_bar;
    bool     first_run;     /* the run in progress began before sampling did */
    uint16_t ticks;
} run_extractor_t;

void run_extractor_init(run_extractor_t *extractor);

/*
 * Push one blackness sample (per-mille).  A Schmitt trigger decides bar vs
 * space.  When a run ends, returns true and sets *run_is_bar / *run_ticks
 * (in samples, saturating at 65535).  The first run is never reported: its
 * start was not seen, so its width is unknown.
 */
bool run_extractor_push(run_extractor_t *extractor,
                        uint16_t blackness,
                        bool *run_is_bar,
                        uint16_t *run_ticks);

/*
 * The same, but each sample counts for 'weight' instead of 1.  With the weight
 * set to the distance the car travelled during the sample, run widths are
 * distances, so a car that accelerates, stutters or stops mid-barcode still
 * gives bars in the right proportion.  A weight of 0 (not moving) adds nothing.
 */
bool run_extractor_push_w(run_extractor_t *extractor,
                          uint16_t blackness,
                          uint16_t weight,
                          bool *run_is_bar,
                          uint16_t *run_ticks);

#endif /* LINE_SENSOR_H */
