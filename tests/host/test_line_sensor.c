/*
 * test_line_sensor.c
 * Host tests for calibration, blackness, line position, line-lost detection
 * and the sample-stream -> bar/space run extractor.  Expected values are
 * hand-derived literals; the ADC levels in the calibration test are the
 * readings taken on the car's bench (right, centre, and the stuck left IR).
 */
#include <stddef.h>
#include <stdint.h>

#include "line_sensor.h"
#include "test_support.h"

/* ---- calibration ----------------------------------------------------- */

/* Bench readings: right 161-177 white / 4073-4088 black, centre 127-153 white /
 * 3841-3997 black, left stuck at ~4000 on both surfaces.  The stuck sensor has
 * no contrast, so it must be excluded.  Catches: trusting a sensor that cannot
 * tell black from white (the car would steer on a constant). */
static void test_calibration_excludes_a_sensor_with_no_contrast(void)
{
    line_cal_t cal;
    /* order: left, centre, right */
    const uint16_t on_white[LINE_CHANNEL_COUNT] = { 4001U, 127U, 161U };
    const uint16_t on_white2[LINE_CHANNEL_COUNT] = { 3998U, 153U, 177U };
    const uint16_t on_black[LINE_CHANNEL_COUNT] = { 4004U, 3997U, 4088U };

    line_cal_init(&cal);
    line_cal_feed(&cal, on_white);
    line_cal_feed(&cal, on_black);
    line_cal_feed(&cal, on_white2);

    CHECK_INT(0x06, line_cal_valid_mask(&cal));   /* centre and right only */
}

/* No frames fed yet: nothing is trustworthy.  Catches: an uninitialised or
 * empty calibration reporting channels as valid. */
static void test_empty_calibration_trusts_nothing(void)
{
    line_cal_t cal;

    line_cal_init(&cal);
    CHECK_INT(0, line_cal_valid_mask(&cal));
}

/* white 150, black 4000: midpoint 2075 is exactly half black.  Catches: wrong
 * scaling, wrong direction, or no clamping outside the calibrated range. */
static void test_blackness_scales_between_white_and_black(void)
{
    line_cal_t cal;
    const uint16_t white[LINE_CHANNEL_COUNT] = { 150U, 150U, 150U };
    const uint16_t black[LINE_CHANNEL_COUNT] = { 4000U, 4000U, 4000U };

    line_cal_init(&cal);
    line_cal_feed(&cal, white);
    line_cal_feed(&cal, black);

    CHECK_INT(0, line_blackness(&cal, LINE_CHANNEL_CENTRE, 150U));
    CHECK_INT(1000, line_blackness(&cal, LINE_CHANNEL_CENTRE, 4000U));
    CHECK_INT(500, line_blackness(&cal, LINE_CHANNEL_CENTRE, 2075U));
    CHECK_INT(0, line_blackness(&cal, LINE_CHANNEL_CENTRE, 40U));      /* below white */
    CHECK_INT(1000, line_blackness(&cal, LINE_CHANNEL_CENTRE, 4095U)); /* above black */
}

/* ---- line position ---------------------------------------------------- */

/* Catches: wrong sign, wrong weighting, or dividing by the wrong total. */
static void test_position_weights_channels_by_blackness(void)
{
    int16_t p = 7;
    const uint16_t centred[LINE_CHANNEL_COUNT] = { 0U, 1000U, 0U };
    const uint16_t half_right[LINE_CHANNEL_COUNT] = { 0U, 500U, 500U };
    const uint16_t left[LINE_CHANNEL_COUNT] = { 1000U, 0U, 0U };
    const uint16_t mostly_left[LINE_CHANNEL_COUNT] = { 300U, 700U, 0U };

    CHECK(line_position(centred, LINE_CHANNEL_ALL_MASK, &p));
    CHECK_INT(0, p);
    CHECK(line_position(half_right, LINE_CHANNEL_ALL_MASK, &p));
    CHECK_INT(500, p);
    CHECK(line_position(left, LINE_CHANNEL_ALL_MASK, &p));
    CHECK_INT(-1000, p);
    CHECK(line_position(mostly_left, LINE_CHANNEL_ALL_MASK, &p));
    CHECK_INT(-300, p);
}

/* All white: no line to name a position for, and the output is not touched.
 * Catches: returning a bogus 0 ("centred") when the line is gone. */
static void test_position_reports_no_line_when_all_white(void)
{
    int16_t p = 123;
    const uint16_t white[LINE_CHANNEL_COUNT] = { 20U, 0U, 40U };

    CHECK(!line_position(white, LINE_CHANNEL_ALL_MASK, &p));
    CHECK_INT(123, p);
}

/* The left sensor reads black (e.g. a barcode beside the line); excluding it
 * must stop it pulling the position.  Catches: ignoring the channel mask. */
static void test_position_ignores_masked_out_channels(void)
{
    int16_t p = 0;
    const uint16_t barcode_on_left[LINE_CHANNEL_COUNT] = { 1000U, 1000U, 0U };

    CHECK(line_position(barcode_on_left, LINE_CHANNEL_ALL_MASK, &p));
    CHECK_INT(-500, p);
    CHECK(line_position(barcode_on_left,
                        (uint8_t)((1U << LINE_CHANNEL_CENTRE) | (1U << LINE_CHANNEL_RIGHT)),
                        &p));
    CHECK_INT(0, p);
}

/* The channel positions are configurable: two close sensors (here at -1000 and
 * +1000, i.e. +/-6 mm of a 6 mm reference) track the line while a third, far
 * sensor is left out of the mask.  Catches: assuming a symmetric three-sensor
 * bar, which cannot track an 18 mm line while another sensor reads a barcode
 * 39 mm away. */
static void test_weighted_position_with_a_custom_layout(void)
{
    const int16_t W[LINE_CHANNEL_COUNT] = { -1000, 1000, 6500 };    /* third is far away */
    uint16_t b[LINE_CHANNEL_COUNT] = { 0U, 1000U, 1000U };
    int16_t pos = 0;

    /* only the two tracking sensors in the mask: the far one is ignored */
    CHECK(line_position_weighted(b, 0x03U, W, &pos));
    CHECK_INT(1000, pos);
    b[0] = 1000U;
    CHECK(line_position_weighted(b, 0x03U, W, &pos));
    CHECK_INT(0, pos);                  /* the line straddles both */
    b[0] = 1000U; b[1] = 0U; b[2] = 1000U;
    CHECK(line_position_weighted(b, 0x03U, W, &pos));
    CHECK_INT(-1000, pos);              /* the far sensor's black does not count */
    /* with the far sensor included it would pull the answer a long way */
    CHECK(line_position_weighted(b, 0x07U, W, &pos));
    CHECK(pos > 1000);
}

/* The weights are signed and need not be symmetric. */
static void test_weighted_position_asymmetric_weights(void)
{
    const int16_t W[LINE_CHANNEL_COUNT] = { 0, 500, 1500 };
    const uint16_t b[LINE_CHANNEL_COUNT] = { 0U, 1000U, 1000U };
    int16_t pos = 0;

    CHECK(line_position_weighted(b, 0x07U, W, &pos));
    CHECK_INT(1000, pos);               /* (500 + 1500) / 2 */
}

/* ---- line lost / found ------------------------------------------------ */

/* Lost after 5 frames without the line, found after 3 with it, one event each.
 * Catches: reporting on the first bad frame (flicker), reporting repeatedly,
 * or never reporting. */
static void test_tracker_debounces_lost_and_found(void)
{
    line_tracker_t t;
    int i;

    line_tracker_init(&t, 5U, 3U);

    for (i = 0; i < 4; i++) {
        CHECK_INT(LINE_EVENT_NONE, line_tracker_update(&t, false));
    }
    CHECK_INT(LINE_EVENT_LOST, line_tracker_update(&t, false));
    CHECK_INT(LINE_EVENT_NONE, line_tracker_update(&t, false));   /* already lost */

    CHECK_INT(LINE_EVENT_NONE, line_tracker_update(&t, true));
    CHECK_INT(LINE_EVENT_NONE, line_tracker_update(&t, true));
    CHECK_INT(LINE_EVENT_FOUND, line_tracker_update(&t, true));
    CHECK_INT(LINE_EVENT_NONE, line_tracker_update(&t, true));    /* already found */
}

/* A single good frame in the middle of a loss restarts the count.  Catches: a
 * counter that is not reset, so scattered bad frames add up to a false loss. */
static void test_tracker_requires_consecutive_bad_frames(void)
{
    line_tracker_t t;
    int i;

    line_tracker_init(&t, 5U, 3U);
    for (i = 0; i < 4; i++) {
        CHECK_INT(LINE_EVENT_NONE, line_tracker_update(&t, false));
    }
    CHECK_INT(LINE_EVENT_NONE, line_tracker_update(&t, true));    /* blip */
    for (i = 0; i < 4; i++) {
        CHECK_INT(LINE_EVENT_NONE, line_tracker_update(&t, false));
    }
    CHECK_INT(LINE_EVENT_LOST, line_tracker_update(&t, false));
}

/* ---- run extractor ---------------------------------------------------- */

typedef struct {
    int      count;
    bool     is_bar[8];
    uint16_t ticks[8];
} runs_t;

static runs_t extract(const uint16_t *blackness, size_t n)
{
    run_extractor_t e;
    runs_t r;
    size_t i;

    r.count = 0;
    run_extractor_init(&e);
    for (i = 0U; i < n; i++) {
        bool is_bar = false;
        uint16_t ticks = 0U;

        if (run_extractor_push(&e, blackness[i], &is_bar, &ticks) && (r.count < 8)) {
            r.is_bar[r.count] = is_bar;
            r.ticks[r.count] = ticks;
            r.count++;
        }
    }
    return r;
}

/* space(2) bar(3) space(4) bar(1) space: the leading run is partial and is
 * not reported; each later run is reported when it ends, with its length in
 * samples.  Catches: off-by-one run lengths, swapped bar/space, reporting the
 * unknown first run. */
static void test_extractor_reports_each_completed_run(void)
{
    const uint16_t s[] = { 0, 0, 900, 900, 900, 0, 0, 0, 0, 900, 0 };
    runs_t r = extract(s, sizeof(s) / sizeof(s[0]));

    CHECK_INT(3, r.count);
    CHECK(r.is_bar[0]);
    CHECK_INT(3, r.ticks[0]);
    CHECK(!r.is_bar[1]);
    CHECK_INT(4, r.ticks[1]);
    CHECK(r.is_bar[2]);
    CHECK_INT(1, r.ticks[2]);
}

/* Samples hovering between the two thresholds must not toggle the state.
 * Catches: a single threshold, which turns ADC noise at an edge into a burst
 * of false 1-sample runs. */
static void test_extractor_ignores_noise_between_thresholds(void)
{
    /* bar, then 550/450/500 wobble (between 400 and 600), then white */
    const uint16_t s[] = { 0, 900, 900, 550, 450, 500, 900, 0, 0 };
    runs_t r = extract(s, sizeof(s) / sizeof(s[0]));

    CHECK_INT(1, r.count);
    CHECK(r.is_bar[0]);
    CHECK_INT(6, r.ticks[0]);   /* samples 1..6, wobble included */
}

/* A run longer than 65535 samples saturates; it must not wrap to a small
 * number that would look like a narrow element.  Catches: counter overflow. */
static void test_extractor_saturates_instead_of_wrapping(void)
{
    run_extractor_t e;
    uint32_t i;
    bool is_bar = true;
    uint16_t ticks = 0U;
    bool emitted = false;

    run_extractor_init(&e);
    (void)run_extractor_push(&e, 900U, &is_bar, &ticks);          /* partial bar */
    for (i = 0U; i < 70000U; i++) {
        (void)run_extractor_push(&e, 0U, &is_bar, &ticks);        /* long white */
    }
    emitted = run_extractor_push(&e, 900U, &is_bar, &ticks);
    CHECK(emitted);
    CHECK(!is_bar);
    CHECK_INT(65535, ticks);
}

int main(void)
{
    RUN(test_calibration_excludes_a_sensor_with_no_contrast);
    RUN(test_empty_calibration_trusts_nothing);
    RUN(test_weighted_position_with_a_custom_layout);
    RUN(test_weighted_position_asymmetric_weights);
    RUN(test_blackness_scales_between_white_and_black);
    RUN(test_position_weights_channels_by_blackness);
    RUN(test_position_reports_no_line_when_all_white);
    RUN(test_position_ignores_masked_out_channels);
    RUN(test_tracker_debounces_lost_and_found);
    RUN(test_tracker_requires_consecutive_bad_frames);
    RUN(test_extractor_reports_each_completed_run);
    RUN(test_extractor_ignores_noise_between_thresholds);
    RUN(test_extractor_saturates_instead_of_wrapping);
    return TEST_REPORT();
}
