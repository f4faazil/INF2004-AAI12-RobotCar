/*
 * test_barcode.c
 * Host tests for the Code 39 decoder, using run widths measured from the
 * lecturer's barcode PDFs (barcode_fixtures.h).
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "barcode.h"
#include "barcode_fixtures.h"
#include "test_support.h"

/* Result of pushing a whole symbol into a decoder. */
typedef struct {
    int  decoded_count;     /* how many pushes returned BARCODE_DECODED */
    int  decoded_at;        /* index of the (last) push that decoded, or -1 */
    char decoded_char;
} push_outcome_t;

/* Push n runs, alternating bar/space starting with a bar. */
static push_outcome_t push_runs(barcode_decoder_t *decoder,
                                const uint16_t *runs, size_t n)
{
    push_outcome_t out = { 0, -1, '\0' };
    size_t i;

    for (i = 0U; i < n; i++) {
        char c = '\0';
        bool is_bar = ((i % 2U) == 0U);

        if (barcode_push_run(decoder, is_bar, runs[i], &c) == BARCODE_DECODED) {
            out.decoded_count++;
            out.decoded_at = (int)i;
            out.decoded_char = c;
        }
    }
    return out;
}

/* Every real barcode page, at three printed scales, decodes to its letter -
 * and only on the final bar.  Catches: wrong Code 39 table entry, wrong
 * window alignment, a decode that fires early or never. */
static void test_decodes_every_real_barcode_page(void)
{
    size_t f;

    for (f = 0U; f < BARCODE_FIXTURE_COUNT; f++) {
        barcode_decoder_t d;
        push_outcome_t out;

        barcode_init(&d);
        out = push_runs(&d, BARCODE_FIXTURES[f].runs, BARCODE_ELEMENT_COUNT);

        CHECK_INT(1, out.decoded_count);
        CHECK_INT((int)BARCODE_ELEMENT_COUNT - 1, out.decoded_at);
        CHECK_INT(BARCODE_FIXTURES[f].expected, out.decoded_char);
    }
}

/* ---- helpers for the rejection tests -------------------------------- */

#define STOP_CHAR_FIRST (20U)
#define SYMBOL_BYTES    (BARCODE_ELEMENT_COUNT * sizeof(uint16_t))

static const uint16_t *fixture_runs(char letter, const char *scale)
{
    size_t f;

    for (f = 0U; f < BARCODE_FIXTURE_COUNT; f++) {
        if ((BARCODE_FIXTURES[f].expected == letter) &&
            (strcmp(BARCODE_FIXTURES[f].scale, scale) == 0)) {
            return BARCODE_FIXTURES[f].runs;
        }
    }
    return NULL;
}

static void copy_symbol(uint16_t *dst, const uint16_t *src)
{
    memcpy(dst, src, SYMBOL_BYTES);
}

static push_outcome_t push_symbol(const uint16_t *symbol)
{
    barcode_decoder_t d;

    barcode_init(&d);
    return push_runs(&d, symbol, BARCODE_ELEMENT_COUNT);
}

/* ---- rejection and tolerance ----------------------------------------- */

/* Junk (a long white, stray bars) before the symbol must not decode, and the
 * symbol must still decode once.  Catches: decoding a misaligned window,
 * i.e. an unanchored start/stop check, which would turn line noise into a
 * wrong turn command. */
static void test_junk_before_symbol_gives_exactly_one_decode(void)
{
    uint16_t stream[12U + BARCODE_ELEMENT_COUNT] = {
        50, 700, 20, 300, 33, 100, 61, 45, 28, 210, 90, 500
    };
    barcode_decoder_t d;
    push_outcome_t out;

    memcpy(&stream[12], fixture_runs('A', "large"), SYMBOL_BYTES);
    barcode_init(&d);
    out = push_runs(&d, stream, 12U + BARCODE_ELEMENT_COUNT);

    CHECK_INT(1, out.decoded_count);
    CHECK_INT(12 + 28, out.decoded_at);
    CHECK_INT('A', out.decoded_char);
}

/* Read backwards, "*A*" looks like "P1P".  The car can meet a barcode from
 * either side (after a U-turn, or a strip laid the other way round), so every
 * real page scanned from its far end must decode to its own letter, once.
 * Catches: accepting a reversed symbol as a different valid letter (a wrong
 * command), or missing it altogether. */
static void test_reversed_symbol_decodes_to_the_same_letter(void)
{
    size_t f;

    for (f = 0U; f < BARCODE_FIXTURE_COUNT; f++) {
        uint16_t symbol[BARCODE_ELEMENT_COUNT];
        push_outcome_t out;
        size_t i;

        for (i = 0U; i < BARCODE_ELEMENT_COUNT; i++) {
            symbol[i] = BARCODE_FIXTURES[f].runs[BARCODE_ELEMENT_COUNT - 1U - i];
        }
        out = push_symbol(symbol);
        CHECK_INT(1, out.decoded_count);
        CHECK_INT(BARCODE_FIXTURES[f].expected, out.decoded_char);
    }
}

/* A valid character 'A' where the start '*' belongs.  Catches: a missing
 * start-character check. */
static void test_wrong_start_character_is_rejected(void)
{
    uint16_t symbol[BARCODE_ELEMENT_COUNT];
    const uint16_t *a = fixture_runs('A', "large");

    copy_symbol(symbol, fixture_runs('F', "large"));
    memcpy(&symbol[0], &a[10], 9U * sizeof(uint16_t));
    CHECK_INT(0, push_symbol(symbol).decoded_count);
}

/* A valid character 'A' where the stop '*' belongs.  Catches: a missing
 * stop-character check (a cut-off or partial symbol would be accepted). */
static void test_wrong_stop_character_is_rejected(void)
{
    uint16_t symbol[BARCODE_ELEMENT_COUNT];
    const uint16_t *a = fixture_runs('A', "large");

    copy_symbol(symbol, fixture_runs('F', "large"));
    memcpy(&symbol[STOP_CHAR_FIRST], &a[10], 9U * sizeof(uint16_t));
    CHECK_INT(0, push_symbol(symbol).decoded_count);
}

/* A narrow space of the 'A' is smeared to 200 px, almost as wide as the three
 * real wide elements (238 px).  The three widest are still exactly the right
 * three, so only a narrow/wide separation check can refuse it.  Catches:
 * trusting "the three widest" without checking that they are clearly wider
 * than everything else (a blurred or dirty bar would read as a wrong letter
 * instead of being rejected). */
static void test_ambiguous_narrow_wide_separation_is_rejected(void)
{
    uint16_t symbol[BARCODE_ELEMENT_COUNT];

    copy_symbol(symbol, fixture_runs('A', "large"));
    symbol[11] = 200U;
    CHECK_INT(0, push_symbol(symbol).decoded_count);
}

/* A data character whose elements are all the same width.  Catches: decoding
 * a character with no narrow/wide separation at all. */
static void test_uniform_widths_is_rejected(void)
{
    uint16_t symbol[BARCODE_ELEMENT_COUNT];
    size_t i;

    copy_symbol(symbol, fixture_runs('A', "large"));
    for (i = 10U; i < 19U; i++) {
        symbol[i] = 79U;
    }
    CHECK_INT(0, push_symbol(symbol).decoded_count);
}

/* A wide gap between characters.  Catches: ignoring the Code 39 rule that
 * the inter-character gap is narrow. */
static void test_wide_inter_character_gap_is_rejected(void)
{
    uint16_t symbol[BARCODE_ELEMENT_COUNT];

    copy_symbol(symbol, fixture_runs('A', "large"));
    symbol[9] = 238U;
    CHECK_INT(0, push_symbol(symbol).decoded_count);
}

/* Scale the elements first..first+8 (one character) by num/den. */
static void scale_char(uint16_t *symbol, size_t first, uint16_t num, uint16_t den)
{
    size_t i;

    for (i = first; i < (first + 9U); i++) {
        symbol[i] = (uint16_t)(((uint32_t)symbol[i] * num) / den);
    }
}

/* A gap that is wide against only one of its two neighbouring characters:
 * the neighbour on the other side is 1.4x larger, so it still looks narrow
 * next to that one.  Both checks are needed.  Catches: comparing the gap with
 * just one neighbour. */
static void test_gap_wide_against_either_neighbour_is_rejected(void)
{
    uint16_t symbol[BARCODE_ELEMENT_COUNT];

    /* first character 1.4x: gap 160 is narrow against it, wide against 'A' */
    copy_symbol(symbol, fixture_runs('A', "large"));
    scale_char(symbol, 0U, 7U, 5U);
    symbol[9] = 160U;
    CHECK_INT(0, push_symbol(symbol).decoded_count);

    /* second character 1.4x: gap 160 is narrow against it, wide against '*' */
    copy_symbol(symbol, fixture_runs('A', "large"));
    scale_char(symbol, 10U, 7U, 5U);
    symbol[9] = 160U;
    CHECK_INT(0, push_symbol(symbol).decoded_count);
}

/* The car speeds up 25% across the symbol: still a valid read.  Catches: a
 * consistency check that is too strict for real speed changes. */
static void test_moderate_speed_change_is_accepted(void)
{
    uint16_t symbol[BARCODE_ELEMENT_COUNT];
    push_outcome_t out;

    copy_symbol(symbol, fixture_runs('A', "large"));
    scale_char(symbol, STOP_CHAR_FIRST, 5U, 4U);
    out = push_symbol(symbol);
    CHECK_INT(1, out.decoded_count);
    CHECK_INT('A', out.decoded_char);
}

/* The stop character is twice the size of the start character: every
 * character is individually valid, but the symbol cannot be one pass at one
 * speed.  Catches: a missing cross-character scale check. */
static void test_large_speed_change_is_rejected(void)
{
    uint16_t symbol[BARCODE_ELEMENT_COUNT];

    copy_symbol(symbol, fixture_runs('A', "large"));
    scale_char(symbol, STOP_CHAR_FIRST, 2U, 1U);
    CHECK_INT(0, push_symbol(symbol).decoded_count);
}

/* Worst case width error of 25%: every narrow element 25% too wide, every
 * wide element 25% too narrow.  Catches: a narrow/wide threshold too strict
 * for real optics and thresholding. */
static void test_worst_case_25_percent_width_error_is_accepted(void)
{
    const char letters[] = { 'A', 'F', 'Z' };
    size_t l;
    size_t i;

    for (l = 0U; l < sizeof(letters); l++) {
        uint16_t symbol[BARCODE_ELEMENT_COUNT];
        push_outcome_t out;

        copy_symbol(symbol, fixture_runs(letters[l], "large"));
        for (i = 0U; i < BARCODE_ELEMENT_COUNT; i++) {
            /* large scale: narrow is 79 px, wide is 238 px */
            symbol[i] = (symbol[i] > 150U) ? (uint16_t)((symbol[i] * 3U) / 4U)
                                           : (uint16_t)((symbol[i] * 5U) / 4U);
        }
        out = push_symbol(symbol);
        CHECK_INT(1, out.decoded_count);
        CHECK_INT(letters[l], out.decoded_char);
    }
}

/* 300000 random runs (deterministic seed) must produce no decode.  Catches:
 * a decoder loose enough to turn line noise into a command. */
static void test_random_runs_never_decode(void)
{
    barcode_decoder_t d;
    uint32_t state = 12345U;
    uint32_t n;
    int decodes = 0;

    barcode_init(&d);
    for (n = 0U; n < 300000U; n++) {
        char c = '\0';
        uint16_t ticks;

        state = (state * 1664525U) + 1013904223U;
        ticks = (uint16_t)(1U + ((state >> 16) % 500U));
        if (barcode_push_run(&d, ((n % 2U) == 0U), ticks, &c) == BARCODE_DECODED) {
            decodes++;
        }
    }
    CHECK_INT(0, decodes);
}


/*
 * Every letter A to Z, built from the standard Code 39 patterns (written out here,
 * independently of the decoder's own table): bar, space, bar ... with n = narrow and
 * w = wide.  Each must decode to itself read forwards, and read from the other end
 * must give the same letter or nothing, never a different letter.  Catches: a wrong
 * pattern in the decoder's table, and a letter that reads as another one backwards,
 * which would send the car the wrong way.
 */
static void test_every_letter_decodes_forward_and_reversed(void)
{
    static const char *PATTERN[26] = {
        "wnnnnwnnw", "nnwnnwnnw", "wnwnnwnnn", "nnnnwwnnw", "wnnnwwnnn", "nnwnwwnnn",
        "nnnnnwwnw", "wnnnnwwnn", "nnwnnwwnn", "nnnnwwwnn", "wnnnnnnww", "nnwnnnnww",
        "wnwnnnnwn", "nnnnwnnww", "wnnnwnnwn", "nnwnwnnwn", "nnnnnnwww", "wnnnnnwwn",
        "nnwnnnwwn", "nnnnwnwwn", "wwnnnnnnw", "nwwnnnnnw", "wwwnnnnnn", "nwnnwnnnw",
        "wwnnwnnnn", "nwwnwnnnn"
    };
    static const char *STAR = "nwnnwnwnn";
    unsigned letter;

    for (letter = 0U; letter < 26U; letter++) {
        uint16_t fwd[BARCODE_ELEMENT_COUNT];
        uint16_t rev[BARCODE_ELEMENT_COUNT];
        const char *parts[3];
        size_t n = 0U;
        unsigned part;
        unsigned i;
        barcode_decoder_t d;
        push_outcome_t out;

        parts[0] = STAR;
        parts[1] = PATTERN[letter];
        parts[2] = STAR;
        for (part = 0U; part < 3U; part++) {
            for (i = 0U; i < 9U; i++) {
                fwd[n++] = (parts[part][i] == 'w') ? 30U : 10U;
            }
            if (part < 2U) {
                fwd[n++] = 10U;                         /* the narrow gap between characters */
            }
        }
        CHECK_INT((int)BARCODE_ELEMENT_COUNT, (int)n);

        barcode_init(&d);
        out = push_runs(&d, fwd, BARCODE_ELEMENT_COUNT);
        CHECK_INT(1, out.decoded_count);
        CHECK_INT((int)('A' + letter), (int)out.decoded_char);

        for (i = 0U; i < BARCODE_ELEMENT_COUNT; i++) {
            rev[i] = fwd[BARCODE_ELEMENT_COUNT - 1U - i];
        }
        barcode_init(&d);
        out = push_runs(&d, rev, BARCODE_ELEMENT_COUNT);
        if (out.decoded_count != 0) {
            CHECK_INT((int)('A' + letter), (int)out.decoded_char);
        }
    }
}


/*
 * A real capture from the car: a Z read from its far end, so the runs arrive reversed, with
 * the sensor's edge bias making some bars short and some gaps long (one narrow element 924,
 * one wide 1356: a ratio of only 1.47).  It must decode as Z.  Catches: a narrow/wide
 * tolerance so strict that a whole, correctly read barcode is refused.
 */
static void test_real_capture_with_edge_bias_decodes(void)
{
    static const uint16_t CAPTURE[BARCODE_ELEMENT_COUNT] = {
        536, 650, 1726, 631, 1657, 671, 579, 1813, 432, 611, 550, 608, 548, 624, 1612, 616,
        1700, 1844, 478, 766, 264, 924, 1356, 825, 1615, 640, 526, 1698, 496
    };
    barcode_decoder_t d;
    push_outcome_t out;

    barcode_init(&d);
    out = push_runs(&d, CAPTURE, BARCODE_ELEMENT_COUNT);
    CHECK_INT(1, out.decoded_count);
    CHECK_INT('Z', out.decoded_char);
}

int main(void)
{
    RUN(test_decodes_every_real_barcode_page);
    RUN(test_every_letter_decodes_forward_and_reversed);
    RUN(test_real_capture_with_edge_bias_decodes);
    RUN(test_junk_before_symbol_gives_exactly_one_decode);
    RUN(test_reversed_symbol_decodes_to_the_same_letter);
    RUN(test_wrong_start_character_is_rejected);
    RUN(test_wrong_stop_character_is_rejected);
    RUN(test_ambiguous_narrow_wide_separation_is_rejected);
    RUN(test_uniform_widths_is_rejected);
    RUN(test_wide_inter_character_gap_is_rejected);
    RUN(test_gap_wide_against_either_neighbour_is_rejected);
    RUN(test_moderate_speed_change_is_accepted);
    RUN(test_large_speed_change_is_rejected);
    RUN(test_worst_case_25_percent_width_error_is_accepted);
    RUN(test_random_runs_never_decode);
    return TEST_REPORT();
}
