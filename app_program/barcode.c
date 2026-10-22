/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * barcode.c
 * Code 39 decoder.  See barcode.h.
 */
#include "barcode.h"

#include <stddef.h>

#define ELEMENTS_PER_CHAR   (9U)
#define WIDE_PER_CHAR       (3U)
#define CHARS_PER_SYMBOL    (3U)
#define CHAR_STRIDE         (10U)   /* 9 elements + 1 inter-character gap */
#define START_STOP_CHAR     ('*')

/*
 * Code 39 nominal wide:narrow is 3:1 and the standard allows 2.2:1 to 3:1.
 * The smallest wide element must be at least 1.6 x the largest narrow one;
 * that leaves room for blur and thresholding error and was checked against
 * a worst case of every narrow +25% and every wide -25% (ratio 1.8).
 */
#define SEPARATION_NUM      (8U)
#define SEPARATION_DEN      (5U)

/*
 * One pass at one speed: the narrow-element size of any two characters in a
 * symbol may differ by at most 1.5 x.
 */
#define SCALE_TOLERANCE_NUM (3U)
#define SCALE_TOLERANCE_DEN (2U)

/*
 * Code 39 characters.  Bit i of the mask is set when element i of the
 * character (bar, space, bar, ... , bar) is wide.  Every character has
 * exactly three wide elements.
 */
typedef struct {
    char     character;
    uint16_t wide_mask;
} code39_entry_t;

static const code39_entry_t CODE39_TABLE[] = {
    { '0', 0x058U },   /* nnnwwnwnn */
    { '1', 0x109U },   /* wnnwnnnnw */
    { '2', 0x10CU },   /* nnwwnnnnw */
    { '3', 0x00DU },   /* wnwwnnnnn */
    { '4', 0x118U },   /* nnnwwnnnw */
    { '5', 0x019U },   /* wnnwwnnnn */
    { '6', 0x01CU },   /* nnwwwnnnn */
    { '7', 0x148U },   /* nnnwnnwnw */
    { '8', 0x049U },   /* wnnwnnwnn */
    { '9', 0x04CU },   /* nnwwnnwnn */
    { 'A', 0x121U },   /* wnnnnwnnw */
    { 'B', 0x124U },   /* nnwnnwnnw */
    { 'C', 0x025U },   /* wnwnnwnnn */
    { 'D', 0x130U },   /* nnnnwwnnw */
    { 'E', 0x031U },   /* wnnnwwnnn */
    { 'F', 0x034U },   /* nnwnwwnnn */
    { 'G', 0x160U },   /* nnnnnwwnw */
    { 'H', 0x061U },   /* wnnnnwwnn */
    { 'I', 0x064U },   /* nnwnnwwnn */
    { 'J', 0x070U },   /* nnnnwwwnn */
    { 'K', 0x181U },   /* wnnnnnnww */
    { 'L', 0x184U },   /* nnwnnnnww */
    { 'M', 0x085U },   /* wnwnnnnwn */
    { 'N', 0x190U },   /* nnnnwnnww */
    { 'O', 0x091U },   /* wnnnwnnwn */
    { 'P', 0x094U },   /* nnwnwnnwn */
    { 'Q', 0x1C0U },   /* nnnnnnwww */
    { 'R', 0x0C1U },   /* wnnnnnwwn */
    { 'S', 0x0C4U },   /* nnwnnnwwn */
    { 'T', 0x0D0U },   /* nnnnwnwwn */
    { 'U', 0x103U },   /* wwnnnnnnw */
    { 'V', 0x106U },   /* nwwnnnnnw */
    { 'W', 0x007U },   /* wwwnnnnnn */
    { 'X', 0x112U },   /* nwnnwnnnw */
    { 'Y', 0x013U },   /* wwnnwnnnn */
    { 'Z', 0x016U },   /* nwwnwnnnn */
    { '-', 0x142U },   /* nwnnnnwnw */
    { '.', 0x043U },   /* wwnnnnwnn */
    { ' ', 0x046U },   /* nwwnnnwnn */
    { '*', 0x052U },   /* nwnnwnwnn */
    { '$', 0x02AU },   /* nwnwnwnnn */
    { '/', 0x08AU },   /* nwnwnnnwn */
    { '+', 0x0A2U },   /* nwnnnwnwn */
    { '%', 0x0A8U }    /* nnnwnwnwn */
};

#define CODE39_TABLE_SIZE   (sizeof(CODE39_TABLE) / sizeof(CODE39_TABLE[0]))

typedef struct {
    char     character;
    uint16_t min_wide;      /* smallest element taken as wide */
    uint32_t narrow_sum;    /* total of the six narrow elements */
} char_info_t;

/* True when big is at least num/den times small. */
static bool at_least(uint16_t big, uint16_t small, uint32_t num, uint32_t den)
{
    return ((uint32_t)big * den) >= ((uint32_t)small * num);
}

/*
 * Mark the three widest of the nine elements as wide, require them to be
 * clearly wider than the rest, and look the pattern up.
 */
static bool classify_character(const uint16_t *elements, char_info_t *info)
{
    uint16_t mask = 0U;
    uint16_t max_narrow = 0U;
    uint8_t  pick;
    uint8_t  i;
    size_t   entry;

    info->min_wide = UINT16_MAX;
    info->narrow_sum = 0U;

    for (pick = 0U; pick < WIDE_PER_CHAR; pick++) {
        uint8_t  widest = 0U;
        uint16_t widest_ticks = 0U;
        bool     found = false;

        for (i = 0U; i < ELEMENTS_PER_CHAR; i++) {
            bool taken = ((mask >> i) & 1U) != 0U;

            if ((!taken) && ((!found) || (elements[i] > widest_ticks))) {
                widest = i;
                widest_ticks = elements[i];
                found = true;
            }
        }
        mask = (uint16_t)(mask | (1U << widest));
    }

    for (i = 0U; i < ELEMENTS_PER_CHAR; i++) {
        if (((mask >> i) & 1U) != 0U) {
            if (elements[i] < info->min_wide) {
                info->min_wide = elements[i];
            }
        } else {
            if (elements[i] > max_narrow) {
                max_narrow = elements[i];
            }
            info->narrow_sum += elements[i];
        }
    }

    if (!at_least(info->min_wide, max_narrow, SEPARATION_NUM, SEPARATION_DEN)) {
        return false;
    }

    for (entry = 0U; entry < CODE39_TABLE_SIZE; entry++) {
        if (CODE39_TABLE[entry].wide_mask == mask) {
            info->character = CODE39_TABLE[entry].character;
            return true;
        }
    }
    return false;
}

/* True when a and b differ by no more than the scale tolerance. */
static bool similar_scale(uint32_t a, uint32_t b)
{
    uint32_t big = (a > b) ? a : b;
    uint32_t small = (a > b) ? b : a;

    return (big * SCALE_TOLERANCE_DEN) <= (small * SCALE_TOLERANCE_NUM);
}

/*
 * Validate a whole "*X*" symbol: three well-formed characters, '*' at both
 * ends, narrow gaps between characters, and one consistent scale.
 */
static bool decode_symbol(const uint16_t *runs, char *decoded)
{
    char_info_t info[CHARS_PER_SYMBOL];
    uint8_t     c;

    for (c = 0U; c < CHARS_PER_SYMBOL; c++) {
        if (!classify_character(&runs[c * CHAR_STRIDE], &info[c])) {
            return false;
        }
    }

    if ((info[0].character != START_STOP_CHAR) ||
        (info[CHARS_PER_SYMBOL - 1U].character != START_STOP_CHAR)) {
        return false;
    }

    for (c = 0U; c < (CHARS_PER_SYMBOL - 1U); c++) {
        uint16_t gap = runs[(c * CHAR_STRIDE) + ELEMENTS_PER_CHAR];

        if ((!at_least(info[c].min_wide, gap, SEPARATION_NUM, SEPARATION_DEN)) ||
            (!at_least(info[c + 1U].min_wide, gap, SEPARATION_NUM, SEPARATION_DEN))) {
            return false;
        }
        if (!similar_scale(info[c].narrow_sum, info[c + 1U].narrow_sum)) {
            return false;
        }
    }

    *decoded = info[1].character;
    return true;
}

void barcode_init(barcode_decoder_t *decoder)
{
    decoder->count = 0U;
}

barcode_result_t barcode_push_run(barcode_decoder_t *decoder,
                                  bool is_bar,
                                  uint16_t ticks,
                                  char *decoded)
{
    uint8_t i;

    if (decoder->count == BARCODE_ELEMENT_COUNT) {
        for (i = 1U; i < BARCODE_ELEMENT_COUNT; i++) {
            decoder->runs[i - 1U] = decoder->runs[i];
        }
        decoder->count--;
    }
    decoder->runs[decoder->count] = ticks;
    decoder->count++;

    if ((!is_bar) || (decoder->count < BARCODE_ELEMENT_COUNT)) {
        return BARCODE_NO_RESULT;
    }
    if (decode_symbol(decoder->runs, decoded)) {
        return BARCODE_DECODED;
    }
    return BARCODE_NO_RESULT;
}
