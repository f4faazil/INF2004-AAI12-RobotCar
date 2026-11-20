/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * barcode.h
 * Code 39 decoder for the course track barcodes ("*A*", "*Z*", ...).
 *
 * The decoder is fed the widths of alternating bars and spaces, as measured
 * by the line sensor, and reports a character when a whole "*X*" symbol has
 * been seen, scanned in either direction.  It works on width ratios only, so it needs no knowledge of the
 * car's speed or the printed scale.  It has no kernel or hardware
 * dependencies.
 */
#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>

/* "*X*": 3 characters x 9 elements, plus 2 inter-character gaps. */
#define BARCODE_ELEMENT_COUNT   (29U)

typedef enum {
    BARCODE_NO_RESULT = 0,
    BARCODE_DECODED
} barcode_result_t;

typedef struct {
    uint16_t runs[BARCODE_ELEMENT_COUNT];   /* most recent runs, oldest first */
    uint8_t  count;                         /* number of valid entries in runs */
} barcode_decoder_t;

void barcode_init(barcode_decoder_t *decoder);

/*
 * Push the next run.  Runs must alternate bar / space / bar ...; the caller
 * guarantees that.  A decode can only complete on a bar.  Returns
 * BARCODE_DECODED and stores the data character in *decoded when the most
 * recent 29 runs form a valid symbol; otherwise BARCODE_NO_RESULT.
 */
barcode_result_t barcode_push_run(barcode_decoder_t *decoder,
                                  bool is_bar,
                                  uint16_t ticks,
                                  char *decoded);

#endif /* BARCODE_H */
