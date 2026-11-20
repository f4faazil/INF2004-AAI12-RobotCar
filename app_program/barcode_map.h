/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * barcode_map.h
 * Parser for the letter -> movement mapping that the PC sends over MQTT
 * (topic <base>/map).  Pure C, no hardware.  The car has no built-in mapping:
 * until one arrives every decoded letter is rejected.
 *
 * Accepted payloads (case-insensitive; entries separated by comma, semicolon,
 * space, tab or newline):
 *
 *   "A=L,B=R,C=F,D=U"      letter=action pairs ('=' or ':').  Actions:
 *                            L / LEFT                      turn left
 *                            R / RIGHT                     turn right
 *                            F / FWD / FORWARD / S / STRAIGHT   go straight on
 *                            U / UTURN / U-TURN            U-turn (to the right)
 *   "LRFULRFU..."          exactly 26 action letters (L R F S U), for A to Z in order
 *   "CLEAR"                remove every mapping
 *
 * Each message REPLACES the whole mapping: letters not named are unmapped.  A
 * message with any error is rejected as a whole and the old mapping is kept.
 */
#ifndef BARCODE_MAP_H
#define BARCODE_MAP_H

#include <stdint.h>

#include "line_nav.h"

#define BMAP_MAX    (26U)

typedef enum {
    BMAP_OK = 0,
    BMAP_ERR_EMPTY,         /* nothing in the message */
    BMAP_ERR_SYNTAX,        /* an entry is not letter=action */
    BMAP_ERR_LETTER,        /* not a letter A to Z */
    BMAP_ERR_ACTION,        /* not a known action */
    BMAP_ERR_DUPLICATE      /* the same letter twice */
} bmap_result_t;

/*
 * Parse a payload.  On BMAP_OK, out[0..*out_len) holds the new mapping (0 entries
 * for "CLEAR").  On any error nothing is written to out_len and out is undefined.
 */
bmap_result_t bmap_parse(const uint8_t *payload, uint16_t len,
                         nav_map_entry_t out[BMAP_MAX], uint8_t *out_len);

/* Short text for an acknowledgement message. */
const char *bmap_result_text(bmap_result_t result);

#endif /* BARCODE_MAP_H */
