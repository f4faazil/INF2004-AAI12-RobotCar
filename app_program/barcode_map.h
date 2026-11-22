/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * barcode_map.h
 * Parser for the letter -> movement mapping that the PC sends over MQTT
 * (topic <base>/map).  Pure C, no hardware.  The car has no built-in mapping:
 * until one arrives every decoded letter is rejected.
 *
 * BINARY form (what the dashboard sends): the type byte TEL_TYPE_MAP (0x12) then ONE BYTE PER LETTER, the number of letters being the length minus 1
 * (no letters at all removes every mapping).  Each byte holds the letter index A = 0 to Z = 25 in bits 7 to 3 and the movement in bits 2 to 0
 * (left 1, right 2, straight 3, u-turn 4), exactly how the map is stored (nav_map_pack): A=L,B=R,F=U is the 4 bytes 12 01 0a 2c.
 * The car answers with TEL_TYPE_MAP_ACK and ONE byte: the result in bits 7 to 5 (bmap_result_t) and the number of letters in bits 4 to 0.
 *
 * TEXT form (kept for bench use with mosquitto_pub; the dashboard no longer sends it).  Accepted text payloads (case-insensitive; entries separated by comma, semicolon,
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
#include "telemetry.h"           /* TEL_TYPE_MAP and TEL_TYPE_MAP_ACK */

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

/*
 * The binary form.  A payload that does not start with TEL_TYPE_MAP is a syntax error.  Duplicates, a letter index above 25 and a command outside
 * 1 to 4 are refused whole, like the text form.  On BMAP_OK, out[0..*out_len) holds the entries.
 */
bmap_result_t bmap_parse_binary(const uint8_t *payload, uint16_t len,
                                nav_map_entry_t out[BMAP_MAX], uint8_t *out_len);

/* Binary if the payload starts with TEL_TYPE_MAP (0x12 is not a printable character, so the two forms cannot be confused), otherwise text. */
bmap_result_t bmap_parse_any(const uint8_t *payload, uint16_t len,
                             nav_map_entry_t out[BMAP_MAX], uint8_t *out_len);

/* Build the binary map payload (1 + n bytes).  Returns the length, or 0 if an entry cannot be packed or capacity is too small. */
uint16_t bmap_encode_map(const nav_map_entry_t *entries, uint8_t n, uint8_t *out, uint16_t capacity);

/* The 2-byte binary reply: TEL_TYPE_MAP_ACK, then the result (3 bits) and the number of letters (5 bits).  Returns 2. */
uint16_t bmap_encode_ack(bmap_result_t result, uint8_t count, uint8_t out[2]);

/* Short text for an acknowledgement message. */
const char *bmap_result_text(bmap_result_t result);

#endif /* BARCODE_MAP_H */
