/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * barcode_map.c
 * Parser for the MQTT letter -> movement mapping.  See barcode_map.h.
 */
#include "barcode_map.h"

#include <stdbool.h>
#include <stddef.h>

#define TOKEN_MAX   (12U)

static bool is_separator(uint8_t c)
{
    return (c == (uint8_t)',') || (c == (uint8_t)';') || (c == (uint8_t)' ') ||
           (c == (uint8_t)'\t') || (c == (uint8_t)'\r') || (c == (uint8_t)'\n') ||
           (c == 0U);
}

static char upper(uint8_t c)
{
    return ((c >= (uint8_t)'a') && (c <= (uint8_t)'z')) ? (char)(c - (uint8_t)'a' + (uint8_t)'A')
                                                          : (char)c;
}

static bool is_letter(char c)
{
    return (c >= 'A') && (c <= 'Z');
}

static bool same(const char *a, const char *b)
{
    while ((*a != '\0') && (*b != '\0') && (*a == *b)) {
        a++;
        b++;
    }
    return (*a == '\0') && (*b == '\0');
}

/* Action word (already upper-case, NUL-terminated) -> command. */
static bool action_of(const char *word, nav_command_t *command)
{
    if (same(word, "L") || same(word, "LEFT")) {
        *command = NAV_CMD_LEFT;
    } else if (same(word, "R") || same(word, "RIGHT")) {
        *command = NAV_CMD_RIGHT;
    } else if (same(word, "F") || same(word, "FWD") || same(word, "FORWARD") ||
               same(word, "S") || same(word, "STRAIGHT")) {
        *command = NAV_CMD_STRAIGHT;
    } else if (same(word, "U") || same(word, "UTURN") || same(word, "U-TURN")) {
        *command = NAV_CMD_UTURN;
    } else {
        return false;
    }
    return true;
}

/* Is every character of the token one of the single-letter actions? */
static bool is_action_string(const char *t, uint8_t n)
{
    uint8_t i;

    for (i = 0U; i < n; i++) {
        if ((t[i] != 'L') && (t[i] != 'R') && (t[i] != 'F') && (t[i] != 'S') && (t[i] != 'U')) {
            return false;
        }
    }
    return true;
}

/* Add letter=command, refusing a letter that is already there. */
static bmap_result_t add_entry(nav_map_entry_t out[BMAP_MAX], uint8_t *count, char letter,
                               nav_command_t command)
{
    uint8_t i;

    for (i = 0U; i < *count; i++) {
        if (out[i].letter == letter) {
            return BMAP_ERR_DUPLICATE;
        }
    }
    out[*count].letter = letter;
    out[*count].command = command;
    (*count)++;
    return BMAP_OK;
}

/*
 * Next token: skips separators, copies up to cap-1 upper-cased characters.  Returns its
 * real length (it can exceed what was copied); 0 when there are no more tokens.
 */
static uint16_t next_token(const uint8_t *payload, uint16_t len, uint16_t *pos, char *buf,
                           uint8_t cap)
{
    uint16_t n = 0U;

    while ((*pos < len) && is_separator(payload[*pos])) {
        (*pos)++;
    }
    while ((*pos < len) && !is_separator(payload[*pos])) {
        if (n < (uint16_t)(cap - 1U)) {
            buf[n] = upper(payload[*pos]);
        }
        n++;
        (*pos)++;
    }
    buf[(n < (uint16_t)(cap - 1U)) ? n : (uint16_t)(cap - 1U)] = '\0';
    return n;
}

/* One "L=ACTION" pair (token already upper-case, length n) into the table. */
static bmap_result_t add_pair(const char *token, uint16_t n, nav_map_entry_t out[BMAP_MAX],
                              uint8_t *count)
{
    nav_command_t command = NAV_CMD_NONE;

    if ((n < 3U) || (n > TOKEN_MAX) || ((token[1] != '=') && (token[1] != ':'))) {
        return BMAP_ERR_SYNTAX;
    }
    if (!is_letter(token[0])) {
        return BMAP_ERR_LETTER;
    }
    if (!action_of(&token[2], &command)) {
        return BMAP_ERR_ACTION;
    }
    return add_entry(out, count, token[0], command);
}

bmap_result_t bmap_parse(const uint8_t *payload, uint16_t len,
                         nav_map_entry_t out[BMAP_MAX], uint8_t *out_len)
{
    char token[BMAP_MAX + 2U];
    uint16_t pos = 0U;
    uint16_t n;
    uint8_t count = 0U;
    uint16_t second_pos;
    char extra[2];

    n = next_token(payload, len, &pos, token, (uint8_t)sizeof(token));
    if (n == 0U) {
        return BMAP_ERR_EMPTY;
    }

    second_pos = pos;
    if (next_token(payload, len, &second_pos, extra, (uint8_t)sizeof(extra)) == 0U) {
        /* a lone token: CLEAR, the 26-letter form, or a single pair */
        if (same(token, "CLEAR")) {
            *out_len = 0U;
            return BMAP_OK;
        }
        if ((n == BMAP_MAX) && is_action_string(token, (uint8_t)n)) {
            uint8_t i;

            for (i = 0U; i < BMAP_MAX; i++) {
                const char word[2] = { token[i], '\0' };
                nav_command_t command = NAV_CMD_NONE;

                (void)action_of(word, &command);
                out[i].letter = (char)('A' + i);
                out[i].command = command;
            }
            *out_len = BMAP_MAX;
            return BMAP_OK;
        }
    }

    /* otherwise every token is a letter=action pair */
    pos = 0U;
    for (;;) {
        bmap_result_t r;

        n = next_token(payload, len, &pos, token, (uint8_t)sizeof(token));
        if (n == 0U) {
            break;
        }
        r = add_pair(token, n, out, &count);
        if (r != BMAP_OK) {
            return r;
        }
    }
    *out_len = count;
    return BMAP_OK;
}

const char *bmap_result_text(bmap_result_t result)
{
    switch (result) {
    case BMAP_OK:
        return "OK";
    case BMAP_ERR_EMPTY:
        return "ERR empty";
    case BMAP_ERR_SYNTAX:
        return "ERR syntax";
    case BMAP_ERR_LETTER:
        return "ERR letter";
    case BMAP_ERR_ACTION:
        return "ERR action";
    case BMAP_ERR_DUPLICATE:
        return "ERR duplicate";
    default:
        return "ERR";
    }
}
