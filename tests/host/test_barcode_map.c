/*
 * test_barcode_map.c
 * Host tests for the MQTT letter -> movement mapping parser.
 */
#include <stdint.h>
#include <string.h>

#include "barcode_map.h"
#include "test_support.h"

static bmap_result_t parse(const char *text, nav_map_entry_t out[BMAP_MAX], uint8_t *count)
{
    *count = 99U;
    return bmap_parse((const uint8_t *)text, (uint16_t)strlen(text), out, count);
}

static nav_command_t lookup(const nav_map_entry_t *m, uint8_t n, char letter)
{
    uint8_t i;

    for (i = 0U; i < n; i++) {
        if (m[i].letter == letter) {
            return m[i].command;
        }
    }
    return NAV_CMD_NONE;
}

/* letter=action pairs with each spelling of each action. */
static void test_pairs_with_every_action_spelling(void)
{
    nav_map_entry_t m[BMAP_MAX];
    uint8_t n;

    CHECK_INT(BMAP_OK, parse("A=L,B=left,C=R,D=RIGHT,E=F,F=fwd,G=forward,H=S,I=straight,"
                             "J=U,K=uturn,L=U-turn", m, &n));
    CHECK_INT(12, n);
    CHECK_INT(NAV_CMD_LEFT, lookup(m, n, 'A'));
    CHECK_INT(NAV_CMD_LEFT, lookup(m, n, 'B'));
    CHECK_INT(NAV_CMD_RIGHT, lookup(m, n, 'C'));
    CHECK_INT(NAV_CMD_RIGHT, lookup(m, n, 'D'));
    CHECK_INT(NAV_CMD_STRAIGHT, lookup(m, n, 'E'));
    CHECK_INT(NAV_CMD_STRAIGHT, lookup(m, n, 'F'));
    CHECK_INT(NAV_CMD_STRAIGHT, lookup(m, n, 'G'));
    CHECK_INT(NAV_CMD_STRAIGHT, lookup(m, n, 'H'));
    CHECK_INT(NAV_CMD_STRAIGHT, lookup(m, n, 'I'));
    CHECK_INT(NAV_CMD_UTURN, lookup(m, n, 'J'));
    CHECK_INT(NAV_CMD_UTURN, lookup(m, n, 'K'));
    CHECK_INT(NAV_CMD_UTURN, lookup(m, n, 'L'));
}

/* Case, the ':' form and any mix of separators are accepted. */
static void test_case_colon_and_separators(void)
{
    nav_map_entry_t m[BMAP_MAX];
    uint8_t n;

    CHECK_INT(BMAP_OK, parse(" a:l ; b=R\n\tc=u,, d=f \r\n", m, &n));
    CHECK_INT(4, n);
    CHECK_INT(NAV_CMD_LEFT, lookup(m, n, 'A'));
    CHECK_INT(NAV_CMD_RIGHT, lookup(m, n, 'B'));
    CHECK_INT(NAV_CMD_UTURN, lookup(m, n, 'C'));
    CHECK_INT(NAV_CMD_STRAIGHT, lookup(m, n, 'D'));
}

/* A full A-Z mapping fits in one MQTT message (the payload limit is 128 bytes). */
static void test_all_26_pairs_fit_one_message(void)
{
    char text[200];
    size_t len = 0U;
    nav_map_entry_t m[BMAP_MAX];
    uint8_t n;
    unsigned i;
    static const char ACTION[4] = { 'L', 'R', 'F', 'U' };

    for (i = 0U; i < 26U; i++) {
        text[len++] = (char)('A' + i);
        text[len++] = '=';
        text[len++] = ACTION[i % 4U];
        text[len++] = ',';
    }
    text[len - 1U] = '\0';
    CHECK(strlen(text) <= 128U);
    CHECK_INT(BMAP_OK, parse(text, m, &n));
    CHECK_INT(26, n);
    CHECK_INT(NAV_CMD_LEFT, lookup(m, n, 'A'));
    CHECK_INT(NAV_CMD_RIGHT, lookup(m, n, 'B'));
    CHECK_INT(NAV_CMD_STRAIGHT, lookup(m, n, 'C'));
    CHECK_INT(NAV_CMD_UTURN, lookup(m, n, 'D'));
    CHECK_INT(NAV_CMD_RIGHT, lookup(m, n, 'Z'));
}

/* Exactly 26 action letters: A to Z in order. */
static void test_the_26_letter_form(void)
{
    nav_map_entry_t m[BMAP_MAX];
    uint8_t n;

    CHECK_INT(BMAP_OK, parse("LRFULRFULRFULRFULRFULRFULU", m, &n));
    CHECK_INT(26, n);
    CHECK_INT('A', m[0].letter);
    CHECK_INT(NAV_CMD_LEFT, m[0].command);
    CHECK_INT('F', m[5].letter);
    CHECK_INT(NAV_CMD_RIGHT, m[5].command);
    CHECK_INT('Z', m[25].letter);
    CHECK_INT(NAV_CMD_UTURN, m[25].command);
    CHECK_INT(BMAP_OK, parse("lrfulrfulrfulrfulrfulrfulu", m, &n));     /* any case */
    CHECK_INT(26, n);
    /* 25 or 27 letters are not a mapping */
    CHECK_INT(BMAP_ERR_SYNTAX, parse("LRFULRFULRFULRFULRFULRFUL", m, &n));
    CHECK_INT(BMAP_ERR_SYNTAX, parse("LRFULRFULRFULRFULRFULRFULUL", m, &n));
    CHECK_INT(BMAP_ERR_SYNTAX, parse("LRFULRFULRFULRFULRFULRFULX", m, &n));
}

/* CLEAR removes every mapping; it is a valid mapping of zero entries. */
static void test_clear_is_a_valid_empty_mapping(void)
{
    nav_map_entry_t m[BMAP_MAX];
    uint8_t n;

    CHECK_INT(BMAP_OK, parse("clear", m, &n));
    CHECK_INT(0, n);
    CHECK_INT(BMAP_OK, parse("  CLEAR \n", m, &n));
    CHECK_INT(0, n);
}

/* Anything wrong rejects the whole message and leaves *out_len alone. */
static void test_errors_reject_the_whole_message(void)
{
    nav_map_entry_t m[BMAP_MAX];
    uint8_t n;

    CHECK_INT(BMAP_ERR_EMPTY, parse("", m, &n));
    CHECK_INT(BMAP_ERR_EMPTY, parse(" ,; \n", m, &n));
    CHECK_INT(99, n);
    CHECK_INT(BMAP_ERR_SYNTAX, parse("A", m, &n));
    CHECK_INT(BMAP_ERR_SYNTAX, parse("A=", m, &n));
    CHECK_INT(BMAP_ERR_SYNTAX, parse("A-L", m, &n));
    CHECK_INT(BMAP_ERR_SYNTAX, parse("A = L", m, &n));        /* spaces separate entries */
    CHECK_INT(BMAP_ERR_SYNTAX, parse("A=L,junk", m, &n));
    CHECK_INT(BMAP_ERR_SYNTAX, parse("junk,A=L", m, &n));      /* junk first is not skipped */
    CHECK_INT(BMAP_ERR_SYNTAX, parse("CLEAR,A=L", m, &n));     /* CLEAR stands alone */
    CHECK_INT(BMAP_ERR_LETTER, parse("1=L", m, &n));
    CHECK_INT(BMAP_ERR_LETTER, parse("A=L,*=R", m, &n));
    CHECK_INT(BMAP_ERR_ACTION, parse("A=X", m, &n));
    CHECK_INT(BMAP_ERR_ACTION, parse("A=L,B=BACKWARDS", m, &n));
    CHECK_INT(BMAP_ERR_DUPLICATE, parse("A=L,B=R,A=U", m, &n));
    CHECK_INT(BMAP_ERR_DUPLICATE, parse("a=L,A=L", m, &n));
    CHECK_INT(99, n);
}

/* No parse reads past the buffer or overflows the table: 40 pairs is refused cleanly. */
static void test_too_many_or_oversized_input_is_safe(void)
{
    nav_map_entry_t m[BMAP_MAX];
    uint8_t n;
    char text[300];
    size_t len = 0U;
    unsigned i;

    for (i = 0U; i < 40U; i++) {
        text[len++] = (char)('A' + (i % 26U));
        text[len++] = '=';
        text[len++] = 'L';
        text[len++] = ',';
    }
    text[len] = '\0';
    CHECK(parse(text, m, &n) != BMAP_OK);                       /* a letter repeats */

    memset(text, 'A', sizeof(text) - 1U);
    text[sizeof(text) - 1U] = '\0';
    CHECK(parse(text, m, &n) != BMAP_OK);                       /* one huge token */
}

/* ---- the binary form ---------------------------------------------------------------------------------------------- */

/* The exact bytes: A=L,B=R,F=U is 12 01 0a 2c (type, then letter index << 3 | command).  Catches: a changed bit layout, which would silently
 * break the dashboard's encoder (its golden bytes are the same ones). */
static void test_binary_map_has_the_documented_bytes(void)
{
    const nav_map_entry_t in[3] = { { 'A', NAV_CMD_LEFT }, { 'B', NAV_CMD_RIGHT }, { 'F', NAV_CMD_UTURN } };
    const uint8_t expect[4] = { 0x12U, 0x01U, 0x0aU, 0x2cU };
    uint8_t out[32];

    CHECK_INT(4, bmap_encode_map(in, 3U, out, sizeof(out)));
    CHECK(memcmp(out, expect, sizeof(expect)) == 0);
    CHECK_INT(1, bmap_encode_map(in, 0U, out, sizeof(out)));           /* no letters: the one type byte clears the map */
    CHECK_INT(0, bmap_encode_map(in, 3U, out, 3U));                    /* capacity too small */
}

/* Encode then parse gives the same table, for every letter and every command. */
static void test_binary_round_trip_for_every_letter_and_command(void)
{
    uint8_t letter;
    uint8_t command;

    for (letter = 0U; letter < 26U; letter++) {
        for (command = (uint8_t)NAV_CMD_LEFT; command <= (uint8_t)NAV_CMD_UTURN; command++) {
            nav_map_entry_t in = { (char)('A' + letter), (nav_command_t)command };
            nav_map_entry_t out[BMAP_MAX];
            uint8_t wire[4];
            uint8_t count = 99U;

            CHECK_INT(2, bmap_encode_map(&in, 1U, wire, sizeof(wire)));
            CHECK_INT((int)BMAP_OK, (int)bmap_parse_binary(wire, 2U, out, &count));
            CHECK_INT(1, count);
            CHECK_INT(in.letter, out[0].letter);
            CHECK_INT((int)in.command, (int)out[0].command);
        }
    }
}

/* All 26 letters fit one message of 27 bytes; a 28th byte is refused. */
static void test_binary_full_alphabet_and_one_too_many(void)
{
    nav_map_entry_t in[BMAP_MAX];
    nav_map_entry_t out[BMAP_MAX];
    uint8_t wire[40];
    uint8_t count = 0U;
    unsigned i;

    for (i = 0U; i < BMAP_MAX; i++) {
        in[i].letter = (char)('A' + i);
        in[i].command = (nav_command_t)(1U + (i % 4U));
    }
    CHECK_INT(27, bmap_encode_map(in, (uint8_t)BMAP_MAX, wire, sizeof(wire)));
    CHECK_INT((int)BMAP_OK, (int)bmap_parse_binary(wire, 27U, out, &count));
    CHECK_INT(26, count);
    wire[27] = 0x01U;
    CHECK_INT((int)BMAP_ERR_SYNTAX, (int)bmap_parse_binary(wire, 28U, out, &count));
}

/* Every error refuses the whole message, like the text form. */
static void test_binary_errors(void)
{
    nav_map_entry_t out[BMAP_MAX];
    uint8_t count = 0U;
    const uint8_t wrong_type[3] = { 0x13U, 0x01U, 0x0aU };
    const uint8_t letter_26[2] = { 0x12U, (uint8_t)((26U << 3) | 1U) };   /* index 26 is not a letter */
    const uint8_t command_0[2] = { 0x12U, (uint8_t)((0U << 3) | 0U) };
    const uint8_t command_5[2] = { 0x12U, (uint8_t)((0U << 3) | 5U) };
    const uint8_t command_7[2] = { 0x12U, (uint8_t)((3U << 3) | 7U) };
    const uint8_t duplicate[3] = { 0x12U, 0x01U, 0x02U };                 /* A twice */
    const uint8_t only_type[1] = { 0x12U };

    CHECK_INT((int)BMAP_ERR_EMPTY, (int)bmap_parse_binary(only_type, 0U, out, &count));
    CHECK_INT((int)BMAP_ERR_SYNTAX, (int)bmap_parse_binary(wrong_type, 3U, out, &count));
    CHECK_INT((int)BMAP_ERR_LETTER, (int)bmap_parse_binary(letter_26, 2U, out, &count));
    CHECK_INT((int)BMAP_ERR_ACTION, (int)bmap_parse_binary(command_0, 2U, out, &count));
    CHECK_INT((int)BMAP_ERR_ACTION, (int)bmap_parse_binary(command_5, 2U, out, &count));
    CHECK_INT((int)BMAP_ERR_ACTION, (int)bmap_parse_binary(command_7, 2U, out, &count));
    CHECK_INT((int)BMAP_ERR_DUPLICATE, (int)bmap_parse_binary(duplicate, 3U, out, &count));
    count = 99U;
    CHECK_INT((int)BMAP_OK, (int)bmap_parse_binary(only_type, 1U, out, &count));    /* just the type: CLEAR */
    CHECK_INT(0, count);
}

/* The robot takes either form; a printable text map and the binary map give the same table. */
static void test_either_form_gives_the_same_table(void)
{
    nav_map_entry_t from_text[BMAP_MAX];
    nav_map_entry_t from_binary[BMAP_MAX];
    const uint8_t wire[4] = { 0x12U, 0x01U, 0x0aU, 0x2cU };
    uint8_t nt = 0U;
    uint8_t nb = 0U;

    CHECK_INT((int)BMAP_OK, (int)parse("A=L,B=R,F=U", from_text, &nt));
    CHECK_INT((int)BMAP_OK, (int)bmap_parse_any(wire, 4U, from_binary, &nb));
    CHECK_INT(nt, nb);
    {
        uint8_t k;

        for (k = 0U; (k < nt) && (k < nb); k++) {                       /* field by field: the structs have padding */
            CHECK_INT(from_text[k].letter, from_binary[k].letter);
            CHECK_INT((int)from_text[k].command, (int)from_binary[k].command);
        }
    }
    nb = 99U;
    CHECK_INT((int)BMAP_OK, (int)bmap_parse_any((const uint8_t *)"\x12", 1U, from_binary, &nb));   /* the type byte alone clears */
    CHECK_INT(0, nb);
    CHECK_INT((int)BMAP_ERR_EMPTY, (int)bmap_parse_any((const uint8_t *)"", 0U, from_binary, &nb));
}

/* The one-byte reply: result in bits 7 to 5, letter count in bits 4 to 0. */
static void test_binary_reply_bytes(void)
{
    uint8_t ack[2];

    CHECK_INT(2, bmap_encode_ack(BMAP_OK, 3U, ack));
    CHECK_INT(0x13, ack[0]);
    CHECK_INT(0x03, ack[1]);                                            /* OK, 3 letters */
    (void)bmap_encode_ack(BMAP_OK, 26U, ack);
    CHECK_INT(0x1a, ack[1]);                                            /* OK, 26 letters */
    (void)bmap_encode_ack(BMAP_ERR_DUPLICATE, 0U, ack);
    CHECK_INT(0xa0, ack[1]);                                            /* duplicate (5), no letters */
    (void)bmap_encode_ack(BMAP_ERR_SYNTAX, 0U, ack);
    CHECK_INT(0x40, ack[1]);                                            /* syntax (2) */
}

/* The stored byte is the wire byte; letters and commands outside the range cannot be packed. */
static void test_packed_entry_limits(void)
{
    uint8_t b = 0U;

    CHECK(nav_map_pack('A', NAV_CMD_LEFT, &b));
    CHECK_INT(0x01, b);
    CHECK(nav_map_pack('Z', NAV_CMD_UTURN, &b));
    CHECK_INT((25 << 3) | 4, b);
    CHECK_INT('Z', nav_map_letter(b));
    CHECK_INT((int)NAV_CMD_UTURN, (int)nav_map_command(b));
    CHECK(!nav_map_pack('a', NAV_CMD_LEFT, &b));                        /* lower case */
    CHECK(!nav_map_pack('1', NAV_CMD_LEFT, &b));
    CHECK(!nav_map_pack('A', NAV_CMD_NONE, &b));
    CHECK(!nav_map_pack('A', (nav_command_t)5, &b));
}

int main(void)
{
    RUN(test_pairs_with_every_action_spelling);
    RUN(test_case_colon_and_separators);
    RUN(test_all_26_pairs_fit_one_message);
    RUN(test_the_26_letter_form);
    RUN(test_clear_is_a_valid_empty_mapping);
    RUN(test_errors_reject_the_whole_message);
    RUN(test_too_many_or_oversized_input_is_safe);
    RUN(test_binary_map_has_the_documented_bytes);
    RUN(test_binary_round_trip_for_every_letter_and_command);
    RUN(test_binary_full_alphabet_and_one_too_many);
    RUN(test_binary_errors);
    RUN(test_either_form_gives_the_same_table);
    RUN(test_binary_reply_bytes);
    RUN(test_packed_entry_limits);
    return TEST_REPORT();
}
