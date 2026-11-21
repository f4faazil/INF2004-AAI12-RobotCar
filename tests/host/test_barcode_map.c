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

int main(void)
{
    RUN(test_pairs_with_every_action_spelling);
    RUN(test_case_colon_and_separators);
    RUN(test_all_26_pairs_fit_one_message);
    RUN(test_the_26_letter_form);
    RUN(test_clear_is_a_valid_empty_mapping);
    RUN(test_errors_reject_the_whole_message);
    RUN(test_too_many_or_oversized_input_is_safe);
    return TEST_REPORT();
}
