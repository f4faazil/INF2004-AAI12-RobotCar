/*
 * test_encoder.c
 * Host tests for quadrature decoding.  The A/B sequences are the standard
 * Gray-code cycle 00 -> 01 -> 11 -> 10 -> 00 for one direction and its
 * reverse for the other.
 */
#include <stddef.h>
#include <stdint.h>

#include "encoder.h"
#include "test_port.h"
#include "test_support.h"

/* Apply a list of 2-bit AB states (A is bit 1, B is bit 0). */
static void feed(encoder_t *e, const uint8_t *states, size_t n)
{
    size_t i;

    for (i = 0U; i < n; i++) {
        encoder_update(e, (states[i] & 2U) != 0U, (states[i] & 1U) != 0U,
                       (uint32_t)(i * 100U));
    }
}

/* One full forward electrical cycle is four counts.  Catches: counting only
 * rising edges (1x/2x) or counting the wrong sign. */
static void test_forward_cycle_counts_four(void)
{
    encoder_t e;
    const uint8_t seq[] = { 0U, 1U, 3U, 2U, 0U };   /* start state + 4 edges */

    encoder_init(&e, false);
    feed(&e, seq, sizeof(seq));
    CHECK_INT(4, e.count);
    CHECK_INT(0, e.illegal);
}

/* The reverse cycle counts down.  Catches: a table that cannot go negative
 * (direction feedback is needed for reverse moves and U-turns). */
static void test_reverse_cycle_counts_minus_four(void)
{
    encoder_t e;
    const uint8_t seq[] = { 0U, 2U, 3U, 1U, 0U };

    encoder_init(&e, false);
    feed(&e, seq, sizeof(seq));
    CHECK_INT(-4, e.count);
}

/* Wiring can swap A and B; invert flips the sign without changing the table.
 * Catches: a motor that "moves backwards" being unfixable without rewiring. */
static void test_invert_flips_direction(void)
{
    encoder_t e;
    const uint8_t seq[] = { 0U, 1U, 3U, 2U, 0U };

    encoder_init(&e, true);
    feed(&e, seq, sizeof(seq));
    CHECK_INT(-4, e.count);
}

/* A repeated state is not an edge.  Catches: counting on every interrupt even
 * when nothing changed (contact bounce / double ISR). */
static void test_repeated_state_is_ignored(void)
{
    encoder_t e;
    const uint8_t seq[] = { 0U, 1U, 1U, 1U, 3U };

    encoder_init(&e, false);
    feed(&e, seq, sizeof(seq));
    CHECK_INT(2, e.count);
}

/* A two-bit jump (00 -> 11) means an edge was missed.  It must not be guessed
 * as +2 or -2 (the direction is unknowable); it is counted as illegal and the
 * count is left alone.  Catches: silently inventing distance. */
static void test_two_bit_jump_is_illegal_and_not_counted(void)
{
    encoder_t e;
    const uint8_t seq[] = { 0U, 3U };

    encoder_init(&e, false);
    feed(&e, seq, sizeof(seq));
    CHECK_INT(0, e.count);
    CHECK_INT(1, e.illegal);
}

/* The first reading only primes the state: starting at AB=11 must not count
 * a spurious edge from the power-up default of 00. */
static void test_first_reading_only_primes_state(void)
{
    encoder_t e;

    encoder_init(&e, false);
    encoder_update(&e, true, true, 10U);
    CHECK_INT(0, e.count);
    CHECK_INT(0, e.illegal);
}

/* The count is a signed 32-bit value and passes through zero cleanly in both
 * directions (the packed motion distance later saturates, this does not). */
static void test_count_crosses_zero_both_ways(void)
{
    encoder_t e;
    const uint8_t rev[] = { 0U, 2U, 3U, 1U, 0U, 2U };    /* 5 edges back: -5 */
    const uint8_t fwd[] = { 0U, 1U, 3U, 2U, 0U, 1U, 3U, 2U };  /* 8 edges on */

    encoder_init(&e, false);
    feed(&e, rev, sizeof(rev));
    CHECK_INT(-5, e.count);
    feed(&e, fwd, sizeof(fwd));
    CHECK_INT(3, e.count);
}

/* The timestamp follows the last counted edge, not repeated states. */
static void test_timestamp_tracks_counted_edges(void)
{
    encoder_t e;

    encoder_init(&e, false);
    encoder_update(&e, false, false, 100U);
    encoder_update(&e, false, true, 200U);
    encoder_update(&e, false, true, 300U);      /* repeat: not an edge */
    CHECK_INT(200, e.last_edge_us);
}

/* Single-channel fallback counts in the commanded direction.  Catches: the
 * fallback ignoring direction (reverse moves would add distance). */
static void test_single_channel_uses_commanded_direction(void)
{
    encoder_t e;

    encoder_init(&e, false);
    encoder_update_single(&e, 1, 10U);
    encoder_update_single(&e, 1, 20U);
    encoder_update_single(&e, -1, 30U);
    CHECK_INT(1, e.count);
    CHECK_INT(30, e.last_edge_us);
}

/* The snapshot copies count, timestamp and illegal count inside one
 * critical section.  Catches: reading the fields unprotected (torn). */
static void test_snapshot_is_taken_inside_a_critical_section(void)
{
    encoder_t e;
    encoder_snapshot_t s;
    const uint8_t seq[] = { 0U, 1U, 3U };

    encoder_init(&e, false);
    feed(&e, seq, sizeof(seq));
    g_port_sections = 0;
    encoder_snapshot(&e, &s);
    CHECK_INT(2, s.count);
    CHECK_INT(200, s.last_edge_us);
    CHECK_INT(0, s.illegal);
    CHECK_INT(1, g_port_sections);
    CHECK_INT(0, g_port_depth);
}

int main(void)
{
    RUN(test_forward_cycle_counts_four);
    RUN(test_reverse_cycle_counts_minus_four);
    RUN(test_invert_flips_direction);
    RUN(test_repeated_state_is_ignored);
    RUN(test_two_bit_jump_is_illegal_and_not_counted);
    RUN(test_first_reading_only_primes_state);
    RUN(test_count_crosses_zero_both_ways);
    RUN(test_timestamp_tracks_counted_edges);
    RUN(test_single_channel_uses_commanded_direction);
    RUN(test_snapshot_is_taken_inside_a_critical_section);
    return TEST_REPORT();
}
