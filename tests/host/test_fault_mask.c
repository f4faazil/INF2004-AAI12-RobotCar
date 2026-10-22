/*
 * test_fault_mask.c
 * Host tests for the shared fault bitmask.  robot_irq_save/restore are
 * counting stubs: they let the test prove every update is wrapped in a
 * balanced critical section, which is the Tribunal's protection rule for a
 * read-modify-write on a Cortex-M0+.
 */
#include <stddef.h>
#include <stdint.h>

#include "fault_mask.h"
#include "robot_port.h"
#include "test_support.h"

static int g_depth = 0;
static int g_saves = 0;
static int g_max_depth = 0;

robot_irq_state_t robot_irq_save(void)
{
    g_saves++;
    g_depth++;
    if (g_depth > g_max_depth) {
        g_max_depth = g_depth;
    }
    return (robot_irq_state_t)(g_depth - 1);
}

void robot_irq_restore(robot_irq_state_t state)
{
    g_depth--;
    CHECK_INT((long)state, g_depth);    /* restores in the order they were saved */
}

/* Setting one fault must leave every other raised fault alone.  Catches: an
 * assignment (=) instead of OR, which would silently drop a fault. */
static void test_set_keeps_other_faults(void)
{
    fault_mask_t m;

    fault_mask_init(&m);
    fault_set(&m, FAULT_IMU);
    fault_set(&m, FAULT_ENCODER);
    CHECK_INT(FAULT_IMU | FAULT_ENCODER, fault_read(&m));
}

/* Clearing one fault must not clear its neighbours.  Catches: clearing the
 * whole mask, or clearing with the wrong polarity. */
static void test_clear_only_clears_named_bits(void)
{
    fault_mask_t m;

    fault_mask_init(&m);
    fault_set(&m, FAULT_IMU | FAULT_ENCODER | FAULT_LINE_STALE);
    fault_clear(&m, FAULT_ENCODER);
    CHECK_INT(FAULT_IMU | FAULT_LINE_STALE, fault_read(&m));
}

/* A freshly initialised mask holds no faults. */
static void test_init_has_no_faults(void)
{
    fault_mask_t m;

    m.bits = 0xFFFFU;
    fault_mask_init(&m);
    CHECK_INT(0, fault_read(&m));
    CHECK(!fault_stop_required(&m));
}

/* Only the stop class stops the car: informational faults are reported but
 * must not trigger a stop.  Catches: telemetry loss halting navigation
 * (Tribunal: network failure must not stop the mission) and an IMU failure
 * stopping the car (Tribunal: the IMU degrades to "hump N/A"). */
static void test_only_stop_class_faults_require_a_stop(void)
{
    fault_mask_t m;

    fault_mask_init(&m);
    fault_set(&m, FAULT_NET_DOWN | FAULT_TELEMETRY_DROP | FAULT_BARCODE_REJECT |
                  FAULT_LINE_OVERFLOW | FAULT_IMU);
    CHECK(!fault_stop_required(&m));

    fault_set(&m, FAULT_NO_SAFE_PATH);
    CHECK(fault_stop_required(&m));
}

/* Every stop-class fault individually requires a stop.  Catches: a bit left
 * out of FAULT_STOP_MASK when a new fault is added. */
static void test_every_stop_fault_requires_a_stop(void)
{
    static const fault_bits_t STOP_FAULTS[] = {
        FAULT_LINE_STALE, FAULT_LINE_CAL, FAULT_LINE_LOST, FAULT_ENCODER,
        FAULT_MOTION, FAULT_ULTRASONIC, FAULT_NO_SAFE_PATH,
        FAULT_RECOVERY_LIMIT
    };
    size_t i;

    for (i = 0U; i < (sizeof(STOP_FAULTS) / sizeof(STOP_FAULTS[0])); i++) {
        fault_mask_t m;

        fault_mask_init(&m);
        fault_set(&m, STOP_FAULTS[i]);
        CHECK(fault_stop_required(&m));
    }
}

/* Every update runs inside one balanced critical section.  Catches: an
 * unprotected read-modify-write (lost-update race) or a missing restore that
 * would leave interrupts masked. */
static void test_updates_are_inside_a_balanced_critical_section(void)
{
    fault_mask_t m;

    g_depth = 0;
    g_saves = 0;
    g_max_depth = 0;
    fault_mask_init(&m);
    fault_set(&m, FAULT_IMU);
    fault_clear(&m, FAULT_IMU);
    CHECK_INT(0, g_depth);
    CHECK(g_saves >= 2);
    CHECK_INT(1, g_max_depth);     /* never nests: the section stays short */
}

int main(void)
{
    RUN(test_set_keeps_other_faults);
    RUN(test_clear_only_clears_named_bits);
    RUN(test_init_has_no_faults);
    RUN(test_only_stop_class_faults_require_a_stop);
    RUN(test_every_stop_fault_requires_a_stop);
    RUN(test_updates_are_inside_a_balanced_critical_section);
    return TEST_REPORT();
}
