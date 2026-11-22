/*
 * test_fault_mask_stress.c
 *
 * Concurrency stress test of the shared fault bitmask (fault_mask.c): the Tribunal 1 test "Shared fault mask
 * (concurrency and safety)".  Several threads stand in for the subsystems (line, encoder, IMU, ultrasonic, motion,
 * network), each setting and clearing its OWN two fault bits as fast as it can, while a monitor thread watches a STOP-class
 * fault that must never disappear.
 *
 * What it checks
 *   1. Lost update: after fault_set(b) the bit b reads set, after fault_clear(b) it reads clear, although every other
 *      thread is read-modify-writing the same word at the same time.
 *   2. Unrelated bits stay unchanged: the final mask equals the expected mask exactly.
 *   3. STOP precedence: fault_stop_required() is true at every sample while the sticky STOP-class fault is raised.
 *
 * The critical section (robot_irq_save/restore) is a mutex here.  On the car it is the kernel spinlock plus interrupt
 * disable on the SMP build (hal_rp2040.c), which gives the same mutual exclusion between two cores and an interrupt handler.
 *
 * NEGATIVE CONTROL: built with -DUNPROTECTED the critical section is empty.  The same test must then report lost updates;
 * if it does not, the test is not able to detect a missing protection and its pass proves nothing.
 *
 *   protected build:   exit 0 when there are no violations
 *   -DUNPROTECTED:     exit 0 when violations were provoked (the control works), exit 1 when none were (inconclusive)
 */
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "fault_mask.h"
#include "robot_port.h"

#ifndef ITERATIONS
#define ITERATIONS 300000UL
#endif
#define WRITERS 6

#ifdef UNPROTECTED
#define MODE_NAME "UNPROTECTED (negative control)"
robot_irq_state_t robot_irq_save(void)
{
    return 0U;
}
void robot_irq_restore(robot_irq_state_t state)
{
    (void)state;
}
#else
#define MODE_NAME "PROTECTED (critical section)"
static pthread_mutex_t g_section = PTHREAD_MUTEX_INITIALIZER;

robot_irq_state_t robot_irq_save(void)
{
    pthread_mutex_lock(&g_section);
    return 0U;
}
void robot_irq_restore(robot_irq_state_t state)
{
    (void)state;
    pthread_mutex_unlock(&g_section);
}
#endif

static fault_mask_t g_mask;
static volatile int g_stop_workers;

/* Two bits per writer: 12 of the 14 defined faults, leaving FAULT_IMU_LIFT for the sticky STOP-class fault. */
static const fault_bits_t WRITER_BITS[WRITERS] = {
    (fault_bits_t)(FAULT_LINE_STALE | FAULT_LINE_CAL),
    (fault_bits_t)(FAULT_LINE_LOST | FAULT_ENCODER),
    (fault_bits_t)(FAULT_MOTION | FAULT_IMU),
    (fault_bits_t)(FAULT_ULTRASONIC | FAULT_NO_SAFE_PATH),
    (fault_bits_t)(FAULT_RECOVERY_LIMIT | FAULT_BARCODE_REJECT),
    (fault_bits_t)(FAULT_LINE_OVERFLOW | FAULT_TELEMETRY_DROP)
};
#define STICKY FAULT_IMU_LIFT

typedef struct {
    int id;
    unsigned long lost_set;
    unsigned long lost_clear;
    unsigned long operations;
} writer_t;

static writer_t g_writer[WRITERS];
static volatile unsigned long g_monitor_samples;
static volatile unsigned long g_stop_breaches;
static volatile unsigned long g_sticky_breaches;

static void *writer_main(void *arg)
{
    writer_t *w = (writer_t *)arg;
    const fault_bits_t own = WRITER_BITS[w->id];
    unsigned long i;

    for (i = 0; i < ITERATIONS; i++) {
        fault_set(&g_mask, own);
        if ((fault_read(&g_mask) & own) != own) {
            w->lost_set++;                      /* another thread's stale write-back erased our bits */
        }
        fault_clear(&g_mask, own);
        if ((fault_read(&g_mask) & own) != 0U) {
            w->lost_clear++;                    /* another thread's stale write-back restored our bits */
        }
        w->operations += 2U;
    }
    return NULL;
}

static void *monitor_main(void *arg)
{
    (void)arg;
    while (!g_stop_workers) {
        g_monitor_samples++;
        if (!fault_stop_required(&g_mask)) {
            g_stop_breaches++;                  /* STOP precedence broken */
        }
        if ((fault_read(&g_mask) & STICKY) == 0U) {
            g_sticky_breaches++;                /* a raised fault vanished */
        }
    }
    return NULL;
}

static double seconds_now(void)
{
    struct timespec t;

    timespec_get(&t, TIME_UTC);
    return (double)t.tv_sec + ((double)t.tv_nsec / 1e9);
}

int main(void)
{
    pthread_t th[WRITERS];
    pthread_t mon;
    unsigned long lost_set = 0;
    unsigned long lost_clear = 0;
    unsigned long ops = 0;
    unsigned long violations;
    double t0;
    double t1;
    int i;
    fault_bits_t expected = STICKY;
    fault_bits_t actual;

    fault_mask_init(&g_mask);
    fault_set(&g_mask, STICKY);                 /* a STOP-class fault that stays raised for the whole run */

    printf("fault mask stress test: %s\n", MODE_NAME);
    printf("  %d writer threads x %lu set/clear pairs, plus a monitor thread\n", WRITERS, ITERATIONS);

    t0 = seconds_now();
    for (i = 0; i < WRITERS; i++) {
        g_writer[i].id = i;
        pthread_create(&th[i], NULL, writer_main, &g_writer[i]);
    }
    pthread_create(&mon, NULL, monitor_main, NULL);
    for (i = 0; i < WRITERS; i++) {
        pthread_join(th[i], NULL);
    }
    g_stop_workers = 1;
    pthread_join(mon, NULL);
    t1 = seconds_now();

    for (i = 0; i < WRITERS; i++) {
        lost_set += g_writer[i].lost_set;
        lost_clear += g_writer[i].lost_clear;
        ops += g_writer[i].operations;
        printf("  writer %d bits=0x%04x  lost-set=%lu lost-clear=%lu\n", i, (unsigned)WRITER_BITS[i],
               g_writer[i].lost_set, g_writer[i].lost_clear);
    }
    actual = fault_read(&g_mask);
    violations = lost_set + lost_clear + g_stop_breaches + g_sticky_breaches + ((actual != expected) ? 1UL : 0UL);

    printf("  expected final mask 0x%04x, actual 0x%04x  (%s)\n", (unsigned)expected, (unsigned)actual,
           (actual == expected) ? "match" : "MISMATCH");
    printf("  lost updates: set=%lu clear=%lu of %lu operations\n", lost_set, lost_clear, ops);
    printf("  STOP precedence: %lu samples, %lu breaches; sticky fault vanished %lu times\n", g_monitor_samples,
           g_stop_breaches, g_sticky_breaches);
    printf("  elapsed %.3f s = %.0f ns per update under contention (HOST timing, not the Cortex-M0+)\n", t1 - t0,
           ((t1 - t0) * 1e9) / (double)ops);

#ifdef UNPROTECTED
    if (violations > 0UL) {
        printf("RESULT: control works: the unprotected update loses %lu updates, so a pass of the protected build is meaningful\n",
               violations);
        return 0;
    }
    printf("RESULT: INCONCLUSIVE: the unprotected build lost nothing; run again or raise ITERATIONS\n");
    return 1;
#else
    if (violations == 0UL) {
        printf("RESULT: PASS: no lost update, no unrelated bit changed, STOP precedence held\n");
        return 0;
    }
    printf("RESULT: FAIL: %lu violations\n", violations);
    return 1;
#endif
}
