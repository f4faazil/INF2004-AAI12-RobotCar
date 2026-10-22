/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * robot_port.h
 * The few platform services the portable robot modules need.  On the car they
 * are provided by hal_rp2040.c; the host tests provide counting stubs.  Keeping
 * them behind this header is what lets every control module run unchanged in
 * a host unit test.
 */
#ifndef ROBOT_PORT_H
#define ROBOT_PORT_H

#include <stdint.h>

typedef uint32_t robot_irq_state_t;

/*
 * Short critical section: masks interrupts and returns the previous state, so
 * calls nest.  The Cortex-M0+ has no atomic read-modify-write, so every
 * update of data shared between tasks and interrupt handlers goes through
 * this pair.  Keep the protected region to a few instructions.
 */
robot_irq_state_t robot_irq_save(void);
void robot_irq_restore(robot_irq_state_t state);

#endif /* ROBOT_PORT_H */
