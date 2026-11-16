/*
 * test_port.h
 * Counting stubs for the platform services in robot_port.h, shared by the
 * host tests of modules that take a critical section.
 */
#ifndef TEST_PORT_H
#define TEST_PORT_H

#include "robot_port.h"

static int g_port_depth = 0;
static int g_port_sections = 0;

robot_irq_state_t robot_irq_save(void)
{
    g_port_sections++;
    g_port_depth++;
    return (robot_irq_state_t)(g_port_depth - 1);
}

void robot_irq_restore(robot_irq_state_t state)
{
    (void)state;
    g_port_depth--;
}

#endif /* TEST_PORT_H */
