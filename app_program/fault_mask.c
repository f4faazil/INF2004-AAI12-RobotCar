/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * fault_mask.c
 * Shared fault bitmask.  See fault_mask.h.
 */
#include "fault_mask.h"
#include "robot_port.h"

void fault_mask_init(fault_mask_t *mask)
{
    mask->bits = 0U;
}

void fault_set(fault_mask_t *mask, fault_bits_t bits)
{
    robot_irq_state_t state = robot_irq_save();

    mask->bits = (fault_bits_t)(mask->bits | bits);
    robot_irq_restore(state);
}

void fault_clear(fault_mask_t *mask, fault_bits_t bits)
{
    robot_irq_state_t state = robot_irq_save();

    mask->bits = (fault_bits_t)(mask->bits & (fault_bits_t)(~bits));
    robot_irq_restore(state);
}

fault_bits_t fault_read(const fault_mask_t *mask)
{
    return mask->bits;      /* a single aligned halfword read is atomic */
}

bool fault_stop_required(const fault_mask_t *mask)
{
    return (fault_read(mask) & FAULT_STOP_MASK) != 0U;
}
