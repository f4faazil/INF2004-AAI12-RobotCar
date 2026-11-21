/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * encoder.c
 * Wheel-encoder pulse counting.  See encoder.h.
 */
#include "encoder.h"
#include "robot_port.h"

#define ENC_FWD         ((int8_t)1)
#define ENC_REV         ((int8_t)-1)
#define ENC_NONE        ((int8_t)0)
#define ENC_BAD         ((int8_t)2)     /* two-bit jump: direction unknowable */
#define ENC_STATE_BITS  (2U)
#define ENC_STATE_MASK  (0x03U)

/*
 * Gray-code transition table, indexed by (previous AB << 2) | next AB.
 * Forward is 00 -> 01 -> 11 -> 10 -> 00.
 */
static const int8_t TRANSITION[16] = {
    /* prev 00 -> 00, 01, 10, 11 */ ENC_NONE, ENC_FWD,  ENC_REV,  ENC_BAD,
    /* prev 01 -> 00, 01, 10, 11 */ ENC_REV,  ENC_NONE, ENC_BAD,  ENC_FWD,
    /* prev 10 -> 00, 01, 10, 11 */ ENC_FWD,  ENC_BAD,  ENC_NONE, ENC_REV,
    /* prev 11 -> 00, 01, 10, 11 */ ENC_BAD,  ENC_REV,  ENC_FWD,  ENC_NONE
};

void encoder_init(encoder_t *enc, bool invert)
{
    enc->count = 0;
    enc->illegal = 0U;
    enc->last_edge_us = 0U;
    enc->last_ab = 0U;
    enc->invert = invert;
    enc->primed = false;
}

void encoder_update(encoder_t *enc, bool a, bool b, uint32_t now_us)
{
    uint8_t next = (uint8_t)((a ? 2U : 0U) | (b ? 1U : 0U));
    int8_t  step;

    if (!enc->primed) {
        enc->last_ab = next;
        enc->primed = true;
        return;
    }

    step = TRANSITION[((uint32_t)enc->last_ab << ENC_STATE_BITS) | next];
    enc->last_ab = (uint8_t)(next & ENC_STATE_MASK);

    if (step == ENC_NONE) {
        return;
    }
    if (step == ENC_BAD) {
        enc->illegal++;
        return;
    }
    if (enc->invert) {
        step = (int8_t)(-step);
    }
    enc->count += step;
    enc->last_edge_us = now_us;
}

void encoder_update_single(encoder_t *enc, int8_t direction, uint32_t now_us)
{
    enc->count += (direction >= 0) ? ENC_FWD : ENC_REV;
    enc->last_edge_us = now_us;
}

void encoder_snapshot(const encoder_t *enc, encoder_snapshot_t *snap)
{
    robot_irq_state_t state = robot_irq_save();

    snap->count = enc->count;
    snap->last_edge_us = enc->last_edge_us;
    snap->illegal = enc->illegal;
    robot_irq_restore(state);
}
