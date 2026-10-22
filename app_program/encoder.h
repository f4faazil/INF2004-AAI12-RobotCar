/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * encoder.h
 * Wheel-encoder pulse counting (Buddy 2).  Quadrature decoding at 4x: every
 * edge of either channel is one count, signed by direction.  Written to be
 * called from a GPIO interrupt handler: encoder_update() does a table lookup
 * and an add, nothing more.
 *
 * Whether the car's encoders are really two-channel quadrature is still
 * unverified (Tribunal assumption A2), so a single-channel fallback is
 * provided: the caller supplies the direction it is commanding.  That
 * fallback cannot detect externally forced reverse rotation.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    volatile int32_t  count;        /* signed edges; + is the forward direction */
    volatile uint32_t illegal;      /* two-bit jumps: a missed edge or noise */
    volatile uint32_t last_edge_us; /* timestamp of the most recent counted edge */
    uint8_t           last_ab;      /* previous 2-bit A/B state */
    bool              invert;       /* swap forward/reverse (wiring dependent) */
    bool              primed;       /* last_ab holds a real reading */
} encoder_t;

/* A consistent copy of one encoder: count, timestamp and health together. */
typedef struct {
    int32_t  count;
    uint32_t last_edge_us;
    uint32_t illegal;
} encoder_snapshot_t;

void encoder_init(encoder_t *enc, bool invert);

/*
 * Quadrature update: call on every edge of A or B with both pin levels.  The
 * first call only records the starting state.
 */
void encoder_update(encoder_t *enc, bool a, bool b, uint32_t now_us);

/* Single-channel update: call on every edge with the commanded direction
 * (+1 forward, -1 reverse). */
void encoder_update_single(encoder_t *enc, int8_t direction, uint32_t now_us);

/*
 * Protected snapshot of count, timestamp and illegal-transition count taken
 * together (Tribunal Version C: an aligned-word read is safe for one value
 * only, a multi-field snapshot can be torn by an interrupt).
 */
void encoder_snapshot(const encoder_t *enc, encoder_snapshot_t *snap);

#endif /* ENCODER_H */
