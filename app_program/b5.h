#ifndef B5_H
#define B5_H
#include <stdint.h>
#include <stdbool.h>
#include "b5_config.h"
/* This file is portable C11. Exactly one owner calls core functions.
 * For cross-task calls use b5_utk.h's protected mailboxes. */
enum { B5_MONITOR, B5_WAIT_STOP, B5_COARSE, B5_FINE,
       B5_READY, B5_BYPASS, B5_RECOVERY, B5_FAULT };
enum { B5_NONE, B5_STOP, B5_LEFT, B5_RIGHT, B5_RESUME };
enum { B5_MODE_NORMAL, B5_MODE_BYPASS, B5_MODE_RECOVERY, B5_MODE_RESET };
enum { B5_F_NO_ECHO=1u, B5_F_BAD_RANGE=2u, B5_F_FEEDBACK=4u,
       B5_F_STOP_TIMEOUT=8u, B5_F_SCAN=16u, B5_F_RECOVERY=32u,
       B5_F_NO_ROUTE=64u };
enum { B5_MEASURE_OK, B5_MEASURE_NO_ECHO, B5_MEASURE_BAD };
/* Logical fields use minimum widths; explicit byte packing is used on the wire.
 * C struct sizes are ABI-dependent and are NOT wire formats. */
typedef struct { unsigned mm:12, valid:1; } b5_range;
typedef struct {
    unsigned mode:2, stopped:1, motion_valid:1, line_valid:1, line_found:1;
    unsigned travel_mm:9; /* 0..500 mm, saturating at 500; cumulative path since BYPASS */
    uint32_t at_ms;         /* platform monotonic time; wrap handled */
} b5_feedback;
typedef struct {
    unsigned state:3, request:3, faults:7, slot:5, count:5;
    b5_range front, closest, width, left, right;
    uint32_t at_ms;
    uint32_t range_ms; /* time latest forward reading was consumed */
} b5_status;
#define B5_STATUS_BYTES 19u
#define B5_FEEDBACK_BYTES 6u
#define B5_SCAN_SLOTS 25u
#define B5_SCAN_BYTES 41u
/* HAL start returns false if busy/unavailable. poll returns once per attempt.
 * servo(slot) sets 30+5*slot degrees, never a raw unbounded pulse. */
typedef struct {
    void (*servo)(uint8_t slot);
    bool (*start)(void);
    bool (*poll)(uint16_t *mm, uint8_t *result);
    void (*cancel)(void);
} b5_hal;
typedef struct {
    b5_hal hw;
    b5_status s;
    uint8_t scan[B5_SCAN_BYTES]; /* 25 consecutive 13-bit samples; top 3 bits zero */
    uint32_t phase_ms, servo_ms, ping_ms, recovery_ms;
    unsigned waiting:1, retries:2, first:5, last:5;
} b5_core;
void b5_init(b5_core *b, b5_hal hw, uint32_t now);
void b5_step(b5_core *b, const b5_feedback *f, uint32_t now);
void b5_encode_status(const b5_status *s, uint8_t out[B5_STATUS_BYTES]);
bool b5_decode_status(const uint8_t in[B5_STATUS_BYTES], b5_status *s);
void b5_encode_feedback(const b5_feedback *f, uint8_t out[B5_FEEDBACK_BYTES]);
bool b5_decode_feedback(const uint8_t in[B5_FEEDBACK_BYTES], b5_feedback *f);
b5_range b5_scan_get(const b5_core *b, unsigned slot);
#endif
