/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * fault_mask.h
 * The shared fault bitmask (Tribunal Version B/C): one bit per fault, updated
 * only through this protected helper because the Cortex-M0+ has no atomic
 * bit update. A raised fault stays recorded until an authorised caller clears
 * it, and unrelated bits are never disturbed. The IMU task may clear
 * FAULT_IMU_LIFT only after its configured safe-rearm condition is satisfied.
 */
#ifndef FAULT_MASK_H
#define FAULT_MASK_H

#include <stdbool.h>
#include <stdint.h>

typedef uint16_t fault_bits_t;

/* Bit positions are part of the telemetry format (telemetry.h), so append,
 * never renumber.  Stop-class faults stop the car; informational faults are
 * reported only.  IMU read failures are informational ("hump N/A"), while a
 * sustained lift/overturn indication is stop-class. */
#define FAULT_LINE_STALE        ((fault_bits_t)(1U << 0))   /* stop: no fresh IR frames */
#define FAULT_LINE_CAL          ((fault_bits_t)(1U << 1))   /* stop: calibration failed */
#define FAULT_LINE_LOST         ((fault_bits_t)(1U << 2))   /* stop: line not recovered */
#define FAULT_ENCODER           ((fault_bits_t)(1U << 3))   /* stop: encoder stale/illegal */
#define FAULT_MOTION            ((fault_bits_t)(1U << 4))   /* stop: motion command failed */
#define FAULT_IMU               ((fault_bits_t)(1U << 5))   /* info: IMU unreadable */
#define FAULT_ULTRASONIC        ((fault_bits_t)(1U << 6))   /* stop: cannot see obstacles */
#define FAULT_NO_SAFE_PATH      ((fault_bits_t)(1U << 7))   /* stop: scan found no route */
#define FAULT_RECOVERY_LIMIT    ((fault_bits_t)(1U << 8))   /* stop: 5 s / 0.5 m exceeded */
#define FAULT_IMU_LIFT          ((fault_bits_t)(1U << 9))   /* stop: sustained lift/overturn */
#define FAULT_BARCODE_REJECT    ((fault_bits_t)(1U << 12))  /* info: unmapped/invalid code */
#define FAULT_LINE_OVERFLOW     ((fault_bits_t)(1U << 13))  /* info: ADC ring overflowed */
#define FAULT_TELEMETRY_DROP    ((fault_bits_t)(1U << 14))  /* info: telemetry buffer full */
#define FAULT_NET_DOWN          ((fault_bits_t)(1U << 15))  /* info: WiFi/MQTT unavailable */

#define FAULT_STOP_MASK         ((fault_bits_t)(FAULT_LINE_STALE | FAULT_LINE_CAL | \
                                  FAULT_LINE_LOST | FAULT_ENCODER | FAULT_MOTION | \
                                  FAULT_ULTRASONIC | FAULT_NO_SAFE_PATH | \
                                  FAULT_RECOVERY_LIMIT | FAULT_IMU_LIFT))

typedef struct {
    volatile fault_bits_t bits;
} fault_mask_t;

void fault_mask_init(fault_mask_t *mask);

/* Raise one or more faults.  Safe from tasks and interrupt handlers. */
void fault_set(fault_mask_t *mask, fault_bits_t bits);

/*
 * Clear faults. Stop-class faults are normally cleared by the Vehicle
 * Controller on the operator's authority. The IMU task may clear only
 * FAULT_IMU_LIFT after a sustained safe-rearm condition. Informational faults
 * may be cleared by the module that owns them.
 */
void fault_clear(fault_mask_t *mask, fault_bits_t bits);

fault_bits_t fault_read(const fault_mask_t *mask);

/* True when any raised fault requires a stop. */
bool fault_stop_required(const fault_mask_t *mask);

#endif /* FAULT_MASK_H */
