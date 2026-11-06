/*
 *  imu.h
 *  IMU / hump subsystem interface.
 *  Owner: Buddy 4 (Suhail)
 */

#ifndef IMU_H
#define IMU_H

#include <tk/tkernel.h>

/* Hump states: 6 states -> 3 bits */
typedef enum {
    IMU_ST_FLAT = 0,
    IMU_ST_RISING,
    IMU_ST_ON_HUMP,
    IMU_ST_DESCENDING,
    IMU_ST_REPORT,
    IMU_ST_DEGRADED
} imu_state_t;

/* Result validity: 3 values -> 2 bits */
#define IMU_VALID           (0U)
#define IMU_UNAVAILABLE     (1U)
#define IMU_LOW_CONFIDENCE  (2U)

#define IMU_HEIGHT_MAX_MM   (126U)  /* 127 is reserved = out of range */
#define IMU_HEIGHT_OOR      (127U)

/*
 * Packed hump event (minimum-bit requirement): 12 bits in a 16-bit word.
 *   bits 0-2  : state     (3 bits)
 *   bits 3-9  : height_mm (7 bits, 0-126, 127 = out of range)
 *   bits 10-11: validity  (2 bits)
 * The IMU fault is one bit in the team's shared fault mask, not here.
 * Masks/shifts (not C bit-fields) so the layout is fixed for MQTT too.
 */
typedef UH hump_evt_t;

typedef struct {
    UW          sequence;
    UW          duration_ms;
    UW          peak_deviation_mg;
    hump_evt_t  packed;
} imu_event_t;

#define HUMP_STATE_SHIFT    (0)
#define HUMP_STATE_MASK     (0x0007U)
#define HUMP_HEIGHT_SHIFT   (3)
#define HUMP_HEIGHT_MASK    (0x03F8U)
#define HUMP_VALID_SHIFT    (10)
#define HUMP_VALID_MASK     (0x0C00U)

Inline hump_evt_t hump_pack(UW state, UW height_mm, UW validity)
{
    return (hump_evt_t)(((state     << HUMP_STATE_SHIFT)  & HUMP_STATE_MASK)  |
                        ((height_mm << HUMP_HEIGHT_SHIFT) & HUMP_HEIGHT_MASK) |
                        ((validity  << HUMP_VALID_SHIFT)  & HUMP_VALID_MASK));
}

Inline UW hump_state(hump_evt_t e)  { return (e & HUMP_STATE_MASK)  >> HUMP_STATE_SHIFT;  }
Inline UW hump_height(hump_evt_t e) { return (e & HUMP_HEIGHT_MASK) >> HUMP_HEIGHT_SHIFT; }
Inline UW hump_valid(hump_evt_t e)  { return (e & HUMP_VALID_MASK)  >> HUMP_VALID_SHIFT;  }

/* Read sequence before/after copying to detect an event update in progress. */
IMPORT volatile imu_event_t imu_latest_event;
IMPORT volatile UW imu_largest_peak_deviation_mg;

/* Task entry, created in usermain() */
IMPORT void imu_task(INT stacd, void *exinf);

#endif /* IMU_H */