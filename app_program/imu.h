/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * imu.h
 * IMU processing (Buddy 4): accelerometer decoding, rest calibration, tilt,
 * lift/overturn detection, hump detection with height estimation, and motion
 * classification.
 *
 * The module has an accelerometer and magnetometer but no gyroscope, so tilt
 * comes from the gravity vector and is only trustworthy when the car is not
 * accelerating hard.  Hump height is the maximum elevation above the
 * pre-hump baseline, accumulated as (distance travelled along the surface) x
 * sin(pitch), which is independent of speed and hump shape.  Missing, stale
 * or disturbed inputs give an UNAVAILABLE or LOW_CONFIDENCE result and never
 * overwrite the highest valid height.
 *
 * Pure C: the caller supplies raw sensor bytes and encoder distance.
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

/* ---- accelerometer decoding ------------------------------------------ */

typedef struct {
    int16_t x;      /* milli-g */
    int16_t y;
    int16_t z;
} imu_vec3_t;

/*
 * The Tribunal names the LSM303DLHC; the bench test found an LSM303D
 * (WHO_AM_I 0x49 at address 0x1D).  Both are supported and told apart by the
 * HAL's probe.
 */
typedef enum {
    IMU_VARIANT_LSM303D = 0,    /* 16-bit, +/-2 g: 0.061 mg per count */
    IMU_VARIANT_LSM303DLHC      /* 12-bit left-justified, +/-2 g high-res: 1 mg per count */
} imu_variant_t;

/* Decode six little-endian bytes (X_L, X_H, Y_L, Y_H, Z_L, Z_H). */
void imu_decode_accel(imu_variant_t variant, const uint8_t raw[6], imu_vec3_t *mg);

/* ---- rest calibration and tilt --------------------------------------- */

#define IMU_AXIS_X      (0U)
#define IMU_AXIS_Y      (1U)
#define IMU_AXIS_Z      (2U)

typedef enum {
    IMU_CAL_COLLECTING = 0,
    IMU_CAL_DONE,
    IMU_CAL_FAILED_MOVING       /* the car was not at rest */
} imu_cal_state_t;

typedef struct {
    uint8_t         forward_axis;   /* which sensor axis points along the car */
    int8_t          forward_sign;   /* +1 or -1 */
    uint16_t        samples_needed;
    uint16_t        samples;
    int32_t         sum[3];
    int16_t         min[3];
    int16_t         max[3];
    int16_t         spread_limit_mg;
    imu_cal_state_t state;
    int16_t         base_f;         /* baseline gravity, forward component */
    int16_t         base_v;         /* ... and vertical component (up-positive) */
    uint8_t         vertical_axis;
    int8_t          vertical_sign;
    int16_t         base_mag;       /* baseline |g| in mg (not assumed to be 1000) */
    uint8_t         lateral_axis;   /* the sideways axis: neither forward nor vertical */
    int16_t         base_l;         /* baseline gravity, sideways component */
} imu_cal_t;

void imu_cal_init(imu_cal_t *cal, uint8_t forward_axis, int8_t forward_sign,
                  uint16_t samples_needed, int16_t spread_limit_mg);

/* Feed one stationary reading; returns the calibration state. */
imu_cal_state_t imu_cal_feed(imu_cal_t *cal, const imu_vec3_t *mg);

/*
 * Pitch relative to the calibrated rest attitude, as sin(pitch) in
 * per-mille; positive = nose up.  Returns 0 until calibration is DONE.
 */
int16_t imu_pitch_permille(const imu_cal_t *cal, const imu_vec3_t *mg);

/*
 * Sideways tilt (roll) relative to the calibrated rest attitude, as sin(roll)
 * in per-mille.  One wheel on a hump lifts that wheel by h, which rolls the car
 * by sin(roll) = h / track.  Measured on the car: positive = the left wheel
 * raised.  Returns 0 until calibration is DONE.
 */
int16_t imu_roll_permille(const imu_cal_t *cal, const imu_vec3_t *mg);

/* Magnitude of the measurement minus the calibrated 1 g (disturbance), mg. */
int16_t imu_disturbance_mg(const imu_cal_t *cal, const imu_vec3_t *mg);

/* ---- sustained lift/overturn detection ------------------------------- */

typedef struct {
    bool     tilt_active;
    uint32_t tilt_since_ms;
    bool     freefall_active;
    uint32_t freefall_since_ms;
    bool     latched;
} imu_lift_detector_t;

void imu_lift_init(imu_lift_detector_t *detector);

/*
 * Re-arming after a lift.  A lifted car is stopped; it may run again only once it is back on the floor.  An accelerometer
 * cannot SEE the floor, so "on the floor" is taken to be: level (within level_cos_min of the calibrated rest attitude),
 * at 1 g (the magnitude within mag_tol_mg of the rest magnitude: not being swung or dropped) and steady (no axis moved more
 * than step_max_mg since the previous sample) for hold_ms in a row.  A car held perfectly level and still in the air
 * satisfies it too: a wheel-contact sensor would be needed to tell that apart.  Re-arming only permits START; it never
 * starts the car.
 */
typedef struct {
    bool       ok_active;
    uint32_t   ok_since_ms;
    bool       have_prev;
    imu_vec3_t prev;
} imu_rearm_t;

void imu_rearm_init(imu_rearm_t *r);

/* Feed every sample.  True once the car has been level, at 1 g and steady for hold_ms. */
bool imu_rearm_update(imu_rearm_t *r, const imu_cal_t *cal, const imu_vec3_t *mg, uint32_t now_ms,
                      uint16_t level_cos_min_permille, uint16_t mag_tol_mg, uint16_t step_max_mg,
                      uint16_t hold_ms);

/*
 * Return true once when a tilt beyond the calibrated cosine limit or a
 * near-freefall reading persists for hold_ms.  Thresholds are supplied by the
 * robot configuration and are provisional until validated on the car.
 */
bool imu_lift_update(imu_lift_detector_t *detector, const imu_cal_t *cal,
                     const imu_vec3_t *mg, uint32_t now_ms,
                     uint16_t tilt_cos_max_permille, uint16_t freefall_max_mg,
                     uint16_t hold_ms);

/* ---- hump detection -------------------------------------------------- */

typedef enum {
    HUMP_FLAT = 0,
    HUMP_RISING,
    HUMP_ON_HUMP,
    HUMP_DESCENDING
} hump_state_t;

typedef enum {
    HUMP_VALID = 0,
    HUMP_UNAVAILABLE,
    HUMP_LOW_CONFIDENCE
} hump_validity_t;

typedef enum {
    HUMP_EVENT_NONE = 0,
    HUMP_EVENT_STARTED,
    HUMP_EVENT_FINISHED
} hump_event_t;

typedef struct {
    int16_t  up_threshold_permille;
    int16_t  down_threshold_permille;
    int16_t  flat_band_permille;
    uint16_t min_duration_ms;           /* a state change must persist this long */
    uint16_t max_duration_ms;           /* longer than this is a slope, not a hump */
    int16_t  min_height_mm;             /* below this is a bump/noise, not reported */
    uint16_t net_elevation_percent;     /* |end elevation| above this % of peak: low confidence */
    int16_t  disturbance_mg;            /* |accel - 1 g| above this: low confidence */
} hump_config_t;

typedef struct {
    int16_t  pitch_permille;            /* from imu_pitch_permille */
    int16_t  disturbance_mg;
    int32_t  distance_mm;               /* signed travel since boot (motion) */
    bool     distance_valid;            /* fresh, valid encoder distance */
    bool     imu_ok;                    /* the accelerometer is being read */
    uint32_t now_ms;
} hump_input_t;

typedef struct {
    hump_config_t   cfg;
    hump_state_t    state;
    int32_t         pitch_filtered;     /* x 16 */
    bool            have_distance;
    int32_t         last_distance_mm;
    int32_t         elevation_um;
    int32_t         peak_um;
    uint32_t        start_ms;
    uint32_t        cond_since_ms;
    bool            cond_active;
    bool            candidate;
    uint32_t        candidate_since_ms;
    bool            inputs_bad;         /* a required input was missing mid-event */
    bool            imu_fault_seen;     /* the accelerometer failed mid-event */
    bool            disturbed;
    int8_t          cond_kind;          /* what cond_since is timing */
    int32_t         highest_valid_mm;   /* -1 until a valid hump has been seen */
    uint16_t        last_event_packed;  /* hump_event_pack() of the latest result */
    uint32_t        humps_seen;
} hump_detector_t;

void hump_init(hump_detector_t *h, const hump_config_t *cfg);

/* Call every IMU period (20 ms).  Returns an event on state transitions. */
hump_event_t hump_update(hump_detector_t *h, const hump_input_t *in);

/* True while a hump is being traversed. */
bool hump_in_progress(const hump_detector_t *h);

/* Highest hump with a VALID result so far, in mm; -1 if none. */
int32_t hump_highest_valid_mm(const hump_detector_t *h);

/* Latest finished hump (height in mm, 127 = out of range) and its validity. */
uint8_t hump_last_height_code(const hump_detector_t *h);
hump_validity_t hump_last_validity(const hump_detector_t *h);

/* ---- roll-based hump detector (one wheel on the hump) ------------------ */

/*
 * Driving shakes the accelerometer and shifts the pitch by about 100 per-mille
 * (the motors' torque and the vibration), so pitch cannot see a 1 cm hump on
 * the move.  Roll stays near zero on the flat, so a hump that lifts one wheel
 * shows up as one smooth bump of the (low-pass filtered) roll.  The height is
 * the peak roll times a lever length fitted on the car with a known hump.
 *
 * The caller low-pass filters the accelerometer vector (about 160 ms) before
 * taking the roll.  Missing or stale inputs give an UNAVAILABLE or
 * LOW_CONFIDENCE result and never overwrite the highest valid height.
 */
typedef struct {
    int16_t  enter_permille;        /* |roll| above this (for enter_ms) starts a hump */
    int16_t  exit_permille;         /* |roll| below this (for exit_ms) ends it */
    uint16_t enter_ms;
    uint16_t exit_ms;
    uint16_t lever_mm;              /* height = peak |roll| x lever / 1000 (fitted) */
    uint16_t min_length_mm;         /* a shorter bump is a knock, not a hump */
    uint16_t max_length_mm;         /* a longer one is a slope or a turn: low confidence */
    int16_t  min_height_mm;         /* below this is not reported */
} roll_hump_config_t;

typedef struct {
    int16_t  roll_permille;         /* from imu_roll_permille, low-pass filtered input */
    int32_t  distance_mm;           /* signed travel since boot (motion) */
    bool     distance_valid;        /* fresh, valid encoder distance */
    bool     imu_ok;                /* the accelerometer is being read */
    uint32_t now_ms;
} roll_hump_input_t;

typedef struct {
    roll_hump_config_t cfg;
    hump_state_t    state;
    int32_t         peak_permille;      /* largest |roll| in this hump */
    int8_t          side;               /* sign of the roll at the peak */
    int32_t         start_mm;
    bool            start_distance_ok;
    uint32_t        cond_since_ms;
    bool            cond_active;
    bool            inputs_bad;         /* a required input was missing mid-event */
    int32_t         highest_valid_mm;   /* -1 until a valid hump has been seen */
    uint16_t        last_event_packed;
    uint32_t        humps_seen;
} roll_hump_t;

void roll_hump_init(roll_hump_t *h, const roll_hump_config_t *cfg);

/* Call every IMU period (20 ms).  Returns an event on state transitions. */
hump_event_t roll_hump_update(roll_hump_t *h, const roll_hump_input_t *in);

bool roll_hump_in_progress(const roll_hump_t *h);
int32_t roll_hump_highest_valid_mm(const roll_hump_t *h);
uint8_t roll_hump_last_height_code(const roll_hump_t *h);
hump_validity_t roll_hump_last_validity(const roll_hump_t *h);

/* ---- the packed 13-bit hump event ------------------------------------ */

/*
 *   bit 12..10  status    (hump_state_t, 3 bits)
 *   bit  9..3   height    (mm, 0..126; 127 = out of range)
 *   bit  2..1   validity  (hump_validity_t, 2 bits)
 *   bit  0      IMU fault
 */
#define HUMP_HEIGHT_OUT_OF_RANGE    (127U)
#define HUMP_HEIGHT_MAX_MM          (126U)

uint16_t hump_event_pack(hump_state_t status, uint8_t height_code,
                         hump_validity_t validity, bool imu_fault);
void hump_event_unpack(uint16_t packed, hump_state_t *status, uint8_t *height_code,
                       hump_validity_t *validity, bool *imu_fault);

/* ---- motion classification ------------------------------------------- */

typedef enum {
    IMU_MOTION_STATIONARY = 0,
    IMU_MOTION_CRUISING,
    IMU_MOTION_ACCELERATING,
    IMU_MOTION_TURNING,
    IMU_MOTION_CLIMBING,
    IMU_MOTION_DESCENDING
} imu_motion_t;

typedef struct {
    int16_t speed_mm_s;                 /* centre speed from motion */
    int16_t turn_rate_dps;              /* from the wheel speed difference */
    int16_t disturbance_mg;
    int16_t accel_mm_s2;                /* change of speed, from motion */
    hump_state_t hump;
} imu_motion_input_t;

imu_motion_t imu_classify_motion(const imu_motion_input_t *in);

#endif /* IMU_H */
