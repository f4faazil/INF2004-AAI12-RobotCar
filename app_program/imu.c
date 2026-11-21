/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * imu.c
 * IMU processing.  See imu.h.
 */
#include "imu.h"

#define LSM303D_UG_PER_COUNT        (61)        /* 0.061 mg per count at +/-2 g */
#define LSM303DLHC_SHIFT_DIV        (16)        /* 12-bit value left-justified in 16 */
#define MG_PER_UG                   (1000)
#define ROUND_HALF                  (500)
#define PERMILLE                    (1000)

#define PITCH_FILTER_ONE            (16)        /* filtered pitch is kept x16 */
#define PITCH_FILTER_DIV            (2)         /* each step moves half way */
#define UM_ROUND_HALF               (500)
#define PERCENT                     (100)

#define KIND_NONE                   ((int8_t)0)
#define KIND_FLAT                   ((int8_t)1)
#define KIND_DOWN                   ((int8_t)2)

#define STATUS_BITS                 (3U)
#define HEIGHT_BITS                 (7U)
#define VALIDITY_BITS               (2U)
#define STATUS_MASK                 (0x07U)
#define HEIGHT_MASK                 (0x7FU)
#define VALIDITY_MASK               (0x03U)
#define FAULT_MASK                  (0x01U)
#define VALIDITY_SHIFT              (1U)
#define HEIGHT_SHIFT                (VALIDITY_SHIFT + VALIDITY_BITS)
#define STATUS_SHIFT                (HEIGHT_SHIFT + HEIGHT_BITS)

#define MOTION_STATIONARY_MM_S      (10)
#define MOTION_TURNING_DPS          (25)
#define MOTION_ACCEL_MM_S2          (150)

/* ---- small helpers --------------------------------------------------- */

static int32_t abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static uint32_t isqrt32(uint32_t n)
{
    uint32_t bit = (uint32_t)1U << 30;
    uint32_t result = 0U;

    while (bit > n) {
        bit >>= 2;
    }
    while (bit != 0U) {
        if (n >= (result + bit)) {
            n -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

static int16_t to_i16(int32_t v)
{
    if (v > INT16_MAX) {
        return INT16_MAX;
    }
    if (v < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)v;
}

/* ---- accelerometer decoding ------------------------------------------ */

static int16_t le16(const uint8_t *p)
{
    return (int16_t)(uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t decode_axis(imu_variant_t variant, int16_t raw)
{
    if (variant == IMU_VARIANT_LSM303DLHC) {
        return (int16_t)(raw / LSM303DLHC_SHIFT_DIV);
    }
    return to_i16(((int32_t)raw * LSM303D_UG_PER_COUNT +
                   ((raw >= 0) ? ROUND_HALF : -ROUND_HALF)) / MG_PER_UG);
}

void imu_decode_accel(imu_variant_t variant, const uint8_t raw[6], imu_vec3_t *mg)
{
    mg->x = decode_axis(variant, le16(&raw[0]));
    mg->y = decode_axis(variant, le16(&raw[2]));
    mg->z = decode_axis(variant, le16(&raw[4]));
}

/* ---- rest calibration and tilt --------------------------------------- */

static int16_t axis_of(const imu_vec3_t *v, uint8_t axis)
{
    if (axis == IMU_AXIS_X) {
        return v->x;
    }
    if (axis == IMU_AXIS_Y) {
        return v->y;
    }
    return v->z;
}

void imu_cal_init(imu_cal_t *cal, uint8_t forward_axis, int8_t forward_sign,
                  uint16_t samples_needed, int16_t spread_limit_mg)
{
    uint8_t i;

    cal->forward_axis = forward_axis;
    cal->forward_sign = (forward_sign < 0) ? (int8_t)-1 : (int8_t)1;
    cal->samples_needed = samples_needed;
    cal->samples = 0U;
    cal->spread_limit_mg = spread_limit_mg;
    cal->state = IMU_CAL_COLLECTING;
    for (i = 0U; i < 3U; i++) {
        cal->sum[i] = 0;
        cal->min[i] = INT16_MAX;
        cal->max[i] = INT16_MIN;
    }
    cal->base_f = 0;
    cal->base_v = 0;
    cal->vertical_axis = IMU_AXIS_Z;
    cal->vertical_sign = 1;
    cal->base_mag = 0;
    cal->lateral_axis = IMU_AXIS_Y;
    cal->base_l = 0;
}

static void finish_calibration(imu_cal_t *cal)
{
    int32_t mean[3];
    int32_t spread = 0;
    uint8_t i;
    uint8_t other_a;
    uint8_t other_b;
    uint32_t mag_sq = 0U;

    for (i = 0U; i < 3U; i++) {
        int32_t s = (int32_t)cal->max[i] - (int32_t)cal->min[i];

        mean[i] = cal->sum[i] / (int32_t)cal->samples;
        if (s > spread) {
            spread = s;
        }
    }
    if (spread > cal->spread_limit_mg) {
        cal->state = IMU_CAL_FAILED_MOVING;
        return;
    }

    /* The vertical axis is whichever non-forward axis carries gravity. */
    other_a = (uint8_t)((cal->forward_axis + 1U) % 3U);
    other_b = (uint8_t)((cal->forward_axis + 2U) % 3U);
    cal->vertical_axis = (abs32(mean[other_a]) >= abs32(mean[other_b])) ? other_a : other_b;
    cal->vertical_sign = (mean[cal->vertical_axis] < 0) ? (int8_t)-1 : (int8_t)1;
    cal->base_v = to_i16(abs32(mean[cal->vertical_axis]));
    cal->base_f = to_i16(mean[cal->forward_axis] * cal->forward_sign);
    /* The axes are 0, 1, 2: the sideways one is what is left over. */
    cal->lateral_axis = (uint8_t)(3U - cal->forward_axis - cal->vertical_axis);
    cal->base_l = to_i16(mean[cal->lateral_axis]);

    for (i = 0U; i < 3U; i++) {
        mag_sq += (uint32_t)(mean[i] * mean[i]);
    }
    cal->base_mag = to_i16((int32_t)isqrt32(mag_sq));
    cal->state = IMU_CAL_DONE;
}

imu_cal_state_t imu_cal_feed(imu_cal_t *cal, const imu_vec3_t *mg)
{
    uint8_t i;

    if (cal->state != IMU_CAL_COLLECTING) {
        return cal->state;
    }
    for (i = 0U; i < 3U; i++) {
        int16_t v = axis_of(mg, i);

        cal->sum[i] += v;
        if (v < cal->min[i]) {
            cal->min[i] = v;
        }
        if (v > cal->max[i]) {
            cal->max[i] = v;
        }
    }
    cal->samples++;
    if (cal->samples >= cal->samples_needed) {
        finish_calibration(cal);
    }
    return cal->state;
}

int16_t imu_pitch_permille(const imu_cal_t *cal, const imu_vec3_t *mg)
{
    int64_t g_f;
    int64_t g_v;
    int64_t num;
    int64_t den;
    uint32_t cur_sq;
    uint32_t base_sq;

    if (cal->state != IMU_CAL_DONE) {
        return 0;
    }
    g_f = (int64_t)axis_of(mg, cal->forward_axis) * cal->forward_sign;
    g_v = (int64_t)axis_of(mg, cal->vertical_axis) * cal->vertical_sign;

    /* z of the cross product of the current and baseline gravity in the
     * forward/vertical plane = |g||g0| sin(angle between them). */
    num = (g_f * cal->base_v) - (g_v * cal->base_f);
    cur_sq = (uint32_t)((g_f * g_f) + (g_v * g_v));
    base_sq = (uint32_t)(((int32_t)cal->base_f * cal->base_f) +
                         ((int32_t)cal->base_v * cal->base_v));
    den = (int64_t)isqrt32(cur_sq) * (int64_t)isqrt32(base_sq);
    if (den == 0) {
        return 0;
    }
    /* Nose up pitches the reaction vector toward -forward, hence the sign. */
    return to_i16((int32_t)((-num * PERMILLE) / den));
}

int16_t imu_roll_permille(const imu_cal_t *cal, const imu_vec3_t *mg)
{
    int64_t g_l;
    int64_t g_v;
    int64_t num;
    int64_t den;
    uint32_t cur_sq;
    uint32_t base_sq;

    if (cal->state != IMU_CAL_DONE) {
        return 0;
    }
    g_l = axis_of(mg, cal->lateral_axis);
    g_v = (int64_t)axis_of(mg, cal->vertical_axis) * cal->vertical_sign;

    /* Same cross product as the pitch, in the sideways/vertical plane. */
    num = (g_l * cal->base_v) - (g_v * cal->base_l);
    cur_sq = (uint32_t)((g_l * g_l) + (g_v * g_v));
    base_sq = (uint32_t)(((int32_t)cal->base_l * cal->base_l) +
                         ((int32_t)cal->base_v * cal->base_v));
    den = (int64_t)isqrt32(cur_sq) * (int64_t)isqrt32(base_sq);
    if (den == 0) {
        return 0;
    }
    return to_i16((int32_t)((-num * PERMILLE) / den));
}

int16_t imu_disturbance_mg(const imu_cal_t *cal, const imu_vec3_t *mg)
{
    uint32_t sq = (uint32_t)(((int32_t)mg->x * mg->x) + ((int32_t)mg->y * mg->y) +
                             ((int32_t)mg->z * mg->z));

    return to_i16((int32_t)isqrt32(sq) - (int32_t)cal->base_mag);
}

static bool condition_held(bool active, bool *tracking, uint32_t *since_ms,
                           uint32_t now_ms, uint16_t hold_ms)
{
    if (!active) {
        *tracking = false;
        return false;
    }
    if (!*tracking) {
        *tracking = true;
        *since_ms = now_ms;
        return hold_ms == 0U;
    }
    return (now_ms - *since_ms) >= hold_ms;
}

void imu_rearm_init(imu_rearm_t *r)
{
    r->ok_active = false;
    r->ok_since_ms = 0U;
    r->have_prev = false;
    r->prev.x = 0;
    r->prev.y = 0;
    r->prev.z = 0;
}

bool imu_rearm_update(imu_rearm_t *r, const imu_cal_t *cal, const imu_vec3_t *mg,
                      uint32_t now_ms, uint16_t level_cos_min_permille,
                      uint16_t mag_tol_mg, uint16_t step_max_mg, uint16_t hold_ms)
{
    int32_t base[3] = { 0, 0, 0 };
    const int32_t x = mg->x;
    const int32_t y = mg->y;
    const int32_t z = mg->z;
    int32_t dx;
    int32_t dy;
    int32_t dz;
    int32_t dm;
    int64_t dot;
    uint32_t magnitude;
    bool level;
    bool at_1g;
    bool steady = false;

    if (cal->state != IMU_CAL_DONE) {
        return false;
    }

    base[cal->forward_axis] = (int32_t)cal->base_f * cal->forward_sign;
    base[cal->vertical_axis] = (int32_t)cal->base_v * cal->vertical_sign;
    base[cal->lateral_axis] = cal->base_l;
    magnitude = isqrt32((uint32_t)(x * x) + (uint32_t)(y * y) + (uint32_t)(z * z));

    dot = ((int64_t)x * base[0]) + ((int64_t)y * base[1]) + ((int64_t)z * base[2]);
    level = (magnitude > 0U) && (cal->base_mag > 0) &&
            ((dot * PERMILLE) >=
             ((int64_t)level_cos_min_permille * cal->base_mag * magnitude));
    dm = (int32_t)magnitude - (int32_t)cal->base_mag;
    if (dm < 0) {
        dm = -dm;
    }
    at_1g = dm <= (int32_t)mag_tol_mg;

    if (r->have_prev) {
        dx = x - r->prev.x;
        dy = y - r->prev.y;
        dz = z - r->prev.z;
        dx = (dx < 0) ? -dx : dx;
        dy = (dy < 0) ? -dy : dy;
        dz = (dz < 0) ? -dz : dz;
        steady = (dx <= (int32_t)step_max_mg) && (dy <= (int32_t)step_max_mg) &&
                 (dz <= (int32_t)step_max_mg);
    }
    r->prev = *mg;
    r->have_prev = true;

    return condition_held(level && at_1g && steady, &r->ok_active, &r->ok_since_ms,
                          now_ms, hold_ms);
}

void imu_lift_init(imu_lift_detector_t *detector)
{
    detector->tilt_active = false;
    detector->tilt_since_ms = 0U;
    detector->freefall_active = false;
    detector->freefall_since_ms = 0U;
    detector->latched = false;
}

bool imu_lift_update(imu_lift_detector_t *detector, const imu_cal_t *cal,
                     const imu_vec3_t *mg, uint32_t now_ms,
                     uint16_t tilt_cos_max_permille, uint16_t freefall_max_mg,
                     uint16_t hold_ms)
{
    int32_t base[3] = { 0, 0, 0 };
    int32_t x = mg->x;
    int32_t y = mg->y;
    int32_t z = mg->z;
    int64_t dot;
    int64_t scaled_dot;
    int64_t tilt_limit;
    uint32_t magnitude_sq;
    uint32_t magnitude;
    bool tilted = false;
    bool near_freefall;
    bool tilt_held;
    bool freefall_held;

    if ((cal->state != IMU_CAL_DONE) || detector->latched) {
        return false;
    }

    base[cal->forward_axis] = (int32_t)cal->base_f * cal->forward_sign;
    base[cal->vertical_axis] = (int32_t)cal->base_v * cal->vertical_sign;
    base[cal->lateral_axis] = cal->base_l;
    magnitude_sq = (uint32_t)(x * x) + (uint32_t)(y * y) + (uint32_t)(z * z);
    magnitude = isqrt32(magnitude_sq);
    near_freefall = magnitude <= freefall_max_mg;

    if (!near_freefall && (cal->base_mag > 0)) {
        dot = ((int64_t)x * base[0]) + ((int64_t)y * base[1]) +
              ((int64_t)z * base[2]);
        scaled_dot = dot * PERMILLE;
        tilt_limit = (int64_t)tilt_cos_max_permille * cal->base_mag * magnitude;
        tilted = scaled_dot <= tilt_limit;
    }

    tilt_held = condition_held(tilted, &detector->tilt_active,
                               &detector->tilt_since_ms, now_ms, hold_ms);
    freefall_held = condition_held(near_freefall, &detector->freefall_active,
                                  &detector->freefall_since_ms, now_ms, hold_ms);
    if (tilt_held || freefall_held) {
        detector->latched = true;
        return true;
    }
    return false;
}

/* ---- packed event ---------------------------------------------------- */

uint16_t hump_event_pack(hump_state_t status, uint8_t height_code,
                         hump_validity_t validity, bool imu_fault)
{
    return (uint16_t)((((uint16_t)status & STATUS_MASK) << STATUS_SHIFT) |
                      (((uint16_t)height_code & HEIGHT_MASK) << HEIGHT_SHIFT) |
                      (((uint16_t)validity & VALIDITY_MASK) << VALIDITY_SHIFT) |
                      (imu_fault ? FAULT_MASK : 0U));
}

void hump_event_unpack(uint16_t packed, hump_state_t *status, uint8_t *height_code,
                       hump_validity_t *validity, bool *imu_fault)
{
    *status = (hump_state_t)((packed >> STATUS_SHIFT) & STATUS_MASK);
    *height_code = (uint8_t)((packed >> HEIGHT_SHIFT) & HEIGHT_MASK);
    *validity = (hump_validity_t)((packed >> VALIDITY_SHIFT) & VALIDITY_MASK);
    *imu_fault = (packed & FAULT_MASK) != 0U;
}

/* ---- hump detection -------------------------------------------------- */

void hump_init(hump_detector_t *h, const hump_config_t *cfg)
{
    h->cfg = *cfg;
    h->state = HUMP_FLAT;
    h->pitch_filtered = 0;
    h->have_distance = false;
    h->last_distance_mm = 0;
    h->elevation_um = 0;
    h->peak_um = 0;
    h->start_ms = 0U;
    h->cond_since_ms = 0U;
    h->cond_active = false;
    h->cond_kind = KIND_NONE;
    h->candidate = false;
    h->candidate_since_ms = 0U;
    h->inputs_bad = false;
    h->imu_fault_seen = false;
    h->disturbed = false;
    h->highest_valid_mm = -1;
    h->last_event_packed = 0U;
    h->humps_seen = 0U;
}

bool hump_in_progress(const hump_detector_t *h)
{
    return h->state != HUMP_FLAT;
}

int32_t hump_highest_valid_mm(const hump_detector_t *h)
{
    return h->highest_valid_mm;
}

uint8_t hump_last_height_code(const hump_detector_t *h)
{
    hump_state_t st;
    uint8_t code;
    hump_validity_t v;
    bool f;

    hump_event_unpack(h->last_event_packed, &st, &code, &v, &f);
    return code;
}

hump_validity_t hump_last_validity(const hump_detector_t *h)
{
    hump_state_t st;
    uint8_t code;
    hump_validity_t v;
    bool f;

    hump_event_unpack(h->last_event_packed, &st, &code, &v, &f);
    return v;
}

static void note_inputs(hump_detector_t *h, const hump_input_t *in)
{
    if (!in->distance_valid) {
        h->inputs_bad = true;
    }
    if (!in->imu_ok) {
        h->inputs_bad = true;
        h->imu_fault_seen = true;
    }
    if (abs32(in->disturbance_mg) > h->cfg.disturbance_mg) {
        h->disturbed = true;
    }
}

static void accumulate(hump_detector_t *h, int32_t ds_mm, int32_t pitch_permille)
{
    /* mm x per-mille = micrometres */
    h->elevation_um += ds_mm * pitch_permille;
    if (h->elevation_um > h->peak_um) {
        h->peak_um = h->elevation_um;
    }
}

static void finish_hump(hump_detector_t *h, bool aborted)
{
    int32_t height_mm = (h->peak_um + UM_ROUND_HALF) / MG_PER_UG;
    hump_validity_t validity;
    uint8_t code;

    if (aborted || h->inputs_bad) {
        validity = HUMP_UNAVAILABLE;
        code = 0U;
    } else {
        validity = HUMP_VALID;
        if (h->disturbed) {
            validity = HUMP_LOW_CONFIDENCE;
        } else if (height_mm < h->cfg.min_height_mm) {
            validity = HUMP_LOW_CONFIDENCE;
        } else if ((abs32(h->elevation_um) * PERCENT) >
                   (h->peak_um * (int32_t)h->cfg.net_elevation_percent)) {
            validity = HUMP_LOW_CONFIDENCE;     /* did not come back down */
        }
        if (height_mm > (int32_t)HUMP_HEIGHT_MAX_MM) {
            code = (uint8_t)HUMP_HEIGHT_OUT_OF_RANGE;
            validity = HUMP_LOW_CONFIDENCE;
        } else {
            code = (uint8_t)((height_mm < 0) ? 0 : height_mm);
        }
    }

    if ((validity == HUMP_VALID) && (height_mm > h->highest_valid_mm)) {
        h->highest_valid_mm = height_mm;
    }
    h->last_event_packed = hump_event_pack(HUMP_FLAT, code, validity, h->imu_fault_seen);
    h->humps_seen++;
    h->state = HUMP_FLAT;
    h->candidate = false;
    h->cond_kind = KIND_NONE;
}

hump_event_t hump_update(hump_detector_t *h, const hump_input_t *in)
{
    int32_t ds = 0;
    int32_t pitch;
    int8_t kind;
    bool sustained;

    h->pitch_filtered += (((int32_t)in->pitch_permille * PITCH_FILTER_ONE) -
                          h->pitch_filtered) / PITCH_FILTER_DIV;
    pitch = h->pitch_filtered / PITCH_FILTER_ONE;

    if (in->distance_valid) {
        if (h->have_distance) {
            ds = in->distance_mm - h->last_distance_mm;
        }
        h->last_distance_mm = in->distance_mm;
        h->have_distance = true;
    } else {
        h->have_distance = false;
    }

    if (h->state == HUMP_FLAT) {
        if (pitch > h->cfg.up_threshold_permille) {
            if (!h->candidate) {
                h->candidate = true;
                h->candidate_since_ms = in->now_ms;
                h->start_ms = in->now_ms;
                h->elevation_um = 0;
                h->peak_um = 0;
                h->inputs_bad = false;
                h->imu_fault_seen = false;
                h->disturbed = false;
            }
            note_inputs(h, in);
            accumulate(h, ds, pitch);
            if ((in->now_ms - h->candidate_since_ms) >= h->cfg.min_duration_ms) {
                h->state = HUMP_RISING;
                h->cond_kind = KIND_NONE;
                return HUMP_EVENT_STARTED;
            }
        } else {
            h->candidate = false;
        }
        return HUMP_EVENT_NONE;
    }

    note_inputs(h, in);
    accumulate(h, ds, pitch);
    if ((in->now_ms - h->start_ms) > h->cfg.max_duration_ms) {
        finish_hump(h, true);
        return HUMP_EVENT_FINISHED;
    }

    if (pitch < -(int32_t)h->cfg.down_threshold_permille) {
        kind = KIND_DOWN;
    } else if (abs32(pitch) <= h->cfg.flat_band_permille) {
        kind = KIND_FLAT;
    } else {
        kind = KIND_NONE;
    }
    if (kind != h->cond_kind) {
        h->cond_kind = kind;
        h->cond_since_ms = in->now_ms;
    }
    sustained = (kind != KIND_NONE) &&
                ((in->now_ms - h->cond_since_ms) >= h->cfg.min_duration_ms);

    if (sustained) {
        if ((h->state != HUMP_DESCENDING) && (kind == KIND_DOWN)) {
            h->state = HUMP_DESCENDING;
        } else if ((h->state == HUMP_RISING) && (kind == KIND_FLAT)) {
            h->state = HUMP_ON_HUMP;
        } else if ((h->state == HUMP_DESCENDING) && (kind == KIND_FLAT)) {
            finish_hump(h, false);
            return HUMP_EVENT_FINISHED;
        } else {
            /* ON_HUMP staying flat: nothing to do */
        }
    }
    return HUMP_EVENT_NONE;
}

/* ---- motion classification ------------------------------------------- */

/* ---- roll-based hump detector ---------------------------------------- */

void roll_hump_init(roll_hump_t *h, const roll_hump_config_t *cfg)
{
    h->cfg = *cfg;
    h->state = HUMP_FLAT;
    h->peak_permille = 0;
    h->side = 1;
    h->start_mm = 0;
    h->start_distance_ok = false;
    h->cond_since_ms = 0U;
    h->cond_active = false;
    h->inputs_bad = false;
    h->highest_valid_mm = -1;
    h->last_event_packed = hump_event_pack(HUMP_FLAT, HUMP_HEIGHT_OUT_OF_RANGE, HUMP_UNAVAILABLE, false);
    h->humps_seen = 0U;
}

bool roll_hump_in_progress(const roll_hump_t *h)
{
    return h->state != HUMP_FLAT;
}

int32_t roll_hump_highest_valid_mm(const roll_hump_t *h)
{
    return h->highest_valid_mm;
}

uint8_t roll_hump_last_height_code(const roll_hump_t *h)
{
    hump_state_t status;
    uint8_t code;
    hump_validity_t validity;
    bool fault;

    hump_event_unpack(h->last_event_packed, &status, &code, &validity, &fault);
    return code;
}

hump_validity_t roll_hump_last_validity(const roll_hump_t *h)
{
    hump_state_t status;
    uint8_t code;
    hump_validity_t validity;
    bool fault;

    hump_event_unpack(h->last_event_packed, &status, &code, &validity, &fault);
    return validity;
}

/* True once 'ok' has held for 'ms'; restarts the timer whenever it breaks. */
static bool held_for(roll_hump_t *h, bool ok, uint32_t now_ms, uint16_t ms)
{
    if (!ok) {
        h->cond_active = false;
        return false;
    }
    if (!h->cond_active) {
        h->cond_active = true;
        h->cond_since_ms = now_ms;
    }
    return (uint32_t)(now_ms - h->cond_since_ms) >= (uint32_t)ms;
}

static bool roll_hump_finish(roll_hump_t *h, const roll_hump_input_t *in)
{
    const int32_t height = ((h->peak_permille * (int32_t)h->cfg.lever_mm) + 500) / 1000;
    hump_validity_t validity = HUMP_VALID;
    uint8_t code;

    h->state = HUMP_FLAT;
    h->cond_active = false;
    if (height < (int32_t)h->cfg.min_height_mm) {
        return false;                       /* too small to be a hump: say nothing */
    }
    if (h->inputs_bad || !h->start_distance_ok || !in->distance_valid) {
        validity = HUMP_LOW_CONFIDENCE;
    } else {
        const int32_t length = in->distance_mm - h->start_mm;
        const int32_t abs_len = (length < 0) ? -length : length;

        if ((abs_len < (int32_t)h->cfg.min_length_mm) || (abs_len > (int32_t)h->cfg.max_length_mm)) {
            validity = HUMP_LOW_CONFIDENCE;
        }
    }
    code = (height > (int32_t)HUMP_HEIGHT_MAX_MM) ? (uint8_t)HUMP_HEIGHT_OUT_OF_RANGE : (uint8_t)height;
    h->last_event_packed = hump_event_pack(HUMP_FLAT, code, validity, h->inputs_bad);
    h->humps_seen++;
    if ((validity == HUMP_VALID) && (height > h->highest_valid_mm) && (code != HUMP_HEIGHT_OUT_OF_RANGE)) {
        h->highest_valid_mm = height;
    }
    return true;
}

hump_event_t roll_hump_update(roll_hump_t *h, const roll_hump_input_t *in)
{
    const int32_t mag = (in->roll_permille < 0) ? -(int32_t)in->roll_permille : (int32_t)in->roll_permille;
    hump_event_t ev = HUMP_EVENT_NONE;

    if (h->state != HUMP_FLAT) {
        if (!in->imu_ok || !in->distance_valid) {
            h->inputs_bad = true;
        }
    }

    switch (h->state) {
    case HUMP_FLAT:
        if (in->imu_ok && held_for(h, mag >= (int32_t)h->cfg.enter_permille, in->now_ms, h->cfg.enter_ms)) {
            h->state = HUMP_RISING;
            h->peak_permille = mag;
            h->side = (in->roll_permille < 0) ? (int8_t)-1 : (int8_t)1;
            h->start_mm = in->distance_mm;
            h->start_distance_ok = in->distance_valid;
            h->inputs_bad = false;
            h->cond_active = false;
            ev = HUMP_EVENT_STARTED;
        }
        break;

    case HUMP_RISING:
    case HUMP_ON_HUMP:
    case HUMP_DESCENDING:
        if (mag > h->peak_permille) {
            h->peak_permille = mag;
            h->side = (in->roll_permille < 0) ? (int8_t)-1 : (int8_t)1;
        }
        if (mag >= (2 * (int32_t)h->cfg.enter_permille)) {
            h->state = (mag >= ((h->peak_permille * 85) / 100)) ? HUMP_ON_HUMP : HUMP_DESCENDING;
        } else if (h->state == HUMP_ON_HUMP) {
            h->state = HUMP_DESCENDING;
        } else {
            /* still climbing toward the top, or already on the way down: keep the state */
        }
        if (held_for(h, mag < (int32_t)h->cfg.exit_permille, in->now_ms, h->cfg.exit_ms)) {
            if (roll_hump_finish(h, in)) {
                ev = HUMP_EVENT_FINISHED;
            }
        }
        break;

    default:
        h->state = HUMP_FLAT;
        break;
    }
    return ev;
}

imu_motion_t imu_classify_motion(const imu_motion_input_t *in)
{
    if (in->hump == HUMP_RISING) {
        return IMU_MOTION_CLIMBING;
    }
    if (in->hump == HUMP_DESCENDING) {
        return IMU_MOTION_DESCENDING;
    }
    if (abs32(in->turn_rate_dps) >= MOTION_TURNING_DPS) {
        return IMU_MOTION_TURNING;
    }
    if (abs32(in->speed_mm_s) < MOTION_STATIONARY_MM_S) {
        return IMU_MOTION_STATIONARY;
    }
    if (abs32(in->accel_mm_s2) >= MOTION_ACCEL_MM_S2) {
        return IMU_MOTION_ACCELERATING;
    }
    return IMU_MOTION_CRUISING;
}
