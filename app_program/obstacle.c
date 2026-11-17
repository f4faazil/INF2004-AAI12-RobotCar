/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * obstacle.c
 * Obstacle scanning, profiling and bypass planning.  See obstacle.h.
 */
#include <stddef.h>

#include "obstacle.h"

#define PERMILLE            (1000)
#define DEG_TO_RAD_MICRO    (17453)             /* pi / 180 x 10^6 */
#define MICRO               (1000000)
#define STRAIGHT_DEG        (90)
#define HALF                (2)
#define MERGE_CAPACITY      (SCAN_MAX_POINTS * 2U)

/* sin() in per-mille for 0..90 degrees. */
static const int16_t SIN_TABLE[91] = {
    0, 17, 35, 52, 70, 87, 105, 122, 139, 156, 174, 191, 208, 225, 242, 259,
    276, 292, 309, 326, 342, 358, 375, 391, 407, 423, 438, 454, 469, 485, 500,
    515, 530, 545, 559, 574, 588, 602, 616, 629, 643, 656, 669, 682, 695, 707,
    719, 731, 743, 755, 766, 777, 788, 799, 809, 819, 829, 839, 848, 857, 866,
    875, 883, 891, 899, 906, 914, 921, 927, 934, 940, 946, 951, 956, 961, 966,
    970, 974, 978, 982, 985, 988, 990, 993, 995, 996, 998, 999, 999, 1000, 1000
};

static int16_t clamp_deg(int16_t deg)
{
    if (deg < 0) {
        return 0;
    }
    if (deg > 180) {
        return 180;
    }
    return deg;
}

int16_t trig_sin_permille(int16_t deg)
{
    int16_t d = clamp_deg(deg);

    if (d > STRAIGHT_DEG) {
        d = (int16_t)(180 - d);
    }
    return SIN_TABLE[d];
}

int16_t trig_cos_permille(int16_t deg)
{
    int16_t d = clamp_deg(deg);

    if (d <= STRAIGHT_DEG) {
        return SIN_TABLE[STRAIGHT_DEG - d];
    }
    return (int16_t)(-SIN_TABLE[d - STRAIGHT_DEG]);
}

static int32_t abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* ---- scan sequencer --------------------------------------------------- */

static void build_angles(scan_t *s, uint8_t lo, uint8_t hi, uint8_t step)
{
    uint16_t a;
    uint8_t st = (step == 0U) ? 1U : step;

    s->angle_count = 0U;
    for (a = lo; (a <= hi) && (s->angle_count < SCAN_MAX_POINTS); a = (uint16_t)(a + st)) {
        s->angles[s->angle_count] = (uint8_t)a;
        s->angle_count++;
    }
}

static void reset_samples(scan_t *s)
{
    s->sample_count = 0U;
    s->valid_count = 0U;
    s->noecho_count = 0U;
    s->taken = 0U;
    s->measure_requested = false;
}

void scan_start(scan_t *s, const obstacle_config_t *cfg, uint32_t now_ms)
{
    s->cfg = *cfg;
    s->fine_phase = false;
    build_angles(s, cfg->coarse_min_deg, cfg->coarse_max_deg, cfg->coarse_step_deg);
    s->angle_index = 0U;
    s->servo_angle = (uint8_t)SCAN_FORWARD_DEG;
    s->state = SCAN_MOVING;
    s->state_since_ms = now_ms;
    s->settle_ms = 0U;
    s->request_seq = 0U;
    s->coarse_count = 0U;
    s->point_count = 0U;
    s->started_ms = now_ms;
    s->move_issued = false;
    s->request_ms = now_ms;
    s->fine_lo = 0U;
    s->fine_hi = 0U;
    reset_samples(s);
}

static uint16_t median_u16(uint16_t *v, uint8_t n)
{
    uint8_t i;
    uint8_t j;

    for (i = 1U; i < n; i++) {
        uint16_t key = v[i];

        for (j = i; (j > 0U) && (v[j - 1U] > key); j--) {
            v[j] = v[j - 1U];
        }
        v[j] = key;
    }
    return v[n / 2U];
}

static scan_point_t make_point(scan_t *s, uint8_t angle)
{
    scan_point_t p;

    p.angle_deg = angle;
    p.kind = SCAN_POINT_INVALID;
    p.distance_mm = 0U;

    if ((uint8_t)(s->valid_count * 2U) > s->taken) {
        p.distance_mm = median_u16(s->samples, s->valid_count);
        p.kind = (p.distance_mm < s->cfg.detect_range_mm) ? SCAN_POINT_OBSTACLE
                                                           : SCAN_POINT_FAR;
    } else if ((uint8_t)(s->noecho_count * 2U) > s->taken) {
        if (s->cfg.no_echo_is_clear) {
            p.kind = SCAN_POINT_FAR;
            p.distance_mm = US_MAX_RANGE_MM;
        } else {
            p.kind = SCAN_POINT_NOECHO;
        }
    } else {
        /* sensor errors or no majority: unusable */
    }
    return p;
}

static void record_sample(scan_t *s, const us_reading_t *r)
{
    s->taken++;
    if (r == NULL) {
        return;                                 /* a timeout: neither */
    }
    if ((r->status == US_VALID) && (s->valid_count < (uint8_t)(sizeof(s->samples) / sizeof(s->samples[0])))) {
        s->samples[s->valid_count] = r->distance_mm;
        s->valid_count++;
    } else if (r->status == US_TOO_CLOSE) {
        if (s->valid_count < (uint8_t)(sizeof(s->samples) / sizeof(s->samples[0]))) {
            s->samples[s->valid_count] = US_MIN_RANGE_MM;
            s->valid_count++;
        }
    } else if ((r->status == US_NO_ECHO) || (r->status == US_OUT_OF_RANGE)) {
        s->noecho_count++;
    } else {
        /* US_ERROR: counted as taken only */
    }
}

static void sort_by_angle(scan_point_t *p, uint8_t n)
{
    uint8_t i;
    uint8_t j;

    for (i = 1U; i < n; i++) {
        scan_point_t key = p[i];

        for (j = i; (j > 0U) && (p[j - 1U].angle_deg > key.angle_deg); j--) {
            p[j] = p[j - 1U];
        }
        p[j] = key;
    }
}

/* Final point list: fine results where they exist, coarse results elsewhere. */
static void merge_points(scan_t *s)
{
    scan_point_t merged[MERGE_CAPACITY];
    uint8_t n = 0U;
    uint8_t i;

    if (s->fine_phase) {
        for (i = 0U; (i < s->coarse_count) && (n < MERGE_CAPACITY); i++) {
            if ((s->coarse[i].angle_deg < s->fine_lo) || (s->coarse[i].angle_deg > s->fine_hi)) {
                merged[n] = s->coarse[i];
                n++;
            }
        }
        for (i = 0U; (i < s->point_count) && (n < MERGE_CAPACITY); i++) {
            merged[n] = s->points[i];
            n++;
        }
    } else {
        for (i = 0U; (i < s->coarse_count) && (n < MERGE_CAPACITY); i++) {
            merged[n] = s->coarse[i];
            n++;
        }
    }
    sort_by_angle(merged, n);
    if (n > SCAN_MAX_POINTS) {
        n = (uint8_t)SCAN_MAX_POINTS;
    }
    for (i = 0U; i < n; i++) {
        s->points[i] = merged[i];
    }
    s->point_count = n;
}

static void start_return(scan_t *s)
{
    merge_points(s);
    s->state = SCAN_RETURNING;
    s->move_issued = false;
}

static void end_of_phase(scan_t *s)
{
    if (!s->fine_phase) {
        uint8_t i;
        bool any = false;
        uint8_t lo = 0U;
        uint8_t hi = 0U;

        for (i = 0U; i < s->coarse_count; i++) {
            if (s->coarse[i].kind == SCAN_POINT_OBSTACLE) {
                if (!any || (s->coarse[i].angle_deg < lo)) {
                    lo = s->coarse[i].angle_deg;
                }
                if (!any || (s->coarse[i].angle_deg > hi)) {
                    hi = s->coarse[i].angle_deg;
                }
                any = true;
            }
        }
        if (!any) {
            start_return(s);
            return;
        }
        {
            int16_t flo = (int16_t)lo - (int16_t)s->cfg.coarse_step_deg;
            int16_t fhi = (int16_t)hi + (int16_t)s->cfg.coarse_step_deg;

            s->fine_lo = servo_clamp_angle(&s->cfg.servo, flo);
            s->fine_hi = servo_clamp_angle(&s->cfg.servo, fhi);
        }
        build_angles(s, s->fine_lo, s->fine_hi, s->cfg.fine_step_deg);
        s->fine_phase = true;
        s->angle_index = 0U;
        s->point_count = 0U;
        s->state = SCAN_MOVING;
        s->move_issued = false;
        return;
    }
    start_return(s);
}

static void finish_point(scan_t *s)
{
    scan_point_t p = make_point(s, s->angles[s->angle_index]);

    if (s->fine_phase) {
        if (s->point_count < SCAN_MAX_POINTS) {
            s->points[s->point_count] = p;
            s->point_count++;
        }
    } else if (s->coarse_count < SCAN_MAX_POINTS) {
        s->coarse[s->coarse_count] = p;
        s->coarse_count++;
    } else {
        /* capacity reached: further points are dropped */
    }

    reset_samples(s);
    if ((uint8_t)(s->angle_index + 1U) < s->angle_count) {
        s->angle_index++;
        s->state = SCAN_MOVING;
        s->move_issued = false;
    } else {
        end_of_phase(s);
    }
}

static void command_move(scan_t *s, uint32_t now_ms, uint8_t target, scan_action_t *act)
{
    s->settle_ms = servo_settle_ms(&s->cfg.servo, s->servo_angle, target);
    s->servo_angle = target;
    s->state_since_ms = now_ms;
    s->move_issued = true;
    act->servo_move = true;
    act->servo_angle = target;
}

bool scan_update(scan_t *s, uint32_t now_ms, const us_reading_t *reading,
                 scan_action_t *act)
{
    act->servo_move = false;
    act->servo_angle = s->servo_angle;
    act->request_measure = false;

    switch (s->state) {
    case SCAN_MOVING:
        if (!s->move_issued) {
            command_move(s, now_ms, s->angles[s->angle_index], act);
        } else if ((now_ms - s->state_since_ms) >= s->settle_ms) {
            s->state = SCAN_MEASURING;
            reset_samples(s);
        } else {
            /* still settling */
        }
        return false;

    case SCAN_MEASURING:
        if (!s->measure_requested) {
            s->measure_requested = true;
            s->request_seq = reading->seq;
            s->request_ms = now_ms;
            act->request_measure = true;
            return false;
        }
        if (reading->seq != s->request_seq) {
            record_sample(s, reading);
        } else if ((now_ms - s->request_ms) >= s->cfg.measure_timeout_ms) {
            record_sample(s, NULL);
        } else {
            return false;                       /* still waiting for the echo */
        }
        if (s->taken >= s->cfg.samples_per_point) {
            finish_point(s);
        } else {
            s->request_seq = reading->seq;
            s->request_ms = now_ms;
            act->request_measure = true;
        }
        return false;

    case SCAN_RETURNING:
        if (!s->move_issued) {
            command_move(s, now_ms, (uint8_t)SCAN_FORWARD_DEG, act);
        } else if ((now_ms - s->state_since_ms) >= s->settle_ms) {
            s->state = SCAN_DONE;
            return true;
        } else {
            /* still returning */
        }
        return false;

    case SCAN_DONE:
        return true;

    default:
        return false;
    }
}

/* ---- profile ----------------------------------------------------------- */

static int32_t lateral_mm(const obstacle_config_t *cfg, uint8_t angle, uint16_t dist)
{
    int32_t x = ((int32_t)dist * trig_cos_permille(angle)) / PERMILLE;

    return cfg->left_is_high_angle ? x : -x;
}

static int32_t forward_of(uint8_t angle, uint16_t dist)
{
    return ((int32_t)dist * trig_sin_permille(angle)) / PERMILLE;
}

/* Half the angular gap to a neighbouring ray, as millimetres at 'dist': the
 * unbiased uncertainty of where the obstacle edge really is. */
static int32_t edge_pad_mm(uint8_t hit_angle, uint8_t miss_angle, uint16_t dist)
{
    int32_t gap = abs32((int32_t)hit_angle - (int32_t)miss_angle);

    return ((int32_t)dist * gap * DEG_TO_RAD_MICRO) / ((int32_t)HALF * MICRO);
}

/* Free width on one side of the obstacle: walk outward through confirmed-free
 * rays and measure, at the obstacle's depth, how far the last of them lies
 * beyond its edge.  Unknown or blocked next to the edge means no clearance. */
static uint16_t side_clearance(const obstacle_config_t *cfg, const scan_point_t *pts,
                               uint8_t n, int step, uint8_t from, int32_t edge_x,
                               int32_t edge_y, bool edge_is_left)
{
    int idx = (int)from + step;
    int last_far = -1;
    uint8_t far_run = 0U;

    while ((idx >= 0) && (idx < (int)n) && (pts[idx].kind == SCAN_POINT_FAR)) {
        last_far = idx;
        far_run++;
        idx += step;
    }
    if (last_far < 0) {
        return 0U;
    }
    if ((cfg->sweep_clear_min_rays > 0U) && (far_run >= cfg->sweep_clear_min_rays) &&
        ((idx < 0) || (idx >= (int)n))) {
        return US_MAX_RANGE_MM;                 /* clear all the way to the end of the sweep */
    }
    {
        int16_t sin_pm = trig_sin_permille(pts[last_far].angle_deg);
        int32_t ray_x;
        int32_t gap;

        if (sin_pm == 0) {
            return 0U;
        }
        ray_x = (edge_y * trig_cos_permille(pts[last_far].angle_deg)) / sin_pm;
        if (!cfg->left_is_high_angle) {
            ray_x = -ray_x;
        }
        gap = edge_is_left ? (edge_x - ray_x) : (ray_x - edge_x);
        return (uint16_t)((gap > 0) ? gap : 0);
    }
}

void obstacle_profile_build(const obstacle_config_t *cfg, const scan_point_t *pts,
                            uint8_t n, obstacle_profile_t *p)
{
    uint8_t i;
    bool real = false;
    int best_lo = -1;
    int best_hi = -1;
    int32_t best_key = 0;
    int32_t best_dist = 0;

    p->valid = false;
    p->obstacle_found = false;
    p->centre_clear = false;
    p->closest_angle_deg = 0U;
    p->closest_mm = 0U;
    p->centre_x_mm = 0;
    p->forward_mm = 0U;
    p->left_edge_x_mm = 0;
    p->right_edge_x_mm = 0;
    p->width_mm = 0U;
    p->left_clearance_mm = 0U;
    p->right_clearance_mm = 0U;

    for (i = 0U; i < n; i++) {
        if ((pts[i].kind == SCAN_POINT_OBSTACLE) || (pts[i].kind == SCAN_POINT_FAR)) {
            real = true;
        }
    }
    if (!real) {
        return;                                 /* silence is not free space */
    }
    p->valid = true;

    {
        int32_t best = 1000;

        for (i = 0U; i < n; i++) {
            int32_t off = abs32((int32_t)pts[i].angle_deg - STRAIGHT_DEG);

            if (off < best) {
                best = off;
                p->centre_clear = (pts[i].kind == SCAN_POINT_FAR);
            }
        }
    }

    /* The obstacle is the run of obstacle points nearest straight ahead. */
    i = 0U;
    while (i < n) {
        if (pts[i].kind == SCAN_POINT_OBSTACLE) {
            uint8_t j = i;
            int32_t key = 1000;
            int32_t dmin = 100000;
            uint8_t k;

            while (((uint8_t)(j + 1U) < n) && (pts[j + 1U].kind == SCAN_POINT_OBSTACLE)) {
                j++;
            }
            for (k = i; k <= j; k++) {
                int32_t off = abs32((int32_t)pts[k].angle_deg - STRAIGHT_DEG);

                if (off < key) {
                    key = off;
                }
                if ((int32_t)pts[k].distance_mm < dmin) {
                    dmin = pts[k].distance_mm;
                }
            }
            if ((best_lo < 0) || (key < best_key) || ((key == best_key) && (dmin < best_dist))) {
                best_lo = i;
                best_hi = j;
                best_key = key;
                best_dist = dmin;
            }
            i = (uint8_t)(j + 1U);
        } else {
            i++;
        }
    }
    if (best_lo < 0) {
        return;
    }

    {
        const bool low_is_left = !cfg->left_is_high_angle;
        int32_t x_min = 100000;
        int32_t x_max = -100000;
        int32_t y_min = 100000;
        int32_t pad_lo = 0;
        int32_t pad_hi = 0;
        int32_t left_pad;
        int32_t right_pad;
        int32_t left_edge;
        int32_t right_edge;
        int32_t y_lo = forward_of(pts[best_lo].angle_deg, pts[best_lo].distance_mm);
        int32_t y_hi = forward_of(pts[best_hi].angle_deg, pts[best_hi].distance_mm);
        uint8_t k;

        p->obstacle_found = true;
        p->closest_mm = 0xFFFFU;
        for (k = (uint8_t)best_lo; k <= (uint8_t)best_hi; k++) {
            int32_t x = lateral_mm(cfg, pts[k].angle_deg, pts[k].distance_mm);
            int32_t y = forward_of(pts[k].angle_deg, pts[k].distance_mm);

            if (x < x_min) { x_min = x; }
            if (x > x_max) { x_max = x; }
            if (y < y_min) { y_min = y; }
            if (pts[k].distance_mm < p->closest_mm) {
                p->closest_mm = pts[k].distance_mm;
                p->closest_angle_deg = pts[k].angle_deg;
            }
        }
        if (best_lo > 0) {
            pad_lo = edge_pad_mm(pts[best_lo].angle_deg, pts[best_lo - 1].angle_deg,
                                 pts[best_lo].distance_mm);
        }
        if ((best_hi + 1) < (int)n) {
            pad_hi = edge_pad_mm(pts[best_hi].angle_deg, pts[best_hi + 1].angle_deg,
                                 pts[best_hi].distance_mm);
        }
        left_pad = low_is_left ? pad_lo : pad_hi;
        right_pad = low_is_left ? pad_hi : pad_lo;
        left_edge = x_min - left_pad;
        right_edge = x_max + right_pad;
        if ((cfg->max_width_mm > 0U) && ((right_edge - left_edge) > (int32_t)cfg->max_width_mm)) {
            const int32_t mid = (left_edge + right_edge) / HALF;

            left_edge = mid - ((int32_t)cfg->max_width_mm / HALF);
            right_edge = mid + ((int32_t)cfg->max_width_mm / HALF);
        }

        p->left_edge_x_mm = (int16_t)left_edge;
        p->right_edge_x_mm = (int16_t)right_edge;
        p->centre_x_mm = (int16_t)((left_edge + right_edge) / HALF);
        p->width_mm = (uint16_t)(right_edge - left_edge);
        p->forward_mm = (uint16_t)y_min;

        /* clearance beyond each end of the run */
        {
            uint16_t below = side_clearance(cfg, pts, n, -1, (uint8_t)best_lo,
                                            low_is_left ? left_edge : right_edge, y_lo,
                                            low_is_left);
            uint16_t above = side_clearance(cfg, pts, n, +1, (uint8_t)best_hi,
                                            low_is_left ? right_edge : left_edge, y_hi,
                                            !low_is_left);

            p->left_clearance_mm = low_is_left ? below : above;
            p->right_clearance_mm = low_is_left ? above : below;
        }
    }
}

/* ---- bypass plan ------------------------------------------------------- */

static void add_step(bypass_plan_t *plan, bool is_turn, int32_t value)
{
    if (plan->step_count < PLAN_MAX_STEPS) {
        plan->steps[plan->step_count].is_turn = is_turn;
        plan->steps[plan->step_count].value = (int16_t)value;
        plan->step_count++;
    }
}

static void plan_clear(bypass_plan_t *plan, plan_action_t action, plan_reason_t reason)
{
    plan->action = action;
    plan->reason = reason;
    plan->step_count = 0U;
    plan->return_leg_mm = 0U;
}

void obstacle_plan(const obstacle_config_t *cfg, const obstacle_profile_t *p,
                   bypass_plan_t *plan)
{
    const int32_t half = (int32_t)cfg->robot_width_mm / HALF;
    const int32_t corridor = half + (int32_t)cfg->margin_mm;
    const int32_t required = (int32_t)cfg->robot_width_mm + (2 * (int32_t)cfg->margin_mm);
    int32_t lateral_left;
    int32_t lateral_right;
    int32_t forward;
    bool left_ok;
    bool right_ok;
    bool go_left;

    plan_clear(plan, PLAN_STOP, PLAN_REASON_INVALID_SCAN);

    if (!p->valid) {
        return;
    }
    if (!p->obstacle_found) {
        if (p->centre_clear) {
            plan_clear(plan, PLAN_CONTINUE, PLAN_REASON_NO_OBSTACLE);
        }
        return;                                 /* cannot confirm the way is free */
    }
    if ((p->left_edge_x_mm > corridor) || (p->right_edge_x_mm < -corridor)) {
        plan_clear(plan, PLAN_CONTINUE, PLAN_REASON_NO_OBSTACLE);
        return;                                 /* beside the path, not in it */
    }
    if ((int32_t)p->forward_mm < (int32_t)cfg->min_turn_distance_mm) {
        plan_clear(plan, PLAN_REVERSE, PLAN_REASON_TOO_CLOSE);
        add_step(plan, false, -((int32_t)cfg->min_turn_distance_mm - (int32_t)p->forward_mm +
                                (int32_t)cfg->margin_mm));
        return;
    }

    lateral_left = corridor - p->left_edge_x_mm;
    lateral_right = corridor + p->right_edge_x_mm;
    left_ok = ((int32_t)p->left_clearance_mm >= required) &&
              (lateral_left <= (int32_t)cfg->recovery_limit_mm);
    right_ok = ((int32_t)p->right_clearance_mm >= required) &&
               (lateral_right <= (int32_t)cfg->recovery_limit_mm);

    if (!left_ok && !right_ok) {
        bool room = ((int32_t)p->left_clearance_mm >= required) ||
                    ((int32_t)p->right_clearance_mm >= required);

        plan_clear(plan, PLAN_STOP,
                   room ? PLAN_REASON_DETOUR_TOO_LARGE : PLAN_REASON_NO_CLEARANCE);
        return;
    }
    go_left = left_ok && (!right_ok || (lateral_left <= lateral_right));

    forward = (int32_t)p->forward_mm + (int32_t)cfg->assumed_depth_mm +
              (int32_t)cfg->robot_length_mm + (int32_t)cfg->margin_mm +
              (int32_t)cfg->forward_extra_mm;

    plan_clear(plan, go_left ? PLAN_BYPASS_LEFT : PLAN_BYPASS_RIGHT, PLAN_REASON_OK);
    {
        const int32_t sign = go_left ? 1 : -1;
        const int32_t lateral = go_left ? lateral_left : lateral_right;

        add_step(plan, true, sign * 90);
        add_step(plan, false, lateral);
        add_step(plan, true, -sign * 90);
        add_step(plan, false, forward);
        add_step(plan, true, -sign * 90);
        plan->return_leg_mm = (uint16_t)lateral;
    }
}

/* How far the outermost rays on one end of the sweep read; a clear or no-echo ray
 * counts as far.  'from_low_end' picks the low angles. */
static int32_t end_distance_mm(const scan_point_t *pts, uint8_t n, bool from_low_end)
{
    int32_t sum = 0;
    uint8_t taken = 0U;
    uint8_t i;

    for (i = 0U; (i < n) && (taken < 2U); i++) {
        const scan_point_t *q = from_low_end ? &pts[i] : &pts[n - 1U - i];

        sum += (q->kind == SCAN_POINT_OBSTACLE) ? (int32_t)q->distance_mm : (int32_t)US_MAX_RANGE_MM;
        taken++;
    }
    return (taken == 0U) ? 0 : (sum / (int32_t)taken);
}

bool obstacle_blind_plan(const obstacle_config_t *cfg, const scan_point_t *pts, uint8_t n,
                         const obstacle_profile_t *p, bypass_plan_t *plan)
{
    const int32_t half = (int32_t)cfg->robot_width_mm / HALF;
    const int32_t corridor = half + (int32_t)cfg->margin_mm;
    int32_t lateral_left;
    int32_t lateral_right;
    int32_t low_end;
    int32_t high_end;
    int32_t right_end;
    int32_t left_end;
    bool go_right;
    int32_t lateral;
    int32_t forward;
    int32_t sign;

    if (!p->valid || !p->obstacle_found || (n == 0U)) {
        return false;
    }
    if ((p->left_edge_x_mm > corridor) || (p->right_edge_x_mm < -corridor)) {
        return false;                           /* beside the path, not in it */
    }
    if ((int32_t)p->forward_mm < (int32_t)cfg->min_turn_distance_mm) {
        return false;                           /* too close to turn */
    }

    lateral_left = corridor - p->left_edge_x_mm;
    lateral_right = corridor + p->right_edge_x_mm;
    low_end = end_distance_mm(pts, n, true);
    high_end = end_distance_mm(pts, n, false);
    right_end = cfg->left_is_high_angle ? low_end : high_end;
    left_end = cfg->left_is_high_angle ? high_end : low_end;
    go_right = (right_end >= left_end);

    if (cfg->blind_side != 0) {
        go_right = (cfg->blind_side < 0);       /* the course says which side is free */
    } else if (go_right && (lateral_right > (int32_t)cfg->recovery_limit_mm)) {
        go_right = false;
    } else if (!go_right && (lateral_left > (int32_t)cfg->recovery_limit_mm)) {
        go_right = true;
    } else {
        /* the preferred side stays */
    }
    lateral = go_right ? lateral_right : lateral_left;
    if (lateral > (int32_t)cfg->recovery_limit_mm) {
        return false;                           /* too far round on both sides */
    }
    sign = go_right ? -1 : 1;
    forward = (int32_t)p->forward_mm + (int32_t)cfg->assumed_depth_mm +
              (int32_t)cfg->robot_length_mm + (int32_t)cfg->margin_mm +
              (int32_t)cfg->forward_extra_mm;

    plan_clear(plan, go_right ? PLAN_BYPASS_RIGHT : PLAN_BYPASS_LEFT, PLAN_REASON_OK);
    add_step(plan, true, sign * 90);
    add_step(plan, false, lateral);
    add_step(plan, true, -sign * 90);
    add_step(plan, false, forward);
    add_step(plan, true, -sign * 90);
    plan->return_leg_mm = (uint16_t)lateral;
    return true;
}
