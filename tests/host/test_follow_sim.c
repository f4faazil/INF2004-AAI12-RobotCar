/*
 * test_follow_sim.c
 * Closed-loop SIMULATION of line following: a kinematic differential-drive car
 * with a three-sensor bar drives a track containing a straight, an S-curve and
 * a barcode strip, using the real calibration + line_nav code at 1 ms sample
 * frames and 10 ms steering steps.  It shows how the steering behaves for
 * different sensor spacings and whether a barcode beside the line disturbs it.
 *
 * SIMULATION ONLY: ideal kinematics (a 20 ms command delay and a 50 ms wheel
 * lag are modelled), a lateral-only spot model, uniform ADC noise.  The real
 * sensor spacing, mounting height and track are not known yet; the numbers
 * here show trends and the shape of the problem, not performance on the car.
 */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "barcode_fixtures.h"
#include "line_nav.h"
#include "test_support.h"

#define PI_D            (3.14159265358979)
#define WHITE           (150.0)
#define BLACK           (4000.0)
#define LINE_HALF_MM    (9.0)
#define SPOT_MM         (2.1)
#define TRACK_MM        (150.0)
#define SENSOR_AHEAD_MM (70.0)
#define STRIP_NEAR_MM   (29.0)      /* barcode near edge, from the line centre */
#define STRIP_WIDTH_MM  (20.0)
#define CRUISE          (150.0)
#define CURVE_SPEED     (90.0)

static const nav_map_entry_t MAP[] = { { 'A', NAV_CMD_LEFT } };

static uint32_t g_rng = 99U;

static double rnd(void)
{
    g_rng = (g_rng * 1664525U) + 1013904223U;
    return ((double)(g_rng >> 8) / 8388608.0) - 1.0;
}

/* ---- the track ------------------------------------------------------------ */

#define S_START     (500.0)
#define S_LEN       (600.0)
#define S_AMP       (30.0)
#define TRACK_END   (1900.0)

static double line_x(double y)
{
    if ((y < S_START) || (y > (S_START + S_LEN))) {
        return 0.0;
    }
    return S_AMP * sin(2.0 * PI_D * (y - S_START) / S_LEN);
}

/* Barcode on the right of the line, on the final straight: bars are black
 * between the edges in 'bar_edges' (mm along y). */
typedef struct {
    bool   enabled;
    double y0;
    double edge[BARCODE_ELEMENT_COUNT + 1U];
} strip_t;

static strip_t make_strip(char letter, double y0, double symbol_mm)
{
    strip_t s;
    size_t f;
    size_t i;
    double total = 0.0;
    double acc = 0.0;

    s.enabled = true;
    s.y0 = y0;
    for (f = 0U; f < BARCODE_FIXTURE_COUNT; f++) {
        if ((BARCODE_FIXTURES[f].expected == letter) &&
            (BARCODE_FIXTURES[f].scale[0] == 'l')) {
            break;
        }
    }
    for (i = 0U; i < BARCODE_ELEMENT_COUNT; i++) {
        total += BARCODE_FIXTURES[f].runs[i];
    }
    s.edge[0] = y0;
    for (i = 0U; i < BARCODE_ELEMENT_COUNT; i++) {
        acc += BARCODE_FIXTURES[f].runs[i];
        s.edge[i + 1U] = y0 + ((acc / total) * symbol_mm);
    }
    return s;
}

static double overlap(double a0, double a1, double b0, double b1)
{
    double lo = (a0 > b0) ? a0 : b0;
    double hi = (a1 < b1) ? a1 : b1;

    return (hi > lo) ? (hi - lo) : 0.0;
}

/* Black coverage (0..1) under a sensor at world (x, y). */
static double coverage(double x, double y, const strip_t *strip)
{
    double lx = line_x(y);
    double cov = overlap(x - (SPOT_MM / 2.0), x + (SPOT_MM / 2.0),
                         lx - LINE_HALF_MM, lx + LINE_HALF_MM) / SPOT_MM;

    if (strip->enabled && (x > (lx + STRIP_NEAR_MM - 1.0)) &&
        (x < (lx + STRIP_NEAR_MM + STRIP_WIDTH_MM + 1.0))) {
        size_t i;

        for (i = 0U; i < BARCODE_ELEMENT_COUNT; i += 2U) {
            cov += overlap(y - (SPOT_MM / 2.0), y + (SPOT_MM / 2.0), strip->edge[i],
                           strip->edge[i + 1U]) / SPOT_MM *
                   overlap(x - (SPOT_MM / 2.0), x + (SPOT_MM / 2.0), lx + STRIP_NEAR_MM,
                           lx + STRIP_NEAR_MM + STRIP_WIDTH_MM) / SPOT_MM;
        }
    }
    return (cov > 1.0) ? 1.0 : cov;
}

/* ---- the car ----------------------------------------------------------------- */

typedef struct {
    double x, y, psi;           /* psi: from +y toward +x (right) */
    double diff_applied;        /* right minus left wheel speed, after lag */
    double diff_pipe[3];        /* 20 ms command delay (2 control cycles) */
    double speed;
} car_t;

typedef struct {
    double max_err_mm;          /* centre sensor vs line, over the whole run */
    double max_err_curve_mm;
    int    lost_events;
    int    junction_events;
    int    barcodes;
    char   letter;
    double end_y;
    bool   finished;
} result_t;

/* A sensor layout: where the three sensors sit, and which do what. */
typedef struct {
    double off_mm[3];           /* lateral offset of LEFT, CENTRE, RIGHT; right positive */
    double ref_mm;              /* spacing that maps to +/-1000 in the position */
    uint8_t track_mask;
    uint8_t lane_mask;
    uint8_t junction_mask;
    int kp;
    int td;
} layout_t;

static layout_t layout_a(void)      /* three close sensors; cannot reach a barcode */
{
    layout_t l = { { -12.0, 0.0, 12.0 }, 12.0, 0x07U, 0x00U, 0x00U, 160, 120 };
    return l;
}

static layout_t layout_b(void)      /* symmetric and wide: reads a barcode, cannot track */
{
    layout_t l = { { -39.0, 0.0, 39.0 }, 39.0, 0x07U, 0x05U, 0x05U, 160, 120 };
    return l;
}

static layout_t layout_c(void)      /* close pair tracks, far sensor reads the barcode */
{
    layout_t l = { { -6.0, 6.0, 39.0 }, 6.0, 0x03U, 0x04U, 0x04U, 80, 120 };
    return l;
}

static line_nav_config_t make_cfg(const layout_t *lay)
{
    line_nav_config_t c;
    int ch;

    c.lost_frames = 5U;
    c.found_frames = 3U;
    c.junction_min_mm = 13U;
    c.junction_merge_ms = 30U;
    c.barcode_quiet_ms = 250U;
    c.instruction_ttl_ms = 20000U;
    c.hump_suppress_max_ms = 1000U;
    c.hump_fresh_ms = 100U;
    c.stale_ms = 50U;
    c.steer_kp_mm_s = (int16_t)lay->kp;
    c.steer_td_ms = (uint16_t)lay->td;
    c.steer_limit_mm_s = 150;
    c.map = MAP;
    c.map_len = 1U;
    for (ch = 0; ch < 3; ch++) {
        c.channel_pos_permille[ch] = (int16_t)lround(lay->off_mm[ch] / lay->ref_mm * 1000.0);
    }
    c.track_mask = lay->track_mask;
    c.lane_mask = lay->lane_mask;
    c.junction_mask = lay->junction_mask;
    return c;
}

static void sensors(const car_t *car, const layout_t *lay, const strip_t *strip, uint16_t adc[3])
{
    double cx = car->x + (SENSOR_AHEAD_MM * sin(car->psi));
    double cy = car->y + (SENSOR_AHEAD_MM * cos(car->psi));
    double rx = cos(car->psi);          /* the car's right-hand direction */
    double ry = -sin(car->psi);
    int ch;

    for (ch = 0; ch < 3; ch++) {
        double off = lay->off_mm[ch];
        double v = WHITE + ((BLACK - WHITE) * coverage(cx + (off * rx), cy + (off * ry), strip)) +
                   (rnd() * 30.0);

        adc[ch] = (uint16_t)((v < 0.0) ? 0.0 : ((v > 4095.0) ? 4095.0 : v));
    }
}

static result_t drive(const layout_t *lay, double speed_mm_s, const strip_t *strip, double start_off_mm)
{
    line_nav_config_t cfg = make_cfg(lay);
    line_nav_t nav;
    car_t car = { start_off_mm, 0.0, 0.0, 0.0, { 0.0, 0.0, 0.0 }, speed_mm_s };
    result_t r = { 0.0, 0.0, 0, 0, 0, '?', 0.0, false };
    uint32_t t = 1000U;
    uint32_t step;
    uint16_t adc[3];
    int i;

    line_nav_init(&nav, &cfg);
    /* calibrate on the bench readings: white 150, black 4000 */
    line_nav_calibrate_begin(&nav);
    for (i = 0; i < 20; i++) {
        uint16_t w[3] = { 150U, 150U, 150U };
        uint16_t b[3] = { 4000U, 4000U, 4000U };

        line_nav_push_frame(&nav, w, t++);
        line_nav_push_frame(&nav, b, t++);
    }
    (void)line_nav_calibrate_end(&nav);

    for (step = 0U; step < 40000U; step++) {            /* 40 s of 10 ms steps at most */
        line_nav_out_t out;
        double v = speed_mm_s;
        int f;

        for (f = 0; f < 10; f++) {
            sensors(&car, lay, strip, adc);
            line_nav_push_frame(&nav, adc, t++);
            car.diff_applied += (car.diff_pipe[0] - car.diff_applied) * 0.02;     /* ~50 ms lag */
            car.psi -= (car.diff_applied / TRACK_MM) * 0.001;
            car.x += v * sin(car.psi) * 0.001;
            car.y += v * cos(car.psi) * 0.001;
        }
        line_nav_step(&nav, t, (int16_t)v, &out);

        if ((out.events & NAV_EVENT_LINE_LOST) != 0U) { r.lost_events++; }
        if ((out.events & NAV_EVENT_JUNCTION) != 0U) { r.junction_events++; }
        if ((out.events & NAV_EVENT_BARCODE) != 0U) {
            nav_instruction_t ins;

            r.barcodes++;
            if (line_nav_take_instruction(&nav, t, &ins)) { r.letter = ins.letter; }
        }

        /* speed schedule as the Vehicle Controller does it */
        if (out.position_valid) {
            double e = fabs((double)out.position) / 1000.0;

            speed_mm_s = CRUISE - ((CRUISE - CURVE_SPEED) * e);
            car.diff_pipe[2] = car.diff_pipe[1];
            car.diff_pipe[1] = car.diff_pipe[0];
            car.diff_pipe[0] = (double)out.steer_diff_mm_s;
        } else {
            speed_mm_s = CURVE_SPEED;
        }
        (void)car.diff_pipe[2];

        {
            double sx = car.x + (SENSOR_AHEAD_MM * sin(car.psi));
            double sy = car.y + (SENSOR_AHEAD_MM * cos(car.psi));
            double err = fabs(sx - line_x(sy));

            if (err > r.max_err_mm) { r.max_err_mm = err; }
            if ((sy > S_START) && (sy < (S_START + S_LEN)) && (err > r.max_err_curve_mm)) {
                r.max_err_curve_mm = err;
            }
            if (car.y > TRACK_END) { r.finished = true; break; }
        }
    }
    r.end_y = car.y;
    return r;
}

/* ---- tests ------------------------------------------------------------------- */

/* Three sensors 12 mm apart (about half the line width) follow the straight
 * and the S-curve at 150 mm/s without losing the line.  SIMULATION. */
static void test_layout_a_close_sensors_follow_an_s_curve(void)
{
    strip_t none = { false, 0.0, { 0.0 } };
    layout_t lay = layout_a();
    result_t r = drive(&lay, CRUISE, &none, 0.0);

    printf("    layout A (12 mm): max error %.1f mm (curve %.1f), lost %d\n",
           r.max_err_mm, r.max_err_curve_mm, r.lost_events);
    CHECK(r.finished);
    CHECK_INT(0, r.lost_events);
    CHECK(r.max_err_mm < 8.0);
}

/* The recommended layout C (a close pair tracks, a far sensor reads the
 * barcode) follows the same course equally well, recovers from a start 10 mm
 * off the line, and works over the course speed range. */
static void test_layout_c_follows_and_recovers(void)
{
    strip_t none = { false, 0.0, { 0.0 } };
    layout_t lay = layout_c();
    result_t base = drive(&lay, CRUISE, &none, 0.0);
    result_t off = drive(&lay, CRUISE, &none, 10.0);
    result_t slow = drive(&lay, 100.0, &none, 0.0);
    result_t fast = drive(&lay, 200.0, &none, 0.0);

    printf("    layout C: 150 mm/s err %.1f | start-off err %.1f | 100 mm/s %.1f | 200 mm/s %.1f\n",
           base.max_err_mm, off.max_err_mm, slow.max_err_mm, fast.max_err_mm);
    CHECK(base.finished && off.finished && slow.finished && fast.finished);
    CHECK_INT(0, base.lost_events);
    CHECK_INT(0, off.lost_events);
    CHECK_INT(0, slow.lost_events);
    CHECK_INT(0, fast.lost_events);
    CHECK(base.max_err_mm < 8.0);
    CHECK(fast.max_err_mm < 10.0);
}

/* With layout C a barcode printed beside the line is read by the far sensor
 * (39 mm out, over the 29-49 mm strip) while the car keeps tracking; the right
 * letter comes out, no junction is invented and the line is not lost. */
static void test_layout_c_reads_a_barcode_beside_the_line(void)
{
    layout_t lay = layout_c();
    static const double SPEEDS[] = { 100.0, 150.0, 200.0 };
    size_t i;

    for (i = 0U; i < 3U; i++) {
        strip_t strip = make_strip('A', 1500.0, 141.0);
        result_t r = drive(&lay, SPEEDS[i], &strip, 0.0);

        printf("    layout C barcode at %.0f mm/s: decoded %d x '%c', junctions %d, lost %d, err %.1f\n",
               SPEEDS[i], r.barcodes, r.letter, r.junction_events, r.lost_events, r.max_err_mm);
        CHECK_INT(1, r.barcodes);
        CHECK_INT('A', r.letter);
        CHECK_INT(0, r.junction_events);
        CHECK_INT(0, r.lost_events);
        CHECK(r.finished);
    }
}

/* THE FINDING (documented, not a defect of the code): a symmetric bar wide
 * enough to read a barcode (39 mm) cannot track an 18 mm line - the line fits
 * in the gap between sensors, so position goes invalid and the line is
 * reported lost.  The layout, not the software, must change. */
static void test_layout_b_wide_symmetric_bar_cannot_track_the_line(void)
{
    strip_t none = { false, 0.0, { 0.0 } };
    layout_t lay = layout_b();
    result_t r = drive(&lay, CRUISE, &none, 0.0);

    printf("    layout B (39 mm): max error %.1f mm, lost %d, finished %d\n",
           r.max_err_mm, r.lost_events, r.finished);
    CHECK(r.lost_events > 0 || !r.finished || r.max_err_mm > 15.0);
}

/* A sensor pair spaced wider than the line leaves a blind gap: 20 mm apart the
 * line is lost on a curve even though the outer sensors are close.  This is the
 * rule behind layout C. */
static void test_a_sensor_gap_wider_than_the_line_loses_it(void)
{
    strip_t none = { false, 0.0, { 0.0 } };
    layout_t lay = { { -20.0, 0.0, 20.0 }, 20.0, 0x07U, 0x00U, 0x00U, 160, 120 };
    result_t r = drive(&lay, CRUISE, &none, 0.0);

    printf("    20 mm spacing: lost %d events, max error %.1f mm\n", r.lost_events, r.max_err_mm);
    CHECK(r.lost_events > 0);
}

int main(void)
{
    RUN(test_layout_a_close_sensors_follow_an_s_curve);
    RUN(test_layout_c_follows_and_recovers);
    RUN(test_layout_c_reads_a_barcode_beside_the_line);
    RUN(test_layout_b_wide_symmetric_bar_cannot_track_the_line);
    RUN(test_a_sensor_gap_wider_than_the_line_loses_it);
    return TEST_REPORT();
}
