/*
 * test_line_nav.c
 * Host tests for line_nav: calibration, steering, line lost/found, hump
 * suppression, barcode lanes, one-shot instructions and junction detection.
 * Frames are synthetic; barcode run widths come from the lecturer's PDFs
 * (barcode_fixtures.h).  Geometry is the assumed one documented in
 * line_nav.h.
 */
#include <stddef.h>
#include <stdint.h>

#include "barcode_fixtures.h"
#include "line_nav.h"
#include "test_support.h"

#define WHITE   (150U)
#define BLACK   (4000U)

/* test-only letter map; the robot's own table is in robot_config.h */
static const nav_map_entry_t MAP[] = {
    { 'A', NAV_CMD_LEFT }, { 'Z', NAV_CMD_RIGHT },
    { 'W', NAV_CMD_STRAIGHT }, { 'X', NAV_CMD_UTURN }
};

static line_nav_config_t make_cfg(void)
{
    line_nav_config_t c;

    c.lost_frames = 3U;
    c.found_frames = 2U;
    c.junction_min_mm = 13U;
    c.junction_merge_ms = 30U;
    c.barcode_quiet_ms = 400U;
    c.instruction_ttl_ms = 20000U;
    c.hump_suppress_max_ms = 1000U;
    c.hump_fresh_ms = 100U;
    c.stale_ms = 50U;
    c.steer_kp_mm_s = 100;
    c.steer_td_ms = 0U;
    c.steer_limit_mm_s = 150;
    c.map = MAP;
    c.map_len = (uint8_t)(sizeof(MAP) / sizeof(MAP[0]));
    c.channel_pos_permille[LINE_CHANNEL_LEFT] = -1000;
    c.channel_pos_permille[LINE_CHANNEL_CENTRE] = 0;
    c.channel_pos_permille[LINE_CHANNEL_RIGHT] = 1000;
    c.track_mask = LINE_CHANNEL_ALL_MASK;
    c.lane_mask = (uint8_t)((1U << LINE_CHANNEL_LEFT) | (1U << LINE_CHANNEL_RIGHT));
    c.junction_mask = (uint8_t)((1U << LINE_CHANNEL_LEFT) | (1U << LINE_CHANNEL_RIGHT));
    c.fixed_white = 0U;
    c.fixed_black = 0U;
    c.fixed_mask = 0U;
    c.smooth_shift = 0U;
    c.barcode_bars = 0U;
    c.barcode_min_bar = 0U;
    return c;
}

/*
 * Layout C: the closed-loop simulation shows an 18 mm line cannot be tracked
 * by sensors more than about 16 mm apart, yet a barcode sits 29-49 mm from the
 * line.  So two close sensors (LEFT at -6 mm, CENTRE at +6 mm) track the line
 * and the RIGHT sensor, 39 mm out, reads the barcode and detects the branch
 * of a junction.
 */
static line_nav_config_t layout_c_cfg(void)
{
    line_nav_config_t c = make_cfg();

    c.channel_pos_permille[LINE_CHANNEL_LEFT] = -1000;
    c.channel_pos_permille[LINE_CHANNEL_CENTRE] = 1000;
    c.channel_pos_permille[LINE_CHANNEL_RIGHT] = 6500;
    c.track_mask = (uint8_t)((1U << LINE_CHANNEL_LEFT) | (1U << LINE_CHANNEL_CENTRE));
    c.lane_mask = (uint8_t)(1U << LINE_CHANNEL_RIGHT);
    c.junction_mask = (uint8_t)(1U << LINE_CHANNEL_RIGHT);
    return c;
}

typedef struct {
    line_nav_t nav;
    uint32_t   t;       /* ms */
} rig_t;

static void frame(rig_t *r, uint16_t l, uint16_t c, uint16_t rt)
{
    uint16_t adc[LINE_CHANNEL_COUNT];

    adc[LINE_CHANNEL_LEFT] = l;
    adc[LINE_CHANNEL_CENTRE] = c;
    adc[LINE_CHANNEL_RIGHT] = rt;
    line_nav_push_frame(&r->nav, adc, r->t);
    r->t++;
}

static void frames(rig_t *r, unsigned n, uint16_t l, uint16_t c, uint16_t rt)
{
    unsigned i;

    for (i = 0U; i < n; i++) {
        frame(r, l, c, rt);
    }
}

/* All three sensors calibrate against 150 (white) and 4000 (black). */
static void rig_init_with(rig_t *r, const line_nav_config_t *cfg)
{
    line_nav_init(&r->nav, cfg);
    r->t = 1000U;
    line_nav_calibrate_begin(&r->nav);
    frames(r, 20U, WHITE, WHITE, WHITE);
    frames(r, 20U, BLACK, BLACK, BLACK);
    frames(r, 20U, WHITE, WHITE, WHITE);
    (void)line_nav_calibrate_end(&r->nav);
}

static void rig_init(rig_t *r)
{
    line_nav_config_t cfg = make_cfg();

    rig_init_with(r, &cfg);
}

/* One 10 ms steering step at 'speed'; the frames of that interval must have
 * been pushed already. */
static line_nav_out_t step(rig_t *r, int16_t speed)
{
    line_nav_out_t out;

    line_nav_step(&r->nav, r->t, speed, &out);
    return out;
}

/* Hold a sensor pattern for 10 ms then step. */
static line_nav_out_t hold(rig_t *r, uint16_t l, uint16_t c, uint16_t rt, int16_t speed)
{
    frames(r, 10U, l, c, rt);
    return step(r, speed);
}

/*
 * Drive a Code 39 symbol past one outer sensor with the centre sensor on the
 * line: 'fpn' frames per narrow element.  Returns the events seen.
 */
static uint8_t drive_barcode_at(rig_t *r, const barcode_fixture_t *fx, unsigned channel_mask,
                                unsigned fpn, int16_t speed)
{
    unsigned i;
    uint8_t events = 0U;
    uint16_t pat[LINE_CHANNEL_COUNT];
    double scale = (double)fpn / (double)fx->runs[0];
    /* a RIGHT sensor that tracks and reads no barcode stays on the line, as on the car */
    const bool right_on_line = (((r->nav.cfg.track_mask >> LINE_CHANNEL_RIGHT) & 1U) != 0U) &&
                               (((r->nav.cfg.lane_mask >> LINE_CHANNEL_RIGHT) & 1U) == 0U);

    for (i = 0U; i <= (BARCODE_ELEMENT_COUNT + 1U); i++) {
        unsigned n = (i == 0U) ? 40U : ((i == (BARCODE_ELEMENT_COUNT + 1U)) ? 80U :
                     (unsigned)((fx->runs[i - 1U] * scale) + 0.5));
        bool black = (i != 0U) && (i != (BARCODE_ELEMENT_COUNT + 1U)) &&
                     (((i - 1U) % 2U) == 0U);
        unsigned k;

        pat[LINE_CHANNEL_LEFT] = WHITE;
        pat[LINE_CHANNEL_RIGHT] = right_on_line ? BLACK : WHITE;
        pat[LINE_CHANNEL_CENTRE] = BLACK;
        if ((channel_mask & (1U << LINE_CHANNEL_LEFT)) != 0U) {
            pat[LINE_CHANNEL_LEFT] = black ? BLACK : WHITE;
        }
        if ((channel_mask & (1U << LINE_CHANNEL_RIGHT)) != 0U) {
            pat[LINE_CHANNEL_RIGHT] = black ? BLACK : WHITE;
        }
        for (k = 0U; k < n; k++) {
            line_nav_push_frame(&r->nav, pat, r->t);
            r->t++;
            if ((k % 10U) == 9U) {
                events |= step(r, speed).events;
            }
        }
    }
    return events;
}

static uint8_t drive_barcode_on(rig_t *r, const barcode_fixture_t *fx, unsigned channel_mask,
                                unsigned fpn)
{
    return drive_barcode_at(r, fx, channel_mask, fpn, 150);
}

static uint8_t drive_barcode(rig_t *r, const barcode_fixture_t *fx, unsigned channel,
                             unsigned fpn)
{
    return drive_barcode_on(r, fx, 1U << channel, fpn);
}

static const barcode_fixture_t *fixture(char letter, const char *scale)
{
    size_t i;

    for (i = 0U; i < BARCODE_FIXTURE_COUNT; i++) {
        if ((BARCODE_FIXTURES[i].expected == letter) &&
            (BARCODE_FIXTURES[i].scale[0] == scale[0])) {
            return &BARCODE_FIXTURES[i];
        }
    }
    return NULL;
}

/* ---- calibration ------------------------------------------------------ */

/* The stuck left sensor (reads ~4000 on both) is left out; centre and right
 * are trusted.  Catches: steering on a sensor that cannot tell black from
 * white (bench: the left IR read 4000+ on white). */
static void test_calibration_excludes_a_stuck_sensor(void)
{
    rig_t r;
    line_nav_config_t cfg = make_cfg();

    line_nav_init(&r.nav, &cfg);
    r.t = 0U;
    line_nav_calibrate_begin(&r.nav);
    frames(&r, 20U, 4001U, WHITE, 161U);
    frames(&r, 20U, 3998U, 3997U, 4088U);
    CHECK(line_nav_calibrate_end(&r.nav));
    CHECK_INT(0x06, r.nav.valid_mask);
}

/* Without contrast on the centre sensor the car cannot follow a line:
 * calibration fails and nothing is trusted. */
static void test_calibration_fails_without_centre_contrast(void)
{
    rig_t r;
    line_nav_config_t cfg = make_cfg();

    line_nav_init(&r.nav, &cfg);
    r.t = 0U;
    line_nav_calibrate_begin(&r.nav);
    frames(&r, 40U, WHITE, 4000U, WHITE);
    CHECK(!line_nav_calibrate_end(&r.nav));
    CHECK_INT(0, r.nav.valid_mask);
}

/* A barcode-only sensor with fixed readings (a digital DO sensor) is trusted
 * from start-up and after a sweep that never crossed black on it; the tracking
 * pair still has to show contrast.  Without the fixed readings the same sweep
 * leaves it out, and its barcodes would silently never decode. */
static void test_fixed_lane_sensor_is_trusted_without_seeing_black(void)
{
    rig_t r;
    line_nav_config_t cfg = layout_c_cfg();

    line_nav_init(&r.nav, &cfg);
    r.t = 0U;
    line_nav_calibrate_begin(&r.nav);
    frames(&r, 20U, WHITE, WHITE, WHITE);
    frames(&r, 20U, BLACK, BLACK, WHITE);               /* the far sensor never sees black */
    CHECK(line_nav_calibrate_end(&r.nav));
    CHECK_INT(0x03, r.nav.valid_mask);                  /* learned: the far sensor is left out */

    cfg.fixed_white = WHITE;
    cfg.fixed_black = BLACK;
    cfg.fixed_mask = (uint8_t)(1U << LINE_CHANNEL_RIGHT);
    line_nav_init(&r.nav, &cfg);
    CHECK_INT(0x04, r.nav.valid_mask);                  /* trusted before any calibration */
    r.t = 0U;
    line_nav_calibrate_begin(&r.nav);
    frames(&r, 20U, WHITE, WHITE, WHITE);
    frames(&r, 20U, BLACK, BLACK, WHITE);
    CHECK(line_nav_calibrate_end(&r.nav));
    CHECK_INT(0x07, r.nav.valid_mask);
}

/* The tracking pair is still checked: with fixed readings for the far sensor,
 * a stuck pair still fails calibration. */
static void test_fixed_lane_sensor_does_not_hide_a_dead_tracking_pair(void)
{
    rig_t r;
    line_nav_config_t cfg = layout_c_cfg();

    cfg.fixed_white = WHITE;
    cfg.fixed_black = BLACK;
    cfg.fixed_mask = (uint8_t)(1U << LINE_CHANNEL_RIGHT);
    line_nav_init(&r.nav, &cfg);
    r.t = 0U;
    line_nav_calibrate_begin(&r.nav);
    frames(&r, 20U, 4000U, 4001U, WHITE);
    frames(&r, 20U, 4002U, 3998U, BLACK);
    CHECK(!line_nav_calibrate_end(&r.nav));
    CHECK_INT(0, r.nav.valid_mask);
}

/* Digital sensors: with fixed readings for every channel, a sweep over a constant
 * surface (the car held in the air) still leaves all three trusted, and a black
 * line under the centre sensor then gives a position. */
static void test_fixed_readings_on_every_channel_need_no_sweep(void)
{
    rig_t r;
    line_nav_config_t cfg = layout_c_cfg();
    line_nav_out_t out;

    cfg.fixed_white = WHITE;
    cfg.fixed_black = BLACK;
    cfg.fixed_mask = LINE_CHANNEL_ALL_MASK;
    line_nav_init(&r.nav, &cfg);
    CHECK_INT(0x07, r.nav.valid_mask);
    r.t = 0U;
    line_nav_calibrate_begin(&r.nav);
    frames(&r, 40U, WHITE, WHITE, WHITE);               /* nothing crosses the line */
    CHECK(line_nav_calibrate_end(&r.nav));
    CHECK_INT(0x07, r.nav.valid_mask);

    frames(&r, 10U, BLACK, BLACK, WHITE);
    out = step(&r, 100);
    CHECK(out.position_valid);
}

/* With smoothing, a sensor pair that flips between black and white every step
 * gives a settled in-between position instead of jumping the full spacing. */
static void test_smoothing_turns_flicker_into_a_small_position(void)
{
    static const unsigned SHIFTS[] = { 0U, 2U };
    unsigned s;

    for (s = 0U; s < 2U; s++) {
        rig_t r;
        line_nav_config_t cfg = layout_c_cfg();
        line_nav_out_t out = { 0 };
        int16_t last_abs = 0;
        unsigned k;

        cfg.smooth_shift = (uint8_t)SHIFTS[s];
        rig_init_with(&r, &cfg);
        for (k = 0U; k < 12U; k++) {
            frames(&r, 10U, (k % 2U) == 0U ? BLACK : WHITE, (k % 2U) == 0U ? WHITE : BLACK, WHITE);
            out = step(&r, 100);
            last_abs = (int16_t)((out.position < 0) ? -out.position : out.position);
        }
        CHECK(out.position_valid);
        if (SHIFTS[s] == 0U) {
            CHECK_INT(1000, last_abs);                  /* raw: the full jump every step */
        } else {
            CHECK(last_abs < 500);                      /* filtered: a small swing */
        }
    }
}

/* ---- steering --------------------------------------------------------- */

/* Line under the left sensor -> veer left (right wheel faster, positive
 * diff); under the right sensor -> veer right; centred -> straight.
 * Catches: steering away from the line. */
static void test_steering_direction(void)
{
    rig_t r;
    line_nav_out_t o;

    rig_init(&r);
    o = hold(&r, BLACK, WHITE, WHITE, 150);
    CHECK(o.position_valid);
    CHECK_INT(-1000, o.position);
    CHECK_INT(100, o.steer_diff_mm_s);

    o = hold(&r, WHITE, WHITE, BLACK, 150);
    CHECK_INT(1000, o.position);
    CHECK_INT(-100, o.steer_diff_mm_s);

    o = hold(&r, WHITE, BLACK, WHITE, 150);
    CHECK_INT(0, o.position);
    CHECK_INT(0, o.steer_diff_mm_s);
}

/* The steering request is limited. */
static void test_steering_is_limited(void)
{
    rig_t r;
    line_nav_config_t cfg = make_cfg();
    line_nav_out_t o;

    line_nav_init(&r.nav, &cfg);
    r.nav.cfg.steer_kp_mm_s = 400;
    r.t = 1000U;
    line_nav_calibrate_begin(&r.nav);
    frames(&r, 20U, WHITE, WHITE, WHITE);
    frames(&r, 20U, BLACK, BLACK, BLACK);
    (void)line_nav_calibrate_end(&r.nav);
    o = hold(&r, BLACK, WHITE, WHITE, 150);
    CHECK_INT(150, o.steer_diff_mm_s);
}

/* The derivative term opposes motion toward the centre, damping overshoot. */
static void test_derivative_damps_steering(void)
{
    rig_t r;
    line_nav_out_t o;

    rig_init(&r);
    r.nav.cfg.steer_kp_mm_s = 100;
    r.nav.cfg.steer_td_ms = 10U;
    (void)hold(&r, WHITE, WHITE, BLACK, 150);              /* +1000 */
    o = hold(&r, WHITE, BLACK, WHITE, 150);                /* back to 0 */
    /* P term 0; D term = -kp x Td x (0 - 1000) / dt / 1000 = +100 for Td = dt */
    CHECK_INT(100, o.steer_diff_mm_s);
}

/* ---- line lost / found ------------------------------------------------- */

/* Lost after 3 white steps, reported once; found after 2 line steps, once. */
static void test_line_lost_and_found_events_are_debounced_and_single(void)
{
    rig_t r;
    line_nav_out_t o;

    rig_init(&r);
    (void)hold(&r, WHITE, BLACK, WHITE, 150);
    o = hold(&r, WHITE, WHITE, WHITE, 150);
    CHECK_INT(0, o.events & NAV_EVENT_LINE_LOST);
    o = hold(&r, WHITE, WHITE, WHITE, 150);
    CHECK_INT(0, o.events & NAV_EVENT_LINE_LOST);
    o = hold(&r, WHITE, WHITE, WHITE, 150);
    CHECK(o.events & NAV_EVENT_LINE_LOST);
    CHECK(o.line_lost);
    o = hold(&r, WHITE, WHITE, WHITE, 150);
    CHECK_INT(0, o.events & NAV_EVENT_LINE_LOST);          /* not repeated */

    o = hold(&r, WHITE, BLACK, WHITE, 150);
    CHECK_INT(0, o.events & NAV_EVENT_LINE_FOUND);
    o = hold(&r, WHITE, BLACK, WHITE, 150);
    CHECK(o.events & NAV_EVENT_LINE_FOUND);
    CHECK(!o.line_lost);
}

/* A hump that tilts the car can make the sensors read white.  While the hump
 * status is valid and fresh, line loss is masked, but only for about 1 s.
 * Catches: a false STOP on a hump, and (the Tribunal's other rule) a hump
 * context that hides a real line loss forever. */
static void test_hump_masks_line_loss_for_a_bounded_time(void)
{
    rig_t r;
    line_nav_out_t o;
    int i;
    int lost_at = -1;

    rig_init(&r);
    (void)hold(&r, WHITE, BLACK, WHITE, 150);
    for (i = 1; i <= 150; i++) {
        line_nav_set_hump(&r.nav, true, r.t);              /* fresh every step */
        o = hold(&r, WHITE, WHITE, WHITE, 150);
        if ((o.events & NAV_EVENT_LINE_LOST) != 0U) {
            lost_at = i;
            break;
        }
        if (i <= 95) {
            CHECK(o.suppressing_lost);          /* masked for the first ~1 s */
        }
    }
    CHECK(lost_at > 90);                                   /* ~1000 ms / 10 ms */
    CHECK(lost_at <= 110);
}

/* A hump report that is no longer fresh (the IMU stopped updating) must not
 * mask anything. */
static void test_stale_hump_status_does_not_mask_line_loss(void)
{
    rig_t r;
    line_nav_out_t o;
    int i;
    bool lost = false;

    rig_init(&r);
    (void)hold(&r, WHITE, BLACK, WHITE, 150);
    line_nav_set_hump(&r.nav, true, r.t);                  /* then never again */
    for (i = 0; i < 20; i++) {
        o = hold(&r, WHITE, WHITE, WHITE, 150);
        lost = lost || ((o.events & NAV_EVENT_LINE_LOST) != 0U);
    }
    CHECK(lost);
}

/* Once the line is genuinely lost, a hump report must not fake a "found":
 * masking only ever hides a LOSS, it never invents a sighting.  Catches: the
 * car resuming normal following with no line under it. */
static void test_hump_context_cannot_fake_a_line_found(void)
{
    rig_t r;
    line_nav_out_t o;
    int i;
    bool found = false;

    rig_init(&r);
    (void)hold(&r, WHITE, BLACK, WHITE, 150);
    for (i = 0; i < 5; i++) {
        o = hold(&r, WHITE, WHITE, WHITE, 150);         /* really lost */
    }
    CHECK(o.line_lost);

    for (i = 0; i < 30; i++) {
        line_nav_set_hump(&r.nav, true, r.t);
        o = hold(&r, WHITE, WHITE, WHITE, 150);
        found = found || ((o.events & NAV_EVENT_LINE_FOUND) != 0U);
    }
    CHECK(!found);
    CHECK(o.line_lost);
}

/* No hump: no masking. */
static void test_no_hump_means_no_masking(void)
{
    rig_t r;
    line_nav_out_t o;
    int i;
    int lost_at = -1;

    rig_init(&r);
    (void)hold(&r, WHITE, BLACK, WHITE, 150);
    line_nav_set_hump(&r.nav, false, r.t);
    for (i = 1; i <= 10; i++) {
        o = hold(&r, WHITE, WHITE, WHITE, 150);
        if (((o.events & NAV_EVENT_LINE_LOST) != 0U) && (lost_at < 0)) {
            lost_at = i;
        }
    }
    CHECK_INT(3, lost_at);
}

/* ---- barcodes --------------------------------------------------------- */

/* A barcode read by the right outer sensor stores a mapped instruction that
 * can be taken once.  Catches: no decode through the real sample path,
 * duplicate execution (Tribunal: each instruction executes once). */
static void test_right_lane_barcode_is_stored_and_taken_once(void)
{
    rig_t r;
    nav_instruction_t ins;
    uint8_t ev;

    rig_init(&r);
    ev = drive_barcode(&r, fixture('A', "large"), LINE_CHANNEL_RIGHT, 20U);
    CHECK(ev & NAV_EVENT_BARCODE);
    CHECK(line_nav_has_instruction(&r.nav, r.t));
    CHECK(line_nav_take_instruction(&r.nav, r.t, &ins));
    CHECK_INT('A', ins.letter);
    CHECK_INT(NAV_CMD_LEFT, ins.command);
    CHECK(!line_nav_take_instruction(&r.nav, r.t, &ins));   /* consumed */
    CHECK(!line_nav_has_instruction(&r.nav, r.t));
}

/* Peeking at the stored instruction does not consume it, so the Vehicle
 * Controller can look, decide at the junction, and only then take it.
 * Catches: a peek that uses the instruction up. */
static void test_peek_shows_the_instruction_without_consuming_it(void)
{
    rig_t r;
    nav_instruction_t a;
    nav_instruction_t b;

    rig_init(&r);
    CHECK(!line_nav_peek_instruction(&r.nav, r.t, &a));
    (void)drive_barcode(&r, fixture('A', "large"), LINE_CHANNEL_RIGHT, 20U);
    CHECK(line_nav_peek_instruction(&r.nav, r.t, &a));
    CHECK(line_nav_peek_instruction(&r.nav, r.t, &b));
    CHECK_INT('A', a.letter);
    CHECK_INT(a.id, b.id);
    CHECK(line_nav_has_instruction(&r.nav, r.t));
    CHECK(!line_nav_peek_instruction(&r.nav, r.t + 20001U, &a));     /* expired */
    CHECK(line_nav_take_instruction(&r.nav, r.t, &a));
    CHECK(!line_nav_peek_instruction(&r.nav, r.t, &a));
}

/* The left outer sensor reads a barcode just as well (which side the strip is
 * on is not yet known). */
static void test_left_lane_barcode_is_decoded(void)
{
    rig_t r;
    nav_instruction_t ins;

    rig_init(&r);
    (void)drive_barcode(&r, fixture('Z', "large"), LINE_CHANNEL_LEFT, 20U);
    CHECK(line_nav_take_instruction(&r.nav, r.t, &ins));
    CHECK_INT('Z', ins.letter);
    CHECK_INT(NAV_CMD_RIGHT, ins.command);
}

/* All three printed scales decode across the speed range (6..30 frames per
 * narrow bar is roughly 0.5x..2x of 3 mm at 150 mm/s, 1 ms frames).  Catches:
 * a decoder or lane that only works at one scale or speed. */
static void test_all_scales_and_speeds_decode(void)
{
    static const char *SCALES[] = { "large", "medium", "small" };
    static const unsigned FPN[] = { 10U, 15U, 20U, 30U };
    size_t si;
    size_t fi;

    for (si = 0U; si < 3U; si++) {
        for (fi = 0U; fi < 4U; fi++) {
            rig_t r;
            nav_instruction_t ins;

            rig_init(&r);
            (void)drive_barcode(&r, fixture('A', SCALES[si]), LINE_CHANNEL_RIGHT, FPN[fi]);
            CHECK(line_nav_take_instruction(&r.nav, r.t, &ins));
            CHECK_INT('A', ins.letter);
        }
    }
}

/* A decoded letter that is not in the command table is rejected and reported;
 * it never becomes an instruction.  Catches: executing an unknown code. */
static void test_unmapped_letter_is_rejected(void)
{
    rig_t r;
    nav_instruction_t ins;
    uint8_t ev;

    rig_init(&r);
    ev = drive_barcode(&r, fixture('F', "large"), LINE_CHANNEL_RIGHT, 20U);
    CHECK(ev & NAV_EVENT_BARCODE_REJECTED);
    CHECK_INT(0, ev & NAV_EVENT_BARCODE);
    CHECK(!line_nav_take_instruction(&r.nav, r.t, &ins));
    CHECK_INT(1, r.nav.rejected_count);
}

/* A stored instruction expires: an old barcode is not executed at some later
 * junction.  Catches: acting on stale barcode data (Tribunal rule). */
static void test_stored_instruction_expires(void)
{
    rig_t r;
    nav_instruction_t ins;

    rig_init(&r);
    (void)drive_barcode(&r, fixture('A', "large"), LINE_CHANNEL_RIGHT, 20U);
    CHECK(!line_nav_take_instruction(&r.nav, r.t + 20001U, &ins));
    CHECK_INT(1, r.nav.expired_count);
}

/* A second, different barcode replaces an unused one (latest wins) and the
 * ids advance. */
static void test_newer_barcode_replaces_the_pending_one(void)
{
    rig_t r;
    nav_instruction_t ins;

    rig_init(&r);
    (void)drive_barcode(&r, fixture('A', "large"), LINE_CHANNEL_RIGHT, 20U);
    r.t += 1500U;                                  /* a clear gap between the two */
    (void)drive_barcode(&r, fixture('Z', "large"), LINE_CHANNEL_RIGHT, 20U);
    CHECK_INT(1, r.nav.overwritten_count);
    CHECK(line_nav_take_instruction(&r.nav, r.t, &ins));
    CHECK_INT('Z', ins.letter);
    CHECK_INT(1, ins.id);
}

/* A strip wide enough to be read by both outer sensors at once is still one
 * barcode, and executes once.  Catches: one physical barcode giving two
 * instructions (Tribunal: each valid instruction executes once). */
static void test_same_barcode_seen_by_both_outer_sensors_is_one_instruction(void)
{
    rig_t r;
    nav_instruction_t ins;

    rig_init(&r);
    (void)drive_barcode_on(&r, fixture('A', "large"),
                           (1U << LINE_CHANNEL_LEFT) | (1U << LINE_CHANNEL_RIGHT), 20U);
    CHECK(line_nav_take_instruction(&r.nav, r.t, &ins));
    CHECK_INT('A', ins.letter);
    CHECK_INT(0, ins.id);
    CHECK(!line_nav_take_instruction(&r.nav, r.t, &ins));
    CHECK_INT(1, r.nav.decoded_count);
}

/* An ADC overflow in the middle voids the attempt: the second half alone is
 * not a barcode.  A clean pass afterwards still decodes.  Catches: decoding
 * across lost samples (Tribunal: overflow invalidates the attempt). */
static void test_overflow_voids_the_barcode_attempt(void)
{
    rig_t r;
    nav_instruction_t ins;
    const barcode_fixture_t *fx = fixture('A', "large");
    unsigned i;
    unsigned fpn = 20U;
    double scale = (double)fpn / (double)fx->runs[0];
    uint16_t pat[LINE_CHANNEL_COUNT];

    rig_init(&r);
    for (i = 0U; i <= (BARCODE_ELEMENT_COUNT + 1U); i++) {
        unsigned n = (i == 0U) ? 40U : ((i == (BARCODE_ELEMENT_COUNT + 1U)) ? 80U :
                     (unsigned)((fx->runs[i - 1U] * scale) + 0.5));
        bool black = (i != 0U) && (i != (BARCODE_ELEMENT_COUNT + 1U)) &&
                     (((i - 1U) % 2U) == 0U);
        unsigned k;

        if (i == 15U) {
            line_nav_note_overflow(&r.nav);
        }
        pat[LINE_CHANNEL_LEFT] = WHITE;
        pat[LINE_CHANNEL_CENTRE] = BLACK;
        pat[LINE_CHANNEL_RIGHT] = black ? BLACK : WHITE;
        for (k = 0U; k < n; k++) {
            line_nav_push_frame(&r.nav, pat, r.t);
            r.t++;
        }
    }
    CHECK(!line_nav_take_instruction(&r.nav, r.t, &ins));
    CHECK_INT(1, r.nav.overflow_count);

    r.t += 1500U;
    (void)drive_barcode(&r, fx, LINE_CHANNEL_RIGHT, 20U);
    CHECK(line_nav_take_instruction(&r.nav, r.t, &ins));
}

/* The overflow is reported as an event for the fault mask. */
static void test_overflow_event_is_reported(void)
{
    rig_t r;
    line_nav_out_t o;

    rig_init(&r);
    line_nav_note_overflow(&r.nav);
    o = hold(&r, WHITE, BLACK, WHITE, 150);
    CHECK(o.events & NAV_EVENT_OVERFLOW);
    o = hold(&r, WHITE, BLACK, WHITE, 150);
    CHECK_INT(0, o.events & NAV_EVENT_OVERFLOW);
}

/* An untrusted (stuck) outer sensor runs no barcode lane. */
static void test_stuck_sensor_does_not_decode(void)
{
    rig_t r;
    line_nav_config_t cfg = make_cfg();
    nav_instruction_t ins;
    const barcode_fixture_t *fx = fixture('A', "large");
    unsigned i;
    double scale = 20.0 / (double)fx->runs[0];

    line_nav_init(&r.nav, &cfg);
    r.t = 0U;
    line_nav_calibrate_begin(&r.nav);
    frames(&r, 20U, 4001U, WHITE, WHITE);
    frames(&r, 20U, 3998U, BLACK, BLACK);
    (void)line_nav_calibrate_end(&r.nav);
    for (i = 0U; i < BARCODE_ELEMENT_COUNT; i++) {
        unsigned n = (unsigned)((fx->runs[i] * scale) + 0.5);

        frames(&r, n, ((i % 2U) == 0U) ? 4100U : 3900U, BLACK, WHITE);
    }
    frames(&r, 80U, 4000U, BLACK, WHITE);
    CHECK(!line_nav_take_instruction(&r.nav, r.t, &ins));
}

/* While a barcode strip passes the outer sensor with the centre on the line,
 * the steering position must not be dragged toward the strip.  Catches: the
 * car swerving into the barcode. */
static void test_barcode_does_not_pull_the_steering(void)
{
    rig_t r;
    const barcode_fixture_t *fx = fixture('A', "large");
    unsigned i;
    unsigned k;
    double scale = 20.0 / (double)fx->runs[0];
    int worst = 0;

    rig_init(&r);
    (void)hold(&r, WHITE, BLACK, WHITE, 150);
    for (i = 0U; i < BARCODE_ELEMENT_COUNT; i++) {
        unsigned n = (unsigned)((fx->runs[i] * scale) + 0.5);
        bool black = ((i % 2U) == 0U);

        for (k = 0U; k < n; k++) {
            uint16_t adc[LINE_CHANNEL_COUNT];

            adc[LINE_CHANNEL_LEFT] = WHITE;
            adc[LINE_CHANNEL_CENTRE] = BLACK;
            adc[LINE_CHANNEL_RIGHT] = black ? BLACK : WHITE;
            line_nav_push_frame(&r.nav, adc, r.t);
            r.t++;
            if ((k % 10U) == 9U) {
                line_nav_out_t o = step(&r, 150);
                int p = (o.position < 0) ? -o.position : o.position;

                if (o.position_valid && (p > worst)) {
                    worst = p;
                }
            }
        }
    }
    CHECK(worst < 100);
}

/* ---- junctions -------------------------------------------------------- */

/*
 * The car's own layout (robot_config.h): CENTRE (-6 mm) and RIGHT (+6 mm) follow
 * the line, the far LEFT sensor (-39 mm) reads the barcode and spots left
 * branches.  A barcode is stored, no junction is invented from its bars, and the
 * command is handed out at the junction that follows.  A barcode met from the
 * other side (read backwards) gives the same letter.
 */
static line_nav_config_t car_layout_cfg(void)
{
    line_nav_config_t c = make_cfg();

    c.channel_pos_permille[LINE_CHANNEL_LEFT] = -6500;
    c.channel_pos_permille[LINE_CHANNEL_CENTRE] = -1000;
    c.channel_pos_permille[LINE_CHANNEL_RIGHT] = 1000;
    c.track_mask = (uint8_t)((1U << LINE_CHANNEL_CENTRE) | (1U << LINE_CHANNEL_RIGHT));
    c.lane_mask = (uint8_t)(1U << LINE_CHANNEL_LEFT);
    c.junction_mask = (uint8_t)(1U << LINE_CHANNEL_LEFT);
    return c;
}

/* A junction needs the far LEFT sensor black together with at least one of the
 * two tracking sensors (centre or right), so two of the three, left being one.
 * The left sensor alone (the car off the line beside a black patch or a barcode
 * bar) is not a junction.  Catches: a junction from the branch sensor alone, one
 * that still needs all three, or one that accepts the pair without the left. */
static void test_junction_needs_left_plus_one_other_sensor_black(void)
{
    static const uint16_t OTHERS[][2] = {
        { BLACK, WHITE },       /* centre only */
        { WHITE, BLACK },       /* right only  */
        { BLACK, BLACK }        /* both        */
    };
    rig_t r;
    line_nav_config_t cfg = car_layout_cfg();
    unsigned i;
    unsigned k;
    uint8_t seen = 0U;

    rig_init_with(&r, &cfg);
    for (i = 0U; i < 20U; i++) {
        seen |= hold(&r, BLACK, WHITE, WHITE, 150).events;
    }
    CHECK((seen & NAV_EVENT_JUNCTION) == 0U);           /* left alone: no */

    rig_init_with(&r, &cfg);
    for (i = 0U; i < 20U; i++) {
        seen |= hold(&r, WHITE, BLACK, BLACK, 150).events;
    }
    CHECK((seen & NAV_EVENT_JUNCTION) == 0U);           /* the pair without the left: no */

    for (k = 0U; k < 3U; k++) {
        rig_init_with(&r, &cfg);
        seen = 0U;
        for (i = 0U; i < 20U; i++) {
            seen |= hold(&r, BLACK, OTHERS[k][0], OTHERS[k][1], 150).events;
        }
        CHECK((seen & NAV_EVENT_JUNCTION) != 0U);       /* left + centre, left + right, all three */
    }
}

/*
 * After a barcode has been read, the next time the left sensor and a tracking
 * sensor are black together is the junction, with no need for the speed (still a placeholder on
 * the car).  Without a barcode waiting, the same black at speed 0 is not a
 * junction (its length is unknown), and with a barcode waiting, one tracking
 * sensor off the line still is not.  Catches: a junction that depends on the
 * wheel-speed estimate, or one that fires on part of the sensor bar.
 */
static void test_junction_after_a_barcode_needs_no_speed(void)
{
    rig_t r;
    line_nav_config_t cfg = car_layout_cfg();
    unsigned i;
    uint8_t seen = 0U;

    rig_init_with(&r, &cfg);
    for (i = 0U; i < 20U; i++) {
        seen |= hold(&r, BLACK, BLACK, BLACK, 0).events;
    }
    CHECK((seen & NAV_EVENT_JUNCTION) == 0U);           /* nothing waiting, no speed */

    rig_init_with(&r, &cfg);
    (void)drive_barcode(&r, fixture('Z', "large"), LINE_CHANNEL_LEFT, 20U);
    CHECK(line_nav_has_instruction(&r.nav, r.t));
    for (i = 0U; i < 20U; i++) {
        seen |= hold(&r, BLACK, WHITE, WHITE, 0).events;
    }
    CHECK((seen & NAV_EVENT_JUNCTION) == 0U);           /* the left alone, no tracking sensor black */

    frames(&r, 500U, WHITE, BLACK, BLACK);
    (void)step(&r, 0);
    for (i = 0U; i < 20U; i++) {
        seen |= hold(&r, BLACK, BLACK, BLACK, 0).events;
    }
    CHECK((seen & NAV_EVENT_JUNCTION) != 0U);           /* all three, with no speed at all */
}

/*
 * The tile's ink may read patchier on the far sensor than the barcode paper's.
 * After a barcode the junction is called from a leaky count, so a left sensor
 * that is black 3 frames in 4 still gives it, while one that is black 1 frame
 * in 4 (a smudge, a stray mark) does not.  Catches: a junction that needs an
 * unbroken run of black, and one that a mostly-white reading can trigger.
 */
static void test_patchy_left_sensor_still_gives_the_junction_after_a_barcode(void)
{
    static const unsigned BLACK_OF_4[] = { 3U, 1U };    /* black frames in every 4 */
    unsigned p;

    for (p = 0U; p < 2U; p++) {
        rig_t r;
        line_nav_config_t cfg = car_layout_cfg();
        uint8_t seen = 0U;
        unsigned i;
        unsigned k;

        rig_init_with(&r, &cfg);
        (void)drive_barcode(&r, fixture('A', "large"), LINE_CHANNEL_LEFT, 20U);
        CHECK(line_nav_has_instruction(&r.nav, r.t));
        frames(&r, 300U, WHITE, BLACK, BLACK);          /* plain line again */
        (void)step(&r, 60);

        for (i = 0U; i < 60U; i++) {                    /* 600 ms on the arm, patchy */
            for (k = 0U; k < 10U; k++) {
                const uint16_t left = ((k % 4U) < BLACK_OF_4[p]) ? BLACK : WHITE;

                frame(&r, left, BLACK, BLACK);
            }
            seen |= step(&r, 60).events;
        }
        if (p == 0U) {
            CHECK((seen & NAV_EVENT_JUNCTION) != 0U);
        } else {
            CHECK((seen & NAV_EVENT_JUNCTION) == 0U);
        }
    }
}

/* Read slowly (60 frames per narrow bar, about 50 mm/s) the barcode's own long
 * bars fill the left sensor's score, and the centre and right are on the line.
 * The score is cleared when the symbol is decoded, so the barcode cannot arm its
 * own junction; the junction is only the next arm.  Catches: a junction fired
 * by the end of the barcode itself. */
static void test_slow_barcode_does_not_arm_its_own_junction(void)
{
    rig_t r;
    line_nav_config_t cfg = car_layout_cfg();
    uint8_t ev;

    rig_init_with(&r, &cfg);
    ev = drive_barcode_at(&r, fixture('A', "large"), 1U << LINE_CHANNEL_LEFT, 60U, 50);
    CHECK((ev & NAV_EVENT_BARCODE) != 0U);
    CHECK((ev & NAV_EVENT_JUNCTION) == 0U);
}

/* pair_on_line is true only while both tracking sensors (CENTRE and RIGHT) see
 * black: that is where a junction turn stops.  The far sensor does not count. */
static void test_pair_on_line_needs_both_tracking_sensors(void)
{
    rig_t r;
    line_nav_config_t cfg = car_layout_cfg();

    rig_init_with(&r, &cfg);
    CHECK(hold(&r, WHITE, BLACK, BLACK, 0).pair_on_line);
    CHECK(!hold(&r, WHITE, BLACK, WHITE, 0).pair_on_line);
    CHECK(!hold(&r, WHITE, WHITE, BLACK, 0).pair_on_line);
    CHECK(!hold(&r, BLACK, WHITE, WHITE, 0).pair_on_line);
}

/* A junction is told from a bar by its length, and the length comes from the
 * measured speed.  With the speed reading double (motion not yet calibrated) a
 * 9 mm wide bar looks 18 mm long, so a sensor in the middle of a barcode is not
 * checked for branches.  Catches: a wide bar turned into a junction. */
static void test_barcode_bar_is_not_a_junction_when_speed_reads_high(void)
{
    rig_t r;
    line_nav_config_t cfg = car_layout_cfg();
    uint8_t ev;

    rig_init_with(&r, &cfg);
    ev = drive_barcode_at(&r, fixture('A', "large"), 1U << LINE_CHANNEL_LEFT, 20U, 300);
    CHECK((ev & NAV_EVENT_BARCODE) != 0U);
    CHECK((ev & NAV_EVENT_JUNCTION) == 0U);
}

/* The same met from the other side.  Read backwards, the start character's
 * first wide bar is already the 3rd element, so the sensor must count as
 * "reading a barcode" after its 2nd run.  Catches: a reversed barcode's first
 * wide bar turned into a junction, which slows the car mid-symbol. */
static void test_reversed_barcode_bar_is_not_a_junction_when_speed_reads_high(void)
{
    static const char LETTERS[] = { 'A', 'Z' };
    unsigned l;

    for (l = 0U; l < 2U; l++) {
        rig_t r;
        line_nav_config_t cfg = car_layout_cfg();
        barcode_fixture_t backwards = *fixture(LETTERS[l], "large");
        uint8_t ev;
        unsigned i;

        for (i = 0U; i < BARCODE_ELEMENT_COUNT; i++) {
            backwards.runs[i] = fixture(LETTERS[l], "large")->runs[BARCODE_ELEMENT_COUNT - 1U - i];
        }
        rig_init_with(&r, &cfg);
        ev = drive_barcode_at(&r, &backwards, 1U << LINE_CHANNEL_LEFT, 20U, 300);
        CHECK((ev & NAV_EVENT_BARCODE) != 0U);
        CHECK((ev & NAV_EVENT_JUNCTION) == 0U);
    }
}

static void test_car_layout_left_sensor_barcode_then_junction(void)
{
    rig_t r;
    line_nav_config_t cfg = car_layout_cfg();
    barcode_fixture_t backwards = *fixture('A', "large");
    nav_instruction_t ins;
    line_nav_out_t o;
    uint8_t ev;
    uint8_t junction = 0U;
    unsigned i;

    rig_init_with(&r, &cfg);
    ev = drive_barcode(&r, fixture('Z', "large"), LINE_CHANNEL_LEFT, 20U);
    CHECK((ev & NAV_EVENT_BARCODE) != 0U);
    CHECK((ev & NAV_EVENT_JUNCTION) == 0U);             /* bars are not a branch */
    CHECK(line_nav_has_instruction(&r.nav, r.t));

    for (i = 0U; (i < 20U) && (junction == 0U); i++) {
        o = hold(&r, BLACK, BLACK, BLACK, 150);         /* a branch to the left */
        if ((o.events & NAV_EVENT_JUNCTION) != 0U) {
            junction = o.junction_mask;
        }
    }
    CHECK_INT(NAV_JUNCTION_LEFT, junction);
    CHECK(line_nav_take_instruction(&r.nav, r.t, &ins));
    CHECK_INT('Z', ins.letter);
    CHECK(!line_nav_take_instruction(&r.nav, r.t, &ins));   /* used once */

    frames(&r, 100U, WHITE, BLACK, BLACK);              /* back to plain line */
    (void)step(&r, 150);
    for (i = 0U; i < BARCODE_ELEMENT_COUNT; i++) {
        backwards.runs[i] = fixture('A', "large")->runs[BARCODE_ELEMENT_COUNT - 1U - i];
    }
    ev = drive_barcode(&r, &backwards, LINE_CHANNEL_LEFT, 20U);
    CHECK((ev & NAV_EVENT_BARCODE) != 0U);
    CHECK(line_nav_take_instruction(&r.nav, r.t, &ins));
    CHECK_INT('A', ins.letter);
}

/* An outer sensor black together with the centre for longer than a barcode
 * element is a junction branch on that side.  At 150 mm/s, 13 mm is reached on
 * the 9th 10 ms step and the event follows the 30 ms merge window (12th step).  Catches: missing the junction, wrong side. */
static void test_long_black_on_an_outer_sensor_is_a_junction(void)
{
    rig_t r;
    line_nav_out_t o;
    int i;
    int at = -1;

    rig_init(&r);
    for (i = 1; i <= 20; i++) {
        o = hold(&r, BLACK, BLACK, WHITE, 150);
        if ((o.events & NAV_EVENT_JUNCTION) != 0U) {
            at = i;
            CHECK_INT(NAV_JUNCTION_LEFT, o.junction_mask);
            break;
        }
    }
    CHECK_INT(12, at);

    rig_init(&r);
    for (i = 1; i <= 20; i++) {
        o = hold(&r, WHITE, BLACK, BLACK, 150);
        if ((o.events & NAV_EVENT_JUNCTION) != 0U) {
            CHECK_INT(NAV_JUNCTION_RIGHT, o.junction_mask);
            break;
        }
    }
}

/* A T-junction or cross shows both sides. */
static void test_both_sides_black_is_a_t_junction(void)
{
    rig_t r;
    line_nav_out_t o;
    int i;
    uint8_t mask = 0U;

    rig_init(&r);
    for (i = 1; i <= 20; i++) {
        o = hold(&r, BLACK, BLACK, BLACK, 150);
        if ((o.events & NAV_EVENT_JUNCTION) != 0U) {
            mask = o.junction_mask;
            break;
        }
    }
    CHECK_INT(NAV_JUNCTION_LEFT | NAV_JUNCTION_RIGHT, mask);
}

/* A T-junction whose two branches reach the outer sensors a step or two
 * apart (the car is slightly skewed) is still ONE junction with both sides.
 * Catches: reporting a T as a left junction followed by a right one, which
 * would execute the stored instruction at the wrong moment. */
static void test_sides_seen_slightly_apart_are_one_t_junction(void)
{
    rig_t r;
    line_nav_out_t o;
    int i;
    int events = 0;
    uint8_t mask = 0U;

    rig_init(&r);
    for (i = 1; i <= 30; i++) {
        uint16_t right = (i >= 3) ? BLACK : WHITE;     /* right arrives 2 steps late */

        o = hold(&r, BLACK, BLACK, right, 150);
        if ((o.events & NAV_EVENT_JUNCTION) != 0U) {
            events++;
            mask |= o.junction_mask;
        }
    }
    CHECK_INT(1, events);
    CHECK_INT(NAV_JUNCTION_LEFT | NAV_JUNCTION_RIGHT, mask);
}

/* The widest barcode bar (9 mm) is shorter than the junction length, so a
 * single wide bar on an outer sensor is NOT a junction at any course speed.
 * Catches: a barcode triggering a turn (Tribunal: do not treat a black pattern
 * as a junction automatically). */
static void test_a_wide_barcode_bar_is_not_a_junction(void)
{
    static const int16_t SPEEDS[] = { 100, 150, 200 };
    size_t s;

    for (s = 0U; s < 3U; s++) {
        rig_t r;
        line_nav_out_t o;
        unsigned steps = (unsigned)(9000 / SPEEDS[s] / 10);     /* 9 mm of travel */
        unsigned i;
        bool junction = false;

        rig_init(&r);
        for (i = 0U; i < steps; i++) {
            o = hold(&r, WHITE, BLACK, BLACK, SPEEDS[s]);
            junction = junction || ((o.events & NAV_EVENT_JUNCTION) != 0U);
        }
        o = hold(&r, WHITE, BLACK, WHITE, SPEEDS[s]);
        junction = junction || ((o.events & NAV_EVENT_JUNCTION) != 0U);
        CHECK(!junction);
    }
}

/* One crossing gives one event; after the sensors clear, a new crossing gives
 * another.  Catches: a junction reported on every step while crossing it. */
static void test_a_junction_is_reported_once_per_crossing(void)
{
    rig_t r;
    line_nav_out_t o;
    int i;
    int events = 0;

    rig_init(&r);
    for (i = 0; i < 30; i++) {
        o = hold(&r, BLACK, BLACK, WHITE, 150);
        if ((o.events & NAV_EVENT_JUNCTION) != 0U) { events++; }
    }
    CHECK_INT(1, events);
    for (i = 0; i < 50; i++) {
        o = hold(&r, WHITE, BLACK, WHITE, 150);         /* clear: 500 ms, 75 mm */
    }
    for (i = 0; i < 30; i++) {
        o = hold(&r, BLACK, BLACK, WHITE, 150);
        if ((o.events & NAV_EVENT_JUNCTION) != 0U) { events++; }
    }
    CHECK_INT(2, events);
}

/* Length needs speed: a stationary car cannot tell a junction from a bar. */
static void test_no_speed_means_no_junction(void)
{
    rig_t r;
    line_nav_out_t o;
    int i;
    bool junction = false;

    rig_init(&r);
    for (i = 0; i < 50; i++) {
        o = hold(&r, BLACK, BLACK, WHITE, 0);
        junction = junction || ((o.events & NAV_EVENT_JUNCTION) != 0U);
    }
    CHECK(!junction);
}

/* ---- layout C: close tracking sensors plus a far barcode sensor ----------- */

/* Only the tracking sensors steer: the far sensor over a black strip must never
 * be mistaken for the line.  Catches: a barcode strip dragging the car, or the
 * far sensor holding the line "visible" when it is really lost. */
static void test_layout_c_tracks_with_the_close_pair_only(void)
{
    rig_t r;
    line_nav_config_t cfg = layout_c_cfg();
    line_nav_out_t o;

    rig_init_with(&r, &cfg);
    o = hold(&r, BLACK, WHITE, WHITE, 150);
    CHECK(o.position_valid);
    CHECK_INT(-1000, o.position);
    o = hold(&r, BLACK, BLACK, WHITE, 150);
    CHECK_INT(0, o.position);                       /* the line straddles the pair */
    o = hold(&r, WHITE, BLACK, WHITE, 150);
    CHECK_INT(1000, o.position);
    o = hold(&r, WHITE, WHITE, BLACK, 150);         /* only the far sensor is black */
    CHECK(!o.position_valid);
}

/* The far sensor reads the barcode while the pair keeps tracking, and a
 * barcode strip causes neither a junction nor a loss of the line. */
static void test_layout_c_reads_a_barcode_without_disturbing_tracking(void)
{
    rig_t r;
    line_nav_config_t cfg = layout_c_cfg();
    nav_instruction_t ins;
    const barcode_fixture_t *fx = fixture('A', "large");
    unsigned i;
    unsigned k;
    double scale = 20.0 / (double)fx->runs[0];
    bool always_valid = true;
    bool junction = false;

    rig_init_with(&r, &cfg);
    (void)hold(&r, WHITE, BLACK, WHITE, 150);
    for (i = 0U; i < BARCODE_ELEMENT_COUNT; i++) {
        unsigned n = (unsigned)((fx->runs[i] * scale) + 0.5);
        bool black = ((i % 2U) == 0U);

        for (k = 0U; k < n; k++) {
            uint16_t adc[LINE_CHANNEL_COUNT];

            adc[LINE_CHANNEL_LEFT] = WHITE;
            adc[LINE_CHANNEL_CENTRE] = BLACK;
            adc[LINE_CHANNEL_RIGHT] = black ? BLACK : WHITE;
            line_nav_push_frame(&r.nav, adc, r.t);
            r.t++;
            if ((k % 10U) == 9U) {
                line_nav_out_t o = step(&r, 150);

                always_valid = always_valid && o.position_valid;
                junction = junction || ((o.events & NAV_EVENT_JUNCTION) != 0U);
            }
        }
    }
    frames(&r, 80U, WHITE, BLACK, WHITE);
    CHECK(always_valid);
    CHECK(!junction);
    CHECK(line_nav_take_instruction(&r.nav, r.t, &ins));
    CHECK_INT('A', ins.letter);
}

/* A sensor outside the lane mask runs no barcode lane. */
static void test_layout_c_does_not_read_barcodes_on_a_tracking_sensor(void)
{
    rig_t r;
    line_nav_config_t cfg = layout_c_cfg();
    nav_instruction_t ins;

    rig_init_with(&r, &cfg);
    (void)drive_barcode(&r, fixture('A', "large"), LINE_CHANNEL_LEFT, 20U);
    CHECK(!line_nav_take_instruction(&r.nav, r.t, &ins));
}

/* Junction branches are seen by the configured sensor only: a long black run
 * on a tracking sensor (the line itself) is not a junction, and the far
 * sensor going long-black together with the line is. */
static void test_layout_c_junction_uses_only_the_branch_sensor(void)
{
    rig_t r;
    line_nav_config_t cfg = layout_c_cfg();
    line_nav_out_t o;
    int i;
    bool junction = false;
    uint8_t mask = 0U;

    rig_init_with(&r, &cfg);
    for (i = 0; i < 30; i++) {
        o = hold(&r, BLACK, BLACK, WHITE, 150);         /* the line itself, long and black */
        junction = junction || ((o.events & NAV_EVENT_JUNCTION) != 0U);
    }
    CHECK(!junction);

    rig_init_with(&r, &cfg);
    for (i = 0; i < 30; i++) {
        o = hold(&r, BLACK, BLACK, BLACK, 150);         /* a crossing line */
        if ((o.events & NAV_EVENT_JUNCTION) != 0U) {
            junction = true;
            mask = o.junction_mask;
        }
    }
    CHECK(junction);
    CHECK_INT(NAV_JUNCTION_RIGHT, mask);
}

/* Calibration needs a usable TRACKING sensor; a good far sensor alone is not
 * enough to follow a line. */
static void test_calibration_needs_a_tracking_sensor(void)
{
    rig_t r;
    line_nav_config_t cfg = layout_c_cfg();

    line_nav_init(&r.nav, &cfg);
    r.t = 0U;
    line_nav_calibrate_begin(&r.nav);
    frames(&r, 20U, 4000U, 4001U, WHITE);               /* tracking pair stuck high */
    frames(&r, 20U, 4002U, 3998U, BLACK);               /* far sensor swings fully */
    CHECK(!line_nav_calibrate_end(&r.nav));
}

/* ---- freshness -------------------------------------------------------- */

/* With no ADC frames for longer than the stale limit, the output says so, so
 * the Vehicle Controller can stop rather than steer on old data. */
static void test_missing_frames_are_reported_stale(void)
{
    rig_t r;
    line_nav_out_t o;

    rig_init(&r);
    frames(&r, 10U, WHITE, BLACK, WHITE);
    o = step(&r, 150);
    CHECK(!o.stale);
    r.t += 60U;                                         /* 60 ms with no frames */
    o = step(&r, 150);
    CHECK(o.stale);
}

/* Nothing stored: nothing to take. */
static void test_no_instruction_when_nothing_decoded(void)
{
    rig_t r;
    nav_instruction_t ins;

    rig_init(&r);
    CHECK(!line_nav_has_instruction(&r.nav, r.t));
    CHECK(!line_nav_take_instruction(&r.nav, r.t, &ins));
}


/*
 * A car that does NOT move at a constant speed.  The barcode is laid out in
 * distance (a narrow element is 20 encoder counts); the car's speed is
 * v0 + accel * t counts per ms, and it stands still from stop_from_ms to
 * stop_to_ms.  With 'use_travel' the encoder distance is reported to the
 * navigator each 10 ms step, as the firmware does.
 */
static void drive_barcode_moving(rig_t *r, const barcode_fixture_t *fx, double v0, double accel,
                                 unsigned stop_from_ms, unsigned stop_to_ms, bool use_travel)
{
    const double lead = 60.0;
    const double trail = 100.0;
    double width[BARCODE_ELEMENT_COUNT];
    double total = 0.0;
    double pos = 0.0;
    double travel = 0.0;
    unsigned elapsed = 0U;
    unsigned i;

    for (i = 0U; i < BARCODE_ELEMENT_COUNT; i++) {
        width[i] = (double)fx->runs[i] * (20.0 / (double)fx->runs[0]);
        total += width[i];
    }
    if (use_travel) {
        line_nav_set_travel(&r->nav, 0, r->t);
    }
    while ((pos < (lead + total + trail)) && (elapsed < 8000U)) {
        uint16_t pat[10][LINE_CHANNEL_COUNT];
        unsigned k;

        for (k = 0U; k < 10U; k++) {
            const bool still = (elapsed >= stop_from_ms) && (elapsed < stop_to_ms);
            const double v = still ? 0.0 : (v0 + (accel * (double)elapsed));
            bool black = false;

            pos += v;
            travel += v;
            if ((pos >= lead) && (pos < (lead + total))) {
                double edge = lead;

                for (i = 0U; i < BARCODE_ELEMENT_COUNT; i++) {
                    edge += width[i];
                    if (pos < edge) {
                        black = ((i % 2U) == 0U);
                        break;
                    }
                }
            }
            pat[k][LINE_CHANNEL_LEFT] = black ? BLACK : WHITE;
            pat[k][LINE_CHANNEL_CENTRE] = BLACK;
            pat[k][LINE_CHANNEL_RIGHT] = BLACK;
            elapsed++;
        }
        if (use_travel) {
            line_nav_set_travel(&r->nav, (int32_t)travel, r->t + 10U);
        }
        for (k = 0U; k < 10U; k++) {
            line_nav_push_frame(&r->nav, pat[k], r->t);
            r->t++;
        }
        (void)step(r, 150);
    }
}

/* Speeding up 2.7x across the symbol: bar widths measured in time are badly
 * distorted, widths measured in distance are not.  Catches: widths measured in
 * time, or the travel never reaching the decoder. */
static void test_barcode_reads_while_accelerating_when_distance_is_known(void)
{
    rig_t r;
    nav_instruction_t ins;

    rig_init(&r);
    drive_barcode_moving(&r, fixture('Z', "large"), 0.4, 0.002, 100000U, 100000U, false);
    CHECK(!line_nav_take_instruction(&r.nav, r.t, &ins));   /* the scenario is a hard one */

    rig_init(&r);
    drive_barcode_moving(&r, fixture('Z', "large"), 0.4, 0.002, 100000U, 100000U, true);
    CHECK(line_nav_take_instruction(&r.nav, r.t, &ins));
    CHECK_INT('Z', ins.letter);
}

/* The car freezes for 350 ms with the sensor in the middle of the symbol: the
 * stopped time adds no distance, so the bars keep their proportions.  Catches:
 * a stopped car still adding width to the bar it stopped on. */
static void test_barcode_reads_through_a_freeze_when_distance_is_known(void)
{
    rig_t r;
    nav_instruction_t ins;

    rig_init(&r);
    drive_barcode_moving(&r, fixture('Z', "large"), 1.5, 0.0, 350U, 700U, true);
    CHECK(line_nav_take_instruction(&r.nav, r.t, &ins));
    CHECK_INT('Z', ins.letter);
}

/* The distance counter running backwards (reversing) adds nothing. */
static void test_travel_going_backwards_adds_no_width(void)
{
    rig_t r;

    rig_init(&r);
    line_nav_set_travel(&r.nav, 1000, r.t);
    line_nav_set_travel(&r.nav, 900, r.t + 10U);
    CHECK_INT(0, (int)r.nav.travel_win_q);
}


/* n equal bars (20 frames bar, 20 frames gap) on the left sensor with the line under the
 * other two, then white: it can never decode as a symbol, only be counted. */
static void drive_equal_bars(rig_t *r, unsigned n)
{
    unsigned i;

    frames(r, 40U, WHITE, BLACK, BLACK);
    for (i = 0U; i < n; i++) {
        unsigned k;

        for (k = 0U; k < 2U; k++) {
            unsigned f;

            for (f = 0U; f < 20U; f++) {
                frame(r, (k == 0U) ? BLACK : WHITE, BLACK, BLACK);
                if ((f % 10U) == 9U) {
                    (void)step(r, 150);
                }
            }
        }
    }
    frames(r, 80U, WHITE, BLACK, BLACK);
    (void)step(r, 150);
}

/*
 * After the left sensor has counted 15 bars the barcode is over, whether or not
 * its letter decoded, and the next time that sensor sees black with the line pair
 * is the junction.  14 bars is not a barcode.  Catches: a junction that waits for
 * a decoded letter, and a count that arms on fewer bars.
 */
static void test_fifteen_bars_arm_the_junction_without_a_decode(void)
{
    rig_t r;
    line_nav_config_t cfg = car_layout_cfg();
    unsigned i;
    uint8_t seen = 0U;

    cfg.barcode_bars = 15U;
    cfg.barcode_min_bar = 0U;

    rig_init_with(&r, &cfg);
    drive_equal_bars(&r, 14U);
    CHECK(!r.nav.bars_done);
    for (i = 0U; i < 20U; i++) {
        seen |= hold(&r, BLACK, BLACK, BLACK, 0).events;
    }
    CHECK((seen & NAV_EVENT_JUNCTION) == 0U);           /* 14 bars: nothing is armed */

    rig_init_with(&r, &cfg);
    drive_equal_bars(&r, 15U);
    CHECK(r.nav.bars_done);
    CHECK(!line_nav_has_instruction(&r.nav, r.t));      /* no letter was decoded */
    CHECK_INT(1, r.nav.attempt_count);                  /* ...and the failed attempt is logged */
    CHECK(r.nav.last_attempt_runs >= 15U);
    seen = 0U;
    for (i = 0U; i < 20U; i++) {
        seen |= hold(&r, BLACK, BLACK, BLACK, 0).events;
    }
    CHECK((seen & NAV_EVENT_JUNCTION) != 0U);           /* the next black is the junction */
    CHECK(!r.nav.bars_done);                            /* used up: it arms only once */
}

/* A bar shorter than barcode_min_bar is a glitch and is not counted. */
static void test_glitches_are_not_counted_as_bars(void)
{
    rig_t r;
    line_nav_config_t cfg = car_layout_cfg();

    cfg.barcode_bars = 15U;
    cfg.barcode_min_bar = 5U;
    rig_init_with(&r, &cfg);
    drive_equal_bars(&r, 15U);                          /* 20-frame bars are real */
    CHECK(r.nav.bars_done);

    cfg.barcode_min_bar = 30U;                          /* now 20 frames is a glitch */
    rig_init_with(&r, &cfg);
    drive_equal_bars(&r, 15U);
    CHECK(!r.nav.bars_done);
}


/*
 * The letter -> command mapping is set at run time (it arrives over MQTT) and the
 * car starts with none: every decoded letter is rejected until one is set.  A new
 * mapping applies to the next barcode, an empty one rejects again, and an
 * instruction already stored keeps the command it was given.  Catches: a built-in
 * mapping that still works with none sent, a mapping that is not used once set, and
 * one that rewrites an instruction already waiting.
 */
static void test_runtime_mapping_is_needed_and_applies_to_the_next_barcode(void)
{
    rig_t r;
    line_nav_config_t cfg = car_layout_cfg();
    nav_instruction_t ins;
    static const nav_map_entry_t Z_UTURN[] = { { 'Z', NAV_CMD_UTURN } };
    static const nav_map_entry_t Z_LEFT[] = { { 'Z', NAV_CMD_LEFT } };
    uint8_t seen;

    cfg.map = NULL;
    cfg.map_len = 0U;
    rig_init_with(&r, &cfg);
    seen = drive_barcode(&r, fixture('Z', "large"), LINE_CHANNEL_LEFT, 20U);
    CHECK((seen & NAV_EVENT_BARCODE_REJECTED) != 0U);               /* no mapping: rejected */
    CHECK(!line_nav_has_instruction(&r.nav, r.t));

    line_nav_set_map(&r.nav, Z_UTURN, 1U);
    seen = drive_barcode(&r, fixture('Z', "large"), LINE_CHANNEL_LEFT, 20U);
    CHECK((seen & NAV_EVENT_BARCODE) != 0U);
    CHECK(line_nav_peek_instruction(&r.nav, r.t, &ins));
    CHECK_INT(NAV_CMD_UTURN, ins.command);

    line_nav_set_map(&r.nav, Z_LEFT, 1U);                           /* remapped after it was stored */
    CHECK(line_nav_peek_instruction(&r.nav, r.t, &ins));
    CHECK_INT(NAV_CMD_UTURN, ins.command);                          /* the stored one is unchanged */
    (void)line_nav_take_instruction(&r.nav, r.t, &ins);
    seen = drive_barcode(&r, fixture('Z', "large"), LINE_CHANNEL_LEFT, 20U);
    CHECK(line_nav_peek_instruction(&r.nav, r.t, &ins));
    CHECK_INT(NAV_CMD_LEFT, ins.command);                           /* the new mapping applies */
    (void)line_nav_take_instruction(&r.nav, r.t, &ins);

    line_nav_set_map(&r.nav, Z_LEFT, 0U);                           /* an empty mapping */
    seen = drive_barcode(&r, fixture('Z', "large"), LINE_CHANNEL_LEFT, 20U);
    CHECK((seen & NAV_EVENT_BARCODE_REJECTED) != 0U);
    CHECK(!line_nav_has_instruction(&r.nav, r.t));
}

/* More than 26 entries are cut to 26, never written past the table. */
static void test_mapping_longer_than_the_alphabet_is_clamped(void)
{
    rig_t r;
    nav_map_entry_t many[40];
    unsigned i;

    rig_init(&r);
    for (i = 0U; i < 40U; i++) {
        many[i].letter = (char)('A' + (i % 26U));
        many[i].command = NAV_CMD_LEFT;
    }
    line_nav_set_map(&r.nav, many, 40U);
    CHECK_INT((int)NAV_MAP_MAX, (int)r.nav.cfg.map_len);
}

int main(void)
{
    RUN(test_barcode_reads_while_accelerating_when_distance_is_known);
    RUN(test_barcode_reads_through_a_freeze_when_distance_is_known);
    RUN(test_travel_going_backwards_adds_no_width);
    RUN(test_fifteen_bars_arm_the_junction_without_a_decode);
    RUN(test_glitches_are_not_counted_as_bars);
    RUN(test_runtime_mapping_is_needed_and_applies_to_the_next_barcode);
    RUN(test_mapping_longer_than_the_alphabet_is_clamped);
    RUN(test_calibration_excludes_a_stuck_sensor);
    RUN(test_calibration_fails_without_centre_contrast);
    RUN(test_fixed_lane_sensor_is_trusted_without_seeing_black);
    RUN(test_fixed_lane_sensor_does_not_hide_a_dead_tracking_pair);
    RUN(test_fixed_readings_on_every_channel_need_no_sweep);
    RUN(test_smoothing_turns_flicker_into_a_small_position);
    RUN(test_steering_direction);
    RUN(test_steering_is_limited);
    RUN(test_derivative_damps_steering);
    RUN(test_line_lost_and_found_events_are_debounced_and_single);
    RUN(test_hump_masks_line_loss_for_a_bounded_time);
    RUN(test_stale_hump_status_does_not_mask_line_loss);
    RUN(test_hump_context_cannot_fake_a_line_found);
    RUN(test_no_hump_means_no_masking);
    RUN(test_right_lane_barcode_is_stored_and_taken_once);
    RUN(test_peek_shows_the_instruction_without_consuming_it);
    RUN(test_left_lane_barcode_is_decoded);
    RUN(test_all_scales_and_speeds_decode);
    RUN(test_unmapped_letter_is_rejected);
    RUN(test_stored_instruction_expires);
    RUN(test_newer_barcode_replaces_the_pending_one);
    RUN(test_same_barcode_seen_by_both_outer_sensors_is_one_instruction);
    RUN(test_overflow_voids_the_barcode_attempt);
    RUN(test_overflow_event_is_reported);
    RUN(test_stuck_sensor_does_not_decode);
    RUN(test_barcode_does_not_pull_the_steering);
    RUN(test_car_layout_left_sensor_barcode_then_junction);
    RUN(test_barcode_bar_is_not_a_junction_when_speed_reads_high);
    RUN(test_pair_on_line_needs_both_tracking_sensors);
    RUN(test_junction_needs_left_plus_one_other_sensor_black);
    RUN(test_junction_after_a_barcode_needs_no_speed);
    RUN(test_patchy_left_sensor_still_gives_the_junction_after_a_barcode);
    RUN(test_slow_barcode_does_not_arm_its_own_junction);
    RUN(test_reversed_barcode_bar_is_not_a_junction_when_speed_reads_high);
    RUN(test_long_black_on_an_outer_sensor_is_a_junction);
    RUN(test_both_sides_black_is_a_t_junction);
    RUN(test_sides_seen_slightly_apart_are_one_t_junction);
    RUN(test_a_wide_barcode_bar_is_not_a_junction);
    RUN(test_a_junction_is_reported_once_per_crossing);
    RUN(test_no_speed_means_no_junction);
    RUN(test_layout_c_tracks_with_the_close_pair_only);
    RUN(test_layout_c_reads_a_barcode_without_disturbing_tracking);
    RUN(test_layout_c_does_not_read_barcodes_on_a_tracking_sensor);
    RUN(test_layout_c_junction_uses_only_the_branch_sensor);
    RUN(test_calibration_needs_a_tracking_sensor);
    RUN(test_missing_frames_are_reported_stale);
    RUN(test_no_instruction_when_nothing_decoded);
    return TEST_REPORT();
}
