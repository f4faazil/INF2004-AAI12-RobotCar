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
static uint8_t drive_barcode_on(rig_t *r, const barcode_fixture_t *fx, unsigned channel_mask,
                                unsigned fpn)
{
    unsigned i;
    uint8_t events = 0U;
    uint16_t pat[LINE_CHANNEL_COUNT];
    double scale = (double)fpn / (double)fx->runs[0];

    for (i = 0U; i <= (BARCODE_ELEMENT_COUNT + 1U); i++) {
        unsigned n = (i == 0U) ? 40U : ((i == (BARCODE_ELEMENT_COUNT + 1U)) ? 80U :
                     (unsigned)((fx->runs[i - 1U] * scale) + 0.5));
        bool black = (i != 0U) && (i != (BARCODE_ELEMENT_COUNT + 1U)) &&
                     (((i - 1U) % 2U) == 0U);
        unsigned k;

        pat[LINE_CHANNEL_LEFT] = WHITE;
        pat[LINE_CHANNEL_RIGHT] = WHITE;
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
                events |= step(r, 150).events;
            }
        }
    }
    return events;
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

int main(void)
{
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
