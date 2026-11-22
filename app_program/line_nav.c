/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * line_nav.c
 * Line-following and barcode navigation.  See line_nav.h.
 */
#include <stddef.h>

#include "line_nav.h"

#define NAV_BLACK_ON            (600U)      /* per-mille: counts as black */
#define NAV_BLACK_OFF           (400U)      /* ...and as white again below this */
/*
 * Runs that make a lane "barcode-active" (no junction checks, speed held).  2, not
 * more: read backwards, the start character's first wide bar is the 3rd element,
 * and every Code 39 symbol starts with a narrow bar from either end.
 */
#define NAV_ACTIVE_RUNS         (2U)
#define UM_PER_MM               (1000U)
#define PERMILLE                (1000)
#define CLEAR_STEPS_TO_RELEASE  (3U)
/*
 * Junction after a barcode: the branch sensor's leaky score (+2 per black 1 kHz
 * frame, -1 per white one, capped) must reach ARMED_JUNCTION_SCORE.  Solid black
 * gets there in 30 frames, enough to ignore a glitch and short enough for the
 * arm of a crossroads at the top course speed (about 18 mm at 300 mm/s); a
 * patchy reading that is black at least about 40% of the time still gets there,
 * and one that is mostly white never does.
 */
#define ARMED_SCORE_UP          (2U)
#define ARMED_SCORE_MAX         (120U)
#define ARMED_JUNCTION_SCORE    (60U)
#define DEFAULT_STEP_MS         (10U)

/* Lane index <-> sensor channel. */
static const uint8_t LANE_CHANNEL[NAV_LANE_COUNT] = { LINE_CHANNEL_LEFT, LINE_CHANNEL_RIGHT };
static const uint8_t LANE_SIDE[NAV_LANE_COUNT] = { NAV_JUNCTION_LEFT, NAV_JUNCTION_RIGHT };

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static void lane_reset(nav_lane_t *lane)
{
    run_extractor_init(&lane->extractor);
    barcode_init(&lane->decoder);
    lane->had_edge = false;
    lane->run_count = 0U;
    lane->bar_count = 0U;
}

static void lanes_reset(line_nav_t *nav)
{
    uint8_t i;

    for (i = 0U; i < NAV_LANE_COUNT; i++) {
        lane_reset(&nav->lane[i]);
        nav->ch_score[LANE_CHANNEL[i]] = 0U;    /* the barcode's bars must not arm its junction */
    }
}

/* Channels with fixed readings are trusted without a sweep: returns 'mask' with their bits set. */
static uint8_t seed_fixed_channels(line_nav_t *nav, uint8_t mask)
{
    uint8_t ch;

    if (nav->cfg.fixed_black <= nav->cfg.fixed_white) {
        return mask;
    }
    for (ch = 0U; ch < LINE_CHANNEL_COUNT; ch++) {
        const uint8_t bit = (uint8_t)(1U << ch);

        if ((nav->cfg.fixed_mask & bit) != 0U) {
            nav->cal.min[ch] = nav->cfg.fixed_white;
            nav->cal.max[ch] = nav->cfg.fixed_black;
            mask = (uint8_t)(mask | bit);
        }
    }
    return mask;
}

void line_nav_init(line_nav_t *nav, const line_nav_config_t *cfg)
{
    uint8_t i;

    nav->cfg = *cfg;
    nav->cfg.map_len = 0U;
    if (cfg->map != NULL) {
        line_nav_set_map(nav, cfg->map, cfg->map_len);     /* packs it: the table the caller passed is not kept */
    }
    nav->cfg.map = NULL;
    line_cal_init(&nav->cal);
    nav->calibrating = false;
    nav->valid_mask = 0U;
    for (i = 0U; i < LINE_CHANNEL_COUNT; i++) {
        nav->blackness[i] = 0U;
        nav->smooth[i] = 0U;
        nav->sum[i] = 0U;
    }
    nav->sum_n = 0U;
    nav->last_frame_ms = 0U;
    nav->have_frame = false;
    nav->travel_have = false;
    nav->travel_on = false;
    nav->travel_win_q = 0U;
    nav->travel_win_ms = 0U;
    nav->travel_carry = 0U;
    for (i = 0U; i < NAV_LANE_COUNT; i++) {
        lane_reset(&nav->lane[i]);
        nav->lane[i].last_edge_ms = 0U;
    }
    for (i = 0U; i < LINE_CHANNEL_COUNT; i++) {
        nav->ch_black[i] = false;
        nav->ch_black_frames[i] = 0U;
        nav->ch_score[i] = 0U;
    }
    nav->pending_valid = false;
    nav->bars_done = false;
    nav->bars_done_ms = 0U;
    nav->bars_done_count = 0U;
    nav->next_id = 0U;
    nav->events = 0U;
    line_tracker_init(&nav->tracker, cfg->lost_frames, cfg->found_frames);
    nav->hump_active = false;
    nav->hump_status_ms = 0U;
    nav->suppressing = false;
    nav->suppress_since_ms = 0U;
    nav->position = 0;
    nav->prev_position = 0;
    nav->have_prev_position = false;
    nav->last_step_ms = 0U;
    nav->have_step = false;
    nav->junction_latched = 0U;
    nav->junction_pending = 0U;
    nav->junction_since_ms = 0U;
    nav->clear_steps = 0U;
    nav->decoded_count = 0U;
    nav->rejected_count = 0U;
    nav->overwritten_count = 0U;
    nav->expired_count = 0U;
    nav->overflow_count = 0U;
    nav->valid_mask = seed_fixed_channels(nav, nav->valid_mask);
}

/* ---- calibration ---------------------------------------------------- */

void line_nav_calibrate_begin(line_nav_t *nav)
{
    line_cal_init(&nav->cal);
    nav->calibrating = true;
    nav->valid_mask = 0U;
}

bool line_nav_calibrate_end(line_nav_t *nav)
{
    uint8_t mask = line_cal_valid_mask(&nav->cal);

    nav->calibrating = false;
    mask = seed_fixed_channels(nav, mask);
    if ((mask & nav->cfg.track_mask) == 0U) {
        nav->valid_mask = 0U;               /* nothing usable to follow a line with */
        return false;
    }
    nav->valid_mask = mask;
    lanes_reset(nav);
    return true;
}

/* ---- frames ----------------------------------------------------------- */

static void handle_decode(line_nav_t *nav, char letter, uint32_t now_ms)
{
    uint8_t i;
    bool found = false;
    nav_command_t command = NAV_CMD_NONE;

    /* a whole barcode has gone by, whatever its letter: the next thing is its junction */
    nav->bars_done = true;
    nav->bars_done_ms = now_ms;
    nav->bars_done_count++;

    /* The symbol is used up.  Resetting both lanes means one physical barcode
     * can never be decoded twice, even if both outer sensors can see it. */
    lanes_reset(nav);

    for (i = 0U; i < nav->cfg.map_len; i++) {
        if (nav_map_letter(nav->map_packed[i]) == letter) {
            command = nav_map_command(nav->map_packed[i]);      /* the last entry for a letter wins */
            found = true;
        }
    }
    if (!found) {
        nav->rejected_count++;
        nav->events |= NAV_EVENT_BARCODE_REJECTED;
        return;
    }
    if (nav->pending_valid) {
        nav->overwritten_count++;
    }
    nav->pending.letter = letter;
    nav->pending.command = command;
    nav->pending.id = nav->next_id;
    nav->pending.timestamp_ms = now_ms;
    nav->next_id++;
    nav->pending_valid = true;
    nav->decoded_count++;
    nav->events |= NAV_EVENT_BARCODE;
}

bool nav_map_pack(char letter, nav_command_t command, uint8_t *packed)
{
    if ((letter < 'A') || (letter > 'Z') || (command < NAV_CMD_LEFT) || (command > NAV_CMD_UTURN)) {
        return false;
    }
    *packed = (uint8_t)((((uint8_t)(letter - 'A')) << NAV_PACKED_LETTER_SHIFT) | (uint8_t)command);
    return true;
}

char nav_map_letter(uint8_t packed)
{
    return (char)('A' + (packed >> NAV_PACKED_LETTER_SHIFT));
}

nav_command_t nav_map_command(uint8_t packed)
{
    return (nav_command_t)(packed & 7U);
}

void line_nav_set_map(line_nav_t *nav, const nav_map_entry_t *entries, uint8_t len)
{
    uint8_t i;
    uint8_t count = 0U;
    const uint8_t n = (len > NAV_MAP_MAX) ? (uint8_t)NAV_MAP_MAX : len;

    for (i = 0U; i < n; i++) {
        uint8_t packed;

        if (nav_map_pack(entries[i].letter, entries[i].command, &packed)) {
            nav->map_packed[count] = packed;
            count++;
        }
    }
    nav->cfg.map = NULL;
    nav->cfg.map_len = count;
}

void line_nav_set_map_packed(line_nav_t *nav, const uint8_t *packed, uint8_t len)
{
    uint8_t i;
    uint8_t count = 0U;
    const uint8_t n = (len > NAV_MAP_MAX) ? (uint8_t)NAV_MAP_MAX : len;

    for (i = 0U; i < n; i++) {
        const uint8_t letter_index = (uint8_t)(packed[i] >> NAV_PACKED_LETTER_SHIFT);
        const uint8_t command = (uint8_t)(packed[i] & 7U);

        if ((letter_index < NAV_MAP_MAX) && (command >= (uint8_t)NAV_CMD_LEFT) && (command <= (uint8_t)NAV_CMD_UTURN)) {
            nav->map_packed[count] = packed[i];
            count++;
        }
    }
    nav->cfg.map = NULL;
    nav->cfg.map_len = count;
}

void line_nav_set_travel(line_nav_t *nav, int32_t travel, uint32_t now_ms)
{
    if (nav->travel_have && (now_ms > nav->travel_last_ms)) {
        const int32_t delta = travel - nav->travel_last;

        nav->travel_win_q = (delta > 0) ? ((uint32_t)delta * 4U) : 0U;
        nav->travel_win_ms = now_ms - nav->travel_last_ms;
        nav->travel_carry = 0U;
        nav->travel_on = true;
    }
    nav->travel_last = travel;
    nav->travel_last_ms = now_ms;
    nav->travel_have = true;
}

/* Weight of one frame: the distance moved during it (quarter counts), or 1 with no travel data. */
static uint16_t frame_weight(line_nav_t *nav, uint32_t now_ms)
{
    uint32_t dt = now_ms - nav->travel_prev_frame_ms;
    uint32_t w;

    nav->travel_prev_frame_ms = now_ms;
    if (!nav->travel_on) {
        return 1U;
    }
    if ((dt == 0U) || (dt > nav->travel_win_ms)) {
        dt = 1U;
    }
    nav->travel_carry += nav->travel_win_q * dt;
    w = nav->travel_carry / nav->travel_win_ms;
    nav->travel_carry -= w * nav->travel_win_ms;
    return (w > UINT16_MAX) ? UINT16_MAX : (uint16_t)w;
}

void line_nav_push_frame(line_nav_t *nav, const uint16_t adc[LINE_CHANNEL_COUNT],
                         uint32_t now_ms)
{
    uint8_t ch;
    uint8_t i;
    uint16_t black[LINE_CHANNEL_COUNT];

    const uint16_t weight = frame_weight(nav, now_ms);

    nav->last_frame_ms = now_ms;
    nav->have_frame = true;

    if (nav->calibrating) {
        line_cal_feed(&nav->cal, adc);
        return;
    }

    for (ch = 0U; ch < LINE_CHANNEL_COUNT; ch++) {
        black[ch] = (((nav->valid_mask >> ch) & 1U) != 0U)
                    ? line_blackness(&nav->cal, ch, adc[ch]) : 0U;
        nav->sum[ch] += black[ch];

        /* length of the current black stretch, counted frame by frame so a
         * blurred or averaged reading can never stretch a barcode bar */
        if (black[ch] >= NAV_BLACK_ON) {
            nav->ch_black[ch] = true;
        } else if (black[ch] <= NAV_BLACK_OFF) {
            nav->ch_black[ch] = false;
        } else {
            /* between the thresholds: keep the previous state */
        }
        if (nav->ch_black[ch] && (((nav->valid_mask >> ch) & 1U) != 0U)) {
            if (nav->ch_black_frames[ch] < UINT32_MAX) {
                nav->ch_black_frames[ch]++;
            }
        } else {
            nav->ch_black_frames[ch] = 0U;
        }
        /* leaky count: tolerates a patchy reading (different ink, a worn tile) that
         * an unbroken-black rule would miss, but never fills on mostly-white */
        if (nav->ch_black[ch] && (((nav->valid_mask >> ch) & 1U) != 0U)) {
            nav->ch_score[ch] = (uint16_t)(((nav->ch_score[ch] + ARMED_SCORE_UP) > ARMED_SCORE_MAX)
                                           ? ARMED_SCORE_MAX : (nav->ch_score[ch] + ARMED_SCORE_UP));
        } else if (nav->ch_score[ch] > 0U) {
            nav->ch_score[ch]--;
        } else {
            /* already empty */
        }
    }
    nav->sum_n++;

    for (i = 0U; i < NAV_LANE_COUNT; i++) {
        nav_lane_t *lane = &nav->lane[i];
        bool is_bar = false;
        uint16_t ticks = 0U;
        const uint8_t channel = LANE_CHANNEL[i];

        if ((((nav->valid_mask >> channel) & 1U) == 0U) ||
            (((nav->cfg.lane_mask >> channel) & 1U) == 0U)) {
            continue;
        }
        if (run_extractor_push_w(&lane->extractor, black[channel], weight, &is_bar, &ticks)) {
            char letter = '\0';

            lane->last_edge_ms = now_ms;
            lane->had_edge = true;
            if (lane->run_count < NAV_RUN_LOG) {
                lane->run_log[lane->run_count] = ticks;
            }
            if (lane->run_count < UINT8_MAX) {
                lane->run_count++;
            }
            if (is_bar && (ticks >= nav->cfg.barcode_min_bar) && (lane->bar_count < UINT8_MAX)) {
                lane->bar_count++;
            }
            if (barcode_push_run(&lane->decoder, is_bar, ticks, &letter) == BARCODE_DECODED) {
                handle_decode(nav, letter, now_ms);
            } else if ((nav->cfg.barcode_bars != 0U) && (lane->bar_count >= nav->cfg.barcode_bars)) {
                /* the count says a whole barcode went by even if the letter did not decode:
                 * log the attempt (try= and the [barlog] widths) so the failure can be seen */
                nav->last_attempt_runs = lane->run_count;
                for (uint8_t k = 0U; k < NAV_RUN_LOG; k++) {
                    nav->attempt_log[k] = lane->run_log[k];
                }
                nav->attempt_count++;
                nav->bars_done = true;
                nav->bars_done_ms = now_ms;
                nav->bars_done_count++;
                lanes_reset(nav);
            }
        }
        if (lane->had_edge && ((now_ms - lane->last_edge_ms) > nav->cfg.barcode_quiet_ms)) {
            if (lane->run_count >= 5U) {        /* a real attempt, not a stray edge */
                nav->last_attempt_runs = lane->run_count;
                for (uint8_t k = 0U; k < NAV_RUN_LOG; k++) {
                    nav->attempt_log[k] = lane->run_log[k];
                }
                nav->attempt_count++;
            }
            lane_reset(lane);                   /* a stale partial symbol */
        }
    }
}

void line_nav_note_overflow(line_nav_t *nav)
{
    lanes_reset(nav);
    nav->overflow_count++;
    nav->events |= NAV_EVENT_OVERFLOW;
}

void line_nav_set_hump(line_nav_t *nav, bool in_progress, uint32_t status_ms)
{
    nav->hump_active = in_progress;
    nav->hump_status_ms = status_ms;
}

/* ---- the steering step -------------------------------------------------- */

static void take_means(line_nav_t *nav)
{
    uint8_t ch;

    if (nav->sum_n == 0U) {
        return;                                 /* keep the previous values */
    }
    for (ch = 0U; ch < LINE_CHANNEL_COUNT; ch++) {
        nav->blackness[ch] = (uint16_t)(nav->sum[ch] / nav->sum_n);
        if (nav->cfg.smooth_shift == 0U) {
            nav->smooth[ch] = nav->blackness[ch];
        } else {
            const int32_t delta = (int32_t)nav->blackness[ch] - (int32_t)nav->smooth[ch];

            nav->smooth[ch] = (uint16_t)((int32_t)nav->smooth[ch] +
                                         (delta / (int32_t)(1U << nav->cfg.smooth_shift)));
        }
        nav->sum[ch] = 0U;
    }
    nav->sum_n = 0U;
}

void line_nav_step(line_nav_t *nav, uint32_t now_ms, int16_t speed_mm_s,
                   line_nav_out_t *out)
{
    const uint32_t junction_um = (uint32_t)nav->cfg.junction_min_mm * UM_PER_MM;
    /* a barcode has been read and not used yet: the next all-black is its junction */
    const bool armed = (nav->pending_valid &&
                        ((now_ms - nav->pending.timestamp_ms) <= nav->cfg.instruction_ttl_ms)) ||
                       (nav->bars_done &&
                        ((now_ms - nav->bars_done_ms) <= nav->cfg.instruction_ttl_ms));
    uint8_t excluded = 0U;
    uint8_t lanes_active = 0U;
    uint8_t any_black = 0U;
    uint8_t newly = 0U;
    uint8_t mask;
    bool visible;
    line_event_t tev;
    uint8_t i;
    const uint32_t dt_ms = (nav->have_step && (now_ms != nav->last_step_ms))
                           ? (now_ms - nav->last_step_ms) : DEFAULT_STEP_MS;

    take_means(nav);
    nav->last_step_ms = now_ms;
    nav->have_step = true;

    out->events = nav->events;
    nav->events = 0U;
    out->junction_mask = 0U;
    out->valid_mask = nav->valid_mask;
    {
        /* the whole tracking pair on a line: where a junction turn stops */
        const uint8_t pair = (uint8_t)(nav->valid_mask & nav->cfg.track_mask);
        uint8_t black = 0U;
        uint8_t k;

        for (k = 0U; k < LINE_CHANNEL_COUNT; k++) {
            if (nav->ch_black[k]) {
                black = (uint8_t)(black | (1U << k));
            }
        }
        out->pair_on_line = (pair != 0U) && ((black & pair) == pair);
        out->junction_armed = armed;
    }

    {
        /* Is a tracking sensor over the line (apart from the branch sensors)?
         * A junction needs the branch sensor (the far left one) black together with
         * at least ONE of them, so two of the three sensors with the left one always
         * among them: the left sensor alone is not enough. */
        uint8_t anchors = (uint8_t)(nav->cfg.track_mask & (uint8_t)(~nav->cfg.junction_mask) &
                                    nav->valid_mask);
        bool anchor_black = (anchors == 0U);
        uint8_t ch;

        for (ch = 0U; ch < LINE_CHANNEL_COUNT; ch++) {
            if ((((anchors >> ch) & 1U) != 0U) && nav->ch_black[ch]) {
                anchor_black = true;
            }
        }

        for (i = 0U; i < NAV_LANE_COUNT; i++) {
            const nav_lane_t *lane = &nav->lane[i];
            const uint8_t channel = LANE_CHANNEL[i];
            const bool valid = ((nav->valid_mask >> channel) & 1U) != 0U;
            const bool is_lane = ((nav->cfg.lane_mask >> channel) & 1U) != 0U;
            const bool is_branch = ((nav->cfg.junction_mask >> channel) & 1U) != 0U;
            const bool tracks = ((nav->cfg.track_mask >> channel) & 1U) != 0U;
            const bool active = is_lane && lane->had_edge &&
                                (lane->run_count >= NAV_ACTIVE_RUNS) &&
                                ((now_ms - lane->last_edge_ms) <= nav->cfg.barcode_quiet_ms);
            const bool black_now = valid && nav->ch_black[channel];
            /* travelled while black: speed (mm/s) x frames (ms) = micrometres */
            const uint32_t black_um = (speed_mm_s > 0)
                                      ? ((uint32_t)nav->ch_black_frames[channel] * (uint32_t)speed_mm_s) : 0U;

            if (black_now && (is_lane || is_branch)) {
                any_black |= LANE_SIDE[i];
            }
            if (active) {
                lanes_active |= (uint8_t)(1U << i);
            }
            /* keep a possible barcode bar out of the steering */
            if (valid && tracks && is_lane &&
                (active || ((nav->junction_latched & LANE_SIDE[i]) != 0U) ||
                 (black_now && anchor_black && (black_um < junction_um)))) {
                excluded |= (uint8_t)(1U << channel);
            }
            /*
             * A junction needs the tracking sensors black together with the
             * branch sensor.  With a barcode waiting that is all it takes: the
             * next time the branch sensor's score fills is the junction, whatever
             * the speed reads and even if the reading is patchy, so the flicker
             * guard is off.  With nothing waiting the branch sensor must be
             * steady and black for longer than any barcode bar, which needs the
             * speed, so a bar is never taken for a junction.
             */
            if (is_branch && anchor_black &&
                (armed ? (nav->ch_score[channel] >= ARMED_JUNCTION_SCORE)
                       : (!active && black_now && (black_um >= junction_um))) &&
                ((nav->junction_latched & LANE_SIDE[i]) == 0U)) {
                newly |= LANE_SIDE[i];
            }
        }
    }

    /* ---- junction: merge sides seen close together into one event ---- */
    if (newly != 0U) {
        if (nav->junction_pending == 0U) {
            nav->junction_since_ms = now_ms;
        }
        nav->junction_pending |= newly;
    }
    if ((nav->junction_pending != 0U) &&
        ((now_ms - nav->junction_since_ms) >= nav->cfg.junction_merge_ms)) {
        out->events |= NAV_EVENT_JUNCTION;
        nav->bars_done = false;                 /* its junction has come */
        out->junction_mask = nav->junction_pending;
        nav->junction_latched |= nav->junction_pending;
        nav->junction_pending = 0U;
    }
    if (any_black == 0U) {
        if (nav->clear_steps < CLEAR_STEPS_TO_RELEASE) {
            nav->clear_steps++;
        }
        if (nav->clear_steps >= CLEAR_STEPS_TO_RELEASE) {
            nav->junction_latched = 0U;
            nav->junction_pending = 0U;
        }
    } else {
        nav->clear_steps = 0U;
    }

    /* ---- line position ---- */
    mask = (uint8_t)(nav->valid_mask & nav->cfg.track_mask & (uint8_t)(~excluded));
    out->position_valid = line_position_weighted(nav->blackness, mask,
                                                 nav->cfg.channel_pos_permille, &out->position);
    if (!out->position_valid) {
        for (i = 0U; i < LINE_CHANNEL_COUNT; i++) {
            nav->smooth[i] = nav->blackness[i];     /* no line: forget the filter */
        }
    } else if (nav->cfg.smooth_shift != 0U) {
        int16_t smoothed;

        if (line_position_weighted(nav->smooth, mask, nav->cfg.channel_pos_permille, &smoothed)) {
            out->position = smoothed;
        }
    } else {
        /* unfiltered */
    }

    /* ---- lost / found, with bounded hump masking ---- */
    visible = out->position_valid;
    out->suppressing_lost = false;
    if (!visible && !nav->tracker.lost && nav->hump_active &&
        ((now_ms - nav->hump_status_ms) <= nav->cfg.hump_fresh_ms)) {
        if (!nav->suppressing) {
            nav->suppressing = true;
            nav->suppress_since_ms = now_ms;
        }
        if ((now_ms - nav->suppress_since_ms) <= nav->cfg.hump_suppress_max_ms) {
            visible = true;
            out->suppressing_lost = true;
        }
    } else if (visible || nav->tracker.lost) {
        nav->suppressing = false;
    } else {
        nav->suppressing = false;               /* no fresh hump context */
    }
    tev = line_tracker_update(&nav->tracker, visible);
    if (tev == LINE_EVENT_LOST) {
        out->events |= NAV_EVENT_LINE_LOST;
    } else if (tev == LINE_EVENT_FOUND) {
        out->events |= NAV_EVENT_LINE_FOUND;
    } else {
        /* no change */
    }
    out->line_lost = nav->tracker.lost;

    /* ---- steering: PD on the position ---- */
    out->steer_diff_mm_s = 0;
    if (out->position_valid) {
        int32_t prev = nav->have_prev_position ? nav->prev_position : out->position;
        /* PD in the form kp x (error + Td x d(error)/dt): the derivative time
         * sets the damping independently of the step period. */
        int32_t rate = ((out->position - prev) * (int32_t)nav->cfg.steer_td_ms) / (int32_t)dt_ms;
        int32_t diff = -(((int32_t)nav->cfg.steer_kp_mm_s * (out->position + rate)) / PERMILLE);

        out->steer_diff_mm_s = (int16_t)clamp_i32(diff, -nav->cfg.steer_limit_mm_s,
                                                  nav->cfg.steer_limit_mm_s);
        nav->prev_position = out->position;
        nav->have_prev_position = true;
        nav->position = out->position;
    } else {
        out->position = nav->position;
    }

    out->stale = !nav->have_frame || ((now_ms - nav->last_frame_ms) > nav->cfg.stale_ms);
    out->barcode_lanes_active = lanes_active;
}

/* ---- instructions ------------------------------------------------------- */

bool line_nav_has_instruction(const line_nav_t *nav, uint32_t now_ms)
{
    return nav->pending_valid &&
           ((now_ms - nav->pending.timestamp_ms) <= nav->cfg.instruction_ttl_ms);
}

bool line_nav_peek_instruction(const line_nav_t *nav, uint32_t now_ms,
                               nav_instruction_t *instruction)
{
    if (!line_nav_has_instruction(nav, now_ms)) {
        return false;
    }
    *instruction = nav->pending;
    return true;
}

bool line_nav_take_instruction(line_nav_t *nav, uint32_t now_ms,
                               nav_instruction_t *instruction)
{
    if (!nav->pending_valid) {
        return false;
    }
    if ((now_ms - nav->pending.timestamp_ms) > nav->cfg.instruction_ttl_ms) {
        nav->pending_valid = false;
        nav->expired_count++;
        return false;
    }
    *instruction = nav->pending;
    nav->pending_valid = false;
    return true;
}
