/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * line_nav.h
 * Line-following and barcode navigation (Buddy 3), built on line_sensor.h
 * and barcode.h.  It turns the 1 ms three-channel ADC frames into what the
 * Vehicle Controller needs:
 *
 *   - the line position and a steering request;
 *   - line-lost / line-found events (debounced, with bounded hump
 *     suppression);
 *   - junction events, told apart from barcodes by physical length;
 *   - validated, mapped barcode instructions, each usable exactly once.
 *
 * It never writes motor outputs.  Pure C.
 *
 * Geometry assumed (to be confirmed on the car): the centre sensor follows
 * the line; the two outer sensors sit beside it.  The barcode strip runs
 * beside the line (near edge about 29 mm from the line centre), so it is
 * read by whichever outer sensor is over it.  Both outer channels therefore
 * run a barcode "lane"; a lane that is seeing bars is excluded from the
 * steering position so a barcode cannot pull the car sideways.
 *
 * A junction is a black run on an outer sensor, together with the centre
 * sensor, that is longer than any barcode element can be (the widest bar is
 * about 9 mm, the crossing line is 18 mm).  Length needs speed, so the caller
 * passes the measured speed; with no speed there is no junction event.
 */
#ifndef LINE_NAV_H
#define LINE_NAV_H

#include <stdbool.h>
#include <stdint.h>

#include "barcode.h"
#include "line_sensor.h"

#define NAV_EVENT_LINE_LOST         (1U << 0)
#define NAV_EVENT_LINE_FOUND        (1U << 1)
#define NAV_EVENT_JUNCTION          (1U << 2)
#define NAV_EVENT_BARCODE           (1U << 3)   /* a mapped instruction was stored */
#define NAV_EVENT_BARCODE_REJECTED  (1U << 4)   /* decoded but not a known command */
#define NAV_EVENT_OVERFLOW          (1U << 5)   /* ADC frames were lost */

#define NAV_JUNCTION_LEFT           (1U << 0)
#define NAV_JUNCTION_RIGHT          (1U << 1)

#define NAV_MAP_MAX                 (26U)       /* letters A to Z */
#define NAV_LANE_COUNT              (2U)        /* the two outer channels */
#define NAV_RUN_LOG                 (32U)       /* runs kept per attempt for the status dump */

typedef enum {
    NAV_CMD_NONE = 0,
    NAV_CMD_LEFT,
    NAV_CMD_RIGHT,
    NAV_CMD_STRAIGHT,
    NAV_CMD_UTURN
} nav_command_t;

typedef struct {
    char          letter;
    nav_command_t command;
} nav_map_entry_t;

typedef struct {
    char          letter;
    nav_command_t command;
    uint16_t      id;               /* increments with every stored instruction */
    uint32_t      timestamp_ms;
} nav_instruction_t;

typedef struct {
    uint8_t  lost_frames;           /* steering frames without a line to declare it lost */
    uint8_t  found_frames;
    uint16_t junction_min_mm;       /* outer black run longer than this is a junction */
    uint16_t junction_merge_ms;     /* sides seen within this report as one junction */
    uint16_t barcode_quiet_ms;      /* a lane is "barcode-active" this long after an edge */
    uint32_t instruction_ttl_ms;    /* a stored instruction expires after this */
    uint16_t hump_suppress_max_ms;  /* longest a hump may mask line loss (about 1 s) */
    uint16_t hump_fresh_ms;         /* hump status older than this is not trusted */
    uint16_t stale_ms;              /* no ADC frame for this long: sensor-data stale */
    int16_t  steer_kp_mm_s;         /* wheel-speed difference at full-scale error */
    uint16_t steer_td_ms;           /* derivative time: damping of the steering */
    int16_t  steer_limit_mm_s;
    const nav_map_entry_t *map;     /* letter -> command table */
    uint8_t  map_len;

    /*
     * Sensor layout.  The closed-loop simulation (tests/host/test_follow_sim.c)
     * shows a sensor pair must be closer together than the line is wide
     * (about 16 mm for an 18 mm line) or the line can fall into the gap
     * between them, yet a barcode sits 29-49 mm from the line.  So the roles
     * are configurable: for example two close sensors track the line while a
     * far one reads the barcode and spots junction branches.
     */
    int16_t  channel_pos_permille[LINE_CHANNEL_COUNT];  /* lateral position, 1000 = reference spacing */
    uint8_t  track_mask;            /* sensors that steer the car (bit per channel) */
    uint8_t  lane_mask;             /* sensors that read barcodes (LEFT and/or RIGHT) */
    uint8_t  junction_mask;         /* sensors that spot junction branches (LEFT and/or RIGHT) */

    /*
     * Fixed white and black readings for the sensors in fixed_mask.  A digital
     * (DO) sensor only ever reads one of two values, so a sweep has nothing to
     * learn from it, and a sweep that never crosses black on it (the car held in
     * the air, or a far sensor that never reaches the line) would leave it
     * untrusted.  With fixed_black > fixed_white those channels are trusted
     * from start-up and after every calibration; the cost is that a dead one is
     * no longer caught by the sweep (watch the edge counts instead).  fixed_mask
     * 0 learns every channel from the sweep.
     */
    uint16_t fixed_white;
    uint16_t fixed_black;
    uint8_t  fixed_mask;            /* bit per channel */

    /*
     * Smoothing of the line position: each sensor's 10 ms mean is low-pass
     * filtered with a gain of 1 / 2^smooth_shift before the position is formed (0 =
     * off).  A digital sensor flickering at the edge of the line then gives a
     * smooth in-between value instead of a hard 0/1 step, which would otherwise
     * jerk the steering and the speed schedule.  Whether the line is visible is
     * still decided on the unfiltered reading, so line-lost timing is unchanged.
     */
    uint8_t  smooth_shift;

    /*
     * Counting bars.  A Code 39 symbol "*X*" always has 15 bars.  When a lane has seen
     * barcode_bars bars in one pass (each at least barcode_min_bar wide, in the same
     * units as a run: distance when the travel is reported, else ms), the barcode is
     * over, decoded or not, and the next thing that sensor sees is the junction.  0
     * switches the count off.
     */
    uint8_t  barcode_bars;
    uint16_t barcode_min_bar;
} line_nav_config_t;

typedef struct {
    run_extractor_t   extractor;
    barcode_decoder_t decoder;
    uint32_t          last_edge_ms;
    bool              had_edge;
    uint8_t           run_count;        /* runs emitted since the lane went quiet */
    uint8_t           bar_count;        /* bars among them (glitches left out) */
    uint16_t          run_log[NAV_RUN_LOG];   /* ticks (ms) of the first runs, for diagnosis */
} nav_lane_t;

typedef struct {
    line_nav_config_t cfg;
    uint8_t           map_packed[NAV_MAP_MAX];  /* the letter map, one byte per entry (see nav_map_pack): 26 bytes instead of 208 */
    line_cal_t        cal;
    bool              calibrating;
    uint8_t           valid_mask;
    uint16_t          blackness[LINE_CHANNEL_COUNT];   /* mean over the last step */
    uint16_t          smooth[LINE_CHANNEL_COUNT];      /* the same, low-pass filtered */
    uint32_t          sum[LINE_CHANNEL_COUNT];
    uint16_t          sum_n;

    uint32_t          last_frame_ms;
    bool              have_frame;

    /* distance travelled, to measure bar widths in distance rather than time */
    bool              travel_have;           /* a previous reading exists */
    bool              travel_on;             /* a usable window: widths are distances */
    int32_t           travel_last;           /* last reading (encoder counts) */
    uint32_t          travel_last_ms;
    uint32_t          travel_win_q;          /* counts moved in the window, in quarter counts */
    uint32_t          travel_win_ms;         /* length of that window */
    uint32_t          travel_carry;          /* remainder kept so no distance is lost */
    uint32_t          travel_prev_frame_ms;

    nav_lane_t        lane[NAV_LANE_COUNT];   /* [0] = LEFT channel, [1] = RIGHT */
    bool              ch_black[LINE_CHANNEL_COUNT];          /* Schmitt state per sensor */
    uint32_t          ch_black_frames[LINE_CHANNEL_COUNT];   /* consecutive 1 ms frames black */
    uint16_t          ch_score[LINE_CHANNEL_COUNT];          /* leaky count: black frames up, white down */

    bool              pending_valid;
    nav_instruction_t pending;
    uint16_t          next_id;
    uint8_t           events;                 /* latched until the next step */

    line_tracker_t    tracker;
    bool              hump_active;
    uint32_t          hump_status_ms;
    bool              suppressing;
    uint32_t          suppress_since_ms;
    int16_t           position;
    int16_t           prev_position;
    bool              have_prev_position;
    uint32_t          last_step_ms;
    bool              have_step;

    uint8_t           junction_latched;       /* sides already reported this crossing */
    uint8_t           junction_pending;       /* sides seen, waiting out the merge window */
    uint32_t          junction_since_ms;
    uint8_t           clear_steps;

    bool              bars_done;             /* a whole barcode went by: the next black is the junction */
    uint32_t          bars_done_ms;
    uint32_t          bars_done_count;       /* barcodes completed, by bar count or by decoding */
    uint16_t          last_attempt_runs;     /* bars+gaps in the last barcode attempt that went quiet */
    uint16_t          attempt_count;         /* attempts of 5+ runs that ended without a decode */
    uint16_t          attempt_log[NAV_RUN_LOG];   /* run widths (ms) of that attempt; first run is a bar */
    uint32_t          decoded_count;
    uint32_t          rejected_count;
    uint32_t          overwritten_count;
    uint32_t          expired_count;
    uint32_t          overflow_count;
} line_nav_t;

typedef struct {
    bool     stale;                 /* no fresh frames: treat as a sensor-data fault */
    bool     position_valid;
    int16_t  position;              /* -1000 (left) .. +1000 (right) */
    int16_t  steer_diff_mm_s;       /* right minus left; + veers left */
    bool     line_lost;             /* debounced state */
    bool     suppressing_lost;      /* a hump is masking line loss */
    uint8_t  events;                /* NAV_EVENT_* since the previous step */
    uint8_t  junction_mask;         /* NAV_JUNCTION_*, valid with NAV_EVENT_JUNCTION */
    uint8_t  valid_mask;            /* channels trusted after calibration */
    uint8_t  barcode_lanes_active;  /* bit per lane currently seeing bars */
    bool     pair_on_line;          /* every trusted tracking sensor sees black now */
    bool     junction_armed;        /* a barcode has gone by (15 bars or decoded): its junction is next */
} line_nav_out_t;

void line_nav_init(line_nav_t *nav, const line_nav_config_t *cfg);

/* ---- calibration ---------------------------------------------------- */

void line_nav_calibrate_begin(line_nav_t *nav);

/*
 * Finish calibration.  Returns true when a tracking sensor saw enough
 * contrast to follow a line; the trusted channels are then available in
 * valid_mask (a sensor with no contrast, like a stuck IR, is left out, except
 * channels with fixed readings, see fixed_mask).  Returns false
 * (and keeps no sensor trusted) otherwise.
 */
bool line_nav_calibrate_end(line_nav_t *nav);

/* ---- per-frame and per-step processing --------------------------------- */

/* Feed one ADC frame (order: left, centre, right).  Call for every 1 ms frame. */
void line_nav_push_frame(line_nav_t *nav, const uint16_t adc[LINE_CHANNEL_COUNT],
                         uint32_t now_ms);

/*
 * Tell the navigator how far the car has travelled (a counter that grows when
 * the car moves forward, such as the sum of both encoder counts).  Call it
 * once per step, just before pushing that step's frames.  From the second call
 * on, barcode bar widths are measured in distance (quarter counts) instead of
 * time, so an accelerating, stuttering or briefly stopped car still reads.
 * Without it, widths are in milliseconds, as before.
 */
void line_nav_set_travel(line_nav_t *nav, int32_t travel, uint32_t now_ms);

/*
 * Replace the letter -> command mapping (the car has none built in; it is sent
 * over MQTT).  Copies up to NAV_MAP_MAX entries; 0 entries removes the mapping, so
 * every decoded letter is rejected.  Affects the barcodes decoded after the call;
 * an instruction already stored keeps the command it was given.
 */
void line_nav_set_map(line_nav_t *nav, const nav_map_entry_t *entries, uint8_t len);

/*
 * One letter -> movement entry in ONE byte, the form it is stored in and the form it has on the MQTT wire: the letter index (A = 0 to Z = 25) in
 * bits 7 to 3 and the command (left 1, right 2, straight 3, u-turn 4) in bits 2 to 0.  Entries outside that range cannot be packed.
 */
#define NAV_PACKED_LETTER_SHIFT     (3U)
bool nav_map_pack(char letter, nav_command_t command, uint8_t *packed);   /* false: letter not A to Z, or no command */
char nav_map_letter(uint8_t packed);
nav_command_t nav_map_command(uint8_t packed);

/* The same as line_nav_set_map() for entries that are already packed (the form the Net task stages them in).  Bytes that do not unpack are skipped. */
void line_nav_set_map_packed(line_nav_t *nav, const uint8_t *packed, uint8_t len);

/* The ADC ring overflowed: whatever barcode attempt was in progress is void. */
void line_nav_note_overflow(line_nav_t *nav);

/* IMU hump status: in_progress and the time it was last updated. */
void line_nav_set_hump(line_nav_t *nav, bool in_progress, uint32_t status_ms);

/* Steering step (every 10 ms).  speed_mm_s is the measured centre speed. */
void line_nav_step(line_nav_t *nav, uint32_t now_ms, int16_t speed_mm_s,
                   line_nav_out_t *out);

/*
 * Take the stored barcode instruction, if there is one and it has not
 * expired.  It is consumed: a second call returns false, so each
 * instruction executes once.
 */
bool line_nav_take_instruction(line_nav_t *nav, uint32_t now_ms,
                               nav_instruction_t *instruction);

/* Copy the stored instruction without consuming it; false if none or expired. */
bool line_nav_peek_instruction(const line_nav_t *nav, uint32_t now_ms,
                               nav_instruction_t *instruction);

/* True while an unexpired instruction is waiting (not consumed). */
bool line_nav_has_instruction(const line_nav_t *nav, uint32_t now_ms);

#endif /* LINE_NAV_H */
