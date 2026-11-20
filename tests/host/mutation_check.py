#!/usr/bin/env python3
"""Mutation check for the host tests.

For each mutant: copy app_program/ and tests/host/ to a temp tree, apply one
deliberate bug to a source file, build and run the named test, and require it
to FAIL.  A mutant the test still passes ("survivor") means the test does not
protect that behaviour.

    python3 mutation_check.py            # run every mutant
    python3 mutation_check.py motion     # only mutants whose test name matches

Each mutant is (test, source file, text to replace, replacement, why it matters).
The text to replace must occur exactly once so a mutant can never silently
apply to the wrong place.
"""
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))

MUTANTS = []


def mutant(test, source, old, new, why):
    MUTANTS.append((test, source, old, new, why))


# ---- fault mask ---------------------------------------------------------
mutant("test_fault_mask", "fault_mask.c",
       "mask->bits = (fault_bits_t)(mask->bits | bits);",
       "mask->bits = bits;",
       "set overwrites other faults")
mutant("test_fault_mask", "fault_mask.c",
       "mask->bits = (fault_bits_t)(mask->bits & (fault_bits_t)(~bits));",
       "mask->bits = (fault_bits_t)(0U * bits);",
       "clear wipes unrelated faults")
mutant("test_fault_mask", "fault_mask.c",
       "    robot_irq_state_t state = robot_irq_save();\n\n    mask->bits = (fault_bits_t)(mask->bits | bits);\n    robot_irq_restore(state);",
       "    mask->bits = (fault_bits_t)(mask->bits | bits);",
       "set is not protected by a critical section")
mutant("test_fault_mask", "fault_mask.h",
       "                                  FAULT_RECOVERY_LIMIT))",
       "                                  0U))",
       "a stop-class fault no longer stops the car")
mutant("test_fault_mask", "fault_mask.h",
       "                                  FAULT_ULTRASONIC | FAULT_NO_SAFE_PATH | \\",
       "                                  FAULT_IMU | FAULT_ULTRASONIC | FAULT_NO_SAFE_PATH | \\",
       "an IMU failure stops the car")

# ---- encoder ------------------------------------------------------------
mutant("test_encoder", "encoder.c",
       "ENC_REV,  ENC_NONE, ENC_BAD,  ENC_FWD,",
       "ENC_FWD,  ENC_NONE, ENC_BAD,  ENC_FWD,",
       "reverse transition counted as forward")
mutant("test_encoder", "encoder.c",
       "        enc->illegal++;\n        return;",
       "        enc->illegal++;\n        enc->count += 2;\n        return;",
       "a missed edge invents distance")
mutant("test_encoder", "encoder.c",
       "    if (enc->invert) {\n        step = (int8_t)(-step);\n    }\n",
       "",
       "invert flag ignored")
mutant("test_encoder", "encoder.c",
       "    robot_irq_state_t state = robot_irq_save();\n\n    snap->count",
       "    robot_irq_state_t state = 0U;\n\n    snap->count",
       "snapshot not taken inside a critical section")

# ---- pid ----------------------------------------------------------------
mutant("test_pid", "pid.c",
       "        sum -= (int64_t)pid->kd * rate;",
       "        sum += (int64_t)pid->kd * rate;",
       "derivative sign wrong")
mutant("test_pid", "pid.c",
       "        pid->integral = clamp(pid->integral + (error * (int32_t)dt_ms),\n                              -pid->integral_limit, pid->integral_limit);",
       "        pid->integral = clamp(pid->integral + (error * (int32_t)dt_ms),\n                              INT32_MIN, INT32_MAX);",
       "integral windup not limited")
mutant("test_pid", "pid.c",
       "    if (sum > (int64_t)pid->out_max) {\n        return pid->out_max;\n    }",
       "",
       "output upper clamp removed")

# ---- motion -------------------------------------------------------------
mutant("test_motion", "motion.c",
       "        if ((int32_t)(now_ms - m->lease_deadline_ms) >= 0) {",
       "        if (false) {",
       "lease never expires")
mutant("test_motion", "motion.c",
       "    if (m->cancelled_valid && (cmd->id == m->cancelled_id)) {\n        return false;\n    }",
       "",
       "an expired or stopped command can be resumed by a refresh")
mutant("test_motion", "motion.c",
       "    if (m->active && (cmd->id == m->cmd.id)) {",
       "    if (false) {",
       "a refresh restarts the manoeuvre")
mutant("test_motion", "motion.c",
       "    if (m->have_id && !id_not_older(cmd->id, m->last_id)) {",
       "    if (m->have_id && !id_not_older(cmd->id, m->last_id) && false) {",
       "stale commands accepted")
mutant("test_motion", "motion.c",
       "    return (int16_t)(uint16_t)(id - reference) >= 0;",
       "    return id >= reference;",
       "id comparison breaks at 16-bit wrap")
mutant("test_motion", "motion.c",
       "    if (m->override_stop) {\n        return false;\n    }",
       "",
       "commands accepted during a safety override")
mutant("test_motion", "motion.c",
       "    return (v > limit) ? limit : v;",
       "    return limit;",
       "no deceleration: the car coasts past the target")
mutant("test_motion", "motion.c",
       "        *target_left = -dir * v;\n        *target_right = dir * v;",
       "        *target_left = dir * v;\n        *target_right = -dir * v;",
       "turn direction reversed")
mutant("test_motion", "motion.c",
       "    if ((abs32(m->speed_left) < (int32_t)m->cfg.stopped_speed_mm_s) &&",
       "    if (true &&",
       "STOPPED reported while the left wheel is still moving")
mutant("test_motion", "motion.c",
       "        (abs32(m->speed_right) < (int32_t)m->cfg.stopped_speed_mm_s)) {",
       "        true) {",
       "STOPPED reported while the right wheel is still moving")
mutant("test_motion", "motion.c",
       "    return (now_ms - *since) >= cfg->encoder_fault_ms;",
       "    return false;",
       "dead encoder never flagged")
mutant("test_motion", "motion.c",
       "    int64_t num = (int64_t)counts * (int64_t)cfg->wheel_diameter_mm * PI_MICRO;",
       "    int64_t num = (int64_t)counts * (int64_t)cfg->wheel_diameter_mm * 3000000LL;",
       "wrong pi in the distance conversion")
mutant("test_motion", "motion.c",
       "        *target_left = (dir * v) - correction;\n        *target_right = (dir * v) + correction;",
       "        *target_left = (dir * v) - (correction * 0);\n        *target_right = (dir * v) + (correction * 0);",
       "no straight-line correction")


# ---- imu ----------------------------------------------------------------
mutant("test_imu", "imu.c",
       "        return (int16_t)(raw / LSM303DLHC_SHIFT_DIV);",
       "        return raw;",
       "LSM303DLHC 12-bit shift forgotten")
mutant("test_imu", "imu.c",
       "    return to_i16((int32_t)((-num * PERMILLE) / den));",
       "    return to_i16((int32_t)((num * PERMILLE) / den));",
       "pitch sign inverted (nose up reads as down)")
mutant("test_imu", "imu.c",
       "    h->elevation_um += ds_mm * pitch_permille;",
       "    h->elevation_um += ((ds_mm * 0) + 3) * pitch_permille;",
       "height ignores the distance travelled (depends on sample count)")
mutant("test_imu", "imu.c",
       "    if (aborted || h->inputs_bad) {",
       "    if (aborted) {",
       "missing distance or IMU fault still gives a height")
mutant("test_imu", "imu.c",
       "    if ((validity == HUMP_VALID) && (height_mm > h->highest_valid_mm)) {",
       "    if ((validity != HUMP_UNAVAILABLE) && (height_mm > h->highest_valid_mm)) {",
       "a low-confidence height becomes the highest valid one")
mutant("test_imu", "imu.c",
       "            if ((in->now_ms - h->candidate_since_ms) >= h->cfg.min_duration_ms) {",
       "            if ((in->now_ms - h->candidate_since_ms) >= 0U) {",
       "minimum-duration filter removed (spikes become humps)")
mutant("test_imu", "imu.c",
       "    return (uint16_t)((((uint16_t)status & STATUS_MASK) << STATUS_SHIFT) |",
       "    return (uint16_t)((((uint16_t)status & STATUS_MASK) << HEIGHT_SHIFT) |",
       "packed status overlaps the height field")
mutant("test_imu", "imu.c",
       "        if (h->disturbed) {\n            validity = HUMP_LOW_CONFIDENCE;\n        } else if",
       "        if (false) {\n            validity = HUMP_LOW_CONFIDENCE;\n        } else if",
       "acceleration disturbance ignored")
mutant("test_imu", "imu.c",
       "        } else if ((abs32(h->elevation_um) * PERCENT) >\n                   (h->peak_um * (int32_t)h->cfg.net_elevation_percent)) {",
       "        } else if (false) {",
       "a step (not a hump) is accepted as a hump")
mutant("test_imu", "imu.c",
       "            code = (uint8_t)HUMP_HEIGHT_OUT_OF_RANGE;\n            validity = HUMP_LOW_CONFIDENCE;",
       "            code = (uint8_t)HUMP_HEIGHT_MAX_MM;\n            validity = HUMP_VALID;",
       "out-of-range height reported as a real number")
mutant("test_imu", "imu.c",
       "    if (spread > cal->spread_limit_mg) {",
       "    if (spread > 30000) {",
       "a moving car is accepted as the rest baseline")
# ---- ultrasonic and servo -------------------------------------------------
mutant("test_ultrasonic", "ultrasonic.c",
       "    mm = (echo_us * SOUND_MM_PER_MS) / ROUND_TRIP_DIV_US;",
       "    mm = (echo_us * SOUND_MM_PER_MS) / MS_PER_S;",
       "round trip not halved (distances doubled)")
mutant("test_ultrasonic", "ultrasonic.c",
       "    if (!echo_complete) {\n        return US_NO_ECHO;\n    }",
       "    if (!echo_complete && false) {\n        return US_NO_ECHO;\n    }",
       "a timeout is read as a very large distance")
mutant("test_ultrasonic", "ultrasonic.c",
       "        return m->obstacle;                     /* neither declares nor clears */",
       "        return false;                           /* neither declares nor clears */",
       "a no-echo clears a held obstacle")
mutant("test_ultrasonic", "ultrasonic.c",
       "    if (m->have_seq && (reading->seq == m->last_seq)) {",
       "    if (false) {",
       "one measurement counted twice")
mutant("test_ultrasonic", "ultrasonic.c",
       "    } else if ((uint32_t)median > release_mm) {",
       "    } else if ((uint32_t)median > ((uint32_t)threshold_mm + (release_mm * 0U))) {",
       "no hysteresis on release")
mutant("test_ultrasonic", "ultrasonic.c",
       "    m->confirm_count = (confirm_count == 0U) ? 1U : confirm_count;",
       "    m->confirm_count = (uint8_t)((confirm_count * 0U) + 1U);",
       "no confirmation count (one reading flips the state)")
mutant("test_ultrasonic", "ultrasonic.c",
       "    return v[m->count / 2U];",
       "    return v[0];",
       "median replaced by minimum")
mutant("test_ultrasonic", "ultrasonic.c",
       "    return (uint16_t)((mm > (int32_t)US_MAX_RANGE_MM) ? (int32_t)US_MAX_RANGE_MM : mm);",
       "    return (uint16_t)mm;",
       "stop threshold not limited to the sensor range")
mutant("test_ultrasonic", "servo.c",
       "    if (cfg->invert) {\n        angle = SERVO_FULL_SWEEP_DEG - angle;\n    }",
       "",
       "servo invert ignored")
mutant("test_ultrasonic", "servo.c",
       "    int32_t angle = (int32_t)servo_clamp_angle(cfg, angle_deg) + cfg->trim_deg;",
       "    int32_t angle = (int32_t)servo_clamp_angle(cfg, angle_deg);",
       "servo trim ignored")
mutant("test_ultrasonic", "servo.c",
       "    return (uint8_t)clamp_i32(angle_deg, cfg->min_angle_deg, cfg->max_angle_deg);",
       "    return (uint8_t)clamp_i32(angle_deg, cfg->min_angle_deg * 0, 255);",
       "servo angle not limited to the safe sweep")

# ---- obstacle -------------------------------------------------------------
mutant("test_obstacle", "obstacle.c",
       "        add_step(plan, true, sign * 90);\n        add_step(plan, false, lateral);",
       "        add_step(plan, true, -sign * 90);\n        add_step(plan, false, lateral);",
       "bypass turns toward the obstacle instead of away")
mutant("test_obstacle", "obstacle.c",
       "    left_ok = ((int32_t)p->left_clearance_mm >= required) &&",
       "    left_ok = true &&",
       "left clearance not checked (could drive into a wall)")
mutant("test_obstacle", "obstacle.c",
       "    if (!real) {\n        return;                                 /* silence is not free space */\n    }\n    p->valid = true;",
       "    if (!real && false) {\n        return;\n    }\n    p->valid = true;",
       "a scan with no echoes is accepted as valid")
mutant("test_obstacle", "obstacle.c",
       "            int16_t flo = (int16_t)lo - (int16_t)s->cfg.coarse_step_deg;",
       "            int16_t flo = (int16_t)lo;",
       "fine scan does not cover the edge of the obstacle")
mutant("test_obstacle", "obstacle.c",
       "    sort_by_angle(merged, n);\n",
       "    sort_by_angle(merged, 0U);\n",
       "merged scan points left unsorted")
mutant("test_obstacle", "obstacle.c",
       "        } else if ((now_ms - s->state_since_ms) >= s->settle_ms) {\n            s->state = SCAN_MEASURING;",
       "        } else if ((now_ms - s->state_since_ms) >= 0U) {\n            s->state = SCAN_MEASURING;",
       "measuring before the servo has settled")
mutant("test_obstacle", "obstacle.c",
       "        } else if ((now_ms - s->request_ms) >= s->cfg.measure_timeout_ms) {",
       "        } else if (false) {",
       "waits forever on a dead sensor")
mutant("test_obstacle", "obstacle.c",
       "              (lateral_left <= (int32_t)cfg->recovery_limit_mm);",
       "              true;",
       "detour longer than the recovery limit accepted")
mutant("test_obstacle", "obstacle.c",
       "    go_left = left_ok && (!right_ok || (lateral_left <= lateral_right));",
       "    go_left = left_ok;",
       "always bypasses left even when right is the shorter detour")
mutant("test_obstacle", "obstacle.c",
       "    if ((int32_t)p->forward_mm < (int32_t)cfg->min_turn_distance_mm) {",
       "    if (false) {",
       "no reverse when too close to turn")
mutant("test_obstacle", "obstacle.c",
       "    if ((p->left_edge_x_mm > corridor) || (p->right_edge_x_mm < -corridor)) {",
       "    if (false) {",
       "an object beside the path is treated as blocking it")
mutant("test_obstacle", "obstacle.c",
       "    return cfg->left_is_high_angle ? x : -x;",
       "    return cfg->left_is_high_angle ? x : x;",
       "servo mounting direction ignored")

mutant("test_barcode", "barcode.c",
       "#define GAP_SEPARATION_NUM  (8U)\n#define GAP_SEPARATION_DEN  (5U)",
       "#define GAP_SEPARATION_NUM  (1U)\n#define GAP_SEPARATION_DEN  (1U)",
       "a wide-looking gap between characters is accepted")
mutant("test_barcode", "barcode.c",
       "#define SEPARATION_NUM      (13U)\n#define SEPARATION_DEN      (10U)",
       "#define SEPARATION_NUM      (1U)\n#define SEPARATION_DEN      (1U)",
       "a character with no clear narrow/wide split is accepted")
mutant("test_barcode", "barcode.c",
       "#define SEPARATION_NUM      (13U)\n#define SEPARATION_DEN      (10U)",
       "#define SEPARATION_NUM      (2U)\n#define SEPARATION_DEN      (1U)",
       "a real barcode with edge bias is refused")

# ---- barcode mapping over MQTT ------------------------------------------
mutant("test_barcode_map", "barcode_map.c",
       "        if (out[i].letter == letter) {\n            return BMAP_ERR_DUPLICATE;",
       "        if ((out[i].letter == letter) && false) {\n            return BMAP_ERR_DUPLICATE;",
       "the same letter can be mapped twice")
mutant("test_barcode_map", "barcode_map.c",
       "    if (!is_letter(token[0])) {\n        return BMAP_ERR_LETTER;",
       "    if (!is_letter(token[0]) && false) {\n        return BMAP_ERR_LETTER;",
       "a non-letter is accepted as a barcode letter")
mutant("test_barcode_map", "barcode_map.c",
       "    if ((n < 3U) || (n > TOKEN_MAX) || ((token[1] != '=') && (token[1] != ':'))) {",
       "    if ((n < 3U) || (n > TOKEN_MAX)) {",
       "an entry with no = or : is accepted")
mutant("test_barcode_map", "barcode_map.c",
       "        if (same(token, \"CLEAR\")) {",
       "        if (true) {",
       "any lone token clears the mapping")
mutant("test_barcode_map", "barcode_map.c",
       "        if ((n == BMAP_MAX) && is_action_string(token, (uint8_t)n)) {",
       "        if ((n == BMAP_MAX) && (is_action_string(token, (uint8_t)n) || true)) {",
       "the 26-letter form accepts letters that are not actions")
mutant("test_line_nav", "line_nav.c",
       "    const uint8_t n = (len > NAV_MAP_MAX) ? (uint8_t)NAV_MAP_MAX : len;",
       "    const uint8_t n = (len > 40U) ? 40U : len;",
       "a mapping longer than the alphabet overruns the table")
mutant("test_line_nav", "line_nav.c",
       "    nav->cfg.map = nav->map_store;\n    nav->cfg.map_len = n;",
       "    nav->cfg.map_len = n;",
       "the new mapping is copied but never used")
mutant("test_mqtt_client", "mqtt_client.c",
       "    if (c->cfg.extra_topic != NULL) {",
       "    if (false) {",
       "the mapping topic is never subscribed")

# ---- line_nav -----------------------------------------------------------
mutant("test_line_nav", "line_nav.c",
       "    if (!nav->travel_on) {\n        return 1U;\n    }",
       "    if (true) {\n        return 1U;\n    }",
       "bar widths ignore the distance travelled")
mutant("test_line_nav", "line_nav.c",
       "        nav->travel_win_q = (delta > 0) ? ((uint32_t)delta * 4U) : 0U;",
       "        nav->travel_win_q = (delta > 0) ? ((uint32_t)delta * 4U) : 4U;",
       "a stopped or reversing car still adds bar width")
mutant("test_line_nav", "line_nav.c",
       "            } else if ((nav->cfg.barcode_bars != 0U) && (lane->bar_count >= nav->cfg.barcode_bars)) {",
       "            } else if ((nav->cfg.barcode_bars != 0U) && (lane->bar_count >= (nav->cfg.barcode_bars - 1U))) {",
       "a barcode is taken as over one bar early")
mutant("test_line_nav", "line_nav.c",
       "            if (is_bar && (ticks >= nav->cfg.barcode_min_bar) && (lane->bar_count < UINT8_MAX)) {",
       "            if (is_bar && (lane->bar_count < UINT8_MAX)) {",
       "glitches are counted as bars")
mutant("test_line_nav", "line_nav.c",
       "        nav->bars_done = false;                 /* its junction has come */\n",
       "",
       "a counted barcode arms every later junction too")
mutant("test_line_nav", "line_nav.c",
       "            if (is_branch && anchor_black &&",
       "            if (is_branch &&",
       "the left sensor alone is a junction")
mutant("test_line_nav", "line_nav.c",
       "        int32_t diff = -(((int32_t)nav->cfg.steer_kp_mm_s * (out->position + rate)) / PERMILLE);",
       "        int32_t diff = (((int32_t)nav->cfg.steer_kp_mm_s * (out->position + rate)) / PERMILLE);",
       "steers away from the line")
mutant("test_line_nav", "line_nav.c",
       "    tev = line_tracker_update(&nav->tracker, visible);",
       "    tev = line_tracker_update(&nav->tracker, true);",
       "line loss never reported")
mutant("test_line_nav", "line_nav.c",
       "        if ((now_ms - nav->suppress_since_ms) <= nav->cfg.hump_suppress_max_ms) {",
       "        if (true) {",
       "a hump masks line loss forever")
mutant("test_line_nav", "line_nav.c",
       "        ((now_ms - nav->hump_status_ms) <= nav->cfg.hump_fresh_ms)) {",
       "        true) {",
       "stale hump status still masks line loss")
mutant("test_line_nav", "line_nav.c",
       "    if (!visible && !nav->tracker.lost && nav->hump_active &&",
       "    if (!visible && nav->hump_active &&",
       "a hump can fake a line-found event")
mutant("test_line_nav", "line_nav.c",
       "    *instruction = nav->pending;\n    nav->pending_valid = false;\n    return true;",
       "    *instruction = nav->pending;\n    return true;",
       "an instruction can be executed more than once")
mutant("test_line_nav", "line_nav.c",
       "    if ((now_ms - nav->pending.timestamp_ms) > nav->cfg.instruction_ttl_ms) {\n        nav->pending_valid = false;",
       "    if (((now_ms - nav->pending.timestamp_ms) > nav->cfg.instruction_ttl_ms) && false) {\n        nav->pending_valid = false;",
       "a stale barcode instruction is executed")
mutant("test_line_nav", "line_nav.c",
       "    if (entry == NULL) {\n        nav->rejected_count++;\n        nav->events |= NAV_EVENT_BARCODE_REJECTED;\n        return;\n    }",
       "    if (entry == NULL) {\n        entry = &nav->cfg.map[0];\n    }",
       "an unknown barcode is executed as the first command")
mutant("test_line_nav", "line_nav.c",
       "void line_nav_note_overflow(line_nav_t *nav)\n{\n    lanes_reset(nav);",
       "void line_nav_note_overflow(line_nav_t *nav)\n{",
       "an ADC overflow does not void the barcode attempt")
mutant("test_line_nav", "line_nav.c",
       "                       : (!active && black_now && (black_um >= junction_um))) &&",
       "                       : (!active && black_now && (black_um >= 0U))) &&",
       "any black on an outer sensor is a junction")
mutant("test_line_nav", "line_nav.c",
       "                (armed ? (nav->ch_score[channel] >= ARMED_JUNCTION_SCORE)",
       "                (armed ? (black_um >= junction_um)",
       "a junction after a barcode still needs the wheel speed")
mutant("test_line_nav", "line_nav.c",
       "                       : (!active && black_now && (black_um >= junction_um))) &&",
       "                       : (black_now && (black_um >= junction_um))) &&",
       "a wide barcode bar becomes a junction when the speed reads high")
mutant("test_line_nav", "line_nav.c",
       "                (armed ? (nav->ch_score[channel] >= ARMED_JUNCTION_SCORE)",
       "                (armed ? (black_now && (nav->ch_black_frames[channel] >= 30U))",
       "a patchy reading misses the junction (an unbroken run of black is required)")
mutant("test_vehicle", "vehicle.c",
       "                vc->turn_lines = (abs32(vc->pending_turn_deg) >= UTURN_MIN_DEG) ? 2U : 1U;",
       "                vc->turn_lines = 1U;",
       "a U-turn stops on the first branch it meets")
mutant("test_vehicle", "vehicle.c",
       "                          (vc->pending_turn_deg > 0) ? spin : (int16_t)(-spin), 0, 0);",
       "                          spin, 0, 0);",
       "every junction turn spins left")
mutant("test_line_nav", "line_nav.c",
       "static const uint8_t LANE_SIDE[NAV_LANE_COUNT] = { NAV_JUNCTION_LEFT, NAV_JUNCTION_RIGHT };",
       "static const uint8_t LANE_SIDE[NAV_LANE_COUNT] = { NAV_JUNCTION_RIGHT, NAV_JUNCTION_LEFT };",
       "junction side reported mirrored")
mutant("test_line_nav", "line_nav.c",
       "        nav->junction_latched |= nav->junction_pending;",
       "",
       "a junction is reported on every step while crossing it")
mutant("test_line_nav", "line_nav.c",
       "                 (black_now && anchor_black && (black_um < junction_um)))) {",
       "                 (black_now && anchor_black && (black_um < junction_um) && (junction_um == 0U)))) {",
       "a barcode bar pulls the steering")
# (skipping an untrusted lane is only an optimisation: its blackness is already
#  zeroed, so removing the skip is an equivalent mutant and is not listed)
mutant("test_line_nav", "line_nav.c",
       "    if ((mask & nav->cfg.track_mask) == 0U) {",
       "    if (false) {",
       "calibration accepts a centre sensor with no contrast")
mutant("test_line_nav", "line_nav.c",
       "        ((now_ms - nav->junction_since_ms) >= nav->cfg.junction_merge_ms)) {",
       "        ((now_ms - nav->junction_since_ms) >= 0U)) {",
       "a T-junction is split into two single-side junctions")
mutant("test_line_nav", "line_nav.c",
       "    out->stale = !nav->have_frame || ((now_ms - nav->last_frame_ms) > nav->cfg.stale_ms);",
       "    out->stale = false;",
       "stale sensor data not reported")
mutant("test_line_nav", "line_nav.c",
       "    if (nav->pending_valid) {\n        nav->overwritten_count++;\n    }",
       "",
       "a replaced instruction is not counted")

# ---- telemetry ----------------------------------------------------------
mutant("test_telemetry", "telemetry.c",
       "        uint8_t shift = (uint8_t)(width - 1U - i);",
       "        uint8_t shift = i;",
       "bits written LSB-first (the laptop would decode garbage)")
mutant("test_telemetry", "telemetry.c",
       "        return (int32_t)(v - ((uint32_t)1U << width));",
       "        return (int32_t)v;",
       "negative values lose their sign")
mutant("test_telemetry", "telemetry.c",
       "    if (pos != s->line_position) {\n        sat |= TEL_SAT_POSITION;\n    }",
       "",
       "a clipped position is not flagged")
mutant("test_telemetry", "telemetry.c",
       "    if (cm > TEL_OBSTACLE_MAX_CM) {\n        cm = TEL_OBSTACLE_MAX_CM;\n        sat |= TEL_SAT_OBSTACLE;\n    }",
       "    if (cm > TEL_OBSTACLE_MAX_CM) {\n        cm = TEL_OBSTACLE_MAX_CM;\n    }",
       "a clipped obstacle distance is not flagged")
mutant("test_telemetry", "telemetry.c",
       "    uint32_t cm = s->obstacle_valid ? s->obstacle_cm : 0U;",
       "    uint32_t cm = s->obstacle_cm;",
       "an invalid obstacle reading is sent as a distance")
mutant("test_telemetry", "telemetry.c",
       "    if (q->count >= TEL_QUEUE_DEPTH) {\n        q->dropped_full++;                      /* drop the NEW record */",
       "    if (q->count >= TEL_QUEUE_DEPTH) {\n        q->head = (uint8_t)((q->head + 1U) % TEL_QUEUE_DEPTH);\n        q->count--;\n        q->dropped_full++;",
       "a full queue drops the OLD record instead of the new one")
mutant("test_telemetry", "telemetry.c",
       "        if ((now_ms - slot->ts_ms) > TEL_MAX_AGE_MS) {",
       "        if (((now_ms - slot->ts_ms) > TEL_MAX_AGE_MS) && false) {",
       "stale telemetry is published late")
mutant("test_telemetry", "telemetry.c",
       "        if ((now_ms - slot->ts_ms) > TEL_MAX_AGE_MS) {",
       "        if ((now_ms - slot->ts_ms) >= TEL_MAX_AGE_MS) {",
       "freshness boundary off by one")
mutant("test_telemetry", "telemetry.c",
       "    state = robot_irq_save();\n    if (q->count >= TEL_QUEUE_DEPTH) {",
       "    state = 0U;\n    if (q->count >= TEL_QUEUE_DEPTH) {",
       "queue push not protected")
# (the explicit length check in open_reader duplicates the reader's own
#  underflow flag, so removing it is an equivalent mutant and is not listed)
mutant("test_telemetry", "telemetry.c",
       "    return bitr_get(r, TEL_TYPE_BITS) == type;",
       "    return (bitr_get(r, TEL_TYPE_BITS) == type) || true;",
       "a message of the wrong type is decoded")
mutant("test_telemetry", "telemetry.c",
       "    if (len > TEL_RECORD_MAX) {\n        return false;\n    }",
       "",
       "an oversize record overflows the queue slot")

# ---- mqtt codec -----------------------------------------------------------
mutant("test_mqtt_codec", "mqtt_codec.c",
       "        if (value > 0U) {\n            digit |= LEN_CONTINUE;\n        }",
       "",
       "remaining length limited to one byte")
mutant("test_mqtt_codec", "mqtt_codec.c",
       "#define TYPE_SUBSCRIBE      (0x82U)",
       "#define TYPE_SUBSCRIBE      (0x80U)",
       "SUBSCRIBE sent with the wrong fixed-header flags")
mutant("test_mqtt_codec", "mqtt_codec.c",
       "    uint8_t flags = FLAG_CLEAN;",
       "    uint8_t flags = 0U;",
       "CONNECT without the clean-session flag")
mutant("test_mqtt_codec", "mqtt_codec.c",
       "    return w.ok ? finish((uint8_t)(TYPE_PUBLISH | (retain ? 1U : 0U)), body, w.len, out, cap)",
       "    return w.ok ? finish((uint8_t)(TYPE_PUBLISH | ((retain && false) ? 1U : 0U)), body, w.len, out, cap)",
       "retain flag ignored")
mutant("test_mqtt_codec", "mqtt_codec.c",
       "        if ((pos > n) || (topic_len > MQTT_MAX_TOPIC)) {",
       "        if (topic_len > MQTT_MAX_TOPIC) {",
       "a PUBLISH with a topic longer than the packet is accepted")
mutant("test_mqtt_codec", "mqtt_codec.c",
       "            if (p->skip_count >= p->remaining) {",
       "            if ((uint16_t)p->skip_count >= p->remaining) {",
       "a 16-bit skip counter never finishes a very large packet")
mutant("test_mqtt_codec", "mqtt_codec.c",
       "                p->stage = (p->remaining > MQTT_MAX_PACKET) ? STAGE_SKIP : STAGE_BODY;",
       "                p->stage = STAGE_BODY;",
       "an oversize packet overflows the body buffer")
mutant("test_mqtt_codec", "mqtt_codec.c",
       "                if (p->length_bytes >= LEN_MAX_BYTES) {",
       "                if (p->length_bytes >= 40U) {",
       "an over-long remaining length is accepted")
mutant("test_mqtt_codec", "mqtt_codec.c",
       "        pkt->connack_code = b[1];",
       "        pkt->connack_code = b[0];",
       "the CONNACK return code is read from the wrong byte")
mutant("test_mqtt_codec", "mqtt_codec.c",
       "    if (c->username != NULL) {\n        flags |= FLAG_USERNAME;\n    }",
       "",
       "username flag not set")

# ---- mqtt client ----------------------------------------------------------
mutant("test_mqtt_client", "mqtt_client.c",
       "    c->backoff_ms *= 2U;",
       "    c->backoff_ms *= 1U;",
       "reconnect backoff never grows (hammers a dead broker)")
mutant("test_mqtt_client", "mqtt_client.c",
       "    if (c->backoff_ms > c->cfg.backoff_max_ms) {\n        c->backoff_ms = c->cfg.backoff_max_ms;\n    }",
       "",
       "reconnect backoff is unbounded")
mutant("test_mqtt_client", "mqtt_client.c",
       "                c->backoff_ms = c->cfg.backoff_initial_ms;\n                if (!send_session_start(c)) {",
       "                if (!send_session_start(c)) {",
       "backoff not reset after a successful connection")
mutant("test_mqtt_client", "mqtt_client.c",
       "    cc.will_topic = c->cfg.status_topic;",
       "    cc.will_topic = NULL;",
       "no last-will message")
mutant("test_mqtt_client", "mqtt_client.c",
       "    if (send_packet(c, buf, n) != SEND_OK) {\n        return false;\n    }\n    if (c->cfg.extra_topic != NULL) {",
       "    if (c->cfg.extra_topic != NULL) {",
       "never subscribes to the command topic")
mutant("test_mqtt_client", "mqtt_client.c",
       "    if (c->state != MQTT_CONNECTED) {\n        c->dropped_offline++;\n        return false;\n    }",
       "    if (c->state != MQTT_CONNECTED) {\n        return false;\n    }",
       "offline drops are not counted")
mutant("test_mqtt_client", "mqtt_client.c",
       "    if (!c->link_up) {\n        if (c->state != MQTT_DISCONNECTED) {\n            c->net.tcp_close(c->net.ctx);\n            c->state = MQTT_DISCONNECTED;\n            parser_reset(c);\n            c->ping_outstanding = false;\n        }",
       "    if (!c->link_up) {\n        if (c->state != MQTT_DISCONNECTED) {\n            fail(c, now_ms);\n        }",
       "losing the WiFi link is counted as a broker failure")
mutant("test_mqtt_client", "mqtt_client.c",
       "    if (sent == 0) {\n        return SEND_BLOCKED;\n    }",
       "",
       "a would-block is treated as a broken connection")
mutant("test_mqtt_client", "mqtt_client.c",
       "    if (sent != (int)n) {\n        return SEND_FAILED;                     /* error or a partial write */\n    }",
       "    if (sent < 0) {\n        return SEND_FAILED;\n    }",
       "a partial write is accepted (the stream desynchronises)")
mutant("test_mqtt_client", "mqtt_client.c",
       "        } else if ((now_ms - c->last_tx_ms) >= ((uint32_t)c->cfg.keepalive_s * MS_PER_S / 2U)) {",
       "        } else if (false) {",
       "keep-alive pings never sent")
mutant("test_mqtt_client", "mqtt_client.c",
       "            if ((now_ms - c->ping_sent_ms) >= PING_RESPONSE_MS) {",
       "            if (false) {",
       "a half-open connection is never detected")
mutant("test_mqtt_client", "mqtt_client.c",
       "                if ((c->state != MQTT_AWAIT_CONNACK) || (c->pkt.connack_code != 0U)) {",
       "                if (c->state != MQTT_AWAIT_CONNACK) {",
       "a refused CONNACK is treated as success")
mutant("test_mqtt_client", "mqtt_client.c",
       "                   ((now_ms - c->attempt_start_ms) >= c->cfg.connect_timeout_ms)) {",
       "                   false) {",
       "waits forever for a CONNACK")
mutant("test_mqtt_client", "mqtt_client.c",
       "        } else if ((now_ms - c->attempt_start_ms) >= c->cfg.connect_timeout_ms) {\n            fail(c, now_ms);\n        } else {\n            /* still connecting */",
       "        } else if (false) {\n            fail(c, now_ms);\n        } else {\n            /* still connecting */",
       "waits forever for the TCP connection")
mutant("test_mqtt_client", "mqtt_client.c",
       "                if (c->ever_connected) {\n                    c->reconnects++;\n                }",
       "",
       "reconnections are not counted")
mutant("test_mqtt_client", "mqtt_client.c",
       "    if ((int32_t)(now_ms - c->next_attempt_ms) >= 0) {",
       "    if (true) {",
       "reconnects ignore the backoff time")

# ---- safety monitor -------------------------------------------------------
mutant("test_safety", "safety.c",
       "    if (in->line_required && in->line_stale) {",
       "    if (in->line_stale) {",
       "stale line data raises a fault even while parked")
mutant("test_safety", "safety.c",
       "    if (in->line_required && in->line_stale) {",
       "    if (in->line_required && in->line_stale && false) {",
       "stale line data while following is not reported")
mutant("test_safety", "safety.c",
       "    if (in->driving && in->encoder_fault) {",
       "    if (in->encoder_fault) {",
       "a silent encoder raises a fault while parked")
mutant("test_safety", "safety.c",
       "    if (in->driving && (in->motion_age_ms > limits->motion_stall_ms)) {",
       "    if (in->driving && (in->motion_age_ms >= limits->motion_stall_ms)) {",
       "motion stall limit off by one")
mutant("test_safety", "safety.c",
       "    if (in->imu_read_failed || (in->imu_age_ms > limits->imu_stale_ms)) {\n        bits |= FAULT_IMU;",
       "    if (in->imu_read_failed || (in->imu_age_ms > limits->imu_stale_ms)) {\n        bits |= FAULT_MOTION;",
       "an IMU failure is reported as a motion (stop-class) fault")
mutant("test_safety", "safety.c",
       "    if (in->obstacle_watch_required &&",
       "    if (true &&",
       "obstacle sensor faults raised while not moving")
mutant("test_safety", "safety.c",
       "        bits |= FAULT_LINE_OVERFLOW;",
       "        bits |= FAULT_LINE_LOST;",
       "an ADC overflow stops the car")

mutant("test_motion", "motion.c",
       "        m->cmd = *cmd;                  /* status.cmd_id now names the STOP */\n",
       "",
       "a stale STOPPED flag can be mistaken for the new STOP having taken effect")

# ---- vehicle controller -----------------------------------------------------
mutant("test_vehicle", "vehicle.c",
       "        if (stop_confirmed(vc, in)) {\n            enter_scan(vc, out, now);",
       "        if (true) {\n            enter_scan(vc, out, now);",
       "the obstacle scan starts before the car is confirmed stopped")
mutant("test_vehicle", "vehicle.c",
       "    return (in->motion.cmd_id == vc->stop_id) &&\n           ((in->motion.flags & MOTION_FLAG_STOPPED) != 0U);",
       "    return ((in->motion.cmd_id == vc->stop_id) || true) &&\n           ((in->motion.flags & MOTION_FLAG_STOPPED) != 0U);",
       "an old STOPPED flag counts as confirming a new STOP")
mutant("test_vehicle", "vehicle.c",
       "        !(vc->have_executed && (in->instruction.id == vc->last_executed_id))) {",
       "        true) {",
       "the same barcode instruction can execute twice")
mutant("test_vehicle", "vehicle.c",
       "            vc->pending_turn_deg = vc->cfg.turn_deg;\n        } else if (ins->command == NAV_CMD_RIGHT) {",
       "            vc->pending_turn_deg = (int16_t)(-vc->cfg.turn_deg);\n        } else if (ins->command == NAV_CMD_RIGHT) {",
       "a LEFT instruction turns right")
mutant("test_vehicle", "vehicle.c",
       "        out->consume_instruction = true;\n        out->consume_id = ins->id;",
       "        out->consume_id = ins->id;",
       "an executed instruction is never consumed")
mutant("test_vehicle", "vehicle.c",
       "    vc->realign_turn_deg = (vc->plan.step_count > 0U && last->is_turn)\n                           ? (int16_t)(-last->value) : 0;",
       "    vc->realign_turn_deg = (vc->plan.step_count > 0U && last->is_turn)\n                           ? (int16_t)(last->value) : 0;",
       "the car realigns the wrong way after recovering the line")
mutant("test_vehicle", "vehicle.c",
       "            } else if ((now - vc->phase_since_ms) >= vc->cfg.recovery_time_limit_ms) {\n                enter_fault(vc, out, now, FAULT_RECOVERY_LIMIT);",
       "            } else if (false) {\n                enter_fault(vc, out, now, FAULT_RECOVERY_LIMIT);",
       "the 5 s recovery limit is not enforced")
mutant("test_vehicle", "vehicle.c",
       "            } else if (abs32(in->motion.distance_mm - vc->start_distance_mm) >=\n                       vc->cfg.recovery_distance_limit_mm) {\n                enter_fault(vc, out, now, FAULT_RECOVERY_LIMIT);",
       "            } else if (false) {\n                enter_fault(vc, out, now, FAULT_RECOVERY_LIMIT);",
       "the 0.5 m recovery limit is not enforced")
mutant("test_vehicle", "vehicle.c",
       "            if (vc->reverse_attempts >= vc->cfg.max_reverse_attempts) {",
       "            if (false) {",
       "the car can reverse and rescan forever")
mutant("test_vehicle", "vehicle.c",
       "                        (in->plan.reason == PLAN_REASON_INVALID_SCAN) ? FAULT_ULTRASONIC\n                                                                     : FAULT_NO_SAFE_PATH);",
       "                        FAULT_NO_SAFE_PATH);",
       "an invalid scan is reported as 'no safe path'")
mutant("test_vehicle", "vehicle.c",
       "        if (fault_stop) {\n            enter_fault(vc, out, now, 0U);          /* someone else raised it */",
       "        if (fault_stop && false) {\n            enter_fault(vc, out, now, 0U);          /* someone else raised it */",
       "a stop-class fault from the safety monitor does not stop the car")
mutant("test_vehicle", "vehicle.c",
       "        if (start || clear_req) {\n            out->clear_faults = clear_req ? in->faults : (in->faults & FAULT_STOP_MASK);",
       "        if (true) {\n            out->clear_faults = clear_req ? in->faults : (in->faults & FAULT_STOP_MASK);",
       "a fault clears itself without the operator")
mutant("test_vehicle", "vehicle.c",
       "        if (running && !allow_remote_stop) {\n            return TEL_ACK_REJECTED_RUNNING;\n        }",
       "        if (running && !allow_remote_stop && false) {\n            return TEL_ACK_REJECTED_RUNNING;\n        }",
       "a remote STOP ignores the team's policy flag")
mutant("test_vehicle", "vehicle.c",
       "        if (running) {\n            return TEL_ACK_REJECTED_RUNNING;    /* never change the mission mid-run */\n        }",
       "",
       "a remote START is accepted while the mission runs")
mutant("test_vehicle", "vehicle.c",
       "        } else if (in->obstacle_present && ((int32_t)(now - vc->blank_until_ms) >= 0)) {",
       "        } else if (in->obstacle_present) {",
       "a cleared obstacle triggers an immediate second scan (no blanking)")
mutant("test_vehicle", "vehicle.c",
       "        if (in->line.stale) {\n            enter_fault(vc, out, now, FAULT_LINE_STALE);",
       "        if (false) {\n            enter_fault(vc, out, now, FAULT_LINE_STALE);",
       "the car keeps steering on stale line data")
mutant("test_vehicle", "vehicle.c",
       "            if (in->calibration_ok) {\n                vc->calibrated = true;",
       "            if (true) {\n                vc->calibrated = true;",
       "the car follows a line after a failed calibration")
mutant("test_vehicle", "vehicle.c",
       "    if (in->hump_in_progress && (speed > vc->cfg.hump_mm_s)) {",
       "    if (false) {",
       "no speed limit on a hump")
mutant("test_vehicle", "vehicle.c",
       "        speed = vc->cfg.curve_mm_s;             /* no position: slow and straight */\n        diff = 0;",
       "        speed = vc->cfg.cruise_mm_s;\n        diff = diff + 0;",
       "full speed with no line position")
mutant("test_vehicle", "vehicle.c",
       "        } else if ((now - vc->state_since_ms) > vc->cfg.stop_confirm_timeout_ms) {\n            enter_fault(vc, out, now, FAULT_MOTION);\n        } else {\n            /* the car has not stopped moving yet: no scanning */",
       "        } else if (false) {\n            enter_fault(vc, out, now, FAULT_MOTION);\n        } else {\n            /* the car has not stopped moving yet: no scanning */",
       "an unconfirmed stop waits forever")
mutant("test_vehicle", "vehicle.c",
       "    return vc->cmd_active && ((now - vc->cmd_started_ms) > vc->cmd_timeout_ms);",
       "    return false && vc->cmd_active && ((now - vc->cmd_started_ms) > vc->cmd_timeout_ms);",
       "a manoeuvre that never completes waits forever")
mutant("test_vehicle", "vehicle.c",
       "    vc->next_id++;\n    return vc->next_id;",
       "    return vc->next_id;",
       "motion commands reuse the same id")
mutant("test_vehicle", "vehicle.c",
       "                vc->cal_idx++;\n                if (vc->cal_idx < CAL_LEGS) {",
       "                vc->cal_idx = 2U;\n                if (vc->cal_idx < CAL_LEGS) {",
       "the calibration sweep is cut short")

mutant("test_line_nav", "line_nav.c",
       "    *instruction = nav->pending;\n    return true;\n}\n\nbool line_nav_take_instruction",
       "    *instruction = nav->pending;\n    ((line_nav_t *)nav)->pending_valid = false;\n    return true;\n}\n\nbool line_nav_take_instruction",
       "peeking at the instruction uses it up")

mutant("test_line_nav", "line_nav.c",
       "    mask = (uint8_t)(nav->valid_mask & nav->cfg.track_mask & (uint8_t)(~excluded));",
       "    mask = (uint8_t)(nav->valid_mask & (uint8_t)(~excluded));",
       "a far barcode sensor is used to steer")
mutant("test_line_nav", "line_nav.c",
       "            (((nav->cfg.lane_mask >> channel) & 1U) == 0U)) {\n            continue;",
       "            false) {\n            continue;",
       "a tracking sensor runs a barcode lane")
mutant("test_line_nav", "line_nav.c",
       "        bool anchor_black = (anchors == 0U);",
       "        bool anchor_black = true;",
       "a junction is reported with no line under the car")
mutant("test_line_sensor", "line_sensor.c",
       "            weighted += (int32_t)blackness[ch] * weight[ch];",
       "            weighted += (int32_t)blackness[ch] * ((weight[ch] * 0) + CHANNEL_POSITION[ch]);",
       "channel positions ignored (assumes a symmetric bar)")

mutant("test_ultrasonic", "ultrasonic.c",
       "    if (reading->distance_mm < urgent_mm) {",
       "    if (reading->distance_mm < (urgent_mm * 0U)) {",
       "a very close reading still waits for confirmation (misses the 150 ms budget)")

def skip_build_products(_directory, names):
    """Leave out compiled test binaries (no dot in the name) and caches."""
    return [n for n in names
            if (n.startswith("test_") and "." not in n)
            or n in ("sim_report", "__pycache__")]


def run(cmd, cwd):
    return subprocess.run(cmd, cwd=cwd, shell=True, capture_output=True, text=True)


def main():
    pattern = sys.argv[1] if len(sys.argv) > 1 else ""
    survivors = []
    ran = 0
    for test, source, old, new, why in MUTANTS:
        if pattern and pattern not in test:
            continue
        with tempfile.TemporaryDirectory() as tmp:
            shutil.copytree(os.path.join(ROOT, "app_program"),
                            os.path.join(tmp, "app_program"))
            shutil.copytree(os.path.join(ROOT, "tests", "host"),
                            os.path.join(tmp, "tests", "host"),
                            ignore=skip_build_products)
            path = os.path.join(tmp, "app_program", source)
            text = open(path).read()
            if text.count(old) != 1:
                print("BAD MUTANT (pattern occurs %d times): %s / %s"
                      % (text.count(old), source, why))
                survivors.append((test, why + " [pattern error]"))
                continue
            open(path, "w").write(text.replace(old, new))
            work = os.path.join(tmp, "tests", "host")
            build = run("make -B %s" % test, work)
            ran += 1
            if build.returncode != 0:
                # A mutant that does not compile proves nothing about the test.
                print("NOCOMPILE  %-18s %s" % (test, why))
                survivors.append((test, why + " [did not compile]"))
                continue
            result = run("./%s" % test, work)
            if result.returncode == 0:
                print("SURVIVED   %-18s %s" % (test, why))
                survivors.append((test, why))
            else:
                print("killed     %-18s %s" % (test, why))
    print("\n%d mutants, %d survivors" % (ran, len(survivors)))
    return 1 if survivors else 0


if __name__ == "__main__":
    sys.exit(main())
