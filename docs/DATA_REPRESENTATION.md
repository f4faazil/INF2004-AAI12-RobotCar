# Data representation (minimum-bit requirement)

Week 13 evidence item 1: every relevant application data field, its valid range, the bits it needs, and how it is represented.
The telemetry tables in section 2 are **generated from the firmware source** (`python3 tools/gen_data_table.py`), and each record's size is
checked against the bytes the unit tests use, so they cannot drift from the code. Sections 3 to 5 are written by hand from the
definitions in `app_program/` and say what was packed and what was deliberately not, with the reason.

## 1. Rules used
- Every field on the wire uses only the bits its range needs, with explicit masks and shifts (`bitw_put`, MSB first). **No C bit-fields on the wire**: their layout is compiler-dependent.
- Each record starts with an 8-bit type, so a receiver knows the layout. Unknown or short records are rejected (`tel_decode_*` returns false).
- A value that does not fit is **saturated and flagged** in the record's `saturated` field. An invalid command is rejected, never clipped into a valid one.
- Calculations stay full width (PID, IMU trigonometry, hump height sum, raw ADC and acceleration). Only stored, shared and transmitted results are packed (Tribunal 1, Version C).
- Control and safety events go straight to the Vehicle Controller; MQTT only carries a copy, so a lost network or a full buffer cannot delay control.

## 2. Telemetry and command records (generated)

Records are at most 18 bytes. The queue slot limit is 32 bytes (`TEL_MAX_MESSAGE`).

#### `status` record: 134 bits = 17 bytes

| Field | Bits | Representation | Range this width holds | Note from the source |
|---|---|---|---|---|
| type | 8 | unsigned, MSB first | 0 to 255 |  |
| ts10 | 16 | unsigned, MSB first | 0 to 65535 |  |
| state | 4 | unsigned, MSB first | 0 to 15 | vehicle state 0..15 |
| nav_cmd | 3 | unsigned, MSB first | 0 to 7 | none/left/right/straight/u-turn |
| line_state | 2 | unsigned, MSB first | 0 to 3 | tracking / lost / masked by hump / stale |
| junction_state | 2 | unsigned, MSB first | 0 to 3 | none / left / right / both |
| barcode_state | 2 | unsigned, MSB first | 0 to 3 | none / pending / rejected / expired |
| line_position | 11 | signed, MSB first | -1024 to 1023 | -1000..+1000 fits +/-1023 |
| left_mm_s | 10 | signed, MSB first | -512 to 511 | +/-511 mm/s (course max ~300) |
| right_mm_s | 10 | signed, MSB first | -512 to 511 | +/-511 mm/s (course max ~300) |
| distance_mm | 16 | signed, MSB first | -32768 to 32767 | +/-32767 mm: reverse and a whole run |
| faults | 16 | unsigned, MSB first | 0 to 65535 |  |
| obstacle_valid | 1 | unsigned, MSB first | 0 to 1 |  |
| obstacle_cm | 9 | unsigned, MSB first | 0 to 511 | 0..510 cm at 1 cm; 511 is reserved |
| hump_packed | 13 | unsigned, MSB first | 0 to 8191 | imu.h: 3 + 7 + 2 + 1 |
| highest_hump_mm | 7 | unsigned, MSB first | 0 to 127 | mm 0..126; 127 = none/out of range |
| saturated | 4 | unsigned, MSB first | 0 to 15 |  |
| (padding) | 2 | zero bits to the next byte | | |

#### `event` record: 53 bits = 7 bytes

| Field | Bits | Representation | Range this width holds | Note from the source |
|---|---|---|---|---|
| type | 8 | unsigned, MSB first | 0 to 255 |  |
| ts10 | 16 | unsigned, MSB first | 0 to 65535 |  |
| code | 5 | unsigned, MSB first | 0 to 31 |  |
| arg | 16 | unsigned, MSB first | 0 to 65535 |  |
| seq | 8 | unsigned, MSB first | 0 to 255 |  |
| (padding) | 3 | zero bits to the next byte | | |

#### `hump` record: 44 bits = 6 bytes

| Field | Bits | Representation | Range this width holds | Note from the source |
|---|---|---|---|---|
| type | 8 | unsigned, MSB first | 0 to 255 |  |
| ts10 | 16 | unsigned, MSB first | 0 to 65535 |  |
| hump_packed | 13 | unsigned, MSB first | 0 to 8191 | imu.h: 3 + 7 + 2 + 1 |
| highest_hump_mm | 7 | unsigned, MSB first | 0 to 127 | mm 0..126; 127 = none/out of range |
| (padding) | 4 | zero bits to the next byte | | |

#### `obstacle` record: 76 bits = 10 bytes

| Field | Bits | Representation | Range this width holds | Note from the source |
|---|---|---|---|---|
| type | 8 | unsigned, MSB first | 0 to 255 |  |
| ts10 | 16 | unsigned, MSB first | 0 to 65535 |  |
| valid | 1 | unsigned, MSB first | 0 to 1 |  |
| CM_MAX | 9 | unsigned, MSB first | 0 to 511 |  |
| CM_MAX | 9 | unsigned, MSB first | 0 to 511 |  |
| CENTRE_CM_LIMIT | 9 | signed, MSB first | -256 to 255 |  |
| CM_MAX | 9 | unsigned, MSB first | 0 to 511 |  |
| CM_MAX | 9 | unsigned, MSB first | 0 to 511 |  |
| action | 3 | unsigned, MSB first | 0 to 7 |  |
| reason | 3 | unsigned, MSB first | 0 to 7 |  |
| (padding) | 4 | zero bits to the next byte | | |

#### `heartbeat` record: 72 bits = 9 bytes

| Field | Bits | Representation | Range this width holds | Note from the source |
|---|---|---|---|---|
| type | 8 | unsigned, MSB first | 0 to 255 |  |
| uptime_s | 32 | unsigned, MSB first | 0 to 4294967295 |  |
| dropped | 16 | unsigned, MSB first | 0 to 65535 |  |
| reconnects | 8 | unsigned, MSB first | 0 to 255 |  |
| queue_depth | 5 | unsigned, MSB first | 0 to 31 |  |
| net_state | 3 | unsigned, MSB first | 0 to 7 |  |
| (padding) | 0 | zero bits to the next byte | | |

#### `sensors` record: 143 bits = 18 bytes

| Field | Bits | Representation | Range this width holds | Note from the source |
|---|---|---|---|---|
| type | 8 | unsigned, MSB first | 0 to 255 |  |
| ts10 | 16 | unsigned, MSB first | 0 to 65535 |  |
| ir_levels | 3 | unsigned, MSB first | 0 to 7 |  |
| ir_valid_mask | 3 | unsigned, MSB first | 0 to 7 |  |
| pair_on_line | 1 | unsigned, MSB first | 0 to 1 |  |
| junction_armed | 1 | unsigned, MSB first | 0 to 1 |  |
| bars_now | 4 | unsigned, MSB first | 0 to 15 |  |
| bars_done | 8 | unsigned, MSB first | 0 to 255 | saturating counter: clipped at 255, the record's saturated bit says so |
| decoded | 8 | unsigned, MSB first | 0 to 255 | saturating counter: clipped at 255, the record's saturated bit says so |
| rejected | 8 | unsigned, MSB first | 0 to 255 | saturating counter: clipped at 255, the record's saturated bit says so |
| junctions | 8 | unsigned, MSB first | 0 to 255 | saturating counter: clipped at 255, the record's saturated bit says so |
| executed | 8 | unsigned, MSB first | 0 to 255 | saturating counter: clipped at 255, the record's saturated bit says so |
| map_entries | 5 | unsigned, MSB first | 0 to 31 |  |
| last_letter | 5 | unsigned, MSB first | 0 to 31 |  |
| motion_state | 2 | unsigned, MSB first | 0 to 3 |  |
| motion_stopped | 1 | unsigned, MSB first | 0 to 1 |  |
| encoder_fault | 1 | unsigned, MSB first | 0 to 1 |  |
| enc_bad_left | 8 | unsigned, MSB first | 0 to 255 | saturating counter: clipped at 255, the record's saturated bit says so |
| enc_bad_right | 8 | unsigned, MSB first | 0 to 255 | saturating counter: clipped at 255, the record's saturated bit says so |
| roll_permille | 11 | signed, MSB first | -1024 to 1023 |  |
| imu_ok | 1 | unsigned, MSB first | 0 to 1 |  |
| lift_latched | 1 | unsigned, MSB first | 0 to 1 |  |
| hump_active | 1 | unsigned, MSB first | 0 to 1 |  |
| mm | 13 | unsigned, MSB first | 0 to 8191 |  |
| us_valid | 1 | unsigned, MSB first | 0 to 1 |  |
| us_obstacle | 1 | unsigned, MSB first | 0 to 1 |  |
| us_fault | 1 | unsigned, MSB first | 0 to 1 |  |
| servo_aim | 2 | unsigned, MSB first | 0 to 3 |  |
| net_state | 3 | unsigned, MSB first | 0 to 7 |  |
| link_up | 1 | unsigned, MSB first | 0 to 1 |  |
| saturated | 1 | unsigned, MSB first | 0 to 1 |  |
| (padding) | 1 | zero bits to the next byte | | |

#### `command` record: 36 bits = 5 bytes

| Field | Bits | Representation | Range this width holds | Note from the source |
|---|---|---|---|---|
| type | 8 | unsigned, MSB first | 0 to 255 |  |
| req_id | 8 | unsigned, MSB first | 0 to 255 |  |
| cmd | 4 | unsigned, MSB first | 0 to 15 |  |
| param | 16 | unsigned, MSB first | 0 to 65535 |  |
| (padding) | 4 | zero bits to the next byte | | |

#### `ack` record: 19 bits = 3 bytes

| Field | Bits | Representation | Range this width holds | Note from the source |
|---|---|---|---|---|
| type | 8 | unsigned, MSB first | 0 to 255 |  |
| req_id | 8 | unsigned, MSB first | 0 to 255 |  |
| result | 3 | unsigned, MSB first | 0 to 7 |  |
| (padding) | 5 | zero bits to the next byte | | |


## 3. Stored and shared data in RAM

"Stored as" is what the variable really occupies in memory. A standalone variable cannot be smaller than a byte, so a 4-bit value
is held in a `uint8_t`; the packed form is what travels on the wire. The last column says whether the field was packed and, if not, why.
Sizes are measured on the Cortex-M0+ (`arm-none-eabi-gcc`).

| Data | Where | Valid range | Bits needed | Stored as | Decision |
|---|---|---|---|---|---|
| Vehicle state | `g_vstate`, `vehicle_state_t` | 16 states (BOOT to DONE) | 4 | `uint8_t` | 4 bits on the wire; a byte in memory |
| Fault mask | `fault_mask_t.bits` | 14 faults defined (bits 0 to 9 and 12 to 15) | 14 | `uint16_t`, one bit per fault | **Packed.** Every set/clear runs in a critical section (spinlock plus interrupt disable on SMP); a read is one halfword. Stress-tested: `docs/evidence/fault_mask_stress.md` |
| Remote request bits | `g_remote_bits` | stop, start, calibrate, clear | 4 | `uint8_t` bit mask | **Packed**; the read-modify-write runs in a critical section |
| Hump event | `hump_packed` | status 3 + height 7 + validity 2 + IMU fault 1 | 13 | `uint16_t`, built with masks/shifts (`hump_event_pack`) | **Packed** everywhere it is stored or sent |
| Highest valid hump | `highest_valid_mm` | 0 to 126 mm, 127 = none | 7 | `int32_t` (internal) | 7 bits on the wire |
| Line junction / valid / lane masks | `line_nav_out_t` | 2, 3 and 2 bits | 2 to 3 | `uint8_t` masks | **Packed** as masks |
| Line position | `line_nav_out_t.position` | -1000 to +1000 | 11 signed | `int16_t` | 11 bits on the wire |
| Barcode letter | `nav_instruction_t.letter` | A to Z | 5 | `char` | 5 bits on the wire |
| Navigation command | `nav_command_t` | none / left / right / straight / u-turn | 3 | enum | 3 bits on the wire |
| Motion command id | `motion_cmd_t.id` | wraps; compared wrap-safe | 16 | `uint16_t` | Kept at 16 bits on purpose: stale and wrapped ids are rejected (`test_stale_and_wrapped_ids`) |
| Motion speed, diff | `motion_cmd_t` | about +-500 mm/s (course max about 300) | 10 signed | `int16_t` | natural; 10 bits on the wire |
| Motion distance, angle, lease | `motion_cmd_t` | bypass legs up to about 1.2 m (scan range 800 mm plus the 405 mm forward leg), +-180 degrees, lease default 50 ms | 12 signed, 9 signed, 11 (holds up to 2047 ms) | `int16_t`, `uint16_t` | natural; the command ring is 8 x 14 = 112 B (packing would save about 40 B) |
| Distance travelled | `motion_status_t.distance_mm` | a run reached 16 441 mm in the logs | 17 or more | `int32_t` | Kept at 32 bits: a 16-bit value overflows past 32.7 m. The wire saturates at +-32 767 mm and flags it |
| Encoder count | `encoder_t.count` | +-2^31 counts (about 18 km) | 32 | `int32_t` | The interrupt is the only writer. A consistent copy (count, timestamp, illegal count) is taken in a critical section (`encoder_snapshot_t`) |
| Encoder illegal transitions | `encoder_t.illegal` | diagnostic counter | 8 on the wire | `uint32_t` | wire: saturating 8-bit counter |
| IR frame ring | `g_ring`, 256 x (3 x `uint16_t` + `uint32_t`) = 3072 B | ADC 0 to 4095 (the digital sensors read 0 or 1) | 12 per channel | 12 B per frame | **Not packed, by team decision:** it is written by the 1 ms interrupt, and Tribunal 1 says to keep raw ADC frames unpacked and to avoid packing in the fastest loop |
| Telemetry queue slot (robot) | `tel_record_t` | at most 18 bytes | | 40 B per slot x 16 = 656 B | **Resized**: was 136 B per slot (2192 B). See `ram_comparison.md` |
| Telemetry inbox slot (radio) | `telemetry_queue_slot_t` | at most 18 bytes | | 40 B per slot x 16 = 668 B | **Resized**: was 2204 B |
| Queue slot timestamp | `ts_ms`, `created_ms` | age compared against 1 s | 10 would do | `uint32_t` | Kept at 32 bits on purpose: a 16-bit stamp wraps after 65 s, and an old record could look fresh again after a long outage |
| Sonar distance | `us_reading_t.distance_mm` | 0 to about 4000 mm | 12 | `uint16_t` | 9 bits on the wire (0 to 510 cm at 1 cm) |
| Servo aim | sensors record | none / left / right | 2 | `uint8_t` | 2 bits on the wire |
| Network state | `g_mqtt_state` | 5 states | 3 | `uint8_t` | 3 bits on the wire |
| Booleans in the module structs | 70 boolean fields declared in `vehicle.h` (24), `motion.h` (11), `line_nav.h` (17), `imu.h` (18) | 1 bit each | 1 | one byte each | **Not packed.** With one instance of each, packing them would save up to about 60 bytes. All the remaining candidates (flags about 60 B, command ring about 40 B, scan points about 32 B, letter map about 180 B) add up to roughly 310 B (estimate), about 2.7 % of the 11.4 KB of application RAM, against 3 072 B (21 %) saved by the queue change. The wire carries the flags as single bits |
| Scan points | `scan_point_t` x 32 | angle (7 positions), kind (4), distance 0 to 4000 mm | 3 + 2 + 12 | 4 B each | natural; about 32 B would be saved |
| Letter map | `nav_map_entry_t` x 26 | letter 5 bits, command 3 bits | 8 per entry | 8 B each | natural; about 180 B would be saved; the MQTT map payload is text (see section 6) |
| Diagnostic counters | `g_bar_*`, `g_junctions_*` and others | debug only | 8 on the wire | `uint32_t` | not application data on the wire; saturate at 255 when sent |

## 4. Local temporaries (not shrunk, one line each)
- PID error, derivative and wheel-speed arithmetic inside one 5 ms cycle: they are not persistent state, and a narrower type would add rounding and packing work to the fastest loop.
- IMU sine/cosine and roll calculations: full width, shrinking would cost accuracy in the 10 mm hump target.
- Hump height sum in micrometres and the peak roll: kept at 32 bits so rounding does not accumulate.
- Line sensor sums and blackness: 16 and 32 bits per 10 ms window, full width so the barcode widths stay exact.

## 5. How packed or shared data is accessed safely

| Shared item | Writers / readers | Mechanism |
|---|---|---|
| Fault mask | any task | all updates through `fault_set` / `fault_clear`, each in one critical section; no other code touches the word |
| Remote request bits | network task writes, vehicle task reads and clears | read-modify-write inside the critical section |
| Encoder counts | the GPIO interrupt writes; Motion reads | one writer; a consistent snapshot of count, timestamp and illegal count is taken in a critical section |
| IR frame ring | the ADC interrupt produces, the Line task consumes | single producer, single consumer; overflow is detected and counted, and voids a barcode in progress |
| Module share structs (`g_line_share`, `g_obstacle_share`, ...) | one task writes, others copy | copied under the critical section |
| Telemetry queue | any task pushes, the network task pops | push and pop inside the critical section; the inbox uses try-lock and never blocks |
| MQTT mailbox | the radio task and the network task | single-producer single-consumer ring with memory barriers; one slot for the reply |
| Hump packed word | the IMU task writes | one writer, one halfword |

The critical section on the SMP build is the kernel spinlock plus interrupt disable, so it also protects against the second core.
The concurrency stress test is in `docs/evidence/fault_mask_stress.md`.

## 6. Known gaps (stated plainly)
- **Stored data is mostly at natural width.** Only the fault mask, the request bits, the hump word and the line masks are packed in memory. The rest was left unpacked where the remaining saving is small (all candidates together are roughly 310 B, about 2.7 % of application RAM) or where the field is written by a fast interrupt. If the marker expects more, the flag groups in the module structs are the next candidates.
- **Some MQTT payloads are text,** not compact binary: the barcode map and its reply (`A=L,B=R,F=U`, `OK n=3`), and the radio diagnostic, heartbeat and online-marker topics. The robot's own records are all binary. The diagnostic topics are debug output; the map and reply are command payloads.
- **Two packing conventions exist.** The robot uses MSB-first (`telemetry.c`). The radio's demo frame (`telemetry_bits.c`, 3-byte hump demo) is LSB-first.
- The hump height is LOW CONFIDENCE in every log: confidence depends on the encoder distance over the bump, and the encoders report bad transitions.
