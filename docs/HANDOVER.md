# Handover checklist - robot firmware (branch `hadi`)

For whoever picks this up next (most likely the integration owner, then each buddy for their own subsystem). It lists what to **re-check**, what **only hardware can settle**, what the **team must decide**, and what is **not done**. The design and interfaces are in [ROBOT_FIRMWARE.md](ROBOT_FIRMWARE.md); this file is the to-do and the trust map.

> **The one-line status:** the portable logic is unit-tested on a PC and the firmware compiles cleanly; **nothing has been run on the car**. Anything below marked ☐ is unverified.

---

## Start here: the sensor layout decides whether this can work at all

Do this **before any tuning**, because every line-following, barcode and junction result depends on it. The track is an **18 mm wide line** with barcodes printed **29-49 mm from the line centre**. A sensor bar has to do two jobs: stay on a line only 18 mm wide, and reach a strip more than 29 mm to the side. A single symmetric bar of three equally spaced sensors cannot do both. In the closed-loop simulation (`tests/host/test_follow_sim.c`, 150 mm/s, S-curve, simulation only):

| Sensor layout | Tracks the line? | Reads a barcode? | Result |
|---|---|---|---|
| Symmetric, 12 mm apart | **Yes**, max error 3.4 mm | **No** - nothing reaches the strip | cannot do the barcodes |
| Symmetric, 39 mm apart (to reach the strip) | **No** - 9 line-loss events, error 29.6 mm | geometrically yes, but not confirmed: the car left the line | cannot follow the line |
| Symmetric, 20 mm apart | **No** - 16 line-loss events | - | the 18 mm line slips into the gap between two 2.1 mm spots |
| **Close pair (±6 mm) + one far sensor (39 mm)** | **Yes**, 3.7 mm at 100-200 mm/s | **Yes** at 100/150/200 mm/s, no false junction | the layout the code assumes |

**The rule:** the sensors that steer must be spaced *closer than the line is wide* (about 16 mm at most), and one sensor must sit *over the barcode strip*. That sensor also detects junction branches.

**What to do, in this order:**

1. ☐ **Measure where each IR sensor actually sits**, in millimetres from the car's centreline (right positive), and note which GPIO each is on (digital DO outputs, AO unconnected: left GP6, centre GP17, right GP3).
2. ☐ **Enter the three numbers** in `ROBOT_SENSOR_MM_LEFT / _CENTRE / _RIGHT` in `app_program/robot_config.h` (currently -6 / +6 / +39, a *proposal*, not the car's geometry). The roles are derived from them: a sensor within 25 mm of the centreline tracks the line, a sensor farther out reads barcodes and junction branches. Set `ROBOT_TRACK_REF_MM` to the distance from the centreline of the outermost *tracking* sensor (6 for the proposed pair at ±6 mm, 12 for a tight symmetric bar at ±12 mm); it scales the steering gain.
3. ☐ **Compare with the table.**
   - *Matches the last row:* carry on with the hardware checklist (§4).
   - *All three close together:* tracking will work but **barcodes can never be read** - a sensor must be moved or added over the strip.
   - *All three wide apart:* **no sensor can track the line** - the code will refuse to calibrate (`FAULT_LINE_CAL`) rather than follow badly - and two sensors must be moved close together.
4. ☐ **Re-check the simulation against the car:** run step 6 (line following) and step 7 (barcodes) of §4 and compare with the 3.7 mm / zero-false-junction figures above. They are simulation results; the car decides.

---

## 0. What you are receiving

- **Branch:** `hadi`, one commit on top of `2e8e6ad` (the lecturer's port). Not pushed. Not merged to `main`.
- **Size:** ~9,600 lines of firmware (`app_program/`), ~8,300 lines of tests and tools.
- **Default build target:** single-core, UART console (`make` in `build_make/`): 77.5 KB flash, 17.5 KB RAM.
- **Original demo:** still builds - set `ROBOT_FIRMWARE` to 0 in `app_program/robot_config.h`.

---

## 1. Confirm my claims first (15-20 minutes, no hardware)

Run these on your own machine before trusting anything. A different result means something differs between our setups, which is worth knowing before debugging hardware.

```sh
cd tests/host
make clean && make test            # expect: 17 programs, 1,954 checks, 0 failures
python3 mutation_check.py          # expect: 151 mutants, 0 survivors (takes several minutes)
./check_arm_warnings.sh            # expect: OK   (needs arm-none-eabi-gcc on PATH)
cd ../.. && python3 tools/test_robot_telemetry.py && python3 tools/test_robot_monitor.py   # expect: OK, OK
cd build_make && make clean && make -j8            # expect: text 77468 / data 4 / bss 17524
```

- ☐ All of the above match. (After moving the repo to a new path, always `make clean`: the build files hold absolute paths.)
- ☐ The mutation check is the evidence that the tests can fail. If you change a module, add a mutant for the rule you changed (see the top of `mutation_check.py` for the format) and confirm it is killed.
- ☐ Optional: also build the other profiles - `make SMP=1`, and the Wi-Fi + MQTT profile (needs the Pico SDK and two local config files, see ROBOT_FIRMWARE.md §4).

**What the tests do not cover** (so a green run says nothing about them): `hal_rp2040.c`, `robot_tasks.c`, `robot_main.c`, `robot_net.c`, `robot_config.c`, `lwip_utk_link.c`. They are compile-checked only.

---

## 2. Review the edits to the lecturer's files (integration owner)

Everything else is new files. These existing files were changed; please review each diff (`git diff 2e8e6ad -- <file>`) because they affect every buddy's build:

| File | Change | Why | Check |
|---|---|---|---|
| `app_program/app_main.c` | wrapped in `#if !ROBOT_FIRMWARE` (7 lines) | both it and `robot_main.c` define `usermain()` | ☐ builds with the switch at 0 and at 1 |
| `build_make/pico_rp2040.mk` | new `WIFI_MQTT` option (default 0), added to the image name and to the profile id | the MQTT link must be opt-in | ☐ default and plain `WIFI=cyw43` builds unchanged |
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` | one `#if TM_WIFI_MQTT` include and one `lwip_utk_link_poll()` call in the radio loop | the link has to run inside the radio owner task | ☐ the call is a no-op until an app task asks for a connection |
| `lib/libnet/lwip/include/lwipopts.h` | TCP is enabled when `TM_WIFI_MQTT` is set (as for `TM_WIFI_TCP`) | the raw-TCP client needs it | ☐ memory (`MEM_SIZE` 12 KB) acceptable |
| `.gitignore` | adds `config/mqtt_config.h` | broker address is local config, like the Wi-Fi credentials | - |

New files outside `app_program/`: `lib/libnet/lwip/lwip_utk_link.[ch]`, `config/mqtt_config.example.h`, `docs/`, `tests/host/`, `tools/robot_*.py`.

---

## 3. Decisions the team must make (they change the build)

| # | Decision | My default | Owner (suggested) |
|---|---|---|---|
| D1 | **IR sensor layout.** The simulation shows a symmetric 3-sensor bar cannot track the 18 mm line *and* read a barcode 29-49 mm away. Default assumes a close pair (±6 mm) tracking plus one far sensor (39 mm) reading barcodes and junction branches. Is the real bar that, and where exactly is each sensor? | `ROBOT_SENSOR_MM_*` = -6 / +6 / +39 | Hadi + Fazil (physical mounting) |
| D2 | **Barcode letter → command map.** Team decision: A left, F right, Z U-turn, executed only at a junction. Still open: which printed scale and which side of the line are on the track? | `ROBOT_BARCODE_MAP` in `robot_config.c` | Hadi, lecturer |
| D3 | **Remote STOP over MQTT.** Tribunal Assumption A1 says an emergency stop during the autonomous mission needs team and lecturer agreement. | allowed (`ROBOT_ALLOW_REMOTE_STOP` 1) | Aamir + team |
| D4 | **Who owns the Vehicle Controller and the Safety Monitor.** The Tribunal left both owners "TBD"; I wrote both (`vehicle.c`, `safety.c`). They need a named owner who understands the state machine. | - | team |
| D5 | **Final pin map, recorded in the repo** (Tribunal risk R3). Settled by the wiring diagram and now in `robot_config.h`: IR GP6/GP17/GP3 (DO), encoders GP28 (left) and GP1 (right), IMU GP26/GP27, TRIG GP4, ECHO GP5, encoders quadrature (left GP7/GP28, right GP0/GP1), motors M1 left GP8/GP9, M2 right GP10/GP11 (Buddy 2's note). | needs the USB console | Fazil, Danish, Suhail |
| D6 | **What happens at a junction with no instruction.** Default: cross it straight without turning. A T-junction has no straight way on, so the car would then search for the line and fault. | cross straight | Hadi + Fazil |
| D7 | **Servo mounting direction** (`left_is_high_angle`) and horn trim; the earlier bench test found the horn not aligned. | true, trim 0 | Danish |

---

## 4. Hardware verification - do in this order

Each step names the setting it determines and the pass criterion. Log the evidence (photo, serial capture, video) next to the step.

| ☐ | Step | Determines | Pass criterion |
|---|---|---|---|
| ☐ | **1. Flash and watch the status line** (1 line/s). | kernel + tick + IR ring work together | `ir=` rises ~1000/s; `do=` shows the raw IR DO levels (left, centre, right) and `edges=` counts their transitions; `ovr=0/0`; tick ISR (`us(...)` last figure) well under 100 µs |
| ☐ | **2. IR DO levels** at mounting height over white floor, the 18 mm line and a barcode, all three sensors; measure sensor lateral positions; check the DO high level is ≤ 3.3 V (a 5 V module needs a divider); set each module's trimmer so DO flips cleanly; confirm the polarity. | `ROBOT_SENSOR_MM_*` (D1), `ROBOT_IR_DARK_LEVEL` (all three IR sensors have fixed white/black readings and are trusted from boot: `vm=7`; set `ROBOT_IR_FIXED_MASK` to 0 to learn them from the sweep instead) | every sensor flips between floor and line; over a barcode the right sensor's DO toggles once per bar, all 29 elements, including the 3 mm ones |
| ☐ | **3. Motors**, wheels raised, low duty. | `ROBOT_MOTOR_*_INVERT` | forward command spins both wheels forward |
| ☐ | **4. Encoders:** signal level (3.3 V safe?), quadrature or single channel, counts per wheel revolution, wheel diameter, track width. | `counts_per_rev`, `wheel_diameter_mm`, `track_width_mm`, `ROBOT_ENCODER_SINGLE_CHANNEL` | rolling 1 m reads 1000 ±50 mm; direction sign correct; `illegal` count stays ~0 |
| ☐ | **5. Speed step response** for the PID. | `kp/ki/kd`, `ff_num/ff_den` | settles within ~0.5 s without oscillation at 100-200 mm/s |
| ☐ | **6. Line following**: straight, then curve. | `steer_kp_mm_s`, `steer_td_ms` | stays on the line, no `LINE_LOST`, at 0.10-0.20 m/s |
| ☐ | **7. Barcodes** at 0.10-0.20 m/s; log decoded / missed / rejected separately. | thresholds, layout (D1, D2) | ≥ 95/100 correct, **0 wrong**; each instruction executes once at the junction |
| ☐ | **8. Junction**: crossing line; measure sensor-bar-to-axle. | `junction_advance_mm`, `junction_min_mm` | turn lands on the junction centre; turns within ±10° in 9/10 |
| ☐ | **9. HC-SR04** (with the 1k/2k divider) at known distances 100-500 mm; servo pulse range, trim, direction. | `servo` config (D7), `ROBOT_OBSTACLE_CFG` | ±20 mm over 100-500 mm; servo reaches 30°-150° cleanly |
| ☐ | **10. Stopping distance** and the **end-to-end obstacle → PWM zero time**. | `ROBOT_BRAKING_MM`, `ROBOT_STOP_RESPONSE_MS` | PWM zero ≤ 150 ms from the obstacle entering range at top speed; car stops before contact (record the physical stop distance separately) |
| ☐ | **11. Obstacle bypass** (scan → plan → bypass → recovery). | robot size/margins, `coarse_step_deg` | clears the obstacle, finds the line within 5 s / 0.5 m |
| ☐ | **12. IMU**: identify the part (LSM303D at 0x1D per the bench test), axis orientation, validate the provisional lift/overturn stop, then known 20 mm and 40 mm humps at two speeds. | `ROBOT_IMU_FORWARD_AXIS/SIGN`, `ROBOT_LIFT_*`, hump thresholds | lift/overturn raises `FAULT_IMU_LIFT` and remains stopped; height error ≤ 10 mm in 9/10 valid runs; invalid cases flagged, not guessed |
| ☐ | **13. Wi-Fi + MQTT**: join, broker connect, telemetry on the laptop (`tools/robot_monitor.py`), forced disconnect, reconnect. | `lwip_utk_link.c` behaviour | reconnect ≤ 30 s in 9/10; **control timing unchanged** while disconnected and reconnecting |
| ☐ | **14. Timing under load**: Wi-Fi + USB console + all sensors at once; read `ovr=` and `us(...)` for several minutes. | task priorities and stack sizes | `ovr` stays 0; worst-case task times far below their periods |

---

## 5. Where I would look for bugs first (highest risk)

`hal_rp2040.c` was written from the RP2040 datasheet and Pico SDK conventions and has never executed. Cross-check each of these against the datasheet; they are the places a wrong constant would fail silently or wedge the board:

- ☐ **Timebase:** `WATCHDOG_TICK` write (0x4005802C, 12 cycles + enable) and that `TIMER` really counts 1 µs. Test: compare `hal_time_us()` against `tk_dly_tsk(1000)`.
- ☐ **1 kHz alarm:** alarm 0 fires on *equality*; the ISR guards against writing a target in the past. Check the tick really is 1 kHz (`ir=` counter) and does not stall.
- ☐ **IRQ numbers and levels:** TIMER_IRQ_0 = 0, IO_IRQ_BANK0 = 13, handler levels 1 and 2 (the kernel needs level ≥ 1 to allow service calls from the ISR).
- ☐ **IO_BANK0 offsets:** INTR0 0x0F0, PROC0_INTE0 0x100, PROC0_INTS0 0x120; 4 bits per pin, edge bits are write-1-to-clear. The encoder/ECHO interrupt depends on all of it.
- ☐ **Encoder ISR** reads both pins once per interrupt. If both channels toggle between two interrupts it records an *illegal* transition. Check the illegal count at maximum wheel speed.
- ☐ **IR sampling:** the tick reads `GPIO_IN` once and maps each DO level to a white or black count; confirm the frame order maps GP6 → left, GP17 → centre, GP3 → right, and that no other peripheral (ECHO interrupt, I2C) is still configured on those pins.
- ☐ **PWM:** 20 kHz motor frequency (TOP 6249 at 125 MHz), compare register A in the low half and B in the high half, servo 50 Hz with a 1 MHz count.
- ☐ **I2C1:** the init follows the SDK's formula; re-targeting the address disables and re-enables the controller; every wait is bounded. Check it recovers from an unplugged sensor.
- ☐ **Pads and muxing:** the template mux GP8/GP9 to I2C0 at boot (`USE_SDEV_DRV`); the robot re-muxes them to PWM. The template's sample ADC/I2C drivers **must stay closed**.
- ☐ **Stacks:** 2048 bytes per task (4096 for the diagnostic task) is a guess. Measure the high-water mark before trusting it.
- ☐ **Priorities** (Motion 1, Line 2, Vehicle/Obstacle 3, IMU 5, Net 10): the USB console task is 3 and the radio task 4, so control outranks both. Confirm with the `ovr`/`us(...)` figures under load (step 14).
- ☐ **SMP profile** builds and pins every task to processor 1, but is untested; the supported profile is single-core.

---

## 6. Known gaps and limits (not bugs, not done)

- **Packed-vs-unpacked evidence (Week 13) is not produced.** What exists: the packed sizes (status 17 B, event 7, hump 6, obstacle 10, heartbeat 9, command 5, ack 3) and the in-memory decode structs on the Cortex-M0+ (status 32 B, event 8, hump 6, obstacle 16, heartbeat 12). The fair comparison the Tribunal asks for - the same robot application built unpacked vs packed, with `arm-none-eabi-size`, `sizeof` and timing logs, commit IDs recorded - still needs a baseline build and measurement.
- **No mission-complete condition** is defined by the course; `VC_DONE` is reserved and unused.
- **Distance and hump height rely on encoder counts** and assume no wheel slip; the IMU has no gyro, so tilt is only valid when the car is not accelerating hard.
- **Lift/overturn detection is provisional** (at least 60° from calibrated rest or at most 300 mg for 300 ms). A single accelerometer cannot detect a level robot lifted evenly, and the lift thresholds need hardware validation before they can be treated as safety-certified.
- **The simulations are simulations.** Speed/sampling, line following, hump height and obstacle scans use idealised models (box-shaped sensor spot, ideal ranging, simple wheel lag). They show trends and that the logic is sound; they are not predictions of the car's numbers.
- **The coarse obstacle scan can miss a narrow off-centre object** (30° rays; default is 15°). Documented in `test_obstacle.c`.
- **The Wi-Fi link is built on the port's development radio profile**, which the lecturer's notes do not call qualified; it has only been compiled.
- **Static analysis beyond the compiler** (MISRA / Barr C tooling) was not run; style was followed by hand.

---

## 7. Traps

- Do not commit `config/wifi_credentials.h` or `config/mqtt_config.h` (both git-ignored; keep them that way).
- The stale-object guard in the makefile discards objects when the profile changes; if a link error mentions a symbol from a profile you are not building, `make clean`.
- `build_make/` may contain old `*.uf2` files from earlier profiles; flash the one named for the profile you just built (`mtk3pico_smp0_uart.uf2` for the default).
- `tests/host/mutation_check.py` works on temporary copies and never modifies the source tree; a mutant that "does not compile" proves nothing about the tests - fix the mutant.
- GP0/GP1 (right encoder) are the UART0 pins: build with `CONSOLE=usb_cdc`. GP28 (left encoder B) is the VBAT-sense pin: keep its jumper open.
- A `PLACEHOLDER` in `robot_config.c` is a starting value, not a result. Do not quote it in the Tribunal or the report as measured.

---

## 8. Evidence to collect as you go (Tribunal / Week 10 / Week 13)

- ☐ Status-line captures for the timing tests (`ovr`, task and tick-interrupt times) with and without Wi-Fi.
- ☐ Video + logs for the Line/Barcode + Motion test: decoded instruction, command id, execution, measured turn angle.
- ☐ Barcode results table: decoded / missed / rejected / wrong, per speed and scale.
- ☐ Obstacle test: timestamped log of obstacle entry, detection, STOP, PWM zero; separate physical stopping distance.
- ☐ Hump test: reference heights vs reported heights, valid and invalid trials counted separately.
- ☐ MQTT: forced-disconnect capture showing overflow counts, stale-record rejection and unchanged control timing; reconnect times.
- ☐ Final pin map and the data representation table (ROBOT_FIRMWARE.md §5) committed with the measured ranges replacing the proposed ones.
- ☐ Packed vs unpacked comparison (§6).

---

## 9. Map of the code

| Area | Files | Tests |
|---|---|---|
| Line, barcode (Buddy 3) | `barcode`, `line_sensor`, `line_nav` | `test_barcode`, `test_line_sensor`, `test_line_nav`, `test_pipeline`, `test_follow_sim` |
| Motion (Buddy 2) | `encoder`, `pid`, `motion` | `test_encoder`, `test_pid`, `test_motion` |
| IMU (Buddy 4) | `imu` | `test_imu` |
| Ultrasonic, servo, obstacle (Buddy 5) | `ultrasonic`, `servo`, `obstacle` | `test_ultrasonic`, `test_obstacle` |
| Telemetry, MQTT (Buddy 1) | `telemetry`, `mqtt_codec`, `mqtt_client`, `robot_net`, `lwip_utk_link` | `test_telemetry`, `test_mqtt_codec`, `test_mqtt_client`, `tools/test_*` |
| Coordination | `vehicle`, `safety`, `fault_mask` | `test_vehicle`, `test_safety`, `test_fault_mask` |
| Hardware and tasks (not unit-tested) | `hal_rp2040`, `robot_tasks`, `robot_main`, `robot_config` | compile + strict-warning check only |
