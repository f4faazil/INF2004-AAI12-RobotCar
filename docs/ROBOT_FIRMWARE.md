# Robot firmware - AAI12

The application for the Pico W + Robo Pico car, on top of the µT-Kernel 3.0 RP2040 port in this repository. It covers all five buddies' subsystems, the Vehicle Controller and the Safety Monitor.

> [!IMPORTANT]
> **Nothing here has run on the car yet.** The logic is unit-tested on a PC and the firmware builds clean, but every hardware-touching path (PWM, IR sampling, I2C, GPIO interrupts, the Wi-Fi link) and every tuning value is unverified until it is bench-tested. Section 7 lists exactly what to measure and in what order. Treat any number in this document marked *simulation* as evidence about the code, not about the car.

---

## 1. What is verified, and how

| Layer | Evidence | Where |
|---|---|---|
| Portable logic (16 modules) | **1,954 checks in 17 host test programs, all passing.** Every module's tests were also attacked with deliberate bugs (mutation check): **151 mutants, 0 survivors.** Mutants that turned out to be equivalent code are noted in `tests/host/mutation_check.py`. | `tests/host/`, run `make test` and `python3 mutation_check.py` |
| Wire formats vs the laptop | The C encoders match byte vectors produced by an independent Python implementation, and the Python decoder passes the same vectors (9 Python tests, incl. a fake-broker end-to-end run of the monitor). | `tools/robot_telemetry.py`, `tests/host/test_telemetry.c` |
| Closed-loop line following | Simulation of a car with a sensor bar on an S-curve, including a barcode strip. | `tests/host/test_follow_sim.c` (simulation only) |
| Firmware build | `arm-none-eabi-gcc` with `-Wall -Wextra -Wshadow`, and `-Wconversion -Wpedantic` on the 16 portable modules: **zero warnings**. Single-core: **77.5 KB flash, 17.5 KB RAM**. The SMP profile and the Wi-Fi + MQTT profile also build. | `build_make/` |
| Wi-Fi + MQTT link | **Compiles and links** against Pico SDK 2.2.0. Never connected to a broker. | `lib/libnet/lwip/lwip_utk_link.c` |
| On the car | **Nothing.** | - |

The code follows the Barr C standard as far as the µT-Kernel glue allows (4-space indent, braces everywhere, `stdint` types, named constants). The glue files (`hal_rp2040.c`, `robot_tasks.c`) use the kernel's own types where its API requires them.

---

## 2. Architecture

```
 IR x3 ──DO───┐                                         ┌── telemetry queue (16 x 128 B, drop-new)
              ▼                                         │            │
   1 kHz tick ISR ──ring(256)──▶ LINE task ──snapshot──▶ VEHICLE ──▶ NET task ──MQTT──▶ laptop
   (TIMER alarm0)               calibrate, barcode,       task        (lowest priority)
                                junction, steering          │  ▲
                                                            │  └── remote START/STOP (policy-checked)
 HC-SR04 ─ECHO irq─┐    ┌── servo (PWM slice 6)             ▼
                   ▼    ▼                               motion cmd (id, lease)
              OBSTACLE task ──scan/plan──▶ VEHICLE ──▶ MOTION task ──PWM──▶ motors (slices 4,5)
                                                            ▲
 encoders ─GPIO irq─────────────────────────────────────────┘ (snapshot)
 IMU ─I2C1─▶ IMU task ──hump status──▶ LINE (suppress false line-lost) and VEHICLE
                 └── sustained lift/overturn ──FAULT_IMU_LIFT──▶ Motion STOP + latched VC_STOPPED
```

| Pri | Task | Period | Owns | File |
|---|---|---|---|---|
| 1 | Motion | 5 ms | the only writer of motor PWM; encoder snapshots; speed PID | `motion.c` |
| 2 | Line | 10 ms | IR ring, calibration, barcode lanes, junctions, steering request | `line_nav.c`, `line_sensor.c`, `barcode.c` |
| 3 | Vehicle | 10 ms | mission state; the only sender of movement commands; safety evaluation | `vehicle.c`, `safety.c` |
| 3 | Obstacle | 10 ms | servo, HC-SR04, scan, profile, bypass plan | `ultrasonic.c`, `servo.c`, `obstacle.c` |
| 5 | IMU | 20 ms | I2C1, tilt, lift/overturn and hump height | `imu.c` |
| 10 | Net | 20 ms | MQTT client, telemetry publishing, remote commands | `mqtt_client.c`, `telemetry.c` |
| 11 | Diag | 1 s | one console status line | `robot_tasks.c` |

The kernel's own services outrank nothing that matters here: the USB console task runs at priority 3 and the CYW43439 radio task at 4 (polling every 2 ms), so the control path (1-3) sits above both. Whether that is enough is a measurement, not a given - see the handover checklist.

The 1 kHz tick is a hardware-timer interrupt, **not** the kernel tick (10 ms): the kernel's timed waits last "timeout + one tick", so 1 ms IR frames and a 5 ms control period cannot come from `tk_dly_tsk`. The interrupt samples the three IR channels into a 256-frame ring and releases the Motion (5 ms), Line/Vehicle (10 ms) and IMU (20 ms) tasks with event-flag bits, counting any tick whose task had not yet handled the previous one.

**Safety structure (Tribunal Version A/B/C):**

- Motion is the only code that writes motor PWM. A command has an **id** and a **50 ms lease**; if the Vehicle Controller stops refreshing it, the wheels stop by themselves. A refresh of the same id never restarts a manoeuvre; an expired, stopped or finished id can never be resumed; commands older than the newest accepted id are rejected (wrap-aware).
- A stop-class fault zeroes the PWM **from inside the Motion task**, without waiting for the Vehicle task.
- The obstacle scan starts only after Motion confirms the car has stopped *for the STOP that was sent* (a stale "stopped" flag from an earlier command does not count).
- An IMU read failure remains informational and degrades to "hump N/A". A sustained
  tilt of at least 60 degrees or acceleration magnitude at most 300 mg for
  300 ms raises stop-class `FAULT_IMU_LIFT`, sends STOP, reports a fault event,
  and latches `VC_STOPPED` until reboot/reset.
  These limits are provisional. A single accelerometer cannot detect a level,
  stationary robot lifted evenly; that needs a wheel-contact sensor or another input.
- Network loss never reaches the control tasks: telemetry is a bounded, non-blocking queue that drops new records when full and discards anything older than 1 s.

---

## 3. Hardware map

Robo Pico GPIO numbers (Tribunal Version B/C map):

| Function | Pins | Notes |
|---|---|---|
| Motors | GP8/GP9 (M1, left), GP10/GP11 (M2, right) | PWM slices 4 and 5, 20 kHz. A = forward, B = backward. Swap with `ROBOT_MOTOR_*_INVERT`. |
| Scan servo | GP12 | PWM slice 6, 50 Hz, parked at 90°. Trim/invert in `ROBOT_OBSTACLE_CFG.servo`. |
| Ultrasonic | TRIG GP4, ECHO GP5 | ECHO is 5 V: needs the 1 kΩ + 2 kΩ divider. Timed by interrupt capture, not busy-waiting. |
| IR sensors | digital DO (AO unconnected): left GP6, centre GP17, right GP3 | Read as plain HIGH/LOW; `ROBOT_IR_DARK_LEVEL` is the level over black. The DO high level must not exceed 3.3 V. |
| IMU | SDA GP26, SCL GP27 (I2C1) | LSM303D at 0x1D, or LSM303DLHC accelerometer at 0x19; probed at start-up. |
| Encoders | left A GP7 / B GP28, right A GP0 / B GP1 (quadrature) | Left counts negative going forward, flipped by `ROBOT_ENC_L_INVERT`. See conflicts 1 and 2 below. |
| Buttons | START GP20, STOP GP21 | Active low, 30 ms debounce. |

**Template pin conflicts (Tribunal risk R3) and how they are handled:**

1. **GP0/GP1 are the UART0 pins** and now carry the right encoder, so the robot needs the **USB console** (`make CONSOLE=usb_cdc PICO_SDK_PATH=...`). With the default UART console the kernel owns GP0/GP1 and the console would be dead after `hal_init()` takes the pins.
2. **GP28 is also the VBAT-sense pin** (left encoder B): the back-of-board jumper must be **open**.
3. **GP8/GP9 are muxed to I2C0 by the template's sample drivers** (`USE_SDEV_DRV`). The robot re-muxes them to PWM in `hal_init()`; the sample ADC/I2C drivers must stay closed because the HAL drives those blocks directly.

---

## 4. Build, flash, monitor

```sh
cd build_make
make -j8                      # single-core, UART console: the default
# -> mtk3pico_smp0_uart.uf2    hold BOOTSEL on the Pico W, copy the .uf2 to the drive
```

Console: USB CDC (build with `CONSOLE=usb_cdc`), because UART0's GP0/GP1 are the right encoder; 115200 8N1. One status line per second (the values below are only an illustration of the format):

```
[robot] st=3 f=0000 pos=2 v=150/149 d=412mm enc=1940/1931 bad=0/0 ir=8123 do=110 edges=12/0/30 vm=7 bc=A bar=1/0 jn=1/1 ex=1 ovf=0 drop=0 net=0 us(m/l/v/isr)=410/880/260/38 ovr=0/0/0
```

(`st` = vehicle state, `f` = fault mask, `enc` = raw encoder counts and `bad` = illegal (missed-edge) transitions per wheel, `us(...)` = worst-case microseconds of the Motion, Line and Vehicle tasks and the tick interrupt, `do` = raw IR DO levels (left, centre, right; 1 = pin high), `edges` = DO transitions per sensor, `vm` = sensors calibration trusts (bit 0 left, 1 centre, 2 right; 0 before the first calibration), `bc` = stored barcode letter or `-`, `bar` = barcodes stored / rejected, `jn` = junctions reported by the Line task / seen by the Vehicle Controller, `ex` = barcode commands executed at a junction, `ovr` = ticks that arrived (Motion / Line / Vehicle) before the previous one was handled. This is the loop-timing evidence the Tribunal asks for.)

To go back to the lecturer's original demo, set `ROBOT_FIRMWARE` to 0 in `app_program/robot_config.h`.

**With Wi-Fi and MQTT** (needs the Pico SDK checkout, see `docs/GETTING_STARTED.md`):

```sh
cp config/wifi_credentials.example.h config/wifi_credentials.h   # edit, set the flag to 1
cp config/mqtt_config.example.h      config/mqtt_config.h        # broker IPv4 address, set the flag to 1
make SMP=0 WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_MQTT=1 \
     PICO_SDK_PATH=$HOME/pico/pico-sdk -j8
```

Both config files are git-ignored. Run a broker (for example `mosquitto`) on the laptop, then:

```sh
python3 tools/robot_monitor.py --broker <laptop-ip>               # live, decoded
python3 tools/robot_monitor.py --broker <laptop-ip> --send stop   # a permitted command
```

Topics (base `aai12/robot`): `telemetry` (status, hump, obstacle, heartbeat), `event`, `ack`, `status` ("online"/"offline", retained, also the MQTT will), and `cmd` (commands in). Reconnection backs off 1, 2, 4, 8, 8… s (bounded) and restarts at 1 s after a success; the simulation of a broker outage reconnects within one capped backoff.

Run the tests:

```sh
cd tests/host && make test && python3 mutation_check.py     # C logic and the mutation check
python3 tools/test_robot_telemetry.py && python3 tools/test_robot_monitor.py
make sim                                                    # barcode decode-rate table (simulation)
```

---

## 5. Interfaces and data

### Motion command (Vehicle Controller → Motion)

`motion_cmd_t { id, type, speed_mm_s, diff_mm_s, distance_mm, angle_deg, lease_ms }` with `type` = `STOP`, `DRIVE` (speed + differential, continuous), `MOVE` (signed distance, straight) or `TURN` (in place, + = left). `motion_status_t` reports the command id, state (`IDLE/RUNNING/DONE/EXPIRED/OVERRIDDEN`), `STOPPED` (both wheels confirmed still), measured wheel speeds, signed distance travelled and the encoder-fault flag.

### Barcode instructions (Line → Vehicle)

A decoded Code 39 letter is mapped to a command by the mapping the PC sent over MQTT (see *MQTT mapping* below) and **stored**; it is executed once at the next junction (advance to put the wheels over it, turn, reacquire the line). It expires after 20 s, a newer barcode replaces it, and an IR-ring overflow voids a barcode in progress. A letter that is not in the mapping (or any letter, before a mapping has arrived) is rejected and reported, never guessed.

### MQTT mapping

The car has **no built-in letter-to-movement mapping**. The PC publishes it to `aai12/robot/map` (`ROBOT_MQTT_BASE "/map"`); the car subscribes to that topic after connecting, so a **retained** message is applied again after every reboot or reconnect. Each message replaces the whole mapping; a message with any error is rejected whole and the old mapping is kept. The car answers on `aai12/robot/map_ack` with `OK n=<entries>` or `ERR empty|syntax|letter|action|duplicate`.

Payload (text, at most 128 bytes, case-insensitive; entries separated by comma, semicolon, space, tab or newline; no spaces around `=`):

| Form | Example |
|---|---|
| `letter=action` pairs (`=` or `:`) | `A=L,B=R,C=F,D=U,E=L,...` |
| 26 action letters, A to Z in order | `LRFULRFULRFULRFULRFULRFULU` |
| remove every mapping | `CLEAR` |

Actions: `L`/`LEFT`, `R`/`RIGHT`, `F`/`FWD`/`FORWARD`/`S`/`STRAIGHT` (go straight on), `U`/`UTURN`/`U-TURN` (U-turn to the right).

Example from a PC with mosquitto: `mosquitto_pub -h <broker> -t aai12/robot/map -r -m "A=L,B=R,C=F,D=U"`. The status line shows `map=<entries>/<rejected messages>`.

### Telemetry: the data representation table (minimum-bit change)

Messages are bit-packed MSB first with a 1-byte type header; **no C bit-fields on the wire**. A value that does not fit is **saturated and flagged** in the message's `saturated` field. Internal calculations (PID, IMU height sum, raw ADC) stay full width.

| Field | Bits | Range / unit |
|---|---|---|
| message type | 8 | status 1, event 2, hump 3, obstacle 4, heartbeat 5, command 0x10, ack 0x11 |
| timestamp | 16 | 10 ms units (wraps at 655 s; records older than 1 s are never sent) |
| vehicle state | 4 | 0-15 (`vehicle_state_t`) |
| pending nav command | 3 | none/left/right/straight/u-turn |
| line state | 2 | tracking / lost / masked by hump / stale |
| junction state | 2 | none / left / right / both |
| barcode state | 2 | none / pending / rejected / expired |
| line position | 11 signed | ±1023 (saturating) |
| wheel speed (each) | 10 signed | ±511 mm/s |
| distance | 16 signed | ±32 767 mm |
| fault mask | 16 | one bit per fault (`fault_mask.h`) |
| obstacle distance | 1 + 9 | valid bit + 0-510 cm at 1 cm |
| hump event | 13 | status 3 + height 7 (0-126 mm, 127 = out of range) + validity 2 + IMU-fault 1 |
| highest hump | 7 | mm, 127 = none |
| saturated flags | 4 | position, speed, distance, obstacle |

Encoded sizes: status **17 bytes**, event 7, hump 6, obstacle 10, heartbeat 9, command 5, ack 3 (against a 128-byte record limit). Report RAM before/after packing with `arm-none-eabi-size` and these `sizeof`s for the Week 13 comparison.

### Remote commands

START, CALIBRATE and CLEAR_FAULTS are refused while the mission runs; no remote command drives the wheels. **A remote STOP is accepted by default** (`ROBOT_ALLOW_REMOTE_STOP`): the Tribunal says that needs team and lecturer agreement, so decide and set the flag. Every accepted request gets an ack when received and another when the controller has handled it.

---

## 6. Findings that need a team decision

These come from the simulations; they change what should be built, not just how.

**1. The IR sensor layout cannot be the symmetric three-sensor bar.** The line is 18 mm wide and a barcode sits 29-49 mm from the line centre. In the closed-loop simulation:

| Layout | Result |
|---|---|
| symmetric, 12 mm apart | follows the S-curve, max error 3.4 mm - but **cannot reach a barcode** |
| symmetric, 39 mm apart (to reach the barcode) | **loses the line** (9 loss events), max error 29.6 mm |
| symmetric, 20 mm apart | **loses the line** (16 loss events): the 18 mm line fits in the gap between two 2.1 mm spots |
| **close pair at ±6 mm + one far sensor at 39 mm** | tracks to 3.7 mm at 100-200 mm/s, recovers from a 10 mm start offset, reads the barcode at 100/150/200 mm/s with **no false junction and no line loss** |

So a sensor pair must be spaced closer than the line is wide (about 16 mm), and a third sensor must sit over the barcode strip. The default configuration assumes the last layout; set the real positions in `robot_config.h` (`ROBOT_SENSOR_MM_*`) - roles (tracking, barcode lane, junction branch) are derived from them. If the real bar is the symmetric one, the code will report calibration failure rather than follow badly.

**2. Barcode-versus-junction is decided by length, so it needs speed.** The widest barcode bar is about 9 mm and the crossing line 18 mm; a black run on a branch sensor longer than 13 mm while the line is under the car is a junction. This works at 0.10-0.20 m/s in simulation and needs the measured speed (no speed, no junction event).

**3. The coarse obstacle scan can miss a narrow object.** With 30° coarse rays a 100 mm box 300 mm ahead and 60 mm off-centre falls between rays (documented in `test_obstacle.c`). The default is a 15° coarse step (about 3 s of scanning).

**4. Steering needs derivative damping.** The first controller (derivative per 10 ms step) was effectively undamped and lost the line even with close sensors; the simulation drove the change to a derivative *time*. Defaults (kp 13 mm/s per mm of reference spacing, Td 120 ms) are simulation-derived starting points.

**5. Obstacle-to-stop timing.** With a 70 ms HC-SR04 period the normal path (median of 3, two confirming readings) can take 140-210 ms. A single valid reading closer than half the stop distance now declares the obstacle at once (worst case roughly 70 + 10 + 5 ms plus braking), but **no end-to-end time has been measured**.

**6. The letter-to-command mapping is not specified by the course.** It is sent from the PC over MQTT and is not built into the firmware (see *MQTT mapping*). A turn runs only at a junction, and a junction with no stored instruction goes straight.

---

## 7. Placeholders to measure, and the bench-test order

Everything in `robot_config.c` marked `PLACEHOLDER`, in the order that unblocks the most:

| # | Measure / verify | Why it matters |
|---|---|---|
| 1 | **Flash and read the status line.** `ir=` should count up at ~1000/s and `ovr=0/0`; the tick interrupt (the last `us(...)` figure) should stay well under 100 µs. These are expectations, not measurements. | proves the kernel, the 1 kHz tick and the IR ring work together |
| 2 | **IR DO levels** at mounting height over white floor, the line and a barcode (all 29 elements must toggle the right sensor's DO), the DO high level (≤ 3.3 V), the trimmer settings, and the sensor lateral positions. | sets `ROBOT_SENSOR_MM_*` and `ROBOT_IR_DARK_LEVEL` |
| 3 | **Motors**: run `hal_motor_set` at low duty with wheels raised; fix direction with `ROBOT_MOTOR_*_INVERT`. | the Motion task is the only thing that may do this later |
| 4 | **Encoders**: signal levels (3.3 V safe?), counts per wheel revolution, wheel diameter, track width. | `counts_per_rev`, `wheel_diameter_mm`, `track_width_mm`; all distances and turns scale with them |
| 5 | **Speed-step response** for the PID gains and the feed-forward `ff_num/ff_den`. | the placeholder gains are only a start |
| 6 | **Line following** on straight, then S-curve: tune `steer_kp_mm_s`, `steer_td_ms`. | |
| 7 | **Barcode decode** at 0.10-0.20 m/s; log decoded, missed and rejected separately. Target: 95/100 with zero wrong. | the simulation says the decoder is sound; the hardware decides |
| 8 | **HC-SR04** with the divider, known distances 100-500 mm (target ±20 mm). **Servo**: pulse range, trim, direction (`left_is_high_angle`). | |
| 9 | **Stopping distance** and the full obstacle-to-PWM-zero time (target 150 ms). | `ROBOT_BRAKING_MM`, `ROBOT_STOP_RESPONSE_MS` |
| 10 | **IMU**: identify the part (LSM303D at 0x1D per the earlier bench test), axis orientation, then known 20 mm and 40 mm humps at two speeds. | `ROBOT_IMU_FORWARD_AXIS/SIGN` |
| 11 | **Wi-Fi + MQTT**: join, broker connect, forced disconnect, reconnect time (target 30 s in 9/10), control timing unchanged. | the link has only ever been compiled |
| 12 | **Calibrated junction advance** (`junction_advance_mm`) and turn accuracy (target ±10° in 9/10). | |

---

## 8. Tribunal traceability

| Tribunal requirement | Implemented in | Tested |
|---|---|---|
| Motion is the only motor writer; command id, expiry, STOP precedence, no restart after stop | `motion.c`, `robot_tasks.c` | `test_motion.c` (lease, stale id, wrap, override, restart) |
| 5 ms Motion period, 50 ms lease | `robot_config.c`, `hal_rp2040.c` | logic only; timing unmeasured |
| Stopped confirmation before scanning | `motion.c` (`STOPPED` for the STOP id), `vehicle.c` | `test_vehicle.c` (incl. a lagging status report) |
| 1 ms three-channel frames into a 256-frame ring; overflow voids a barcode; stale data → fault + stop | `hal_rp2040.c`, `line_nav.c`, `safety.c` | `test_line_nav.c`, `test_safety.c` |
| Barcode vs junction discrimination; instruction stored, executed at the junction, once | `line_nav.c`, `vehicle.c` | `test_line_nav.c`, `test_vehicle.c`, `test_follow_sim.c` |
| Hump suppresses false line-lost only while valid, fresh and ≤ 1 s | `line_nav.c` | `test_line_nav.c` |
| Hump height = max accumulated elevation, invalid/unavailable never overwrites the highest valid | `imu.c` | `test_imu.c` (20/40 mm humps, 100-200 mm/s, noise: simulation) |
| Obstacle: stop → confirm → coarse+fine scan → profile → bypass → line recovery ≤ 5 s / 0.5 m | `obstacle.c`, `vehicle.c` | `test_obstacle.c`, `test_vehicle.c` |
| No echo ≠ free space; invalid scan or no route keeps the car stopped with a fault | `ultrasonic.c`, `obstacle.c`, `vehicle.c` | `test_ultrasonic.c`, `test_obstacle.c`, `test_vehicle.c` |
| Shared fault mask: one bit per fault, protected helper, latched until authorised clearing | `fault_mask.c` | `test_fault_mask.c` |
| Minimum-bit data, explicit masks/shifts, 1-byte type header, saturate-and-flag | `telemetry.c`, `imu.c` | `test_telemetry.c` + Python cross-check |
| Bounded telemetry buffer 16 × 128 B, drop-new, 1 s freshness, non-blocking | `telemetry.c` | `test_telemetry.c` |
| MQTT: separate topics, bounded backoff, heartbeat, remote actuation refused during RUN | `mqtt_client.c`, `vehicle.c` | `test_mqtt_client.c`, `test_vehicle.c` |
| Encoder snapshot: count + timestamp + validity together, protected | `encoder.c` | `test_encoder.c` |

---

## 9. Not done / known limits

- **No hardware validation of any kind** (section 7). The register-level code in `hal_rp2040.c` follows the RP2040 datasheet but is the highest-risk file.
- **Single-core is the supported profile.** The SMP profile builds and pins all robot tasks to processor 1 (RP2040 core 0, which owns the peripheral interrupts), but is untested; hardware barriers would be needed if tasks were split across cores.
- **The Wi-Fi link** uses the radio profile, which the port documents as a development profile, not the qualified release.
- **No mission-complete condition** exists in the course material, so `VC_DONE` is reserved and unused.
- **Distance travelled is encoder-based** and assumes no wheel slip; hump height inherits that and the ±10 mm target is untested on real humps.
- **The remote-STOP policy and the letter map are placeholders** for team decisions.
