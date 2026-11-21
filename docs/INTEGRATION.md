# Integration notes: the final robot firmware

This is what the single `release/final-car` branch is made of, what had to change when the team's branches were put
together, and what is known to be unfinished. The design of the robot application itself is in
[ROBOT_FIRMWARE.md](ROBOT_FIRMWARE.md); [HANDOVER.md](HANDOVER.md) is the older to-do list written before anything ran on the car.

## Who contributed what (from the git history)

| Part | Branch merged | Author(s) |
|---|---|---|
| Robot application: five subsystems, Vehicle Controller, Safety Monitor, first MQTT client | `hadi` | Hadi |
| IR sensors, barcode (distance-based widths, 15-bar junction arming), runtime letter map over MQTT | `feature/hadi-irsensor` | Hadi |
| IMU (LSM303D), latched lift stop, I2C timeout | `imu-suhail` | Suhail |
| Motor drive model and calibration | `feature/motor` | Hadi, Fazil |
| IMU roll-based hump detector (fitted lever), IMU bring-up apps | `feature/imu` | Fazil |
| Obstacle scan, profile, bypass planner, servo calibration, bypass on the black-line track | `feature/obstacles` | Fazil |
| WiFi/DHCP recovery, MQTT session and lwIP adapter, diagnostics, telemetry demo and dashboard base | `feature/aamir-robot-integration`, `feature/aamir-wifi-mqtt-telemetry` | Aamir |
| Buddy 5 standalone obstacle module (`b5_*`, `APP_BUDDY5` test modes) and its docs | `feature/danish-obstacles` | Danish |
| Integration, telemetry record for every sensor, dashboard, MQTT receive path, fixes below | `integration/telemetry` | Fazil |

Every teammate's commits keep their own authorship in this branch.

## What changed when the branches were put together

**Two MQTT stacks, one kept.** Hadi's `mqtt_client` (inside the robot's Net task) and Aamir's `mqtt_lwip` session (inside the
radio task) both wanted the socket. The radio build now uses Aamir's: the session lives in the radio owner task and the robot
hands it telemetry records through `telemetry_robot_submit()` (non-blocking, so a slow network never stalls a control task).
`lwip_utk_link` is no longer polled from `cyw43_utk.c`. Hadi's client is still used by builds without the radio owner.

**Receiving over MQTT (new).** The session now sends SUBSCRIBE and accepts SUBACK and incoming PUBLISH (QoS 0 only). The car
subscribes to `aai12/robot/map` and `aai12/robot/cmd`. Messages cross from the radio task to the robot through a small
lock-free mailbox (`mqtt_mailbox.c`); the answer to a map goes back on `aai12/robot/map_ack`. `cmd` carries START and STOP and
goes through the same policy as the button: refused while the car is picked up, START refused while running. Commands are
never retained, so one is never replayed when the car reconnects.

**Barcode letter map.** The car has no built-in table. The dashboard sends one (`A=L,B=R,F=U`), retained, and the car says
`OK n=<count>` or `ERR ...`. Until a map arrives every decoded barcode is counted as rejected.

**Telemetry.** Hadi's bit-packed records plus a new per-sensor record (type 0x06, 18 bytes: IR levels, barcode progress, motors
and encoders, IMU tilt and lift, ultrasonic and servo, WiFi/MQTT state). Decoders: `tools/robot_telemetry.py` and
`dashboard/server/robot-frames.js`, both checked against bytes produced by the C encoder.

**Dashboard.** Aamir's page is now `/network`; `/` is the robot page with every sensor, a barcode-instructions card,
START / STOP (START needs an arming tick), and a built-in MQTT broker with retained messages so no Mosquitto is needed.
`npm run map -- "A=L,B=R,F=U"` sends a map from a terminal. See `dashboard/README.md`.

**Bugs found only once everything ran together on the radio build:**
- The init task's 1 KB stack overflowed (hard fault, USB would not enumerate): `INITTASK_STKSZ` is now 4 KB.
- The radio's 2576-byte status block was copied onto a 2 KB task stack and stalled the car after START: it is now read in the Net
  task only (`robot_net.c`).
- The GPIO edge interrupt (encoders and sonar echo) was switched on from the init task, which can run on the other core, so it
  never fired: it is now switched on from a task pinned to processor 1.
- `time_us_64()` added to the HAL for builds without the radio.

**Lift stop.** Suhail's latched stop is kept. It now clears by itself once the car is back on the floor (level to about 15
degrees, 1 g within 120 mg, steady for 2 s, at least 1 s after the lift). That only permits START; the car never starts by
itself. The test that asserted "latched until reboot" was updated.

**Obstacle bypass (Buddy 5).** Servo mapping measured on the car (1818 us straight, 10.9 us per degree); scan 54 to 126
degrees in 12 degree steps; stop at 330 mm of sonar reading; forward leg from the car's geometry; the servo looks at the box
on the side leg and the leg is cut short when the box ends; a sonar guard (100 mm) stops every straight leg; a crossing line is
ignored on the way past; recovery accepts the line only after the expected return distance. Danish's `b5_*` module is merged as an
opt-in build (`BUDDY5=1..4`); the default build is unchanged (checked: the robot firmware is byte-identical with and without it).

**Conflicts resolved:** `build_make/pico_rp2040.mk`, `lwipopts.h`, `cyw43_utk.c` (Aamir's side), and the add/add conflicts with
Danish's branch in `app_main.c`, `config.h`, `hw_setting.c`, `pico_rp2040.mk` (both sides kept).

## Build

```sh
cd build_make
# robot, no WiFi
mingw32-make -j8 PICO_SDK_PATH=<sdk> SMP=1 CONSOLE=usb_cdc
# robot with WiFi + MQTT (needs the two local config files below)
mingw32-make -j8 PICO_SDK_PATH=<sdk> SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_MQTT=1
# Danish's Buddy 5 bench modes
mingw32-make -j8 PICO_SDK_PATH=<sdk> SMP=1 CONSOLE=usb_cdc BUDDY5=1
```

Two files are **local only** and git-ignored; copy them from the `.example.h` next to them and fill in your own network:
`config/wifi_credentials.h` (SSID and password) and `config/mqtt_test_config.h` (broker address, client id). Never commit them,
and never share a UF2 built with them: the password is inside the firmware.

## Tests

- Robot logic: `make -C tests/host test` (19 programs). Some host compilers reject one sign-conversion warning in `line_nav.c`
  under `-Werror`; it passes without it.
- MQTT and telemetry: `tests/Makefile.mqtt_wire`, `_stream`, `_session`, `_mailbox`, `_heartbeat`, `_lwip` (builds with
  `-DAAI12_MQTT_SUBSCRIBE=0`: its packet counts predate subscribing), `telemetry_inbox`, `telemetry_queue`.
  `telemetry_rtos` needs the Pico SDK headers.
- Buddy 5: `sh tests/buddy5/run_host_tests.sh`.
- Dashboard: `cd dashboard && npm install && npm test` (49 tests).

## Known limits (not fixed)

- **Hump height confidence.** Every hump event in the logs was LOW CONFIDENCE: confidence depends on the encoder distance over
  the bump (60 to 400 mm) and the encoders report many illegal transitions (the counters saturate at 255). Handling the car by
  hand also looks like a hump. The 76 mm lever is a fit to one 20 mm hump, so the height is not linear.
- **Lift stop** triggers at about 60 degrees of tilt or on free-fall; a level lift is invisible to an accelerometer.
- **Obstacle planner:** the box centre comes from a wide sonar beam; the width is clamped to 220 mm; under about 30 cm the sonar
  over-reads. A hand or object in front mid-bypass holds the car for 4 s and then stops it with a fault (no re-plan).
- **IMU info flag** (`f=0020`) stays set although the IMU is healthy.
- **Unverified on the car:** a run powered from the power bank only; the START and STOP acknowledgements on every run; the
  barcode-to-turn chain end to end on the track.
