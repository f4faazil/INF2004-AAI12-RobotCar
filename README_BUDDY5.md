# Buddy 5: ultrasonic and servo on µT-Kernel

This adds Buddy 5 to the supplied **INF2004-AAI12-RobotCar** GNU Make project.
It does not use the Pico SDK runtime or CMake. The SDK is still needed for the
repository's USB console sources, as in the original project.

Start with the **stationary distance test**. This package is a first integration,
not a complete autonomous robot: this repository still contains the demo app,
not your teammates' Vehicle Controller, Motion, Line or Telemetry implementations.
Physical range accuracy, interrupt timing and servo geometry remain to be tested.

## 1. Copy into your branch

Check `git status` first. Commit or otherwise preserve existing work. Copy this
ZIP's source files into the **repository root** (the folder containing
`app_program`, `build_make`, `config`, etc.), preserving the folder paths.
Do not create another repository inside your existing one. `.git` is not included.
The changes are also provided as `buddy5.patch` for review; use either the source
files or the patch, not both. If your local versions differ from the uploaded
repository, review the changes before replacing those files.

Existing files changed:

- `app_program/app_main.c`: starts Buddy 5 when selected; original demo remains mode 0.
- `build_make/pico_rp2040.mk`: adds BUDDY5 modes, image names and stale-object guard.
- `kernel/sysdepend/pico_rp2040/hw_setting.c`: releases GP16 from the demo LED in Buddy 5 modes.
- `include/sys/sysdepend/pico_rp2040/sysdef.h`: rejects GPIO diagnostic scan with Buddy 5.
- `config/config.h`: disables sample device drivers in Buddy 5 modes. Their I2C0 pin assignment uses GP8/9, which your robot uses for motors.

New files: `app_program/b5*`, `tests/buddy5/*`, and the Buddy 5 documentation.

## 2. Wiring for the distance test

| Signal | GPIO |
|---|---|
| Ultrasonic TRIG | GP16 |
| Ultrasonic ECHO | GP17 |
| Servo signal (later) | GP12 |

Retain the previously verified sensor supply and common ground. Confirm ECHO is
safe for a 3.3 V Pico input; the power-bank supply label does not establish the
ECHO signal voltage. Do not connect an unconditioned 5 V ECHO to GP17.
No other device or demo LED may drive GP16.

Lift the wheels off the table. Bench modes hold GP8–GP11 low and start no motor
task. Do not press the board's manual motor-test buttons during these tests.

## 3. First test: distance only

Build from `build_make` using the terminal/toolchain that builds this repository.
On Windows this needs a GNU-compatible environment (for example, the team's
existing MSYS2/WSL setup), GNU Make, host `g++`, and `arm-none-eabi-gcc` on PATH.
The Pico VS Code CMake “Compile Project” command is not this project's build.

```sh
cd build_make
make SMP=1 CONSOLE=usb_cdc BUDDY5=1 PICO_SDK_PATH=/your/path/to/pico-sdk -j8
```

Use your SDK path; forward-slash paths avoid Windows escaping problems. The port
was build-tested with Arm GNU 13.2.Rel1 and Pico SDK 2.2.0 plus its TinyUSB submodule.
Other installed versions have not been qualified by this package.

The output is `mtk3pico_smp1_usb_cdc_b5mode1.uf2`. A prebuilt copy of this exact
range-only profile is in `firmware/`, so you can test without building first.
Hold BOOTSEL while connecting USB, then copy the UF2 to the Pico's boot drive.
After reboot, open its USB serial port in your serial monitor (115200 if asked).
Close any other program using that COM port. Allow USB to enumerate.

Expect a line roughly every 0.5 s:

```text
[b5] mm=300 state=0 request=1 faults=0x0 samples=0
```

`mm` is millimetres. `request=1` means STOP: bench firmware never authorises
movement. `samples=0` is normal here because it counts scan samples, not pings.
`range=INVALID` means no usable distance; it does not mean a clear path.

Place a flat, hard object at measured distances of 100, 200 and 500 mm from the
sensor face. Record the reported values and variation. Remove or angle the object
and check that invalid readings are reported. This tests sensing only: it does
not demonstrate obstacle stopping, avoidance or the 150 ms response target.

## 4. Servo calibration, then stationary scan

`BUDDY5=2` sends the centre/right/centre/left pulse values in `b5_config.h`, holding
each for four seconds. Defaults are deliberately narrow: 1500/1400/1500/1600 µs.
These are **not confirmed angles**. Watch the mechanism and adjust pulses gradually;
stop if it binds. Establish forward mechanically, then confirm the positions
correspond to right 30°, forward 90°, left 150°. Rebuild after each change.
The interpolation assumes approximately linear angle-to-pulse mapping on each
side of forward; validate this before relying on width estimates.

Only after calibration, set `B5_SERVO_CALIBRATED` to 1 and build `BUDDY5=3`.
Uncalibrated scan/control modes print a message and exit without scanning.
Mode 3 uses explicitly simulated stopped feedback for this stationary bench only.
Bring an obstacle within the proposed 250 mm threshold; it should request a stop,
scan coarse positions, refine the obstacle profile and report READY or a fault.
No wheel movement follows. Move/no-echo tests should be done with the car stationary.
The scan is one-shot; reboot to repeat. `b5_read_scan()` exposes the packed rays.

## 5. Controller integration is a separate step

`BUDDY5=4` runs the state machine using real feedback through `b5_utk.h`, without
simulated stopped reports or periodic console logging. The team must add its
controller/motion/line/telemetry task startup to this profile. At present only the
Buddy 5 task is started, so no feedback correctly causes a STOP/fault.

See `docs/BUDDY5_INTERFACE.md` before integration. A direction result is a bypass
candidate, not proof the whole car fits. The controller must plan the manoeuvre,
respect all stop/fault requests and use motion/line feedback. Buddy 5 never writes
motor outputs in controller mode. There is no autonomous reverse manoeuvre here.

## 6. Review and commit

From the repo root:

```sh
git diff --stat
git diff
```

After reviewing and testing, stage the changed files and commit on your existing
branch. Do not commit generated build objects or local SDK/toolchain folders.
`BUDDY5=0` selects the original demo; it still blinks GP16, so disconnect ultrasonic
TRIG before running that profile. Do not flash demo firmware on the wired car
without reviewing its pin use.

See `docs/BUDDY5_VALIDATION.md` for checks performed and checks still outstanding.
