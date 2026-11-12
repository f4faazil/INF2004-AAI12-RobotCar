# Motor calibration (Buddy 2)

Measures what `ROBOT_MOTION_CFG` needs and cannot be read from a datasheet: the dead zone of each motor, the PWM-to-speed line of each wheel, the real distance for a 1000 mm command, and the real angle for a 360 degree turn.

> **Status:** the state machine (`motor_cal.c`), the new per-wheel drive model in `motion.c` and the fitting script are tested on a PC. The on-car glue (`motor_cal_task.c`) has been syntax-checked against stub kernel headers only. It has not been built for the board and has not run on the car.

## Before you start

- Floor test, not wheels raised: the dead zone and speed curve with the car's weight on the tyres are what the mission sees. Clear **2 m of floor** for test 3.
- Fully charged battery or power bank, the same one you will race on.
- **Measure the track width** now: centre of the left tyre contact patch to centre of the right, in mm. Put it in `track_width_mm`.
- Measure the tyre diameter with calipers and put it in `wheel_diameter_mm` (and keep `counts_per_rev` so `counts_per_rev / (pi x D)` still equals your rolled counts per mm: `counts_per_rev = 11.77 x pi x D`).

## Build and run

1. In `app_program/robot_config.h` set `ROBOT_MOTOR_CAL` to `1`.
2. Build with the USB console (GP0/GP1 carry the right encoder): `make CONSOLE=usb_cdc PICO_SDK_PATH=...` in `build_make/`.
3. Flash, open the USB serial port, and log everything to a file.
4. Press START (GP20) to run each test in order. STOP (GP21) aborts the running test.

| Test | What the car does | Space | Output |
|---|---|---|---|
| 1 | Left wheel only: 400 ms pulses of rising PWM from 10 %, 600 ms rest, until it moves (BREAKAWAY); then kicked above that and lowered in steps of 0.5 % until it stalls (SUSTAIN) | the car pivots on the right wheel | `BREAKAWAY`, `SUSTAIN`, `SAMPLE` lines |
| 2 | The same for the right wheel | pivots on the left wheel | same |
| 3 | Both wheels open loop at 15, 20, 25, 30, 35, 40, 45 % | about 2 m | `SPEED pwm= l= r=` (counts per second) |
| 4 | Closed-loop 1000 mm at 120 mm/s | 1.2 m | `CLOSED-DONE`, encoder counts |
| 5, 6 | Closed-loop 360 degree turn left, then right | turns in place | `CLOSED-DONE` |

A test that cannot find a breakaway up to 45 % PWM, or in which a wheel travels more than about 1.7 m, prints `ABORT` and stops the motors. That usually means a dead encoder or a disconnected motor.

## Fit

```sh
python3 tools/motor_cal_fit.py cal_log.txt
python3 tools/motor_cal_fit.py cal_log.txt --counts-per-mm 11.863 11.767   # left right, the default
```

It prints the dead zone, the speed table in mm/s, the fitted line `pwm = offset + slope x speed` for each wheel, and the lines to paste into `ROBOT_MOTION_CFG`: `ff_num/ff_den`, `ff_right_num/ff_right_den`, `pwm_offset_left/right` and `creep_speed_mm_s`. Rows below 20 mm/s are left out of the fit because stiction dominates them.

Rebuild with the new values, then run tests 4-6.

## Check the result

- **Test 4:** measure the real distance **D** with a tape. The firmware thinks 1000 mm. If D differs by more than about 2 %, set `counts_per_rev = counts_per_rev x 1000 / D`.
- **Tests 5 and 6:** mark the start heading, measure the real angle **A** the car turned (it should be 360). Set `track_width_mm = track_width_mm x 360 / A`. A left and right result that disagree point to uneven wheels or slip, not to the track width.
- The `RUN` lines during test 4 show the wheel speeds and PWM every 250 ms: both wheels should settle on 120 mm/s within about 0.5 s without oscillating. If not, tune `kp` and `ki`.

When finished, set `ROBOT_MOTOR_CAL` back to `0`.

## What each setting does

| Setting | Effect |
|---|---|
| `pwm_offset_left/right` | PWM added in the direction of motion before the feed-forward term: the dead zone of the motor |
| `ff_num/ff_den`, `ff_right_num/ff_right_den` | PWM per-mille per mm/s, per wheel (`ff_right_den` = 0 shares the left wheel's value) |
| `pwm_limit` | cap on the PWM of both wheels, per-mille (0 = no cap) |
| `creep_speed_mm_s` | slowest speed the Motion module uses to finish a move: keep it above the slowest speed the wheels hold (the fit script suggests 2 times it) |
