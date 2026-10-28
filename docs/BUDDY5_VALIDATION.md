# Buddy 5 validation — 27 October 2026

## Completed on the development host

- Portable core and codec: **15 scenarios**, including 4,001 codec round trips.
- AddressSanitizer and UndefinedBehaviorSanitizer: same tests passed. Leak detection
  was disabled because the execution environment does not support it.
- Checked clear-range monitoring, first-invalid STOP, repeated-invalid fault,
  stationary scan/refinement, packed scan boundaries/padding, controller bypass
  acknowledgement, line recovery request, stale feedback, stopped acknowledgement,
  scan movement fault, invalid ranges, malformed binary packets, no candidate route,
  500 mm / 5 s limits, unsigned time wrap, and stop-confirmation timeout.
- Geometry uses a constant Q15 sine table, avoiding a runtime libm dependency.
- ARM compile/link and UF2 generation succeeded with GNU Make, Arm GNU 13.2.Rel1,
  and Pico SDK 2.2.0/TinyUSB 86ad6e56 for the USB profile:

| SMP | Console | BUDDY5 | Purpose | Result |
|---:|---|---:|---|---|
| 0 | UART | 0 | Original demo remains available | Pass |
| 0 | UART | 1 | Stationary range | Pass |
| 1 | USB CDC | 1 | Stationary range | Pass |
| 0 | UART | 2 | Servo calibration | Pass |
| 1 | UART | 3 | Stationary scan | Pass |
| 1 | UART | 4 | Controller interface | Pass |

Mode 3/4 default to calibration disabled; the build result does not establish
that a physical scan or controller integration works. No hardware was connected
to this development environment.

Run portable tests from the repository root in a shell with a host C compiler:

```sh
sh tests/buddy5/run_host_tests.sh
```

The source ZIP contains the dual-core USB range-only UF2. Other images are omitted
so an uncalibrated scan image is not mistaken for the first test.

## Hardware/integration checks still required

1. Confirm GP16/GP17 pin ownership and Pico-safe ECHO voltage; verify fixed-distance
   readings and no-echo/stuck-high/stuck-low handling on the actual car.
2. Calibrate servo centre, endpoints, orientation and settling time. Measure pulse
   timing; check range/width results against known objects and clipped profiles.
3. Merge GPIO IRQ ownership with the encoder module; test simultaneous encoder,
   USB, I2C and ultrasonic activity. Measure ECHO timing jitter under load.
4. Add the real Vehicle Controller task and feedback; verify no-motion startup,
   real stopped confirmation, stale data, reset acknowledgement, no-route stopping
   and bounded line recovery. Check that travel counts absolute path and saturates.
5. Measure end-to-end obstacle-to-PWM stop response against the 150 ms target,
   including sensor period, timeout, scheduler, inter-task delivery and Motion.
   No deadline is claimed from these compile or host tests.
6. Exercise concurrent mailbox readers/writer on both cores, delayed publishers
   and busy acquisition. Verify consumers stop on stale or unavailable status.
7. Agree timestamp widths, priorities, clearance/timeout settings and the team's
   binary MQTT schema. Measure target RAM, stack high-water use, message sizes
   and CPU cost against an equivalent pre-change implementation.

The initial demo's IPC pipeline is not an unpacked implementation of Buddy 5.
Comparing the two image sizes would mix feature and representation changes and
would not establish savings attributable to the minimum-bit requirement.

## Linker section sizes (bytes)

These are image section counts, not peak RAM/stack measurements.

| SMP / console / mode | text | data | bss |
|---|---:|---:|---:|
| 0 / uart / 1 | 42237 | 0 | 9604 |
| 1 / usb_cdc / 1 | 64809 | 36 | 19800 |
| 0 / uart / 2 | 41430 | 0 | 9484 |
| 1 / uart / 3 | 52337 | 12 | 13676 |
| 1 / uart / 4 | 52160 | 12 | 13676 |
| 0 / uart / 0 | 40542 | 192 | 9684 |
