# Evidence: shared fault mask concurrency stress test

Run on 2026-11-21 14:47 from branch `evidence/min-bit` (based on commit `54accea`), compiler `gcc (x86_64-win32-seh-rev0, Built by MinGW-Builds project) 15.2.0`, host build.

Requirement covered: Tribunal 1 test *Shared fault mask (concurrency and safety)*: faults raised from several subsystems during
overlapping set/clear operations, including a fault requiring STOP. Success criteria: every raised fault stays recorded until it is
cleared, unrelated bits stay unchanged, STOP keeps precedence during concurrent updates.

## What the test does
`tests/host/test_fault_mask_stress.c`: 6 threads stand in for the subsystems. Each owns two fault bits and repeats `fault_set`,
check the bit reads set, `fault_clear`, check the bit reads clear, 300 000 times, all on one shared `fault_mask_t`. A seventh
thread samples `fault_stop_required()` continuously while a STOP-class fault (`FAULT_IMU_LIFT`) stays raised. A lost update shows
up as a bit that does not read back as just written.

The critical section is a mutex in the host build. On the car, `robot_irq_save()` is the kernel spinlock plus interrupt disable on
the SMP build (`hal_rp2040.c`), which gives the same mutual exclusion between two cores and an interrupt handler.

**Negative control:** the same test built with `-DUNPROTECTED` (empty critical section). If this build did not lose updates the
test could not detect a missing protection and the pass would prove nothing.

Reproduce: `make -C tests/host stress`.

## Results

### Protected build, 5 runs (each 3 600 000 updates)

| Run | Lost set | Lost clear | STOP samples | STOP breaches | Raised fault vanished | Result |
|---|---|---|---|---|---|---|
| 1 | 0 | 0 | 145,070,947 | 0 | 0 | PASS |
| 2 | 0 | 0 | 148,592,258 | 0 | 0 | PASS |
| 3 | 0 | 0 | 160,247,739 | 0 | 0 | PASS |
| 4 | 0 | 0 | 159,171,378 | 0 | 0 | PASS |
| 5 | 0 | 0 | 151,283,951 | 0 | 0 | PASS |

Heavier run (3 000 000 pairs per thread): 36,000,000 updates, lost set=0, lost clear=0, STOP samples=1,611,172,533, breaches=0.

### Negative control (-DUNPROTECTED), 5 runs

| Run | Lost set | Lost clear | Total lost |
|---|---|---|---|
| 1 | 345 | 13 | 358 |
| 2 | 365 | 65 | 430 |
| 3 | 58 | 11 | 69 |
| 4 | 21 | 5 | 26 |
| 5 | 122 | 12 | 134 |

### Full output of one protected run

```
fault mask stress test: PROTECTED (critical section)
  6 writer threads x 300000 set/clear pairs, plus a monitor thread
  writer 0 bits=0x0003  lost-set=0 lost-clear=0
  writer 1 bits=0x000c  lost-set=0 lost-clear=0
  writer 2 bits=0x0030  lost-set=0 lost-clear=0
  writer 3 bits=0x00c0  lost-set=0 lost-clear=0
  writer 4 bits=0x1100  lost-set=0 lost-clear=0
  writer 5 bits=0x6000  lost-set=0 lost-clear=0
  expected final mask 0x0200, actual 0x0200  (match)
  lost updates: set=0 clear=0 of 3600000 operations
  STOP precedence: 162233931 samples, 0 breaches; sticky fault vanished 0 times
  elapsed 0.405 s = 113 ns per update under contention (HOST timing, not the Cortex-M0+)
RESULT: PASS: no lost update, no unrelated bit changed, STOP precedence held
```

## Reading the result
- With the protected update: no lost update, the final mask equals the expected mask exactly, STOP precedence never broke.
- Without it: updates are lost every run, so the test does detect the race it is meant to guard against.

## Limits (stated plainly)
- This is a host test. The mutex models the car's spinlock; it does not exercise the real interrupt handlers or the RP2040's two cores.
- The "ns per update" printed by the test is host timing and is **not** the Cortex-M0+ critical-section time. That figure has to
  be measured on the car (a cycle count around `fault_set`) and is still open.
- "Motor-output response" (PWM reaching zero when a STOP fault is raised) is covered by the vehicle tests (`test_vehicle.c`) and
  the timing test on the car, not by this test.
