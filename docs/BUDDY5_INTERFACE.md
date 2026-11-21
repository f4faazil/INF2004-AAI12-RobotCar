# Buddy 5 interface and ownership

`b5_start()` is called once from usermain before any consumers start. It creates a
4096-byte-stack task at provisional priority 6. Under SMP it is pinned to processor
1 (RP2040 core 0). The same processor owns ECHO IRQ13. Do not remove this affinity:
local interrupt masking cannot protect capture data from another core.

The default firmware is a stationary bench. For real integration select mode 4,
calibrate the servo, and add the team's tasks to the Buddy 5 branch of usermain.
These APIs are task-context only:

```c
/* Vehicle Controller: timestamp the actual underlying measurement snapshot. */
bool accepted = b5_submit_feedback(mode, stopped, motion_valid,
                                  line_valid, line_found, cumulative_path_mm,
                                  measured_at_ms);

/* Vehicle Controller / Telemetry: copy a complete binary status packet. */
uint8_t packet[B5_STATUS_BYTES];
b5_status status;
if (b5_read_packet(packet) && b5_decode_status(packet, &status)) {
    /* Check publication and forward-reading age using b5_now_ms().
       Apply STOP on stale/invalid data; never infer clearance from no echo. */
}
```

`b5_now_ms()` and all timestamps use the same local monotonic clock. A caller must
not refresh the timestamp on unchanged stale measurements. Feedback should arrive
well within the proposed 100 ms maximum age. If motion and line measurements have
different ages, use the older relevant timestamp or mark the stale component
invalid. Stop confirmation must describe actual stopped motion, not just a sent
motor-stop command. Its timestamp must be newer than the stop request.

The controller is the sole feedback producer. `mode` is NORMAL=0, BYPASS=1,
RECOVERY=2, RESET=3. Cumulative travelled path starts at zero when bypass begins;
count absolute path through both bypass and recovery, including reverse motion if
the team later implements it. The API saturates values above 500 mm to 500 before
packing, so overflow cannot conceal a limit violation. Reset requires fresh,
valid, stopped feedback. Send RESET to acknowledge a latched fault, then return
to NORMAL; holding RESET continuously restarts the state machine continuously.

Requests: NONE=0, STOP=1, LEFT=2, RIGHT=3, RESUME=4. Faults are latched until reset.
LEFT/RIGHT are stationary-scan candidate directions. The controller acknowledges
authorised bypass by supplying BYPASS mode, then RECOVERY during line search.
RESUME is a request requiring controller acknowledgement, not motor actuation.

States: MONITOR=0, WAIT_STOP=1, COARSE=2, FINE=3, READY=4, BYPASS=5,
RECOVERY=6, FAULT=7. Fault bits: no echo=0, bad range=1, stale/invalid feedback=2,
stop-confirmation timeout=3, scan failure=4, recovery limit=5, no route=6.
These are module-local fault bits. The Vehicle Controller maps/combines them with
other modules; Buddy 5 does not modify the team's global fault mask.

All cross-task packed mailboxes use a µT-Kernel mutex with polling acquisition.
A false API return means busy/unavailable/invalid input: retry on a later task
cycle, without spinning. The owner retains the previous feedback when a snapshot
is busy; stale-feedback checks still run. Publication also skips a busy mailbox;
consumers must reject an old publication. Consumers must not hold the mutex or
access the private byte arrays directly. Separate status/scan API calls are not a
single combined snapshot; consume rays only once the stopped scan is complete.

ECHO ISR/task capture uses a single-core critical section around each complete
packed update, with compiler barriers. The raw IRQ handler calls no kernel or
console functions. This port reserves **IO_BANK0 IRQ13**. It rejects already-enabled
core-0 GPIO sources at startup, but cannot discover a dormant handler. Before
Buddy 2 adds encoder IRQs, merge an agreed GPIO interrupt dispatcher; never install
another handler for IRQ13 or enable unrelated sources behind this handler.

GP12 uses PWM slice 6, including GP13's shared period/divider. Reserve the entire
slice. Motor slices 4/5 are not reset by Buddy 5. The hardware timer and watchdog
1 µs tick are shared platform resources initialised by this profile; coordinate
any future timebase/watchdog changes. No alarms are allocated. A 10 µs trigger
pulse uses a bounded critical section; ECHO timing is interrupt-driven, not polled.
Interrupt masking elsewhere can still distort pulse timing: measure it on hardware.

Proposed limits in b5_config.h require team acceptance: 250 mm stop, 400 mm side
ray clearance, 70 ms ping period, 30 ms ECHO timeout, 500 ms stopped-confirmation
limit, 15 s scan limit, and 5 s or 500 mm for the **whole detour**, whichever occurs
first. The last rule is conservative and stricter than a recovery-only limit.
Sonar rays do not prove continuous free space; observed width is approximate and
invalid when its sampled clear boundaries are missing or its field of view clips
an obstacle. Do not treat these parameters as demonstrated performance.
