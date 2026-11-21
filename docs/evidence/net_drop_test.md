# Evidence: WiFi / telemetry drop and recovery (Tribunal 1, "WiFi/Telemetry + Motion - timing")

Run on 2026-11-21 with the finalised robot firmware (WiFi + MQTT build, v21) on the car at a desk, and `dashboard/tools/net-drop-test.mjs`
standing in for the dashboard.

**Test case (Tribunal 1):** disconnect WiFi/MQTT until the telemetry buffer fills, then restore service while Motion runs.
**Success criteria:** Motion never waits for network recovery or buffer space; overflow is counted and records older than 1 s are
discarded; no missed 5 ms Motion deadlines; reconnection within 30 s in 9 of 10 trials.

## Method
`net-drop-test.mjs` runs the dashboard's own MQTT broker. In each of 10 trials it closes the broker (the car loses its session), keeps it
down for 20 s, restarts it, and times how long the car takes to (a) connect again and (b) deliver its first telemetry record. The car's
serial console was logged in parallel (`desk3.txt`; the window 15:54:30 to 15:59:30 is in `net_drop/`).

## Result: 10 of 10 trials within the limit

| Trial | Down (s) | Reconnected after (s) | First record after (s) | Records dropped by the car (running total) | Result |
|---|---|---|---|---|---|
| 1 | 20 | 0.3 | 0.5 | 628 | PASS |
| 2 | 20 | 0.3 | 0.5 | 856 | PASS |
| 3 | 20 | 0.3 | 0.5 | 1084 | PASS |
| 4 | 20 | 0.3 | 0.5 | 1312 | PASS |
| 5 | 20 | 0.3 | 0.4 | 1539 | PASS |
| 6 | 20 | 0.3 | 0.5 | 1767 | PASS |
| 7 | 20 | 0.3 | 0.4 | 1994 | PASS |
| 8 | 20 | 0.3 | 0.5 | 2222 | PASS |
| 9 | 20 | 0.3 | 0.5 | 2449 | PASS |
| 10 | 20 | 0.3 | 0.5 | 2676 | PASS |

Criterion: reconnection within 30 s in at least 9 of 10. Measured: 10 of 10, in 0.5 s or less (limit 30 s). Data: `net_drop/net-drop-run2.csv`.
An earlier run (`net-drop-run1-no-serial-log.csv`, 10 of 10, first trial 3.3 s) was made before the serial log was recording, so it
is kept only as a repeat of the result.

## Motion timing during the same window (serial log, `tools/log_summary.py`)

| Measure | Value |
|---|---|
| Status lines in the window | 273 |
| New tick overruns (Motion / Control / Vehicle) | +0 / +0 / +0 |
| Worst Motion run time so far | 152 us (period 5000 us) |
| Worst Line run time | 275 us (period 10000 us) |
| Worst Vehicle run time | 512 us |
| Worst ADC interrupt | 195 us (period 1000 us) |
| Telemetry records dropped in the window | +2724 (counted, buffer full) |

The car's MQTT state cycles as designed on every trial: 4 (backoff) when the broker disappears, 1 (connecting) while it is down, 3
(online) 0.3 s after the broker is back (`net_drop/serial_log_*.txt`, `mqtt=` in the `[wifi]` lines).

## What this shows
- **Motion does not wait for the network:** no tick overrun in 4.5 minutes of repeated drops, and the worst Motion run time is 3 % of its period.
- **Overflow is counted, not blocked on:** about 11 records per second were dropped while the broker was away, the drop counter rose
  by 2724, and the queue was empty again (depth 0) after each reconnection.
- **Recovery is fast:** 10 of 10 trials delivered data again within 0.5 s of the broker returning.

## Limits (stated plainly)
- The car was **idle** (state IDLE, wheels not driven) during this run, so Motion was not under driving load. Repeat with START and the
  wheels off the floor for a harder case.
- The test removes the **broker** (the TCP session), not the **WiFi link**. Losing the hotspot itself also needs the car to rejoin WiFi and
  get a new address, which is slower and was not exercised here.
- "Records older than 1 s are discarded" is **not measured by this test**; it is covered by the telemetry queue host tests. The drop counter
  here counts records dropped because the buffer was full.
- Reconnection times of 0.3 s reflect the car retrying continuously while the broker is down; they are not a worst-case bound.
