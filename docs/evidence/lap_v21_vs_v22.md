# Evidence: laps on the mini track, v21 (before) and v22 (after the smaller telemetry queues)

Two laps recorded without a USB cable by `dashboard/tools/record-feed.mjs` on 2026-11-21 and summarised by `dashboard/tools/feed-summary.mjs`.
Full reports: `laps/lap_v21_1_report.md` (v21, 5:01 pm, 176 s) and `laps/lap_v22_1_report.md` (v22, 5:05 pm, 264 s). The raw recordings stay in the dashboard folder (2 to 3 MB each).

## Comparison

| | v21 (before) | v22 (after) |
|---|---|---|
| Recording length | 175.8 s | 264.4 s |
| Distance run | 8 556 mm | 15 205 mm |
| Barcodes read in the recording (letter F) | 2, none rejected | 4, none rejected |
| Instructions executed at the junction | 2 | 4 |
| Obstacle stops, scans and bypass plans | 2, both BYPASS LEFT, both rejoined the line | 4, all BYPASS LEFT, all rejoined the line |
| Scanned obstacle (closest / width) | 28 cm / 21 cm | 28 to 29 cm / 21 to 22 cm |
| Hump readings | 2, both 9 mm, VALID | 5: 10, 9, 9, 9 mm VALID and 11 mm LOW CONFIDENCE |
| Time in FOLLOW / BYPASS / SEARCH LINE | 38 % / 15 % / 12 % | 43 % / 21 % / 11 % |
| Line lost events per minute | 7.8 | 9.3 |
| Time from one barcode to the next | 69.3 s | 66.6, 52.9, 68.9 s (mean 62.8 s) |
| Telemetry records dropped by the car | 0 | 0 |
| Feed gaps over 2 s | 4 (2.2, 3.1, 2.4, 9.7 s) | 3 (2.6 s each) |
| Broker disconnects | 0 | 0 |
| Lift stop | IMU_LIFT fault at 152 s after the car was picked up (tilt 90 degrees) | not exercised in this recording |
| Encoder bad transitions at the end | 231 left, 240 right | 255, 255 (the counters stop at 255) |

## What this shows
- Both builds follow the line, read the barcode every time it was reached (6 of 6, none rejected), execute the instruction at the junction, stop for the box, scan it and rejoin the line (6 of 6).
- Hump heights are VALID in 6 of 7 readings (9 to 11 mm); the one LOW CONFIDENCE is 11 mm.
- Nothing separates v22 from v21 in lap behaviour: the lap time varies more from lap to lap (52.9 to 69.3 s) than between the two builds.
- No telemetry was dropped by the car in either run, and the MQTT connection never dropped.
- v21 shows the lift stop working: the IMU_LIFT fault appears when the car was picked up.

## Limits (stated plainly)
- The recording itself cannot show which firmware was loaded: the feed carries no version. The person who ran the laps confirmed that `lap_v21_1` was recorded on v21 and `lap_v22_1` on v22 (the order of the files, 5:01 pm and 5:05 pm, agrees).
- Six barcode reads, six bypasses and seven humps are small samples. The Tribunal test asks for 95 of 100 decodes.
- No timing: task run times and tick overruns are only in the serial log, so the before/after timing comparison is still open.
- The scanned width (21 to 22 cm) is not the box width: the planner clamps the width at 22 cm, and the box is about 15 cm.
- The hump height has not been checked against a reference measurement of the track's hump.
- The barcode letter F was executed as LEFT in both runs; this must match the map that was sent.
- The sonar sensor-fault flag appears in both runs, but the vehicle's ultrasonic fault was never raised.
- Gaps of 2 to 3 s in the feed (and one of 9.7 s right after the lift in v21) are stalls in delivery: the car dropped no records and the broker saw no disconnect. The serial log would show the cause.
