# Desk tests (no track needed)

Three tests from the Tribunal 1 test plan that can be done at a desk. Total about 45 minutes. The fault-mask test is already done on the
PC: see `fault_mask_stress.md`.

**Safety:** keep the car's wheels OFF the floor, and keep a hand near it, for every test where the motors could run.

Open two terminals. In the first, record the car's serial console for the whole session:

```
cd C:\Users\cools\Github
python3 serial_console.py --log desk1.txt
```

Work in `C:\Users\cools\Github\aai12-dashboard` for the rest. Flash the firmware you want to test first (the same file for all three tests).

## 1. Sonar accuracy against a ruler (about 10 minutes)

Needs: the dashboard running and the car connected, a ruler, a flat board at least 20 cm wide.

```
node tools/desk-ruler.mjs sonar
```

It asks for 100, 150, 200, 250, 300, 400 and 500 mm. Measure from the sonar's FRONT FACE to the board, keep the board square to the
sonar and the car still. It reads the dashboard's sonar value for 8 s at each distance and writes a CSV.
- Criterion (Tribunal 1): error within +-20 mm over 100 to 500 mm.
- Expected: it will probably FAIL at the close distances: the sonar is known to over-read below about 30 cm. That is a real result, and
  the CSV gives the correction table.

## 2. Encoder distance and direction against a ruler (about 10 minutes)

Needs: the dashboard running and the car connected, a ruler or tape with a 500 mm length marked, the motors off (car in IDLE or STOPPED).

```
node tools/desk-ruler.mjs encoder
```

It asks you to roll the car forward 500 mm along the ruler, then back to the start mark, three times. The sonar keeps pinging, so the
encoder and ECHO interrupts run together. It checks direction, distance error and the encoder's bad-transition count.
- The 2 % limit it prints is a proposal, not from the Tribunal text. A hand roll cannot prove an exact pulse count; say so in the report.

## 3. WiFi drop and recovery (about 8 minutes)

Needs: the car on and joined to the hotspot. Press START with the wheels off the floor if you want Motion under load.

STOP THE DASHBOARD first (Ctrl+C in its terminal), because the test uses the same port for its own broker:

```
node tools/net-drop.mjs --trials 10 --down 20
```

Each trial takes the broker away for 20 s (the car's buffer fills and records are dropped), brings it back, and times how long the car
takes to reconnect and deliver telemetry. It writes a CSV.
- Criterion (Tribunal 1): reconnection within 30 s in at least 9 of 10 trials.
- Also check the serial log for the same period (use the times the test printed):
  `python3 log_summary.py desk1.txt --from HH:MM:SS --to HH:MM:SS`
  Success: no new Motion tick overrun in the window, and the telemetry drop counter grows while the broker is down.
- When it ends, start the dashboard again.

## What to keep as evidence
The three CSV files, `desk1.txt`, and the terminal output of each run (screenshot or copy). Put them in `docs/evidence/` with the date.

## 4. Timing before and after the RAM change (about 25 minutes, Week 13 item 4)

Compares the firmware you tested (`robot_wifi_telemetry_v21_startstop_fazfaz_smp1.uf2`, BEFORE) with the one with the smaller telemetry queues
(`robot_wifi_telemetry_v22_minbit_fazfaz_smp1.uf2`, AFTER). Do exactly the same thing with each, one after the other, with the same power and the same
position of the car. Both files contain your hotspot password: keep them off GitHub.

For EACH firmware:
1. Flash it and start the serial log: `python3 serial_console.py --log desk_before.txt` (use `desk_after.txt` for the second).
2. Start the dashboard (`npm start`) and wait until the Robot pill says "sending".
3. Leave the car idle for 5 minutes. (Optional and better: put a strip of black tape under the line sensors, wheels OFF the floor, press START and let it run for 2 minutes; then press STOP.)
4. Stop the log with Ctrl+C.
5. Run `python3 log_summary.py desk_before.txt` (or `desk_after.txt`) and keep the output.

Compare the two: worst Motion, Line and Vehicle run times, new tick overruns, and the telemetry drop counter. No new Motion overrun and similar worst-case times mean no timing penalty.

Also check on the AFTER firmware (about 5 minutes): the dashboard shows every card updating; START and STOP answer RECEIVED then COMPLETED; sending the barcode map answers `OK n=3`.

## 5. The 150 ms obstacle stop (Tribunal 1 test 1): read this before running it

From the code (not yet measured), the time from an obstacle appearing to the motors stopping is made of:
- the sonar pings every 70 ms;
- the monitor takes the median of the last 3 readings and then needs 2 readings in a row below the 330 mm stop distance, so a new obstacle is declared on the 3rd ping after it appears: about 140 ms, plus up to 70 ms more if it appears just after a ping;
- an obstacle already closer than half the stop distance (165 mm) is declared on the first reading (about 85 to 100 ms in total);
- the Vehicle Controller (10 ms cycle) and Motion (5 ms) add up to about 15 ms.

So an obstacle that enters the 330 mm range from further away is likely to take **about 155 to 250 ms**, longer than 150 ms, while one that appears close takes about 100 ms. This is an
analysis, not a measurement. The test needs a timestamp of the moment the motor output reaches zero, which the firmware does not log yet. If the measured time exceeds 150 ms the honest
options are to report it, or to change the confirmation (for example act on the first reading and keep the median only for release).

### How to run the 150 ms measurement (about 30 minutes; floor space not needed for the timing itself)

Use the timing firmware `robot_wifi_telemetry_v22_stoptiming_fazfaz_smp1.uf2`. It is the v22 firmware plus one `[stop]` line per obstacle stop (built with
`ROBOT_STOP_TIMING=1`; with the flag off the firmware is byte-identical to v22). It contains your hotspot password: keep it off GitHub.

The line reports, in microseconds: the sonar's first reading inside the stop distance, the monitor declaring the obstacle, the Vehicle Controller posting STOP,
Motion setting the motor output to zero, and the wheels confirmed still.

1. Flash the timing firmware and start the serial log: `python3 serial_console.py --log desk_stop.txt`.
2. Put the car with the **wheels off the floor** and a strip of black tape under the line sensors so it keeps following. Start the dashboard if you want to watch it.
3. Press START (GP20 or the dashboard). The wheels turn in the air.
4. Bring a board or a book into the sonar's path **from far to near**, quickly, so it enters the 330 mm range from further away (start at about 50 cm and move it in to about 25 cm). Also, now and then, put it in suddenly **very close** (about 10 to 15 cm). Do both kinds: they take different paths in the code.
5. The car stops for the obstacle and prints a `[stop]` line within about a second. Take the board away, press STOP, wait for the car to settle, press START and repeat. Do at least **10 stops**.
6. Stop the log (Ctrl+C) and run `python3 stop_timing_summary.py desk_stop.txt`.

What it reports: each stop broken down into the four hand-overs, and two verdicts against 150 ms: measured from the sonar's first reading, and (conservatively) from the earliest possible entry, which adds one ping period (70 ms) because the car cannot see an obstacle between two pings.

The physical stopping distance on the floor is a separate measurement (tape on the floor, car at 100 mm/s towards a board, mark where it stops); this test measures time to zero motor output.

## 6. A lap on the mini track without a USB cable (record the dashboard feed)

When the car cannot be tethered, record what it sends over WiFi instead. This gives the lap, barcode, hump, obstacle and fault evidence. It cannot give the timing
comparison or the `[stop]` lines: those exist only in the serial log, which needs the USB cable.

1. Start the dashboard (`npm start` in `C:\Users\cools\Github\aai12-dashboard`) and wait until the Robot pill says "sending".
2. Send the barcode map from the dashboard card (default `A=L,B=R,F=U`) and check the page says `OK n=3`.
3. In a second terminal start the recorder, one file per lap and per firmware:
   `node tools/record-feed.mjs --out lap_v22_1.jsonl`
   It prints a line every 5 s with the state and the last event, and warns if the car goes silent.
4. Put the car on the track and press START (GP20 or the dashboard). Keep a hand near the car for the first lap.
5. When the lap is over, press Ctrl+C in the recorder terminal. It saves the file and prints the lap report at once.
   Reprint it any time with `node tools/feed-summary.mjs lap_v22_1.jsonl`.

The report gives: time in each vehicle state and the timeline, distance run and top speed, every barcode / junction / hump / obstacle / plan / fault event with its time,
the barcode, encoder, IMU and sonar counters, hump and obstacle-scan tables, faults seen, gaps in the feed, broker connections, and the car's own drop counter.

To compare the two firmwares, record the same lap with each (same start, same battery, at least 3 laps) and compare the reports: states and times, faults, distance, drops, and whether the barcode and hump were read.
Event times have 10 ms resolution.

## 7. Collecting the raw sonar rays for the obstacle-width fix (about 25 minutes, needs the USB cable)

The car reports the box as 21 to 22 cm wide when it is about 15 cm, because the sonar beam is wide and the planner clamps the width at 22 cm. To design a better estimate we need the
raw rays (angle and distance) the scan sees for a box of known size and position. This firmware build prints them.

Use `robot_wifi_telemetry_v22_rays_fazfaz_smp1.uf2` (v22 plus a `[ray]` line per ray after every scan; with the flag off it is byte-identical to v22). It contains your hotspot
password: keep it off GitHub.

1. Flash it and start the serial log: `python3 serial_console.py --log rays1.txt`.
2. Put the car with the **wheels off the floor**, black tape under the line sensors. The box must stand with its flat 15 cm face square to the car (the box is about 15 cm wide, 10 cm deep, 20 cm high).
3. For each placement below, put the box in place FIRST (measure with the tape: from the sonar's front face to the box's front face, and sideways from the car's centre line to the middle of the box), then press START. The car sees the box, stops and scans, and prints the rays.
4. After the scan the car starts its bypass in the air: press STOP, move the box to the next placement, press START again. Do each placement **twice**.
5. Write the true distance and offset of every scan in order, in a note next to the log.

| Scan | Distance (cm) | Sideways offset of the box centre (cm, + is right) |
|---|---|---|
| 1, 2 | 28 | 0 |
| 3, 4 | 35 | 0 |
| 5, 6 | 22 | 0 |
| 7, 8 | 30 | -5 (left) |
| 9, 10 | 30 | +5 (right) |

6. Stop the log and run `python3 ray_report.py rays1.txt --csv rays1.csv`. Send me the log, the CSV and your note of the true positions.

With real rays I can check any new width estimate against the true 15 cm before it goes into the firmware.
