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
