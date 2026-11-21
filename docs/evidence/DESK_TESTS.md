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
node tools/net-drop-test.mjs --trials 10 --down 20
```

Each trial takes the broker away for 20 s (the car's buffer fills and records are dropped), brings it back, and times how long the car
takes to reconnect and deliver telemetry. It writes a CSV.
- Criterion (Tribunal 1): reconnection within 30 s in at least 9 of 10 trials.
- Also check the serial log for the same period: no new Motion tick overrun (`ovr=` first number stays 0), and `drop=` grows while the broker is down.
- When it ends, start the dashboard again.

## What to keep as evidence
The three CSV files, `desk1.txt`, and the terminal output of each run (screenshot or copy). Put them in `docs/evidence/` with the date.
