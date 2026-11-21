# Lap report: lap_v21_1.jsonl

Recorded 21/11/2026, 5:01:55 pm for 175.8 s, 2406 messages (2386 robot records).
Records per second: status 9.8, sensors 2.0, heartbeat 1.0, event 0.8, hump 0.0, obstacle 0.0

## Vehicle states (72 changes)
| State | Time (s) | Share |
|---|---|---|
| FOLLOW | 67.7 | 38 % |
| BYPASS | 26.1 | 15 % |
| STOPPED | 23.6 | 13 % |
| SEARCH LINE | 20.5 | 12 % |
| OBSTACLE SCAN | 11.2 | 6 % |
| RECOVER LINE | 9.1 | 5 % |
| JUNCTION ADVANCE | 6.1 | 3 % |
| IDLE | 4.6 | 3 % |
| JUNCTION TURN | 4.6 | 3 % |
| CALIBRATE | 2.2 | 1 % |
| OBSTACLE STOP | 0.1 | 0 % |
| REACQUIRE | 0.1 | 0 % |

Timeline: 0.0 s IDLE  >  4.6 s CALIBRATE  >  6.9 s SEARCH LINE  >  7.1 s FOLLOW  >  10.0 s SEARCH LINE  >  11.9 s FOLLOW  >  12.3 s SEARCH LINE  >  14.2 s FOLLOW  >  15.4 s SEARCH LINE  >  16.7 s FOLLOW  >  21.0 s JUNCTION ADVANCE  >  21.0 s FOLLOW  >  23.8 s SEARCH LINE  >  24.0 s FOLLOW  >  24.3 s SEARCH LINE  >  24.5 s FOLLOW  >  24.8 s SEARCH LINE  >  25.1 s FOLLOW  >  25.3 s SEARCH LINE  >  25.5 s FOLLOW  >  25.9 s SEARCH LINE  >  26.1 s FOLLOW  >  44.3 s JUNCTION ADVANCE  >  45.3 s JUNCTION TURN  >  47.7 s FOLLOW  >  48.1 s OBSTACLE STOP  >  48.2 s OBSTACLE SCAN  >  55.3 s BYPASS  >  67.4 s RECOVER LINE  >  71.9 s SEARCH LINE  >  73.5 s FOLLOW  >  73.8 s JUNCTION ADVANCE  >  74.8 s SEARCH LINE  >  75.1 s FOLLOW  >  78.7 s SEARCH LINE  >  80.9 s FOLLOW  >  81.0 s SEARCH LINE  >  82.6 s FOLLOW  >  83.3 s SEARCH LINE  >  84.6 s FOLLOW  >  85.4 s SEARCH LINE  >  87.0 s FOLLOW  >  89.0 s JUNCTION ADVANCE  >  90.1 s FOLLOW  >  93.5 s SEARCH LINE  >  93.8 s FOLLOW  >  94.1 s SEARCH LINE  >  94.3 s FOLLOW  >  94.6 s SEARCH LINE  >  95.0 s FOLLOW  >  95.2 s SEARCH LINE  >  95.5 s FOLLOW  >  95.7 s SEARCH LINE  >  96.0 s FOLLOW  >  113.7 s JUNCTION ADVANCE  >  114.7 s JUNCTION TURN  >  117.0 s REACQUIRE  >  117.0 s FOLLOW  >  118.5 s OBSTACLE STOP  >  118.5 s OBSTACLE SCAN  >  122.7 s BYPASS  >  136.6 s RECOVER LINE  >  141.2 s SEARCH LINE  >  142.7 s FOLLOW  >  142.8 s JUNCTION ADVANCE  >  143.8 s SEARCH LINE  >  145.9 s FOLLOW  >  149.2 s SEARCH LINE  >  149.3 s FOLLOW  >  149.9 s JUNCTION ADVANCE  >  150.8 s FOLLOW  >  152.3 s STOPPED

## Motion
Distance run: 8556 mm. Fastest wheel: 480 mm/s.

## Events (143)
Counts: STATE 77, LINE LOST 23, LINE FOUND 23, JUNCTION 7, HUMP 2, BARCODE 2, NAV EXEC 2, OBSTACLE 2, PLAN 2, RECOVERED 2, FAULT 1

| Time | Event | Detail |
|---|---|---|
| 21.0 s | JUNCTION | LEFT |
| 22.9 s | HUMP | FLAT 9 mm |
| 29.9 s | BARCODE | F |
| 44.3 s | NAV EXEC | LEFT |
| 44.3 s | JUNCTION | LEFT |
| 48.0 s | OBSTACLE | 0 |
| 55.3 s | PLAN | BYPASS LEFT |
| 71.9 s | RECOVERED | 0 |
| 73.7 s | JUNCTION | LEFT |
| 88.9 s | JUNCTION | LEFT |
| 92.8 s | HUMP | FLAT 9 mm |
| 99.2 s | BARCODE | F |
| 113.7 s | NAV EXEC | LEFT |
| 113.7 s | JUNCTION | LEFT |
| 118.5 s | OBSTACLE | 0 |
| 122.7 s | PLAN | BYPASS LEFT |
| 141.1 s | RECOVERED | 0 |
| 142.8 s | JUNCTION | LEFT |
| 149.8 s | JUNCTION | LEFT |
| 152.3 s | FAULT | IMU_LIFT |

## Barcode, encoders, IMU, sonar (last sensor record, and extremes)
- Barcode: decoded 2, rejected 0, completed 2, last letter F, letters mapped 3, junctions 7, executed 2
- Encoder bad transitions: left 231, right 240 (counters stop at 255); encoder fault seen: no
- IMU: ok in 345/345 records; lift latched seen: YES; largest sideways tilt 90.0 degrees
- Sonar: 226/345 valid readings, closest 110 mm; sensor fault seen: YES
- Map in the car: 3 letters

## Hump records (2)
| Height (mm) | Validity | Highest valid so far |
|---|---|---|
| 9 | VALID | 9 |
| 9 | VALID | 9 |

## Obstacle scans (2)
| Time | Closest (cm) | Width (cm) | Centre (cm) | Plan | Reason |
|---|---|---|---|---|---|
| 55.3 s | 28 | 21 | 4 | BYPASS LEFT | OK |
| 122.7 s | 28 | 21 | 4 | BYPASS LEFT | OK |

## Faults seen in status records
IMU (1723 records), IMU_LIFT (201 records)

## Feed health
Gaps of more than 2 s with no robot data: 90.2 s for 2.2 s, 146.0 s for 3.1 s, 154.0 s for 2.4 s, 156.6 s for 9.7 s
Broker events: 0 connects, 0 disconnects, 0 keep-alive timeouts
Telemetry records dropped by the car (counter): 133 -> 133 (+0); queue depth now 0

Not in this report (serial log only): task run times, tick overruns, [stop] timing lines.
