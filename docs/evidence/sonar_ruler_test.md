# Evidence: ultrasonic accuracy against a ruler (Tribunal 1, "GPIO ISR dispatcher (shared resource)")

Run on 2026-11-21 at a desk with the finalised firmware (WiFi + MQTT build) and `dashboard/tools/desk-ruler.mjs sonar`.
Data: `sonar/sonar-ruler.csv`.

**Criterion (Tribunal 1):** ultrasonic error within +-20 mm over 100 to 500 mm.

## Method
A flat board was placed square to the sonar at each distance, measured with a tape from the sonar's front face to the board. The car was
still, the servo straight ahead. The tool took the median of the dashboard's sonar readings over 8 s (16 readings) at each distance.

## Result: PASS, 7 of 7 distances within +-20 mm (worst error 4 mm)

| True (mm) | Median (mm) | Error (mm) | Min..Max (mm) | Valid readings |
|---|---|---|---|---|
| 100 | 96 | -4 | 96..96 | 16/16 |
| 150 | 153 | +3 | 149..154 | 16/16 |
| 200 | 201 | +1 | 201..205 | 16/16 |
| 250 | 247 | -3 | 247..251 | 16/16 |
| 300 | 303 | +3 | 302..307 | 16/16 |
| 400 | 404 | +4 | 400..405 | 16/16 |
| 500 | 504 | +4 | 499..504 | 16/16 |

Every reading was valid, and the spread at each distance is 5 mm or less.

## Limits (stated plainly)
- One flat target, square to the sonar, in a clear area. This is the best case: a small or angled object (like the 15 cm box at a bypass) can
  read differently because of the wide beam. The scan's box-width estimate is a separate question and is not covered here.
- The tape measure and board placement are good to a few millimetres, so errors of 3 to 4 mm are within the measuring tolerance.
- Only 100 to 500 mm was tested, as the criterion asks.
- The encoder interrupts were idle in this test; the interrupt-sharing part is in the encoder test, where the sonar pings continuously.
