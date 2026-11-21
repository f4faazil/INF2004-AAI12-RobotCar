# Evidence: encoder distance and direction against a ruler (Tribunal 1, "GPIO ISR dispatcher (shared resource)")

Run on 2026-11-21 at a desk with the finalised firmware and `dashboard/tools/desk-ruler.mjs encoder`. The car was rolled by hand (motors idle) 500 mm
forward and 500 mm back along a tape, three cycles, while the sonar pinged continuously, so the encoder and ECHO interrupts ran together.
Data: `encoder/encoder-ruler.csv`.

**Criterion (Tribunal 1):** no lost or extra counts against the known input; correct direction and wrap handling.
A hand roll cannot prove an exact pulse count. This test checks direction and distance against a ruler; the 2 % limit used below is a proposal.

## Result

| Cycle | Leg | Ruler (mm) | Car reports (mm) | Error (mm) | Error (%) | Direction | Bad transitions added |
|---|---|---|---|---|---|---|---|
| 1 | forward | 500 | 463 | -37 | -7.4 | correct | 8 |
| 1 | back | -500 | -486 | -14 | -2.8 | correct | 13 |
| 2 | forward | 500 | 504 | +4 | +0.8 | correct | 10 |
| 2 | back | -500 | -499 | -1 | -0.2 | correct | 7 |
| 3 | forward | 500 | 497 | -3 | -0.6 | correct | 17 |
| 3 | back | -500 | -496 | -4 | -0.8 | correct | 7 |

- **Direction:** correct on 6 of 6 legs (forward positive, back negative).
- **Distance:** 4 of 6 legs within 1 % (4 mm or less). Two legs outside the proposed 2 %: the first forward leg (-37 mm) and the first back leg (-14 mm).
- **Bad (illegal) transitions:** 62 in total over the six legs. At about 118 counts per cm, one 500 mm roll is roughly 5900 counts per wheel, so
  62 bad transitions is about 0.1 % of all transitions, far too small to explain a 37 mm error (about 440 counts).

## Reading it
The two outliers are both in the first cycle and the next four legs are accurate to 4 mm. That pattern fits the way the car was started
rolling (a slip or an imprecise stop on the mark) more than an encoder fault, but this test cannot prove that. A repeat with more cycles would show
whether the first-cycle error returns. An earlier attempt (`encoder/encoder-first-attempt-aborted.csv`) recorded 0 mm for its first leg and was
abandoned, so it is not counted.

## Limits (stated plainly)
- Hand rolling, ruler and mark placement limit the accuracy to a few millimetres at best.
- The encoder bad-transition counter is not zero even at hand speed. It is small against the total, but it is the same counter that saturated at 255 in the
  long runs, so it deserves a look (noise or a missed edge at slow speed).
- Wrap handling and a known pulse sequence were not tested here. They are covered by the host tests, not by this desk test.
