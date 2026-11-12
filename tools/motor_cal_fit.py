#!/usr/bin/env python3
"""Turn a motor calibration log into the drive-model values for ROBOT_MOTION_CFG.

Build the firmware with ROBOT_MOTOR_CAL = 1, run tests 1-3 with the START
button, and save the "[cal]" lines from the USB console to a file:

    python3 tools/motor_cal_fit.py cal_log.txt
    python3 tools/motor_cal_fit.py cal_log.txt --counts-per-mm 11.863 11.767

What it reads
    "[cal] RUN 1 dead zone, LEFT ..."     which wheel the next lines belong to
    "[cal] BREAKAWAY pwm=230 l=.. r=.."   PWM that started the wheel from rest
    "[cal] SUSTAIN pwm=190 l=.. r=.."     lowest PWM that kept it turning
    "[cal] SPEED pwm=250 l=.. r=.."       open-loop speed table, counts/second

What it prints
    the fitted line  pwm = offset + slope x speed  for each wheel, and the
    fields to paste into ROBOT_MOTION_CFG (robot_config.c).

The speeds in the log are encoder counts per second; they are converted to
mm/s with the counts-per-mm of each wheel (default: the three 100 cm rolls,
118.63 and 117.67 counts/cm).  Rows where the wheel did not turn are left out
of the fit.
"""
import argparse
import re
import sys

DEFAULT_COUNTS_PER_MM = (11.863, 11.767)    # left, right
MIN_FIT_SPEED_MM_S = 20.0                   # slower rows are dominated by stiction
FF_DEN = 100                                # feed-forward is written as num / 100
CREEP_MARGIN = 1.5                          # creep speed = slowest stable speed x this

LINE = re.compile(r"\[cal\]\s+(\w+)\s*(.*)")
FIELD = re.compile(r"(\w+)=(-?\d+)")


def parse(text):
    """Return (dead_zone, speed_rows): dead_zone[wheel] = {breakaway, sustain, ...}."""
    dead = {"L": {}, "R": {}}
    speed = []
    wheel = None
    # A log replayed from the car's RAM sits between LOG-BEGIN and LOG-END; if the
    # file also holds a live copy of the same lines, use only the last replay.
    begin = text.rfind("LOG-BEGIN")
    if begin >= 0:
        end = text.find("LOG-END", begin)
        text = text[begin:end] if end >= 0 else text[begin:]
    for raw in text.splitlines():
        m = LINE.search(raw)
        if not m:
            continue
        kind, rest = m.group(1), m.group(2)
        if kind == "RUN":
            if "dead zone, LEFT" in rest:
                wheel = "L"
            elif "dead zone, RIGHT" in rest:
                wheel = "R"
            else:
                wheel = None
            continue
        f = {k: int(v) for k, v in FIELD.findall(rest)}
        if kind in ("BREAKAWAY", "SUSTAIN") and wheel and "pwm" in f:
            dead[wheel][kind.lower()] = f["pwm"]
            dead[wheel][kind.lower() + "_cps"] = abs(f.get("l" if wheel == "L" else "r", 0))
        elif kind == "SPEED" and {"pwm", "l", "r"} <= f.keys():
            speed.append((f["pwm"], f["l"], f["r"]))
    return dead, speed


def fit_line(points):
    """Least squares pwm = offset + slope * speed; points are (speed, pwm)."""
    n = len(points)
    if n < 2:
        return None
    sx = sum(p[0] for p in points)
    sy = sum(p[1] for p in points)
    sxx = sum(p[0] * p[0] for p in points)
    sxy = sum(p[0] * p[1] for p in points)
    den = n * sxx - sx * sx
    if den == 0:
        return None
    slope = (n * sxy - sx * sy) / den
    offset = (sy - slope * sx) / n
    return offset, slope


def fit_wheel(speed_rows, index, counts_per_mm):
    """index 1 = left column, 2 = right column of a speed row."""
    pts = []
    for row in speed_rows:
        v = row[index] / counts_per_mm
        if v >= MIN_FIT_SPEED_MM_S:
            pts.append((v, float(row[0])))
    result = fit_line(pts)
    return result, pts


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("log", nargs="?", help="saved console log (default: stdin)")
    ap.add_argument("--counts-per-mm", nargs=2, type=float, metavar=("LEFT", "RIGHT"),
                    default=DEFAULT_COUNTS_PER_MM)
    args = ap.parse_args(argv)

    if args.log:
        with open(args.log, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
    else:
        text = sys.stdin.read()
    dead, speed = parse(text)
    cpm = {"L": args.counts_per_mm[0], "R": args.counts_per_mm[1]}

    print("Dead zone (per-mille of full PWM)")
    for w, name in (("L", "left"), ("R", "right")):
        d = dead[w]
        print("  %-5s breakaway %s   sustain %s" % (
            name, d.get("breakaway", "not measured"), d.get("sustain", "not measured")))

    print("\nSpeed table (mm/s)")
    for row in speed:
        print("  pwm %4d   left %6.1f   right %6.1f" % (
            row[0], row[1] / cpm["L"], row[2] / cpm["R"]))

    fits = {}
    for w, idx in (("L", 1), ("R", 2)):
        fits[w], pts = fit_wheel(speed, idx, cpm[w])
        if fits[w] is None:
            print("\n%s wheel: not enough rows at or above %.0f mm/s to fit"
                  % ("left" if w == "L" else "right", MIN_FIT_SPEED_MM_S))
        else:
            off, slope = fits[w]
            print("\n%s wheel fit: pwm = %.1f + %.3f x v   (%d points)"
                  % ("left" if w == "L" else "right", off, slope, len(pts)))

    if fits["L"] is None or fits["R"] is None:
        print("\nCannot produce config values: run the speed table (test 3) first.")
        return 1

    off_l, slope_l = fits["L"]
    off_r, slope_r = fits["R"]
    off_l = max(0, round(off_l))
    off_r = max(0, round(off_r))
    ff_l = max(1, round(slope_l * FF_DEN))
    ff_r = max(1, round(slope_r * FF_DEN))

    creep = 0.0
    for w in ("L", "R"):
        cps = dead[w].get("sustain_cps")
        if cps:
            creep = max(creep, cps / cpm[w] * CREEP_MARGIN)

    print("\nPaste into ROBOT_MOTION_CFG in app_program/robot_config.c:")
    print("    .ff_num = %d," % ff_l)
    print("    .ff_den = %d,                 /* left: %.2f per-mille per mm/s */" % (FF_DEN, slope_l))
    print("    .ff_right_num = %d," % ff_r)
    print("    .ff_right_den = %d,           /* right: %.2f per-mille per mm/s */" % (FF_DEN, slope_r))
    print("    .pwm_offset_left = %d," % off_l)
    print("    .pwm_offset_right = %d," % off_r)
    if creep > 0.0:
        print("    .creep_speed_mm_s = %d,      /* slowest stable speed x %.1f */"
              % (int(round(creep)), CREEP_MARGIN))
    else:
        print("    /* creep_speed_mm_s: run the dead-zone tests to get a value */")
    return 0


if __name__ == "__main__":
    sys.exit(main())
