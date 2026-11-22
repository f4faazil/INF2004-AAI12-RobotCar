#!/usr/bin/env python3
"""Pull the raw sonar rays of every scan out of a serial log of the ROBOT_SCAN_RAYS firmware build.

    python3 ray_report.py desk_rays.txt [--csv rays.csv]

For each scan it prints the car's own estimate (closest distance, width, centre, plan) from the [scan] line and the rays from the [ray] lines
(angle in degrees as the servo is commanded, 90 = straight ahead; kind O obstacle echo, F far echo, N no echo, I invalid; distance in mm).
With --csv it also writes one row per ray, for the width analysis.
"""
import argparse
import csv
import re
import sys

ap = argparse.ArgumentParser()
ap.add_argument("log")
ap.add_argument("--csv")
a = ap.parse_args()

SCAN = re.compile(r"\[scan\] closest=(-?\d+)mm forward=(-?\d+)mm width=(-?\d+)mm centre=(-?\d+)mm left-clear=(-?\d+)mm right-clear=(-?\d+)mm plan=(\d+)")
HEAD = re.compile(r"\[ray\] scan=(\d+) rays=(\d+)")
RAY = re.compile(r"\[ray\] (\d+):([OFNI]):(\d+)\s*$")

scans, last_scan_line, cur = [], None, None
for line in open(a.log, encoding="utf-8", errors="replace"):
    m = SCAN.search(line)
    if m:
        last_scan_line = tuple(int(x) for x in m.groups())
        continue
    m = HEAD.search(line)
    if m:
        cur = dict(n=int(m.group(1)), want=int(m.group(2)), rays=[], summary=last_scan_line)
        scans.append(cur)
        continue
    m = RAY.search(line)
    if m and cur is not None:
        cur["rays"].append((int(m.group(1)), m.group(2), int(m.group(3))))

if not scans:
    sys.exit("no [ray] lines found: is this the log of the ROBOT_SCAN_RAYS build, and did the car stop and scan?")

PLAN = ["continue", "bypass left", "bypass right", "reverse", "stop"]
print(f"{len(scans)} scans\n")
for s in scans:
    summ = s["summary"]
    head = f"scan {s['n']}: {len(s['rays'])}/{s['want']} rays"
    if summ:
        head += f"; the car estimated closest {summ[0]} mm, width {summ[2]} mm, centre {summ[3]:+d} mm, plan {PLAN[summ[6]] if summ[6] < 5 else summ[6]}"
    print(head)
    print("  " + "  ".join(f"{ang}:{k}:{mm}" for ang, k, mm in s["rays"]))
if a.csv:
    with open(a.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scan", "angle_deg", "kind", "distance_mm", "car_closest_mm", "car_width_mm", "car_centre_mm"])
        for s in scans:
            summ = s["summary"] or (None,) * 7
            for ang, k, mm in s["rays"]:
                w.writerow([s["n"], ang, k, mm, summ[0], summ[2], summ[3]])
    print(f"\nwrote {a.csv}")
