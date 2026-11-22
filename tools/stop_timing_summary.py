#!/usr/bin/env python3
"""Summarise the obstacle-to-stop timing from a serial log of the ROBOT_STOP_TIMING firmware build.

    python3 stop_timing_summary.py desk_stop.txt [--limit-ms 150] [--ping-ms 70]

Each obstacle stop prints one line like
    [stop] #3 first-reading=285mm threshold=330mm  sonar-to-declared=140100us declared-to-STOP=6200us STOP-to-PWM0=2400us  total=148700us  wheels-still=312000us
where "total" runs from the sonar's FIRST reading inside the stop distance to the motor output reaching zero.

The obstacle was physically inside the range some time BEFORE that first reading (up to one ping period, 70 ms), which the car cannot see. So two verdicts are given:
  from the first reading:              total <= limit
  from the earliest possible entry:    total + one ping period <= limit   (the conservative reading of "within 150 ms of entering the range")
"""
import argparse
import re
import statistics
import sys

LINE = re.compile(
    r"\[stop\] #(\d+) first-reading=(\d+)mm threshold=(\d+)mm\s+sonar-to-declared=(\d+)us declared-to-STOP=(\d+)us "
    r"STOP-to-PWM0=(\d+)us\s+total=(\d+)us\s+wheels-still=(>?)(\d+)us")

ap = argparse.ArgumentParser()
ap.add_argument("log")
ap.add_argument("--limit-ms", type=float, default=150.0)
ap.add_argument("--ping-ms", type=float, default=70.0)
a = ap.parse_args()

rows = []
with open(a.log, encoding="utf-8", errors="replace") as f:
    for line in f:
        m = LINE.search(line)
        if m:
            n, mm, thr, det, veh, mot, tot, gt, still = m.groups()
            rows.append(dict(n=int(n), mm=int(mm), thr=int(thr), det=int(det) / 1000, veh=int(veh) / 1000,
                             mot=int(mot) / 1000, tot=int(tot) / 1000, still=int(still) / 1000, still_missed=bool(gt)))
if not rows:
    sys.exit("no [stop] lines found: is this the log of the ROBOT_STOP_TIMING build, and did the car stop for an obstacle?")

print(f"{len(rows)} obstacle stops, limit {a.limit_ms:.0f} ms, one ping period {a.ping_ms:.0f} ms\n")
print("| # | First reading (mm) | Sonar to declared (ms) | Declared to STOP (ms) | STOP to PWM zero (ms) | Total (ms) | Total + ping (ms) | Wheels still (ms) | Path |")
print("|---|---|---|---|---|---|---|---|---|")
for r in rows:
    path = "urgent (under half the range)" if r["mm"] < r["thr"] // 2 else "median + 2 readings"
    print(f"| {r['n']} | {r['mm']} | {r['det']:.1f} | {r['veh']:.1f} | {r['mot']:.1f} | {r['tot']:.1f} | {r['tot'] + a.ping_ms:.1f} | "
          f"{'>' if r['still_missed'] else ''}{r['still']:.0f} | {path} |")

tot = [r["tot"] for r in rows]
ok_first = sum(1 for t in tot if t <= a.limit_ms)
ok_entry = sum(1 for t in tot if t + a.ping_ms <= a.limit_ms)
print(f"\ntotal from the first reading: min {min(tot):.1f}, median {statistics.median(tot):.1f}, max {max(tot):.1f} ms")
print(f"within {a.limit_ms:.0f} ms measured from the first reading:        {ok_first}/{len(rows)}")
print(f"within {a.limit_ms:.0f} ms measured from the earliest possible entry: {ok_entry}/{len(rows)}")
slow = [r for r in rows if r["mm"] >= r["thr"] // 2]
fast = [r for r in rows if r["mm"] < r["thr"] // 2]
for name, grp in (("urgent path (reading under half the range)", fast), ("median + 2 readings path", slow)):
    if grp:
        t = [r["tot"] for r in grp]
        print(f"  {name}: {len(grp)} stops, {min(t):.1f} to {max(t):.1f} ms")
print("\nRESULT:", "PASS in every stop, both ways" if ok_entry == len(rows) else
      ("PASS from the first reading, FAIL from the earliest possible entry" if ok_first == len(rows) else "FAIL: some stops exceed the limit even from the first reading"))
