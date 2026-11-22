#!/usr/bin/env python3
"""Summarise the robot's serial log for a time window: tick overruns, worst task times and the telemetry drop counter.

    python3 log_summary.py desk1.txt                          the whole file
    python3 log_summary.py desk1.txt --from 14:20:00 --to 14:30:00

The "[robot]" status line carries cumulative counters since boot, so "new overruns during the test" is the value at the end of
the window minus the value at its start:
    ovr=a/b/c          tick overruns of Motion (5 ms), Control/Line (10 ms), Vehicle
    us(m/l/v/isr)=...  worst run time in microseconds of Motion, Line, Vehicle and the ADC interrupt so far
    drop=N             telemetry records dropped because the buffer was full
    net=3              MQTT session online (anything else: not online)
"""
import argparse
import re
import sys

LINE = re.compile(
    r"^(\d\d:\d\d:\d\d)\.\d+\s+\[robot\].*?drop=(\d+) net=(\d+) us\(m/l/v/isr\)=(\d+)/(\d+)/(\d+)/(\d+) ovr=(\d+)/(\d+)/(\d+)")

ap = argparse.ArgumentParser()
ap.add_argument("log")
ap.add_argument("--from", dest="t_from", default="00:00:00")
ap.add_argument("--to", dest="t_to", default="23:59:59")
a = ap.parse_args()

rows = []
with open(a.log, encoding="utf-8", errors="replace") as f:
    for line in f:
        m = LINE.match(line)
        if m and a.t_from <= m.group(1) <= a.t_to:
            g = m.groups()
            rows.append((g[0], int(g[1]), int(g[2]), *map(int, g[3:7]), *map(int, g[7:10])))

if not rows:
    sys.exit("no [robot] status lines in that window (check the file and the --from/--to times)")

first, last = rows[0], rows[-1]
online = sum(1 for r in rows if r[2] == 3)
print(f"window {first[0]} to {last[0]}: {len(rows)} status lines, MQTT online in {online} of them ({100 * online // len(rows)} %)")
print(f"telemetry drop counter: {first[1]} -> {last[1]}  (+{last[1] - first[1]} records dropped in the window)")
print(f"tick overruns at start {first[7]}/{first[8]}/{first[9]}, at end {last[7]}/{last[8]}/{last[9]}  "
      f"(new in the window: motion +{last[7] - first[7]}, control +{last[8] - first[8]}, vehicle +{last[9] - first[9]})")
print(f"worst task times so far (us): motion {max(r[3] for r in rows)}, line {max(r[4] for r in rows)}, "
      f"vehicle {max(r[5] for r in rows)}, ADC interrupt {max(r[6] for r in rows)}   (periods: motion 5000, line 10000)")
print("RESULT:", "no new Motion overrun in the window" if last[7] == first[7] else "NEW MOTION OVERRUNS in the window")
