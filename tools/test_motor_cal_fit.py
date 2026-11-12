#!/usr/bin/env python3
"""Tests for motor_cal_fit.py: a synthetic log with a known motor must give
back the same offset, slope and dead zone."""
import io
import os
import sys
import unittest
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import motor_cal_fit as fit  # noqa: E402

CPM = (11.863, 11.767)


def make_log(off_l, slope_l, off_r, slope_r):
    """pwm = off + slope * v  ->  v = (pwm - off) / slope; counts/s = v * cpm."""
    lines = [
        "[cal] motor calibration.",
        "[cal] RUN 1 dead zone, LEFT wheel (car pivots on the right wheel)",
        "[cal] SAMPLE pwm=100 l=0 r=0",
        "[cal] BREAKAWAY pwm=230 l=410 r=0",
        "[cal] SAMPLE pwm=310 l=900 r=0",
        "[cal] SUSTAIN pwm=190 l=235 r=0",
        "[cal] DONE",
        "[cal] RUN 2 dead zone, RIGHT wheel (car pivots on the left wheel)",
        "[cal] BREAKAWAY pwm=250 l=0 r=380",
        "[cal] SUSTAIN pwm=210 l=0 r=200",
        "[cal] DONE",
        "[cal] RUN 3 speed table, both wheels (about 2 m of clear floor)",
    ]
    for pwm in (150, 200, 250, 300, 350, 400, 450):
        vl = max(0.0, (pwm - off_l) / slope_l)
        vr = max(0.0, (pwm - off_r) / slope_r)
        lines.append("[cal] SPEED pwm=%d l=%d r=%d" % (pwm, round(vl * CPM[0]), round(vr * CPM[1])))
    lines.append("[cal] DONE")
    return "\n".join(lines)


class FitTests(unittest.TestCase):
    def test_parse_assigns_dead_zone_to_the_right_wheel(self):
        dead, speed = fit.parse(make_log(100, 1.7, 110, 1.9))
        self.assertEqual(dead["L"]["breakaway"], 230)
        self.assertEqual(dead["L"]["sustain"], 190)
        self.assertEqual(dead["R"]["breakaway"], 250)
        self.assertEqual(dead["R"]["sustain"], 210)
        self.assertEqual(len(speed), 7)

    def test_fit_recovers_the_motor_model(self):
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_cal_test_log.txt")
        try:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(make_log(100, 1.7, 110, 1.9))
            out = io.StringIO()
            with redirect_stdout(out):
                code = fit.main([path])
        finally:
            if os.path.exists(path):
                os.remove(path)
        text = out.getvalue()
        self.assertEqual(code, 0)
        self.assertIn(".ff_num = 170", text)
        self.assertIn(".ff_right_num = 190", text)
        self.assertIn(".pwm_offset_left = 100", text)
        self.assertIn(".pwm_offset_right = 110", text)
        # slowest stable speed: left 235 / 11.863 = 19.8, right 200 / 11.767 = 17.0
        # -> 19.8 x 2.0 = 39.6 mm/s
        self.assertIn(".creep_speed_mm_s = 40", text)

    def test_rows_where_the_wheel_stood_still_are_excluded(self):
        # off=300 puts the lowest rows at zero speed: they must not drag the line
        dead, speed = fit.parse(make_log(300, 1.7, 300, 1.7))
        res, pts = fit.fit_wheel(speed, 1, CPM[0])
        self.assertIsNotNone(res)
        self.assertAlmostEqual(res[0], 300.0, delta=3.0)
        self.assertAlmostEqual(res[1], 1.7, delta=0.05)
        self.assertTrue(all(p[1] >= 350 for p in pts))

    def test_replayed_ram_log_is_not_counted_twice(self):
        live = make_log(100, 1.7, 110, 1.9)
        text = live + "\n[cal] LOG-BEGIN\n" + live + "\n[cal] LOG-END bytes=1\n"
        dead, speed = fit.parse(text)
        self.assertEqual(len(speed), 7)
        self.assertEqual(dead["L"]["breakaway"], 230)

    def test_missing_speed_table_is_reported(self):
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_cal_test_log2.txt")
        try:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write("[cal] RUN 1 dead zone, LEFT wheel\n[cal] BREAKAWAY pwm=230 l=400 r=0\n")
            out = io.StringIO()
            with redirect_stdout(out):
                code = fit.main([path])
        finally:
            if os.path.exists(path):
                os.remove(path)
        self.assertEqual(code, 1)
        self.assertIn("run the speed table", out.getvalue())


if __name__ == "__main__":
    unittest.main()
