#!/usr/bin/env python3
"""Checks the Python reference codec against the byte vectors the C firmware
tests use (tests/host/test_telemetry.c), so both sides agree on the layout."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_telemetry as t  # noqa: E402


def h(text):
    return bytes.fromhex(text)


class VectorTests(unittest.TestCase):
    def test_status_vector(self):
        d = t.decode_status(h("01 01 02 52 07 ff ff 40 7f ff e8 00 1b 21 25 aa 00"))
        self.assertEqual(d["ts10"], 0x0102)
        self.assertEqual(d["state"], 5)
        self.assertEqual(d["line_position"], -1)
        self.assertEqual((d["left_mm_s"], d["right_mm_s"]), (-3, 7))
        self.assertEqual(d["distance_mm"], -2)
        self.assertEqual(d["faults"], 0x8001)
        self.assertTrue(d["obstacle_valid"])
        self.assertEqual(d["obstacle_cm"], 200)
        self.assertEqual(d["hump_packed"], 2349)
        self.assertEqual(d["highest_hump_mm"], 40)
        self.assertEqual(d["saturated"], 0)
        self.assertEqual(d["hump"], {"status": 2, "height_code": 37,
                                     "validity": 2, "imu_fault": True})

    def test_saturated_status_vector(self):
        d = t.decode_status(h("01 01 02 52 03 ff 7f e0 17 ff f8 00 1f f9 25 aa 3c"))
        self.assertEqual(d["line_position"], 1023)
        self.assertEqual((d["left_mm_s"], d["right_mm_s"]), (511, -511))
        self.assertEqual(d["distance_mm"], 32767)
        self.assertEqual(d["obstacle_cm"], 510)
        self.assertEqual(d["saturated"], 15)

    def test_event_hump_obstacle_heartbeat_ack(self):
        self.assertEqual(t.decode_event(h("02 12 34 10 02 08 38")),
                         {"ts10": 0x1234, "code": 2, "arg": 65, "seq": 7, "name": "BARCODE"})
        self.assertEqual(t.decode_hump(h("03 00 ff 49 6a 80")),
                         {"ts10": 255, "hump_packed": 2349, "highest_hump_mm": 40})
        o = t.decode_obstacle(h("04 00 64 87 81 5f c1 80 1c 80"))
        self.assertEqual((o["closest_cm"], o["width_cm"], o["centre_cm"]), (30, 10, -4))
        hb = t.decode_heartbeat(h("05 00 00 0e 4d 00 03 02 2a"))
        self.assertEqual((hb["uptime_s"], hb["dropped"], hb["reconnects"]), (3661, 3, 2))
        self.assertEqual(t.decode_ack(h("11 09 40"))["result_name"], "REJECTED_RUNNING")

    def test_sensors_vector_from_the_firmware(self):
        """The bytes tel_encode_sensors produces for the sample record in tests/host/test_telemetry.c."""
        data = h("063039be90302010402d6903323bbd0cb49c")
        self.assertEqual(len(data), 18)
        d = t.decode_sensors(data)
        self.assertEqual(d["ts10"], 12345)
        self.assertEqual((d["ir_levels"], d["ir_valid_mask"]), (5, 7))
        self.assertTrue(d["pair_on_line"])
        self.assertEqual((d["bars_now"], d["bars_done"], d["decoded"], d["rejected"]), (9, 3, 2, 1))
        self.assertEqual((d["junctions"], d["executed"], d["map_entries"], d["last_letter"]), (4, 2, 26, 26))
        self.assertEqual((d["enc_bad_left"], d["enc_bad_right"]), (12, 200))
        self.assertEqual(d["roll_permille"], -137)
        self.assertTrue(d["imu_ok"] and d["hump_active"] and not d["lift_latched"])
        self.assertEqual((d["us_mm"], d["us_valid"], d["servo_aim"]), (406, True, 2))
        self.assertEqual((d["net_state"], d["link_up"], d["saturated"]), (3, True, 0))
        self.assertEqual(t.encode_sensors(d), data)

    def test_command_vector(self):
        self.assertEqual(t.encode_command({"req_id": 9, "cmd": 2, "param": 0x1234}),
                         h("10 09 21 23 40"))

    def test_round_trip(self):
        s = dict(ts10=65535, state=15, nav_cmd=4, line_state=3, junction_state=3,
                 barcode_state=3, line_position=-1023, left_mm_s=-511, right_mm_s=511,
                 distance_mm=-32767, faults=0xFFFF, obstacle_valid=True,
                 obstacle_cm=510, hump_packed=0x1FFF, highest_hump_mm=127)
        d = t.decode_status(t.encode_status(s))
        for k, v in s.items():
            self.assertEqual(d[k], v, k)
        self.assertEqual(d["saturated"], 0)

    def test_wrong_type_is_rejected(self):
        with self.assertRaises(ValueError):
            t.decode(h("7f 00"))


if __name__ == "__main__":
    unittest.main()
