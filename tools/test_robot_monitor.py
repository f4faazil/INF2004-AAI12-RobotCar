#!/usr/bin/env python3
"""Tests the laptop monitor against an in-process fake broker, and checks that
the MQTT packets it builds are the ones the firmware's codec expects."""
import io
import os
import socket
import sys
import threading
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_monitor as mon  # noqa: E402
import robot_telemetry as telem  # noqa: E402


class FakeBroker(threading.Thread):
    def __init__(self, to_send):
        super().__init__(daemon=True)
        self.sock = socket.socket()
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(1)
        self.port = self.sock.getsockname()[1]
        self.to_send = to_send
        self.received = []

    def run(self):
        conn, _ = self.sock.accept()
        conn.settimeout(5.0)
        reader = mon.PacketReader()
        sent = False
        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                for ptype, flags, body in reader.feed(data):
                    self.received.append((ptype, flags, body))
                    if ptype == 1:
                        conn.sendall(bytes([0x20, 0x02, 0x00, 0x00]))
                    if ptype == 8 and not sent:
                        conn.sendall(bytes([0x90, 0x03, 0x00, 0x01, 0x00]))
                        for topic, payload in self.to_send:
                            conn.sendall(mon.encode_publish(topic, payload))
                        sent = True
        except socket.timeout:
            pass
        finally:
            conn.close()
            self.sock.close()


class PacketTests(unittest.TestCase):
    def test_packets_match_the_firmware_codec_vectors(self):
        # the same bytes tests/host/test_mqtt_codec.c expects
        self.assertEqual(mon.encode_connect("abc", 60).hex(), "100f00044d5154540402003c0003616263")
        self.assertEqual(mon.encode_publish("a/b", b"hi").hex(), "300700036 12f626869".replace(" ", ""))
        self.assertEqual(mon.encode_subscribe(1, "a/b").hex(), "82080001000361 2f6200".replace(" ", ""))
        self.assertEqual(mon.encode_pingreq(), bytes([0xC0, 0x00]))

    def test_reader_handles_fragmentation_and_two_packets(self):
        r = mon.PacketReader()
        data = mon.encode_publish("a/b", b"x" * 130) + bytes([0xD0, 0x00])
        out = []
        for i in range(len(data)):
            out.extend(r.feed(data[i:i + 1]))
        self.assertEqual([p[0] for p in out], [3, 13])


class EndToEnd(unittest.TestCase):
    def test_monitor_decodes_robot_messages_and_sends_a_command(self):
        status = telem.encode_status(dict(
            ts10=100, state=3, nav_cmd=1, line_state=0, junction_state=0, barcode_state=1,
            line_position=-12, left_mm_s=150, right_mm_s=148, distance_mm=420, faults=0,
            obstacle_valid=True, obstacle_cm=88, hump_packed=0, highest_hump_mm=127))
        event = telem.encode_event(dict(ts10=101, code=2, arg=ord("A"), seq=3))
        broker = FakeBroker([("aai12/robot/telemetry", status), ("aai12/robot/event", event),
                             ("aai12/robot/status", b"online")])
        broker.start()
        out = io.StringIO()
        code = mon.run("127.0.0.1", broker.port, "aai12/robot", "stop", 9, 1.2, out=out)
        self.assertEqual(code, 0)
        text = out.getvalue()
        self.assertIn('"state_name": "FOLLOW"', text)
        self.assertIn('"line_position": -12', text)
        self.assertIn('"name": "BARCODE"', text)
        self.assertIn('"status": "online"', text)
        publishes = [r for r in broker.received if r[0] == 3]
        self.assertEqual(len(publishes), 1)
        topic, payload = mon.parse_publish(publishes[0][1], publishes[0][2])
        self.assertEqual(topic, "aai12/robot/cmd")
        self.assertEqual(payload, telem.encode_command({"req_id": 9, "cmd": 1, "param": 0}))


if __name__ == "__main__":
    unittest.main()
