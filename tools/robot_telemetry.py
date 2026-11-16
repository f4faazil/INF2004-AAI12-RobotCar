#!/usr/bin/env python3
"""Reference encoder/decoder for the robot's binary MQTT payloads.

This is the laptop-side counterpart of app_program/telemetry.c.  It follows
the layout in app_program/telemetry.h exactly: a 1-byte message-type header,
then fields packed MSB first with the widths below, zero-padded to a byte.
Multi-byte fields are therefore big-endian; signed fields are two's
complement in their stated width.

    python3 robot_telemetry.py decode 01 00 64 ...      # hex bytes, spaces optional
    python3 robot_telemetry.py decode 0100640...

Standard library only.
"""
import json
import sys

TYPE_STATUS = 0x01
TYPE_EVENT = 0x02
TYPE_HUMP = 0x03
TYPE_OBSTACLE = 0x04
TYPE_HEARTBEAT = 0x05
TYPE_COMMAND = 0x10
TYPE_ACK = 0x11

STATE_NAMES = ["BOOT", "IDLE", "CALIBRATE", "FOLLOW", "JUNCTION_ADVANCE",
               "JUNCTION_TURN", "REACQUIRE", "SEARCH_LINE", "OBSTACLE_STOP",
               "OBSTACLE_SCAN", "BYPASS", "REVERSE", "RECOVER_LINE", "STOPPED",
               "FAULT", "DONE"]
EVENT_NAMES = {1: "STATE", 2: "BARCODE", 3: "JUNCTION", 4: "NAV_EXEC", 5: "HUMP",
               6: "OBSTACLE", 7: "PLAN", 8: "FAULT", 9: "FAULT_CLEAR",
               10: "LINE_LOST", 11: "LINE_FOUND", 12: "RECOVERED", 13: "DONE"}
FAULT_NAMES = {0: "LINE_STALE", 1: "LINE_CAL", 2: "LINE_LOST", 3: "ENCODER",
               4: "MOTION", 5: "IMU", 6: "ULTRASONIC", 7: "NO_SAFE_PATH",
               8: "RECOVERY_LIMIT", 12: "BARCODE_REJECT", 13: "LINE_OVERFLOW",
               14: "TELEMETRY_DROP", 15: "NET_DOWN"}


class BitWriter:
    def __init__(self):
        self.bits = []

    def put(self, value, width):
        value &= (1 << width) - 1
        for i in range(width - 1, -1, -1):
            self.bits.append((value >> i) & 1)

    def put_signed(self, value, width):
        self.put(value & ((1 << width) - 1), width)

    def to_bytes(self):
        bits = self.bits + [0] * (-len(self.bits) % 8)
        out = bytearray()
        for i in range(0, len(bits), 8):
            byte = 0
            for b in bits[i:i + 8]:
                byte = (byte << 1) | b
            out.append(byte)
        return bytes(out)


class BitReader:
    def __init__(self, data):
        self.data = data
        self.pos = 0

    def get(self, width):
        value = 0
        for _ in range(width):
            byte = self.data[self.pos // 8]
            value = (value << 1) | ((byte >> (7 - self.pos % 8)) & 1)
            self.pos += 1
        return value

    def get_signed(self, width):
        v = self.get(width)
        return v - (1 << width) if v & (1 << (width - 1)) else v


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def encode_status(s):
    """Fields saturate to their width; the 'saturated' bits record that."""
    sat = 0
    pos = clamp(s["line_position"], -1023, 1023)
    if pos != s["line_position"]:
        sat |= 1
    left = clamp(s["left_mm_s"], -511, 511)
    right = clamp(s["right_mm_s"], -511, 511)
    if (left, right) != (s["left_mm_s"], s["right_mm_s"]):
        sat |= 2
    dist = clamp(s["distance_mm"], -32767, 32767)
    if dist != s["distance_mm"]:
        sat |= 4
    cm = clamp(s["obstacle_cm"], 0, 510) if s["obstacle_valid"] else 0
    if s["obstacle_valid"] and cm != s["obstacle_cm"]:
        sat |= 8
    w = BitWriter()
    w.put(TYPE_STATUS, 8)
    w.put(s["ts10"], 16)
    w.put(s["state"], 4)
    w.put(s["nav_cmd"], 3)
    w.put(s["line_state"], 2)
    w.put(s["junction_state"], 2)
    w.put(s["barcode_state"], 2)
    w.put_signed(pos, 11)
    w.put_signed(left, 10)
    w.put_signed(right, 10)
    w.put_signed(dist, 16)
    w.put(s["faults"], 16)
    w.put(1 if s["obstacle_valid"] else 0, 1)
    w.put(cm, 9)
    w.put(s["hump_packed"], 13)
    w.put(s["highest_hump_mm"], 7)
    w.put(sat, 4)
    return w.to_bytes()


def decode_status(data):
    r = BitReader(data)
    if r.get(8) != TYPE_STATUS:
        raise ValueError("not a status message")
    d = {}
    d["ts10"] = r.get(16)
    d["state"] = r.get(4)
    d["nav_cmd"] = r.get(3)
    d["line_state"] = r.get(2)
    d["junction_state"] = r.get(2)
    d["barcode_state"] = r.get(2)
    d["line_position"] = r.get_signed(11)
    d["left_mm_s"] = r.get_signed(10)
    d["right_mm_s"] = r.get_signed(10)
    d["distance_mm"] = r.get_signed(16)
    d["faults"] = r.get(16)
    d["obstacle_valid"] = bool(r.get(1))
    d["obstacle_cm"] = r.get(9)
    d["hump_packed"] = r.get(13)
    d["highest_hump_mm"] = r.get(7)
    d["saturated"] = r.get(4)
    d["state_name"] = STATE_NAMES[d["state"]]
    d["fault_names"] = [n for b, n in FAULT_NAMES.items() if d["faults"] & (1 << b)]
    h = d["hump_packed"]
    d["hump"] = {"status": (h >> 10) & 7, "height_code": (h >> 3) & 0x7F,
                 "validity": (h >> 1) & 3, "imu_fault": bool(h & 1)}
    return d


def encode_event(e):
    w = BitWriter()
    w.put(TYPE_EVENT, 8)
    w.put(e["ts10"], 16)
    w.put(e["code"], 5)
    w.put(e["arg"], 16)
    w.put(e["seq"], 8)
    return w.to_bytes()


def decode_event(data):
    r = BitReader(data)
    if r.get(8) != TYPE_EVENT:
        raise ValueError("not an event message")
    d = {"ts10": r.get(16), "code": r.get(5), "arg": r.get(16), "seq": r.get(8)}
    d["name"] = EVENT_NAMES.get(d["code"], "UNKNOWN")
    return d


def encode_hump(h):
    w = BitWriter()
    w.put(TYPE_HUMP, 8)
    w.put(h["ts10"], 16)
    w.put(h["hump_packed"], 13)
    w.put(h["highest_hump_mm"], 7)
    return w.to_bytes()


def decode_hump(data):
    r = BitReader(data)
    if r.get(8) != TYPE_HUMP:
        raise ValueError("not a hump message")
    return {"ts10": r.get(16), "hump_packed": r.get(13), "highest_hump_mm": r.get(7)}


def encode_obstacle(o):
    w = BitWriter()
    w.put(TYPE_OBSTACLE, 8)
    w.put(o["ts10"], 16)
    w.put(1 if o["valid"] else 0, 1)
    w.put(clamp(o["closest_cm"], 0, 511), 9)
    w.put(clamp(o["width_cm"], 0, 511), 9)
    w.put_signed(clamp(o["centre_cm"], -255, 255), 9)
    w.put(clamp(o["left_clear_cm"], 0, 511), 9)
    w.put(clamp(o["right_clear_cm"], 0, 511), 9)
    w.put(o["action"], 3)
    w.put(o["reason"], 3)
    return w.to_bytes()


def decode_obstacle(data):
    r = BitReader(data)
    if r.get(8) != TYPE_OBSTACLE:
        raise ValueError("not an obstacle message")
    return {"ts10": r.get(16), "valid": bool(r.get(1)), "closest_cm": r.get(9),
            "width_cm": r.get(9), "centre_cm": r.get_signed(9),
            "left_clear_cm": r.get(9), "right_clear_cm": r.get(9),
            "action": r.get(3), "reason": r.get(3)}


def encode_heartbeat(h):
    w = BitWriter()
    w.put(TYPE_HEARTBEAT, 8)
    w.put(h["uptime_s"], 32)
    w.put(h["dropped"], 16)
    w.put(h["reconnects"], 8)
    w.put(h["queue_depth"], 5)
    w.put(h["net_state"], 3)
    return w.to_bytes()


def decode_heartbeat(data):
    r = BitReader(data)
    if r.get(8) != TYPE_HEARTBEAT:
        raise ValueError("not a heartbeat message")
    return {"uptime_s": r.get(32), "dropped": r.get(16), "reconnects": r.get(8),
            "queue_depth": r.get(5), "net_state": r.get(3)}


def encode_command(c):
    w = BitWriter()
    w.put(TYPE_COMMAND, 8)
    w.put(c["req_id"], 8)
    w.put(c["cmd"], 4)
    w.put(c["param"], 16)
    return w.to_bytes()


def decode_ack(data):
    r = BitReader(data)
    if r.get(8) != TYPE_ACK:
        raise ValueError("not an ack message")
    d = {"req_id": r.get(8), "result": r.get(3)}
    d["result_name"] = ["RECEIVED", "COMPLETED", "REJECTED_RUNNING", "INVALID"][d["result"]] \
        if d["result"] < 4 else "UNKNOWN"
    return d


DECODERS = {TYPE_STATUS: decode_status, TYPE_EVENT: decode_event,
            TYPE_HUMP: decode_hump, TYPE_OBSTACLE: decode_obstacle,
            TYPE_HEARTBEAT: decode_heartbeat, TYPE_ACK: decode_ack}


def decode(data):
    """Decode any robot -> laptop message."""
    if not data or data[0] not in DECODERS:
        raise ValueError("unknown message type")
    return DECODERS[data[0]](data)


def main(argv):
    if len(argv) >= 3 and argv[1] == "decode":
        text = "".join(argv[2:]).replace("0x", "").replace(",", "")
        print(json.dumps(decode(bytes.fromhex(text)), indent=2))
        return 0
    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
