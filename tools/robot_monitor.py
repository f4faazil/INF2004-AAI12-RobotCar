#!/usr/bin/env python3
"""Laptop-side monitor for the robot's MQTT telemetry (standard library only).

Connects to the broker, subscribes to <base>/#, decodes the robot's binary
messages with robot_telemetry.py and prints them.  It can also send one of the
permitted commands (the robot refuses START / CALIBRATE / CLEAR_FAULTS while a
mission is running, and no command ever drives the wheels).

    python3 robot_monitor.py --broker 192.168.1.50
    python3 robot_monitor.py --broker 192.168.1.50 --send stop
    python3 robot_monitor.py --broker 192.168.1.50 --send start --req 7

Topics (base aai12/robot, see app_program/robot_config.h):
    <base>/telemetry   status, hump, obstacle, heartbeat (binary)
    <base>/event       events (binary)
    <base>/ack         command acknowledgements (binary)
    <base>/status      "online" / "offline" (retained text, the MQTT will)
    <base>/cmd         commands to the robot (binary)
"""
import argparse
import json
import socket
import sys
import time

import robot_telemetry as telem

BASE = "aai12/robot"
COMMANDS = {"stop": 1, "start": 2, "calibrate": 3, "clear_faults": 4, "status": 5}


def _remaining_length(n):
    out = bytearray()
    while True:
        digit = n % 128
        n //= 128
        out.append(digit | (0x80 if n else 0))
        if not n:
            return bytes(out)


def _string(s):
    b = s.encode()
    return len(b).to_bytes(2, "big") + b


def encode_connect(client_id, keepalive=30):
    body = _string("MQTT") + bytes([4, 0x02]) + keepalive.to_bytes(2, "big") + _string(client_id)
    return bytes([0x10]) + _remaining_length(len(body)) + body


def encode_subscribe(packet_id, topic_filter):
    body = packet_id.to_bytes(2, "big") + _string(topic_filter) + bytes([0])
    return bytes([0x82]) + _remaining_length(len(body)) + body


def encode_publish(topic, payload):
    body = _string(topic) + payload
    return bytes([0x30]) + _remaining_length(len(body)) + body


def encode_pingreq():
    return bytes([0xC0, 0x00])


class PacketReader:
    """Splits an MQTT byte stream into (type, flags, body) packets."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        while True:
            if len(self.buf) < 2:
                return
            mult, length, i = 1, 0, 1
            while True:
                if i >= len(self.buf):
                    return
                b = self.buf[i]
                length += (b & 0x7F) * mult
                mult *= 128
                i += 1
                if not b & 0x80:
                    break
                if i > 4:
                    raise ValueError("malformed remaining length")
            if len(self.buf) < i + length:
                return
            first = self.buf[0]
            body = bytes(self.buf[i:i + length])
            del self.buf[:i + length]
            yield first >> 4, first & 0x0F, body


def parse_publish(flags, body):
    topic_len = int.from_bytes(body[:2], "big")
    topic = body[2:2 + topic_len].decode()
    pos = 2 + topic_len
    if (flags >> 1) & 3:
        pos += 2
    return topic, body[pos:]


def describe(topic, payload):
    if topic.endswith("/status"):
        return {"status": payload.decode(errors="replace")}
    try:
        return telem.decode(payload)
    except (ValueError, IndexError) as exc:
        return {"undecodable": payload.hex(), "reason": str(exc)}


def run(host, port, base, send, req, seconds, out=sys.stdout):
    sock = socket.create_connection((host, port), timeout=5)
    sock.settimeout(0.25)             # short, so a --seconds deadline is honoured
    reader = PacketReader()
    sock.sendall(encode_connect("aai12-laptop"))
    sock.sendall(encode_subscribe(1, base + "/#"))
    if send:
        payload = telem.encode_command({"req_id": req & 0xFF, "cmd": COMMANDS[send], "param": 0})
        sock.sendall(encode_publish(base + "/cmd", payload))
        print("sent %s (req %d)" % (send, req), file=out)
    deadline = time.time() + seconds if seconds else None
    last_ping = time.time()
    while deadline is None or time.time() < deadline:
        try:
            data = sock.recv(1024)
            if not data:
                print("broker closed the connection", file=out)
                return 1
        except socket.timeout:
            data = b""
        for ptype, flags, body in reader.feed(data):
            if ptype == 3:
                topic, payload = parse_publish(flags, body)
                print(json.dumps({"topic": topic, "msg": describe(topic, payload)}), file=out)
        if time.time() - last_ping > 15:
            sock.sendall(encode_pingreq())
            last_ping = time.time()
    return 0


def main(argv):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument("--broker", required=True, help="broker IPv4 address or name")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--base", default=BASE)
    p.add_argument("--send", choices=sorted(COMMANDS), help="send a command, then keep listening")
    p.add_argument("--req", type=int, default=1, help="request id carried in the command")
    p.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = run until Ctrl-C)")
    a = p.parse_args(argv)
    try:
        return run(a.broker, a.port, a.base, a.send, a.req, a.seconds)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
