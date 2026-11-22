#!/usr/bin/env python3
"""Generate the telemetry part of docs/DATA_REPRESENTATION.md from the firmware source.

Reads the width macros in app_program/telemetry.h and the field order of every tel_encode_*() in app_program/telemetry.c, so the
table cannot drift from the code. Each record's size is checked against the bytes the unit tests use (the golden records).

    python3 tools/gen_data_table.py            prints the markdown
    python3 tools/gen_data_table.py --check    only verifies sizes (exit 1 on a mismatch)
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
H = open(os.path.join(ROOT, "app_program", "telemetry.h"), encoding="utf-8").read()
C = open(os.path.join(ROOT, "app_program", "telemetry.c"), encoding="utf-8").read()

# bytes of the firmware's own records, as checked by the unit tests (dashboard/test/robot-frames.test.js, tests/host/test_telemetry.c)
GOLDEN_BYTES = {"status": 17, "event": 7, "hump": 6, "obstacle": 10, "heartbeat": 9, "sensors": 18, "command": 5, "ack": 3}

macros = {}
for m in re.finditer(r"#define\s+(TEL_\w+)\s+\((\d+)U\)[ \t]*(?:/\*\s*(.*?)\s*\*/)?", H):
    macros[m.group(1)] = (int(m.group(2)), (m.group(3) or "").strip())


def width_of(token):
    token = token.strip()
    if token in macros:
        return macros[token][0], macros[token][1]
    m = re.fullmatch(r"\(?(\d+)U?\)?", token)
    if m:
        return int(m.group(1)), ""
    return None, ""


def field_name(expr):
    expr = expr.strip()
    if expr.startswith("TEL_TYPE_"):
        return "type"
    expr = expr.split("?")[0].strip()
    names = re.findall(r"[A-Za-z_]\w*", expr)
    names = [n for n in names if n not in ("uint32_t", "uint8_t", "int32_t", "int16_t", "uint16_t", "cm")] or ["cm"]
    n = names[-1]
    return {"sat": "saturated", "cm": "obstacle_cm", "pos": "line_position", "left": "left_mm_s", "right": "right_mm_s", "dist": "distance_mm"}.get(n, n)


records = {}
for fm in re.finditer(r"uint16_t tel_encode_(\w+)\([^)]*\)\s*\{(.*?)\n\}\n", C, re.S):
    name, body = fm.group(1), fm.group(2)
    fields = []
    # bitw_put / bitw_put_signed with an explicit width, and put_count(): a counter clipped at TEL_COUNT_MAX and flagged in `saturated`
    for pm in re.finditer(r"bitw_put(_signed)?\(&w,\s*(.+?),\s*([A-Za-z_0-9]+)\);|put_count\(&w,\s*(.+?),\s*&sat\)", body):
        if pm.group(4) is not None:
            bits = macros["TEL_COUNT_BITS"][0]
            fields.append((field_name(pm.group(4)), bits, False, f"saturating counter: clipped at {(1 << bits) - 1}, the record's saturated bit says so"))
            continue
        bits, note = width_of(pm.group(3))
        fields.append((field_name(pm.group(2)), bits, bool(pm.group(1)), note))
    if fields:
        records[name] = fields

ok = True
lines = []
for name in GOLDEN_BYTES:
    fields = records.get(name)
    if not fields or any(f[1] is None for f in fields):
        lines.append(f"<!-- {name}: could not be read from the encoder -->")
        ok = False
        continue
    total = sum(f[1] for f in fields)
    size = (total + 7) // 8
    good = size == GOLDEN_BYTES[name]
    ok &= good
    lines.append(f"#### `{name}` record: {total} bits = {size} bytes" + ("" if good else f"  **(MISMATCH: tests use {GOLDEN_BYTES[name]})**"))
    lines.append("")
    lines.append("| Field | Bits | Representation | Range this width holds | Note from the source |")
    lines.append("|---|---|---|---|---|")
    for fname, bits, signed, note in fields:
        rng = f"{-(1 << (bits - 1))} to {(1 << (bits - 1)) - 1}" if signed else f"0 to {(1 << bits) - 1}"
        lines.append(f"| {fname} | {bits} | {'signed' if signed else 'unsigned'}, MSB first | {rng} | {note} |")
    pad = size * 8 - total
    lines.append(f"| (padding) | {pad} | zero bits to the next byte | | |")
    lines.append("")

if "--check" in sys.argv:
    print("sizes match the golden records" if ok else "MISMATCH between the encoders and the golden record sizes")
    sys.exit(0 if ok else 1)
print("\n".join(lines))
sys.exit(0 if ok else 1)
