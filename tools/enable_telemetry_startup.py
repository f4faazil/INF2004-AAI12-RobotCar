#!/usr/bin/env python3
"""Apply a tiny, idempotent, fail-closed startup patch to app_main.c.

Run from repo root:
  python3 tools/enable_telemetry_startup.py --check
  python3 tools/enable_telemetry_startup.py --apply

Only writes app_program/app_main.c, preserving all other content, including
local DHCP diagnostics. Does not inspect private WiFi credentials.
"""
from pathlib import Path
import argparse
import sys

ROOT = Path(__file__).resolve().parent.parent
TARGET = ROOT / "app_program" / "app_main.c"

INCLUDE_ANCHOR = '#include "demo_tasks.h"\n'
INCLUDE_PATCH = INCLUDE_ANCHOR + '''\n#if TM_WIFI_MQTT
/* Narrow startup ABI: avoids the micro T-Kernel size_t header clash. */
#include "telemetry_tkernel_port.h"
#endif
'''

BOOT_ANCHOR = 'EXPORT INT usermain(void)\n{\n\tID\ttid;\n'
BOOT_PATCH = BOOT_ANCHOR + '''
#if TM_WIFI_MQTT
    /* Create queue mutex BEFORE any radio/producer task starts. */
    if (!telemetry_rtos_boot_init()) {
        tm_printf((UB *)"[init] telemetry inbox mutex creation failed\\n");
        return 1;
    }
#endif
'''


def patch_once(src, anchor, replacement, label):
    if replacement in src:
        return src, False
    n = src.count(anchor)
    if n != 1:
        raise ValueError(f"{label}: expected anchor once, got {n}; no changes made")
    return src.replace(anchor, replacement, 1), True


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--check', action='store_true')
    group.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    if not TARGET.is_file():
        print(f"ERROR: Missing {TARGET}", file=sys.stderr)
        return 1
    original = TARGET.read_text()
    try:
        updated, change1 = patch_once(original, INCLUDE_ANCHOR, INCLUDE_PATCH, 'include')
        updated, change2 = patch_once(updated, BOOT_ANCHOR, BOOT_PATCH, 'usermain')
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    if args.check:
        print('Preflight passed; no files modified.')
        print('app_program/app_main.c:', 'needs update' if change1 or change2 else 'already patched')
        return 0
    if change1 or change2:
        TARGET.write_text(updated)
        print('Applied guarded telemetry startup to app_main.c only.')
    else:
        print('Already applied; no files modified.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
