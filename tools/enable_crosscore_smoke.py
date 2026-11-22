#!/usr/bin/env python3
"""Opt-in, reversible processor-2 telemetry demonstration in app_main.c.

Run from repository root:
  python3 tools/enable_crosscore_smoke.py --check
  python3 tools/enable_crosscore_smoke.py --apply
  python3 tools/enable_crosscore_smoke.py --undo

Only edits two anchored locations in app_program/app_main.c; never touches
credential files or replaces the professor's application skeleton.
"""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
TARGET = ROOT / "app_program" / "app_main.c"

INCLUDE_ANCHOR = '#include "demo_tasks.h"\n'
INCLUDE_BLOCK = '''\n#if TM_WIFI_MQTT && TK_SUPPORT_SMP
/* Stage 5C5 hardware QUALIFICATION ONLY. Remove before robot integration. */
#include "telemetry_crosscore_demo.h"
#endif
'''

CONSUMER_ANCHOR = '\t\tconsumed++;\n\t\tlast_seen_on = THIS_PRC();\n\t\ttk_set_flg(flgid, FLG_CONSUMED);\n'
CONSUMER_BLOCK = '''
#if TM_WIFI_MQTT && TK_SUPPORT_SMP
        /* TEMPORARY cross-core test: the consumer is pinned to processor 2.
         * This only submits encoded bytes to a non-waiting protected inbox.
         * All network calls remain in the processor-1 CYW43 task. */
        if (THIS_PRC() == 2) {
            const int r = telemetry_crosscore_demo_emit((unsigned int)msg.seq);
            if (msg.seq <= 2U || (msg.seq % 20U) == 0U || r != 0) {
                tm_printf((UB *)"[telemetry-demo] prc=%d seq=%u submit=%d\\n",
                          THIS_PRC(), msg.seq, r);
            }
        }
#endif
'''

def apply_block(src, anchor, block, label):
    if block in src:
        return src, False
    if src.count(anchor) != 1:
        raise ValueError(f"{label}: expected anchor once, found {src.count(anchor)}")
    return src.replace(anchor, anchor + block, 1), True


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--apply', action='store_true')
    mode.add_argument('--undo', action='store_true')
    args = parser.parse_args()
    if not TARGET.is_file():
        print(f"ERROR: missing {TARGET}", file=sys.stderr)
        return 1
    original = TARGET.read_text()
    if args.undo:
        if (INCLUDE_BLOCK in original) != (CONSUMER_BLOCK in original):
            print("ERROR: partial/mismatched patch; no changes made", file=sys.stderr)
            return 1
        updated = original.replace(INCLUDE_BLOCK, '', 1).replace(CONSUMER_BLOCK, '', 1)
        if updated != original:
            TARGET.write_text(updated)
            print("Cross-core smoke hook removed from app_main.c")
        else:
            print("No cross-core smoke hook present")
        return 0
    try:
        updated, c1 = apply_block(original, INCLUDE_ANCHOR, INCLUDE_BLOCK, 'include')
        updated, c2 = apply_block(updated, CONSUMER_ANCHOR, CONSUMER_BLOCK, 'consumer')
    except ValueError as exc:
        print(f"ERROR: {exc}; no files modified", file=sys.stderr)
        return 1
    if args.check:
        print("Preflight passed; no files modified.")
        print("app_program/app_main.c:", "needs update" if c1 or c2 else "already patched")
        return 0
    if c1 or c2:
        TARGET.write_text(updated)
        print("Cross-core demo hook applied to app_main.c only.")
    else:
        print("Cross-core demo hook already applied; no changes made.")
    return 0

if __name__ == '__main__':
    sys.exit(main())
