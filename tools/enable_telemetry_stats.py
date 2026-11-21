#!/usr/bin/env python3
"""Stage 5D1: patch queue diagnostics without including standard headers in app_main."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
APP = ROOT / 'app_program' / 'app_main.c'
RTOS = ROOT / 'app_program' / 'telemetry_rtos.c'
PORT = ROOT / 'app_program' / 'telemetry_tkernel_port.h'

APP_ANCHOR = '/* One line every REPORT_MS cannot overrun a 4 KB ring, but say'
APP_BLOCK = '''#if TM_WIFI_MQTT
            /* Stage 5D1: best-effort, non-waiting stats snapshot.
             * No mutex is held while printing. */
            {
                unsigned int queued = 0U;
                unsigned int full = 0U;
                unsigned int stale = 0U;
                if (telemetry_rtos_stats_copy(&queued, &full, &stale)) {
                    tm_printf((UB *)"[telemetry-stats] queued=%u full=%u stale=%u\\n",
                              queued, full, stale);
                }
            }
#endif

            '''
RTOS_ANCHOR = '/* Small startup ABI for app_main.c'
RTOS_BLOCK = '''/* Narrow stats ABI: app_main.c never needs stddef.h or telemetry_inbox.h.
 * An unavailable mutex gives a skipped sample rather than blocking. */
int telemetry_rtos_stats_copy(unsigned int *queued,
                              unsigned int *dropped_full,
                              unsigned int *dropped_stale)
{
    telemetry_inbox_stats_t snapshot;
    if (queued == NULL || dropped_full == NULL || dropped_stale == NULL) {
        return 0;
    }
    if (telemetry_rtos_snapshot(&snapshot) != TELEMETRY_INBOX_OK) {
        return 0;
    }
    *queued = (unsigned int)snapshot.queued;
    *dropped_full = (unsigned int)snapshot.dropped_full;
    *dropped_stale = (unsigned int)snapshot.dropped_stale;
    return 1;
}

'''
PORT_ANCHOR = 'int telemetry_rtos_boot_init(void);'
PORT_BLOCK = '''
/* Non-waiting queue statistics snapshot. Returns 1 on success. */
int telemetry_rtos_stats_copy(unsigned int *queued,
                              unsigned int *dropped_full,
                              unsigned int *dropped_stale);'''


def insert_once(source, anchor, block, before=False):
    if block in source:
        return source, False
    if source.count(anchor) != 1:
        raise ValueError(f'expected anchor exactly once: {anchor!r}, got {source.count(anchor)}')
    return (source.replace(anchor, block + anchor if before else anchor + block, 1), True)


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--check', action='store_true')
    group.add_argument('--apply', action='store_true')
    opts = parser.parse_args()
    for f in (APP, RTOS, PORT):
        if not f.is_file():
            print(f'ERROR: missing {f}; no files modified', file=sys.stderr)
            return 1
    old = {f: f.read_text() for f in (APP, RTOS, PORT)}
    try:
        new_app, ca = insert_once(old[APP], APP_ANCHOR, APP_BLOCK, before=True)
        new_rtos, cr = insert_once(old[RTOS], RTOS_ANCHOR, RTOS_BLOCK, before=True)
        new_port, ch = insert_once(old[PORT], PORT_ANCHOR, PORT_BLOCK)
    except ValueError as e:
        print(f'ERROR: {e}; no files modified', file=sys.stderr)
        return 1
    changes = {APP: (new_app, ca), RTOS: (new_rtos, cr), PORT: (new_port, ch)}
    for f, (_, modified) in changes.items():
        print(f'{f.relative_to(ROOT)}: {"needs update" if modified else "already patched"}')
    if opts.check:
        print('Preflight passed; no files modified.')
        return 0
    for f, (new_data, modified) in changes.items():
        if modified:
            f.write_text(new_data)
    print('Stage 5D1 telemetry stats monitor patch applied.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
