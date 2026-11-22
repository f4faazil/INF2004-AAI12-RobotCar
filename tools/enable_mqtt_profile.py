#!/usr/bin/env python3
"""Opt-in, fail-closed patch for the AAI12 lecturer micro T-Kernel template.

Run from repository root:
    python3 tools/enable_mqtt_profile.py --check
    python3 tools/enable_mqtt_profile.py --apply

Only edits build_make/pico_rp2040.mk, lwipopts.h, and cyw43_utk.c.
No repository/network operations, no generated binaries, no passwords read.
"""

from pathlib import Path
import argparse
import sys

ROOT = Path(__file__).resolve().parent.parent


def transform_mk(src: str) -> str:
    substitutions = [
        (
            "ifeq ($(WIFI_NETIF),1)\nifneq ($(WIFI_JOIN),1)",
            """# AAI12 Buddy 1: opt-in MQTT TCP client. Do not enable the echo TCP
# test at the same time: that client is sequenced after the UDP test.
WIFI_MQTT ?= 0
ifneq ($(filter $(WIFI_MQTT),0 1),$(WIFI_MQTT))
$(error Unknown WIFI_MQTT '$(WIFI_MQTT)'; use 0 or 1)
endif
ifeq ($(WIFI_MQTT),1)
ifneq ($(WIFI_DHCP),1)
$(error WIFI_MQTT=1 requires WIFI_DHCP=1)
endif
ifneq ($(WIFI_TCP),0)
$(error WIFI_MQTT=1 currently requires WIFI_TCP=0)
endif
ifeq ($(wildcard ../config/mqtt_test_config.h),)
$(error WIFI_MQTT=1 requires config/mqtt_test_config.h)
endif
endif

ifeq ($(WIFI_NETIF),1)
ifneq ($(WIFI_JOIN),1)""",
        ),
        (
            "ifeq ($(WIFI_TCPBULK),1)\nWIFI_SUFFIX := _wifi_tcpbulk\nendif",
            """ifeq ($(WIFI_TCPBULK),1)
WIFI_SUFFIX := _wifi_tcpbulk
endif
ifeq ($(WIFI_MQTT),1)
WIFI_SUFFIX := _wifi_mqtt
endif""",
        ),
        (
            "-tcp$(WIFI_TCP)-bulk$(WIFI_TCPBULK)",
            "-tcp$(WIFI_TCP)-bulk$(WIFI_TCPBULK)-mqtt$(WIFI_MQTT)",
        ),
        (
            "ifeq ($(WIFI_TCPBULK),1)\nLWIP_PORT_OBJS += ./mtkernel_3/lib/libnet/lwip/lwip_utk_tcpbulk.o\nendif",
            """ifeq ($(WIFI_TCPBULK),1)
LWIP_PORT_OBJS += ./mtkernel_3/lib/libnet/lwip/lwip_utk_tcpbulk.o
endif
ifeq ($(WIFI_MQTT),1)
LWIP_PORT_OBJS += ./mtkernel_3/lib/libnet/lwip/mqtt_lwip.o
endif""",
        ),
        (
            'WIFI_INCLUDES := -I"$(WIFIDIR)" \\\n',
            'WIFI_INCLUDES := -I"$(WIFIDIR)" \\\n                 -I"../app_program" \\\n',
        ),
        (
            '-DTM_WIFI_TCP=$(WIFI_TCP) -DTM_WIFI_TCPBULK=$(WIFI_TCPBULK) \\',
            '-DTM_WIFI_TCP=$(WIFI_TCP) -DTM_WIFI_TCPBULK=$(WIFI_TCPBULK) \\\n          -DTM_WIFI_MQTT=$(WIFI_MQTT) \\',
        ),
    ]
    return replace_exact(src, substitutions, "pico_rp2040.mk")


def transform_lwipopts(src: str) -> str:
    substitutions = [
        (
            "#define LWIP_TCP                        TM_WIFI_TCP",
            "#define LWIP_TCP                        (TM_WIFI_TCP || TM_WIFI_MQTT)",
        ),
        (
            "#if TM_WIFI_TCP\n",
            "#if TM_WIFI_TCP || TM_WIFI_MQTT\n",
        ),
    ]
    return replace_exact(src, substitutions, "lwipopts.h", counts=[1, 2])


def transform_cyw43(src: str) -> str:
    substitutions = [
        (
            '#include "cyw43_utk.h"\n',
            '#include "cyw43_utk.h"\n#if TM_WIFI_MQTT\n#include "mqtt_lwip.h"\n#endif\n',
        ),
        (
            '#if TM_WIFI_DHCP\n        update_dhcp_status();\n#endif\n',
            '#if TM_WIFI_DHCP\n        update_dhcp_status();\n#endif\n'
            '#if TM_WIFI_MQTT\n'
            '        mqtt_lwip_poll(&cyw43_state.netif[CYW43_ITF_STA]);\n'
            '#endif\n',
        ),
    ]
    return replace_exact(src, substitutions, "cyw43_utk.c")


def replace_exact(src: str, changes, name: str, counts=None) -> str:
    for i, (old, new) in enumerate(changes):
        count = (counts[i] if counts else 1)
        if new in src:
            pass  # already applied, safe to re-run
        elif src.count(old) == count:
            src = src.replace(old, new)
        else:
            raise ValueError(
                f"{name}: expected anchor {i + 1} exactly {count} time(s), "
                f"got {src.count(old)}. No files will be changed."
            )
    return src


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--check", action="store_true")
    group.add_argument("--apply", action="store_true")
    args = parser.parse_args()

    files = [
        (ROOT / "build_make/pico_rp2040.mk", transform_mk),
        (ROOT / "lib/libnet/lwip/include/lwipopts.h", transform_lwipopts),
        (ROOT / "lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c", transform_cyw43),
    ]
    pending = []
    try:
        for path, fn in files:
            old = path.read_text(encoding="utf-8")
            new = fn(old)
            pending.append((path, old, new))
    except (OSError, ValueError) as exc:
        print(f"STOP: {exc}", file=sys.stderr)
        return 1

    for path, old, new in pending:
        status = "needs update" if old != new else "already updated"
        print(f"{path.relative_to(ROOT)}: {status}")
    if args.apply:
        for path, old, new in pending:
            if old != new:
                path.write_text(new, encoding="utf-8")
        print("Applied opt-in MQTT profile to local working tree only.")
    else:
        print("Preflight passed; no files modified.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
