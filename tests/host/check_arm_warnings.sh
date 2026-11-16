#!/usr/bin/env bash
# Compile every robot source file with the ARM cross compiler and strict
# warnings, so a change that is clean on the PC but warns on the target is
# caught.  Prints nothing but "OK" when clean; exits non-zero on any warning.
#
#   tests/host/check_arm_warnings.sh
set -u
cd "$(dirname "$0")/../.." || exit 2
FLAGS="-mcpu=cortex-m0plus -mthumb -ffreestanding -std=gnu11 -O2 -mfloat-abi=soft \
 -D_PICO_RP2040_ -Iinclude -Iconfig -Ikernel/knlinc"
PORTABLE="barcode line_sensor fault_mask encoder pid motion imu ultrasonic servo obstacle \
line_nav telemetry mqtt_codec mqtt_client safety vehicle"
GLUE="hal_rp2040 robot_tasks robot_main robot_net robot_config"
bad=0
for f in $PORTABLE; do
  out=$(arm-none-eabi-gcc $FLAGS -Wall -Wextra -Wshadow -Wconversion -Wpedantic -Wcast-align \
        -Wundef -c "app_program/$f.c" -o /dev/null 2>&1 | grep -E "warning|error")
  [ -n "$out" ] && { echo "== $f"; echo "$out"; bad=1; }
done
for f in $GLUE; do
  out=$(arm-none-eabi-gcc $FLAGS -Wall -Wextra -Wshadow -c "app_program/$f.c" -o /dev/null 2>&1 \
        | grep -E "warning|error")
  [ -n "$out" ] && { echo "== $f"; echo "$out"; bad=1; }
done
[ "$bad" -eq 0 ] && echo OK
exit "$bad"
