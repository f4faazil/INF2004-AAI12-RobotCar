# Evidence: RAM before and after (Week 13 item 2)

Static RAM of the robot firmware (WiFi + MQTT build) measured with `arm-none-eabi-size` and the linker map, **before** and **after** the change.

| | Commit | Source |
|---|---|---|
| Before | `e8256a2` | byte-identical to the v21 firmware that was tested on the car |
| After | `25a6a25` | the telemetry queue slots resized to the real record size |

Same toolchain (Arm GNU 15.2), same flags: `SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_MQTT=1`.

## What changed
The telemetry queue and the radio's telemetry inbox both held 128-byte records, but the largest record the firmware sends is 18 bytes (the per-sensor
record) and the demo frames are 3. Both slots are now 32 bytes (`TEL_RECORD_MAX`, `TELEMETRY_QUEUE_MAX_BYTES`). The boundary tests use the macro.

## Result

| Measure | Before | After | Change |
|---|---|---|---|
| `.bss` (zero-initialised RAM) | 67 356 B | 64 212 B | **-3 144 B (-4.7 %)** |
| `.data` (initialised RAM) | 2 700 B | 2 700 B | 0 |
| `.text` (flash) | 378 957 B | 377 893 B | -1 064 B |
| Robot telemetry queue `g_tel_q` | 2 192 B | 656 B | -1 536 B (-70 %) |
| Radio telemetry inbox `shared_inbox` | 2 204 B | 668 B | -1 536 B (-70 %) |
| Application static RAM (`app_program/*.o`) | 14 482 B | 11 338 B | **-3 144 B (-22 %)** |
| Static RAM of the whole image | 69 914 B | 66 770 B | -4.5 % |

## Where the application's RAM goes now

| Item | Size | Note |
|---|---|---|
| `g_ring` (IR frames, 256 x 12 B) | 3072 B | not packed by team decision: written by the 1 ms interrupt, raw ADC kept (Tribunal 1) |
| `g_status` (radio driver status copy) | 2576 B | the radio driver's own structure, not application data |
| `g_tel_q` | 656 B | was 2192 B |
| `shared_inbox` | 668 B | was 2204 B |
| `g_rx` (MQTT mailbox) | 648 B | still sized for 128 bytes; the binary map needs at most 27, so this could shrink (not done) |
| `nav` / `motion` / `scan` / `vc` structs | 636 / 464 / 404 / 264 B | module state; packing their flags would save up to about 60 B in total (see `DATA_REPRESENTATION.md`, section 3) |

## What this does and does not show
- The saving is real and measured: 3 072 bytes from the two queues (2 x 1 536 B), plus 72 bytes from storing the letter map packed (v23), 3 144 bytes in all. `.text` fell by 1 064 bytes as well, because the text builders for the heartbeat and diagnostics were removed.
- It is a reduction of the **queue slot size**, a data-representation decision (the old slots were sized for a 128-byte limit the data never needs), and not a change to any message.
- **There was no unpacked "before" build of the firmware**: telemetry was packed from the start. So this compares the shipped firmware against the improved one. The unpacked
  comparison for the *messages* is in `message_size.md`.
- Functionality is unchanged: all host tests pass, including the telemetry round-trip tests and the queue boundary tests.

## Reproduce
```
cd build_make
mingw32-make -j8 PICO_SDK_PATH=<sdk> SMP=1 CONSOLE=usb_cdc WIFI=cyw43 WIFI_JOIN=1 WIFI_NETIF=1 WIFI_DHCP=1 WIFI_MQTT=1
arm-none-eabi-size mtk3pico_smp1_usb_cdc_wifi_mqtt.elf
arm-none-eabi-nm -S --size-sort -t d mtk3pico_smp1_usb_cdc_wifi_mqtt.elf    # per-symbol sizes
```
(needs the two local config files; see `docs/INTEGRATION.md`).
