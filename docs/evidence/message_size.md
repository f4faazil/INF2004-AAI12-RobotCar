# Evidence: message size before and after (Week 13 item 3)

The same eight records in three representations: **packed** (what the firmware sends), **natural-width binary** (the same fields as plain
C `uint8/16/32` and one byte per boolean, no padding, with the 1-byte type header) and **compact text** (JSON, counting only the raw numeric and boolean
fields, so real JSON would be larger). Because telemetry was packed from the start, natural-width binary and text are the "unpacked" baselines
that the packed form is compared with.

How the numbers were obtained:
- **Packed:** the length of the golden records produced by the C encoders and checked by the unit tests (`tests/host/test_telemetry.c`, `dashboard/test/robot-frames.test.js`).
- **Natural width:** `sizeof` of the record structs on the Cortex-M0+ compiled with `arm-none-eabi-gcc -fpack-struct=1`: status 26, event 6, hump 5, obstacle 15, heartbeat 9, sensors 54, command 4, ack 2 bytes, plus 1 byte of type.
- **Text:** `dashboard/tools/message-size-report.mjs` serialises each decoded record.
- **MQTT packet:** adds the fixed header, remaining length and the 21-character topic `aai12/robot/telemetry` (25 to 26 bytes). TCP/IP headers are not counted.

## Result

| Record | Packed (bytes) | Natural-width binary | Compact text | Packed saves vs natural | Packed saves vs text | MQTT packet, packed | MQTT packet, text |
|---|---|---|---|---|---|---|---|
| status | 17 | 27 | 259 | 37 % smaller | 93 % smaller | 42 | 285 |
| event | 7 | 7 | 38 | same | 82 % smaller | 32 | 63 |
| hump | 6 | 6 | 49 | same | 88 % smaller | 31 | 74 |
| obstacle | 10 | 16 | 124 | 38 % smaller | 92 % smaller | 35 | 149 |
| heartbeat | 9 | 10 | 71 | 10 % smaller | 87 % smaller | 34 | 96 |
| sensors | 18 | 55 | 475 | 67 % smaller | 96 % smaller | 43 | 501 |
| command | 5 | 5 | 29 | same | 83 % smaller | 30 | 54 |
| ack | 3 | 3 | 22 | same | 86 % smaller | 28 | 47 |
| **all 8 records** | **75** | **129** | **1067** | **42 % smaller** | **93 % smaller** | | |

Steady stream (status 10 Hz + sensors 2 Hz + heartbeat 1 Hz), payload only: packed 215 B/s, natural 390 B/s, text 3611 B/s.


## The other MQTT payloads (binary since v23)

| Payload | Before (text) | Now (binary) | Note |
|---|---|---|---|
| Barcode map, 3 letters (`A=L,B=R,F=U`) | 11 B | 4 B | type 0x12 plus one byte per letter |
| Barcode map, all 26 letters | up to 100 B | 27 B | |
| Map reply (`OK n=3`, `ERR duplicate`) | 6 to 13 B | 2 B | type 0x13 plus result (3 bits) and count (5 bits) |
| Heartbeat | 25 to 43 B | 13 B | text length grows with the numbers; shown here from all-zero to all-ten-digit values |
| Online marker | 19 B | 1 B | `AAI12_PICO_W_ONLINE` is now the byte 0x22 |
| Diagnostic part 1 | 81 to 207 B | 50 B | same range of values |
| Diagnostic part 2 | 84 to 210 B | 58 B | |

The binary sizes are the lengths asserted in the firmware tests (`tests/host/test_barcode_map.c`, `tests/test_mqtt_heartbeat.c`, `tests/test_mqtt_lwip.c`). The text sizes
are those of the old text formats, computed for the smallest and the largest values the fields can hold. The diagnostic parts still carry full 32-bit counters, so they are
compact rather than minimum-bit; they are debug output that is sent once per connection.

## What this shows
- The large records shrink a lot: the per-sensor record is **67 % smaller** than natural-width binary (18 B against 55 B) and 96 % smaller than text; the status record is 37 % smaller.
- The small records (event, hump, command, ack) are **already as small as natural width**: their fields are bytes or 16-bit values anyway. Packing gains nothing there.
- Against text the packed form is 82 to 96 % smaller, and one status record fits in a 42-byte MQTT packet instead of 285.
- Over all eight records: 75 bytes packed, 129 natural-width binary (**42 % smaller**), 1067 compact text (**93 % smaller**).
- At the current rates (status 10 per second, sensors 2, heartbeat 1) the car sends about 215 bytes per second of payload packed, against 390 natural-width and 3611 as text.

## Limits (stated plainly)
- The text baseline counts only raw fields and no spaces, so it understates real JSON; the saving against text is, if anything, larger.
- Natural width and text are reconstructions made for this comparison, not firmware that ever ran. The packed numbers are from the real encoders.
- The table above is the robot's own records. The other MQTT payloads are in the second table below.
- These are message sizes only. Timing before and after is in the timing evidence.
