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


## What this shows
- The large records shrink a lot: the per-sensor record is **67 % smaller** than natural-width binary (18 B against 55 B) and 96 % smaller than text; the status record is 37 % smaller.
- The small records (event, hump, command, ack) are **already as small as natural width**: their fields are bytes or 16-bit values anyway. Packing gains nothing there.
- Against text the packed form is 82 to 96 % smaller, and one status record fits in a 42-byte MQTT packet instead of 285.
- Over all eight records: 75 bytes packed, 129 natural-width binary (**42 % smaller**), 1067 compact text (**93 % smaller**).
- At the current rates (status 10 per second, sensors 2, heartbeat 1) the car sends about 215 bytes per second of payload packed, against 390 natural-width and 3611 as text.

## Limits (stated plainly)
- The text baseline counts only raw fields and no spaces, so it understates real JSON; the saving against text is, if anything, larger.
- Natural width and text are reconstructions made for this comparison, not firmware that ever ran. The packed numbers are from the real encoders.
- The barcode map (`A=L,B=R,F=U`, up to 128 bytes) and the radio diagnostic and heartbeat topics are text and are not covered by this table; see `DATA_REPRESENTATION.md`, section 6.
- These are message sizes only. Timing before and after is in the timing evidence.
