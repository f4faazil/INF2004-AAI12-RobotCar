# Evidence: firmware v23 on the car (binary map, heartbeat, marker and diagnostics)

Date: 22 November 2026. The car was stationary (idle, wheels not driven) on the USB cable, joined to the PC's WiFi hotspot, with the dashboard's own MQTT broker
on the PC. Firmware: `robot_wifi_telemetry_v23_binary_fazfaz_smp1.uf2` (branch `evidence/min-bit`, commit cde6ae6). Nothing was driven; no START was sent.

## What was checked and what came back

| Check | Result |
|---|---|
| Flash v23 over the BOOTSEL drive, car restarts | Boots, WiFi joins the hotspot, DHCP address, MQTT online (serial `mqtt=3`) |
| Online marker (1 byte, `0x22`) | Decoded by the dashboard as `AAI12_PICO_W_ONLINE` |
| Diagnostic part 1 (50 B, `0x23`) and part 2 (58 B, `0x24`) | Both decoded to the named fields; boot nonce `cda5a5a4` matches the heartbeat's boot id |
| Heartbeat (13 B, `0x21`) | `seq` 3, 4, 5 at 10 s spacing (`up` 158 499, 168 499, 178 499 ms), same boot id |
| Map `A=L,B=R,F=U` sent from the dashboard | Sent as the 4 bytes `12 01 0a 2c`; car replied `13 03`; dashboard showed `OK n=3` |
| 26-letter map (binary, 27 B) | reply `13 1a` = OK, 26 letters |
| Clear (`12`) | reply `13 00` = OK, 0 letters |
| Same letter twice (`12 01 02`) | reply `13 a0` = ERR duplicate |
| Command 0 and command 7 (`12 00`, `12 07`) | reply `13 80` = ERR action (both) |
| 27 letters | reply `13 40` = ERR syntax |
| Old text map `C=F,D=U` | still accepted; reply is binary `13 02` |
| Serial status after the tests | `map=3/4` (3 letters, 4 rejected maps = the 4 bad ones above), no new tick overruns, worst task times unchanged (motion 160 us, line 289 us, vehicle 501 us) |
| Steady feed over 20 s | status 10.0 / s, sensors 2.0 / s, heartbeat 1.0 / s received; the car's telemetry drop counter did not move (1402 to 1402) |

## Not shown by this test
- The car was not driven, so the barcode behaviour on the track with a binary map is not tested here (the decode path is unchanged and covered by the host tests).
- The drop counter rose by 71 in one 6-second window during the first map tests. It was followed up (see "Drop and stall follow-up" below): the cause is stalls on the WiFi link, not the map messages.

## Drop and stall follow-up (same day, v23, car stationary)
The car's heartbeat record carries the drop counter, so it can be watched from the dashboard without the USB cable.

| Run | What happened |
|---|---|
| 10 binary maps in 1 second | Drop counter did not change (10 706 before and after); queue depth stayed 0 |
| Opening the serial port for 6 seconds | Drop counter did not change |
| Control, 60 s, no maps and no serial | One 7 s gap and one 2.4 s gap in the heartbeats (1 Hz); drop counter flat |
| 100 s with the serial log open | Status records (10 Hz): three gaps, 8.3 s, 2.6 s and 2.6 s. During the 8.3 s gap the car stayed `mqtt=3` (online) with no new tick overruns; its drop counter started to rise only after about 6 s of the gap (22 records), when its 16-slot queue filled |

Reading: the feed stalls for 2 to 8 seconds from time to time even when nothing else is going on, the records arrive in a bunch afterwards, and the car only drops
records when a stall lasts longer than its queue can absorb. Maps and the serial port do not cause it. This points at the WiFi link (the PC's radio carries the hotspot and its
own connection to the other network at the same time), not at the MQTT payload change. It is not proved: the same measurement on v22 was not made, because putting the car into BOOTSEL
needs a hand on the button. A way to test it: the same 100 s run with the PC disconnected from its other WiFi network, or on a separate router.
- The firmware was flashed once; no repeated reconnect test was run on v23.

Tool used for the map cases: `dashboard/tools/map-cases.mjs` (needs the dashboard running with the car connected). Raw logs are in the session scratchpad, not in the repo.

## v24 (smaller MQTT mailbox) on the car, same day, car stationary
Firmware `robot_wifi_telemetry_v24_mailbox_fazfaz_smp1.uf2` (commit b60eca1): the MQTT mailbox payload is 32 bytes instead of 128 (`.bss` 63 732 B).

| Check | Result |
|---|---|
| Online marker, both diagnostic parts, heartbeat | Decoded as on v23 (new boot id `b5aab54b` in all of them) |
| Map `A=L,B=R,F=U` | `OK n=3` |
| The seven binary map cases (26 letters, clear, duplicate, action 0, action 7, 27 letters, valid again) | Same replies as v23: `13 1a`, `13 00`, `13 a0`, `13 80`, `13 80`, `13 40`, `13 03` |
| Text map of 15 bytes | accepted, reply `13 04` (4 letters) |
| Text map of 35 bytes (over the new 32-byte limit) | no reply: dropped by the mailbox, as intended |
| Steady feed over 20 s | status 10.1 / s, sensors 2.0 / s, heartbeat 1.0 / s; drop counter flat (151 to 151) |
| Serial status | `map=3/4`, no tick overruns (0/0/0), worst task times motion 156 us, line 273 us, vehicle 492 us |

The 35-byte text map is the one behaviour that changed from v23 (v23 accepted it); the dashboard sends binary, so this affects hand-typed text maps only. The car was not driven.
