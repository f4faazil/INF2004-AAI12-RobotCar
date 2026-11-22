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
- The drop counter rose by 71 in one 6-second window during the burst of map tests (reply messages go out ahead of telemetry). It was flat in steady state. The cause was not isolated.
- The firmware was flashed once; no repeated reconnect test was run on v23.

Tool used for the map cases: `dashboard/tools/map-cases.mjs` (needs the dashboard running with the car connected). Raw logs are in the session scratchpad, not in the repo.
