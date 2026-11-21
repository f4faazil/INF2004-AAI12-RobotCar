# AAI12 Robot Telemetry Dashboard

A responsive, read-only dashboard that relays local MQTT traffic to the browser over WebSocket. It has no motor-control commands and makes no changes to the Pico W firmware.

## Quick demo

Requires Node.js 20 or newer.

```sh
cd dashboard
npm install
npm run demo
```

Open <http://localhost:3000>. Demo mode generates clearly labelled, simulated versions of the existing online marker and two-part connection diagnostic snapshot. It does not simulate unconfirmed robot sensors and does not need an MQTT broker or physical robot.

## Connect to MQTT

```sh
cd dashboard
cp .env.example .env
# Edit .env with the broker URL and, if needed, credentials.
npm start
```

The backend subscribes to MQTT; broker credentials never reach the browser. `.env` and `node_modules/` are ignored by Git. Plain `mqtt://` is suitable only for a trusted local network; use `mqtts://` when the broker supports TLS.

Useful settings:

| Variable | Default | Purpose |
|---|---|---|
| `PORT` | `3000` | Local dashboard port |
| `DEMO_MODE` | `false` | Use simulated telemetry instead of MQTT |
| `MQTT_URL` | `mqtt://127.0.0.1:1883` | Broker connection URL |
| `MQTT_USERNAME`, `MQTT_PASSWORD` | empty | Optional broker credentials |
| `MQTT_TEST_TOPIC` | `aai12/test` | Pico W online smoke-test marker |
| `MQTT_DIAGNOSTIC_TOPIC` | `aai12/diagnostic` | Compact MQTT/network diagnostic records |
| `MQTT_TELEMETRY_TOPIC` | `aai12/robot/telemetry` | Provisional binary diagnostic frames |
| `MQTT_HEARTBEAT_TOPIC` | `aai12/robot/heartbeat` | Ten-second Pico W liveness heartbeat |
| `MQTT_JSON_TOPIC` | empty | Opt-in future team telemetry topic |

Restart the backend after changing `.env`.

## What the firmware publishes today

The dashboard implementation was based on commit `b97bf5e` of `feature/aamir-robot-integration`.

- The configured test topic (default `aai12/test`) sends `AAI12_PICO_W_ONLINE` once after each MQTT connection.
- `aai12/diagnostic` sends two compact comma-separated `key=value` records after connecting. The dashboard expands the firmware's abbreviated keys into readable names.
- `aai12/robot/telemetry` carries the temporary three-byte Stage 5C diagnostic format: `type`, `status`, `height`, `validity`, and `imu_fault`. It is explicitly marked by the firmware as diagnostic-only and not the approved robot schema.
- `aai12/robot/heartbeat` sends `v=1,b=<boot-id>,seq=<sequence>,up=<uptime-ms>` approximately every ten seconds while the MQTT session is online. A heartbeat less than 30 seconds old is the dashboard's only current Pico-online evidence.

The firmware does **not** currently publish a confirmed movement state, accelerometer axes, gyroscope axes, or IMU temperature. Those measurements are therefore absent from the main live dashboard. Reserved placeholders show where agreed teammate telemetry can be integrated later.

The dashboard deliberately distinguishes two different states:

- **Broker connected** means the MacBook relay has an MQTT connection.
- **Pico W message received** means one of the configured firmware topics was observed at the displayed last-seen time. Without a periodic heartbeat, it is not proof that the Pico remains online afterward.

While the Node.js backend remains running, it keeps an in-memory latest-known cache of the Pico online marker, both diagnostic snapshot parts, and the latest valid heartbeat. A refreshed or newly opened browser receives these records immediately with their original receive times. Heartbeat freshness is calculated from that original time—not replay time—so a cached heartbeat older than 30 seconds remains stale. Restarting the backend clears this in-memory cache.

The diagnostic records are published in two parts after MQTT connects. They contain a saved snapshot of counters and the most recently captured failure context. They are labelled as connection-time snapshots throughout the UI and must not be interpreted as continuously updating statistics.

## Future team integration

Once the team agrees on a JSON schema, set `MQTT_JSON_TOPIC` to its topic. The current adapter accepts an object shaped like this:

```json
{
  "movement": "FORWARD",
  "imu": {
    "accelX": 0.02, "accelY": -0.01, "accelZ": 1.0,
    "gyroX": 0.1, "gyroY": 0.2, "gyroZ": 0.3,
    "temperature": 29.5
  }
}
```

These names remain a dashboard proposal for Hadi/team review. Future telemetry is not shown as live data until a configured topic actually publishes it.

## Test with the actual Pico W

1. Run a local MQTT broker reachable by both the MacBook and Pico W. Note the MacBook's LAN/hotspot IP.
2. Keep the existing Pico configuration pointed at that IP and port, then flash the existing `WIFI_MQTT=1` build. No firmware change is required for connection snapshots.
3. In `dashboard/.env`, set `DEMO_MODE=false` and set `MQTT_URL` to the same broker, for example `mqtt://192.168.1.20:1883`. Add credentials only if the broker requires them.
4. Start the dashboard with `npm start`, open <http://localhost:3000>, then power-cycle the Pico W so its one-time connection messages are emitted while the dashboard is subscribed.
5. Confirm the event log receives `aai12/test`, followed by two `aai12/diagnostic` records. Check that the boot nonce matches across both parts and that the RP2040 timer card reports `524` as verified.
6. Within about ten seconds, confirm `aai12/robot/heartbeat` appears with the same boot ID, sequence `0`, and an uptime. Further heartbeats should arrive about every ten seconds with increasing sequences.
7. `aai12/robot/telemetry` appears only if the firmware build has a producer submitting the provisional diagnostic frame. The dashboard logs a valid or invalid payload but does not present it as real sensor telemetry.

For an independent broker check, run `mosquitto_sub -v -h <broker-ip> -t 'aai12/#'` before power-cycling the Pico.

To test stale detection, disconnect or power off the Pico. The dashboard changes to `OFFLINE / STALE` 30 seconds after the last heartbeat while continuing to report the MacBook relay's broker connection separately.

## Tests

```sh
npm test
```

Tests cover configuration, compact diagnostic parsing, real-topic routing, preservation of raw snapshot evidence, provisional binary decoding, unknown-payload handling and the future JSON boundary.

---

# Robot telemetry page (added when the robot firmware was integrated)

`/` is now the **robot dashboard**: every sensor the car has, decoded from the robot's own bit-packed records
(`app_program/telemetry.h`).  The WiFi / MQTT diagnostics page described above is still here, at `/network`.

## Run it on the laptop that hosts the website

```sh
cd dashboard
npm install          # once
npm start            # website on http://localhost:3000 and an MQTT broker for the robot on port 1883
```

The dashboard now contains its **own small MQTT broker** (`server/broker.js`, no Mosquitto needed): the robot connects to
this laptop's address on the hotspot (the address built into the firmware, `AAI12_MQTT_BROKER_IP` in
`config/mqtt_test_config.h`), and the page shows what it publishes.  Open `http://<laptop-ip>:3000` from any device on the
hotspot to see it.  The broker covers CONNECT, PUBLISH (QoS 0/1/2 acknowledged), SUBSCRIBE (+ and #), PINGREQ and
retained messages (kept in memory while it runs); it has no wills or login.  To use a real broker instead: `EMBEDDED_BROKER=false` and `MQTT_URL=mqtt://...` in `.env`.

Windows asks once whether Node may accept connections: allow it on the hotspot network (private and public), or the robot
cannot reach port 1883 / the page cannot be opened from a phone.

Try the page without the robot: `DEMO_MODE=true npm start` plays a scripted lap (follow the line, meet a box, scan, bypass,
rejoin, barcode Z, U-turn, hump, picked up) through exactly the same decoding.  The page is marked SIMULATION.

## What the page shows

| Card | From |
|---|---|
| Vehicle state, faults (stop faults in red), distance run, robot uptime, records per second | status record, 10 Hz; heartbeat, 1 Hz |
| Line and IR sensors: the three digital levels, which are trusted, line position, line state, junction seen/armed | status + sensor records |
| Barcode: bars counted (of 15), last letter, instruction pending, decoded/rejected, barcodes completed, junctions/executed, letters mapped | sensor record, 2 Hz |
| Motors and encoders: left/right speed with a chart, motion state, wheels stopped, encoder faults and bad transitions | status + sensor records |
| IMU: sideways tilt, lift/pick-up latched, hump now, last hump height, highest hump | sensor + status + hump records |
| Ultrasonic and servo: distance with a chart, valid, obstacle flag, fault, where the sonar points | sensor record |
| Last obstacle scan: closest, width, centre, clear left/right, the plan and why, with a top-down picture | obstacle record |
| Network: WiFi link, MQTT session, queue depth, records dropped, the robot's connection and PINGREQ interval seen by the broker | heartbeat + broker |
| Events: state changes, barcodes, junctions, humps, plans, faults | event records |

## Feeding the barcode instructions to the car

The car has **no built-in barcode table**: until one arrives over MQTT every barcode it reads is rejected. The page's
**Barcode instructions** card sets what each letter does (Left, Right, Straight, U-turn; “—” leaves it unmapped) and
**Send to the robot** publishes it, *retained*, on `aai12/robot/map`. The car subscribes to that topic when it connects (so
it gets the table even if it connects after you sent it) and answers on `aai12/robot/map_ack` (`OK n=3`, or `ERR ...` and
it keeps its old table). The answer shows on the card.

From a terminal instead (the dashboard must be running):

```sh
npm run map -- "A=L,B=R,F=U"      # A left, B right, F U-turn   (L left, R right, F straight, U U-turn)
npm run map -- CLEAR              # forget everything on the car
curl -X POST --data "A=L,B=R,F=U" http://localhost:3000/api/map
```

Each send REPLACES the whole table. The dashboard checks the text with the robot's own rules (`server/barcode-map.js`
mirrors `app_program/barcode_map.c`) and refuses a bad one before it is sent.

Anything that can reach the dashboard's port or the broker can change the table, so keep both on the hotspot.

## START and STOP from the page

The **START / STOP** card at the top sends the car's own command record (`TEL_TYPE_COMMAND`, `server/commands.js`) on
`aai12/robot/cmd`. START does what the GP20 button does (calibrate, then follow the line); STOP stops the motors. The car
answers with an ACK record carrying the same request id: RECEIVED, then COMPLETED when its vehicle controller has acted
on it, or REFUSED when its own policy says no (picked up by the IMU lift stop, or START while already running).

Safety, in layers:
- START is greyed out until you tick "I am next to the car and its wheels are clear", and the tick is used up by one START.
- A command is never retained and never queued: with no car connected the dashboard says so and sends nothing, and a car
  that reconnects is not given an old START.
- The page's WebSocket must come from this dashboard's own address (a page served by another site cannot send commands).
- The car still refuses START while picked up, and its physical stop conditions (lift, faults) still win.

The dashboard's port and the broker have no login: anyone who can reach them on the hotspot can press START. Keep the
hotspot to your team, and keep the car's wheels off the floor for the first tries.


## The per-sensor record

`TEL_TYPE_SENSORS` (0x06), 18 bytes, every field with its range and the bits that range needs: see `telemetry.h`.
`server/robot-frames.js` decodes it and is checked against bytes produced by the C encoder (`test/robot-frames.test.js`);
`tools/robot_telemetry.py` is the Python decoder of the same layout.

## Tests

`npm test` now also covers the robot-record decoder (against the firmware's bytes), the routing, the replay cache for a newly opened page,
the demo lap and the built-in broker (connect, publish at QoS 0 and 1, ping, segmented packets, subscribe, retained
messages), and the barcode map path end to end (`test/map.test.js`: validation, the bytes the car receives, the car's answer).
