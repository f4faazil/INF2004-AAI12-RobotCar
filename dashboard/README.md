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
