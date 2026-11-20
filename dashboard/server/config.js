const bool = (value, fallback = false) => value == null ? fallback : /^(1|true|yes)$/i.test(value);

export function loadConfig(env = process.env) {
  const port = Number.parseInt(env.PORT ?? "3000", 10);
  if (!Number.isInteger(port) || port < 1 || port > 65535) throw new Error("PORT must be between 1 and 65535");

  return {
    port,
    demoMode: bool(env.DEMO_MODE),
    // The built-in broker: the robot connects to this machine and the dashboard reads what it sends, so no
    // Mosquitto is needed.  Set EMBEDDED_BROKER=false to use an external broker through MQTT_URL instead.
    embeddedBroker: bool(env.EMBEDDED_BROKER, true),
    brokerPort: Number.parseInt(env.BROKER_PORT ?? "1883", 10),
    brokerHost: env.BROKER_HOST ?? "0.0.0.0",
    mqtt: {
      url: env.MQTT_URL ?? "mqtt://127.0.0.1:1883",
      username: env.MQTT_USERNAME || undefined,
      password: env.MQTT_PASSWORD || undefined,
      clientId: env.MQTT_CLIENT_ID ?? "aai12-dashboard",
      topics: {
        test: env.MQTT_TEST_TOPIC ?? "aai12/test",
        diagnostic: env.MQTT_DIAGNOSTIC_TOPIC ?? "aai12/diagnostic",
        telemetry: env.MQTT_TELEMETRY_TOPIC ?? "aai12/robot/telemetry",
        heartbeat: env.MQTT_HEARTBEAT_TOPIC ?? "aai12/robot/heartbeat",
        event: env.MQTT_EVENT_TOPIC ?? "aai12/robot/event",
        ack: env.MQTT_ACK_TOPIC ?? "aai12/robot/ack",
        status: env.MQTT_STATUS_TOPIC ?? "aai12/robot/status",
        map: env.MQTT_MAP_TOPIC ?? "aai12/robot/map",                  // dashboard -> car: barcode letter map
        mapAck: env.MQTT_MAP_ACK_TOPIC ?? "aai12/robot/map_ack",        // car -> dashboard: "OK n=3" or "ERR ..."
        json: env.MQTT_JSON_TOPIC || undefined
      }
    }
  };
}
