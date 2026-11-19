const bool = (value, fallback = false) => value == null ? fallback : /^(1|true|yes)$/i.test(value);

export function loadConfig(env = process.env) {
  const port = Number.parseInt(env.PORT ?? "3000", 10);
  if (!Number.isInteger(port) || port < 1 || port > 65535) throw new Error("PORT must be between 1 and 65535");

  return {
    port,
    demoMode: bool(env.DEMO_MODE),
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
        json: env.MQTT_JSON_TOPIC || undefined
      }
    }
  };
}
