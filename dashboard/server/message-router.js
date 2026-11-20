import { parseDiagnostic, parseFutureJson, parseHeartbeat, parseProvisionalTelemetry } from "./parsers.js";

export function routeMqttMessage(topic, payload, topics, now = () => new Date()) {
  const base = { receivedAt: now().toISOString(), topic, source: "firmware-confirmed" };
  if (topic === topics.test) {
    return { ...base, type: "online-marker", data: payload.toString("utf8") };
  }
  if (topic === topics.diagnostic) {
    const parsed = parseDiagnostic(payload);
    return { ...base, type: "diagnostic-snapshot", data: parsed.fields, raw: parsed.raw };
  }
  if (topic === topics.telemetry) {
    return { ...base, type: "provisional-telemetry", data: parseProvisionalTelemetry(payload) };
  }
  if (topic === topics.heartbeat) {
    const parsed = parseHeartbeat(payload);
    return { ...base, type: "heartbeat", data: parsed.fields, raw: parsed.raw };
  }
  if (topics.json && topic === topics.json) {
    return { ...base, type: "future-telemetry", source: "future-json-topic", data: parseFutureJson(payload) };
  }
  return null;
}
