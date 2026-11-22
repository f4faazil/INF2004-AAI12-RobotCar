import { parseDiagnostic, parseFutureJson, parseHeartbeat, parseProvisionalTelemetry } from "./parsers.js";
import { decodeRobotFrame } from "./robot-frames.js";
import { decodeMapAck } from "./barcode-map.js";

export function routeMqttMessage(topic, payload, topics, now = () => new Date()) {
  const base = { receivedAt: now().toISOString(), topic, source: "firmware-confirmed" };
  if (topic === topics.test) {
    return { ...base, type: "online-marker", data: payload.toString("utf8") };
  }
  if (topics.mapAck && topic === topics.mapAck) {
    return { ...base, type: "map-ack", data: decodeMapAck(payload).text };
  }
  if (topic === topics.diagnostic) {
    const parsed = parseDiagnostic(payload);
    return { ...base, type: "diagnostic-snapshot", data: parsed.fields, raw: parsed.raw };
  }
  if (topic === topics.telemetry && payload.length === 3) {
    return { ...base, type: "provisional-telemetry", data: parseProvisionalTelemetry(payload) };
  }
  // The robot's own records (app_program/telemetry.h): the first byte says which one.  The firmware build that
  // runs the radio owner task sends them all on the telemetry topic; the application's own MQTT client would
  // spread them over the event, ack and status topics, so those are read too.
  if ([topics.telemetry, topics.event, topics.ack, topics.status].filter(Boolean).includes(topic)) {
    const frame = decodeRobotFrame(payload);
    return { ...base, type: "robot-frame", kind: frame.kind, data: frame, hex: payload.toString("hex") };
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
