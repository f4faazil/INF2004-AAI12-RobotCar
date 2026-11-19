import test from "node:test";
import assert from "node:assert/strict";
import { routeMqttMessage } from "../server/message-router.js";

const topics = {
  test: "aai12/test",
  diagnostic: "aai12/diagnostic",
  telemetry: "aai12/robot/telemetry",
  heartbeat: "aai12/robot/heartbeat"
};
const fixedNow = () => new Date("2026-11-19T12:00:00.000Z");

test("routes the Pico online marker without claiming continuous presence", () => {
  const result = routeMqttMessage(topics.test, Buffer.from("AAI12_PICO_W_ONLINE"), topics, fixedNow);
  assert.equal(result.type, "online-marker");
  assert.equal(result.source, "firmware-confirmed");
  assert.equal(result.receivedAt, "2026-11-19T12:00:00.000Z");
  assert.equal(result.data, "AAI12_PICO_W_ONLINE");
});

test("routes a diagnostic message as a historical snapshot and keeps raw evidence", () => {
  const raw = "v=1,b=00af10cc,tk=524,r=0,s=3,pq=4,pr=4,wc=2,dc=2";
  const result = routeMqttMessage(topics.diagnostic, Buffer.from(raw), topics, fixedNow);
  assert.equal(result.type, "diagnostic-snapshot");
  assert.equal(result.raw, raw);
  assert.deepEqual(result.data, {
    schemaVersion: 1, bootNonce: "00af10cc", timerTickConfig: 524,
    failureReason: 0, mqttState: 3, pingsQueued: 4,
    pingResponses: 4, wifiLinkChanges: 2, dhcpChanges: 2
  });
});

test("ignores MQTT topics outside the configured dashboard inputs", () => {
  assert.equal(routeMqttMessage("aai12/unrelated", Buffer.from("data"), topics, fixedNow), null);
});

test("routes a valid heartbeat with its original receive timestamp", () => {
  const result = routeMqttMessage(topics.heartbeat,
    Buffer.from("v=1,b=12abcdef,seq=7,up=70000"), topics, fixedNow);
  assert.equal(result.type, "heartbeat");
  assert.equal(result.receivedAt, "2026-11-19T12:00:00.000Z");
  assert.deepEqual(result.data,
    { schemaVersion: 1, bootId: "12abcdef", sequence: 7, uptimeMs: 70000 });
});
