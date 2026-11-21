import test from "node:test";
import assert from "node:assert/strict";
import { routeMqttMessage } from "../server/message-router.js";
import { LatestPicoState } from "../server/latest-state.js";
import { loadConfig } from "../server/config.js";
import { scenario, createRobotSimulator } from "../server/robot-simulator.js";

const topics = loadConfig({}).mqtt.topics;
const fixedNow = () => new Date("2026-11-20T12:00:00.000Z");
const SENSORS = Buffer.from("063039be90302010402d6903323bbd0cb49c", "hex");

test("a robot record on the telemetry topic is decoded, not treated as the 3-byte diagnostic frame", () => {
  const result = routeMqttMessage(topics.telemetry, SENSORS, topics, fixedNow);
  assert.equal(result.type, "robot-frame");
  assert.equal(result.kind, "sensors");
  assert.equal(result.data.lastLetterChar, "Z");
  assert.equal(result.hex, SENSORS.toString("hex"));
});

test("the application client's separate event, ack and status topics are read too", () => {
  const event = Buffer.from("02009608006038", "hex");
  assert.equal(routeMqttMessage("aai12/robot/event", event, topics, fixedNow).kind, "event");
  assert.equal(routeMqttMessage("aai12/robot/ack", Buffer.from("110720", "hex"), topics, fixedNow).kind, "ack");
});

test("a garbage record is an error the server reports, not a crash", () => {
  assert.throws(() => routeMqttMessage(topics.telemetry, Buffer.from("7f0000000000", "hex"), topics, fixedNow), /unknown robot record type/);
});

test("a new page gets the latest record of each kind and the last events", () => {
  const state = new LatestPicoState();
  const frame = (kind, at, extra = {}) => ({ type: "robot-frame", kind, receivedAt: at, topic: topics.telemetry, source: "firmware-confirmed", data: { kind, ...extra } });
  state.record(frame("status", "2026-11-20T10:00:00.000Z", { n: 1 }));
  state.record(frame("status", "2026-11-20T10:00:01.000Z", { n: 2 }));
  state.record(frame("sensors", "2026-11-20T10:00:01.500Z"));
  state.record(frame("event", "2026-11-20T10:00:02.000Z", { name: "STATE" }));
  state.record(frame("event", "2026-11-20T10:00:03.000Z", { name: "BARCODE" }));
  const replay = state.replay(new Date("2026-11-20T10:05:00.000Z"));
  assert.deepEqual(replay.map(m => m.kind), ["status", "sensors", "event", "event"]);
  assert.equal(replay[0].data.n, 2);                       // only the latest status
  assert.ok(replay.every(m => m.cached === true));
});

test("the demo lap covers every state the dashboard shows, and ends picked up", () => {
  const seen = new Set();
  for (let t = 0; t < 72; t += 0.25) seen.add(scenario(t).state);
  for (const needed of [2, 3, 4, 5, 8, 9, 10, 12, 14]) assert.ok(seen.has(needed), `state ${needed} missing from the lap`);
  assert.equal(scenario(66).lift, true);
  assert.equal(scenario(66).faults & (1 << 9), 1 << 9);
  assert.ok(scenario(26).aim === 2 && scenario(26).us < 400);   // looking at the box while passing it
});

test("the demo simulator emits records that decode through the real router", async () => {
  const got = [];
  const stop = createRobotSimulator(m => got.push(m), { intervalMs: 5 });
  await new Promise(resolve => setTimeout(resolve, 400));
  stop();
  const kinds = new Set(got.map(m => m.kind));
  for (const k of ["status", "sensors", "heartbeat", "event"]) assert.ok(kinds.has(k), `${k} not emitted`);
  assert.ok(got.every(m => m.source === "demo-simulated"));
});
