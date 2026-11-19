import test from "node:test";
import assert from "node:assert/strict";
import { loadConfig } from "../server/config.js";

test("loads safe defaults without credentials", () => {
  const config = loadConfig({});
  assert.equal(config.demoMode, false);
  assert.equal(config.mqtt.topics.telemetry, "aai12/robot/telemetry");
  assert.equal(config.mqtt.password, undefined);
});

test("reads demo and custom topic settings", () => {
  const config = loadConfig({ PORT: "4100", DEMO_MODE: "true", MQTT_JSON_TOPIC: "aai12/robot/v1" });
  assert.equal(config.port, 4100);
  assert.equal(config.demoMode, true);
  assert.equal(config.mqtt.topics.json, "aai12/robot/v1");
});
