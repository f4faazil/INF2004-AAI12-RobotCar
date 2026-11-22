import test from "node:test";
import assert from "node:assert/strict";
import "../public/heartbeat-state.js";

test("heartbeat remains online before 30 seconds and becomes stale at the boundary", () => {
  const received = "2026-11-19T10:00:00.000Z";
  const start = Date.parse(received);
  assert.equal(globalThis.heartbeatFreshness(received, start + 29999).online, true);
  assert.equal(globalThis.heartbeatFreshness(received, start + 30000).online, false);
});

test("refresh freshness uses original receipt time rather than replay time", () => {
  const original = "2026-11-19T10:00:00.000Z";
  const replayedAt = Date.parse("2026-11-19T10:05:00.000Z");
  const result = globalThis.heartbeatFreshness(original, replayedAt);
  assert.equal(result.online, false);
  assert.equal(result.ageMs, 300000);
});
