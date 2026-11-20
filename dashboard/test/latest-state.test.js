import test from "node:test";
import assert from "node:assert/strict";
import { LatestPicoState } from "../server/latest-state.js";

const message = (type, receivedAt, data = {}) => ({ type, receivedAt, topic: type === "online-marker" ? "aai12/test" : "aai12/diagnostic", source: "firmware-confirmed", data });

test("replays the latest marker and both diagnostic parts as historical data", () => {
  const state = new LatestPicoState();
  state.record(message("online-marker", "2026-11-19T10:00:00.000Z", "AAI12_PICO_W_ONLINE"));
  state.record(message("diagnostic-snapshot", "2026-11-19T10:00:01.000Z", { bootNonce: "abc", timerTickConfig: 524 }));
  state.record(message("diagnostic-snapshot", "2026-11-19T10:00:02.000Z", { bootNonce: "abc", tcpErrors: 0, joinAttempts: 1 }));

  const replay = state.replay(new Date("2026-11-19T10:05:00.000Z"));
  assert.equal(replay.length, 3);
  assert.ok(replay.every(item => item.cached === true));
  assert.ok(replay.every(item => item.replayedAt === "2026-11-19T10:05:00.000Z"));
  assert.deepEqual(replay.map(item => item.receivedAt), ["2026-11-19T10:00:00.000Z", "2026-11-19T10:00:01.000Z", "2026-11-19T10:00:02.000Z"]);
});

test("new messages replace cached values instead of growing an event history", () => {
  const state = new LatestPicoState();
  state.record(message("online-marker", "2026-11-19T10:00:00.000Z", "old"));
  state.record(message("online-marker", "2026-11-19T10:01:00.000Z", "new"));
  assert.equal(state.replay().length, 1);
  assert.equal(state.replay()[0].data, "new");
});

test("a new boot nonce drops diagnostic parts from the previous boot", () => {
  const state = new LatestPicoState();
  state.record(message("diagnostic-snapshot", "2026-11-19T10:00:01.000Z", { bootNonce: "old", timerTickConfig: 524 }));
  state.record(message("diagnostic-snapshot", "2026-11-19T10:00:02.000Z", { bootNonce: "old", tcpErrors: 2 }));
  state.record(message("diagnostic-snapshot", "2026-11-19T10:05:01.000Z", { bootNonce: "new", timerTickConfig: 524 }));
  const diagnostics = state.replay().filter(item => item.type === "diagnostic-snapshot");
  assert.equal(diagnostics.length, 1);
  assert.equal(diagnostics[0].data.bootNonce, "new");
});

test("browser refresh replays the latest heartbeat with its original time", () => {
  const state = new LatestPicoState();
  state.record({ type: "heartbeat", topic: "aai12/robot/heartbeat",
    source: "firmware-confirmed", receivedAt: "2026-11-19T10:00:10.000Z",
    data: { bootId: "12abcdef", sequence: 3, uptimeMs: 30000 } });
  const replay = state.replay(new Date("2026-11-19T10:00:20.000Z"));
  assert.equal(replay[0].cached, true);
  assert.equal(replay[0].receivedAt, "2026-11-19T10:00:10.000Z");
  assert.equal(replay[0].replayedAt, "2026-11-19T10:00:20.000Z");
});
