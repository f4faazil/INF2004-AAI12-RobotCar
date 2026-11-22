import test from "node:test";
import assert from "node:assert/strict";
import { decodeRobotFrame, encodeRobotFrame } from "../server/robot-frames.js";

// Bytes produced by the C firmware's own encoders (app_program/telemetry.c) for the records named below.
// Regenerate them with a small program that fills the struct and calls tel_encode_*; they are the contract.
const FIRMWARE = {
  sensors: "063039be90302010402d6903323bbd0cb49c",
  status: "010064a24f061685806e4c02088525a2c0",
  event: "02009608006038",
  hump: "0300c84968b0",
  obstacle: "0400fa8a02c17c800080",
  heartbeat: "0500000e100005021b",
  ack: "110720"
};
const decode = hex => decodeRobotFrame(Buffer.from(hex, "hex"));

test("the per-sensor record decodes to the values the firmware encoded", () => {
  const d = decode(FIRMWARE.sensors);
  assert.equal(d.kind, "sensors");
  assert.equal(d.byteLength, 18);
  assert.equal(d.ts10, 12345);
  assert.deepEqual(d.ir, { left: true, centre: false, right: true });
  assert.deepEqual(d.irValid, { left: true, centre: true, right: true });
  assert.equal(d.pairOnLine, true);
  assert.equal(d.junctionArmed, false);
  assert.deepEqual([d.barsNow, d.barsDone, d.decoded, d.rejected, d.junctions, d.executed], [9, 3, 2, 1, 4, 2]);
  assert.deepEqual([d.mapEntries, d.lastLetter, d.lastLetterChar], [26, 26, "Z"]);
  assert.deepEqual([d.motionState, d.motionStateName, d.motionStopped, d.encoderFault], [1, "RUNNING", false, false]);
  assert.deepEqual([d.encBadLeft, d.encBadRight], [12, 200]);
  assert.equal(d.rollPermille, -137);
  assert.equal(d.rollDeg, -7.9);
  assert.deepEqual([d.imuOk, d.liftLatched, d.humpActive], [true, false, true]);
  assert.deepEqual([d.usMm, d.usValid, d.usObstacle, d.usFault], [406, true, false, false]);
  assert.deepEqual([d.servoAim, d.servoAimName], [2, "RIGHT"]);
  assert.deepEqual([d.netState, d.netStateName, d.linkUp, d.saturated], [3, "ONLINE", true, false]);
});

test("the status record decodes the vehicle, line, motors, faults, sonar and hump fields", () => {
  const d = decode(FIRMWARE.status);
  assert.equal(d.kind, "status");
  assert.deepEqual([d.ts10, d.state, d.stateName], [100, 10, "BYPASS"]);
  assert.deepEqual([d.navCmd, d.navCmdName, d.lineStateName, d.junctionName, d.barcodeName], [1, "LEFT", "TRACKING", "RIGHT", "INSTRUCTION PENDING"]);
  assert.deepEqual([d.linePosition, d.leftMmS, d.rightMmS, d.distanceMm], [-250, 90, 88, 1764]);
  assert.equal(d.faults, 0xc020);
  assert.deepEqual(d.faultNames, ["IMU", "TELEMETRY_DROP", "NET_DOWN"]);
  assert.equal(d.stopFault, false);                       // those three are information only
  assert.deepEqual([d.obstacleValid, d.obstacleCm], [true, 33]);
  assert.equal(d.hump.statusName, "ON HUMP");
  assert.equal(d.hump.heightMm, 37);
  assert.equal(d.highestHump, 11);
});

test("a lift fault is a stop fault", () => {
  const bytes = encodeRobotFrame("status", { ts10: 1, state: 14, faults: 1 << 9 });
  const d = decodeRobotFrame(bytes);
  assert.deepEqual(d.faultNames, ["IMU_LIFT"]);
  assert.equal(d.stopFault, true);
});

test("event, hump, obstacle, heartbeat and ack records decode", () => {
  const e = decode(FIRMWARE.event);
  assert.deepEqual([e.ts10, e.name, e.detail, e.seq], [150, "STATE", "RECOVER LINE", 7]);
  const h = decode(FIRMWARE.hump);
  assert.deepEqual([h.ts10, h.hump.statusName, h.hump.heightMm, h.highestHump], [200, "ON HUMP", 37, 11]);
  const o = decode(FIRMWARE.obstacle);
  assert.deepEqual([o.valid, o.closestCm, o.widthCm, o.centreCm, o.leftClearCm, o.rightClearCm, o.actionName, o.reasonName],
    [true, 40, 22, 23, 400, 0, "BYPASS LEFT", "OK"]);
  const b = decode(FIRMWARE.heartbeat);
  assert.deepEqual([b.uptimeS, b.dropped, b.reconnects, b.queueDepth, b.netStateName], [3600, 5, 2, 3, "ONLINE"]);
  const a = decode(FIRMWARE.ack);
  assert.deepEqual([a.reqId, a.resultName], [7, "COMPLETED"]);
});

test("the JavaScript encoder produces the firmware's bytes", () => {
  const sensors = decode(FIRMWARE.sensors);
  assert.equal(encodeRobotFrame("sensors", sensors).toString("hex"), FIRMWARE.sensors);
  const status = decode(FIRMWARE.status);
  assert.equal(encodeRobotFrame("status", status).toString("hex"), FIRMWARE.status);
  const event = decode(FIRMWARE.event);
  assert.equal(encodeRobotFrame("event", event).toString("hex"), FIRMWARE.event);
  const obstacle = decode(FIRMWARE.obstacle);
  assert.equal(encodeRobotFrame("obstacle", obstacle).toString("hex"), FIRMWARE.obstacle);
  const heartbeat = decode(FIRMWARE.heartbeat);
  assert.equal(encodeRobotFrame("heartbeat", heartbeat).toString("hex"), FIRMWARE.heartbeat);
  const hump = decode(FIRMWARE.hump);
  assert.equal(encodeRobotFrame("hump", hump).toString("hex"), FIRMWARE.hump);
});

test("an unknown type and a short record are errors, not data", () => {
  assert.throws(() => decodeRobotFrame(Buffer.from([0x7f, 0, 0])), /unknown robot record type/);
  assert.throws(() => decodeRobotFrame(Buffer.from(FIRMWARE.sensors.slice(0, 20), "hex")), /too short/);
});

test("the lift state and the tilt come through the sensor record", () => {
  const d = decodeRobotFrame(encodeRobotFrame("sensors", { ts10: 5, rollPermille: 866, liftLatched: true, imuOk: true }));
  assert.equal(d.liftLatched, true);
  assert.equal(d.rollDeg, 60);                              // asin(0.866) is 60 degrees: the lift policy's limit
});
