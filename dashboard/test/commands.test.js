import test from "node:test";
import assert from "node:assert/strict";
import net from "node:net";
import { spawn } from "node:child_process";
import { encodeCommand, sameOrigin, createRequestIds } from "../server/commands.js";
import { decodeRobotFrame } from "../server/robot-frames.js";

test("the command bytes are exactly what the firmware's tel_encode_command() produces", () => {
  // golden bytes from the C encoder (scratchpad/cmd_golden.c)
  assert.equal(encodeCommand("start", 7).toString("hex"), "1007200000");
  assert.equal(encodeCommand("start", 1).toString("hex"), "1001200000");
  assert.equal(encodeCommand("start", 255).toString("hex"), "10ff200000");
  assert.equal(encodeCommand("stop", 7).toString("hex"), "1007100000");
  assert.equal(encodeCommand("stop", 1).toString("hex"), "1001100000");
  assert.equal(encodeCommand("stop", 255).toString("hex"), "10ff100000");
});

test("only START and STOP exist, and bad request ids are refused", () => {
  assert.throws(() => encodeCommand("calibrate", 1), /unknown command/);
  assert.throws(() => encodeCommand("drive", 1), /unknown command/);
  assert.throws(() => encodeCommand("start", 256), /request id/);
  assert.throws(() => encodeCommand("start", -1), /request id/);
});

test("request ids run 1 to 255 and wrap, never 0", () => {
  const next = createRequestIds();
  assert.equal(next(), 1);
  for (let i = 0; i < 253; i++) next();
  assert.equal(next(), 255);
  assert.equal(next(), 1);
});

test("a page served by another site is not trusted to drive the car", () => {
  assert.equal(sameOrigin("http://192.168.137.1:3000", "192.168.137.1:3000"), true);
  assert.equal(sameOrigin("http://evil.example", "192.168.137.1:3000"), false);
  assert.equal(sameOrigin("not a url", "192.168.137.1:3000"), false);
  assert.equal(sameOrigin(undefined, "192.168.137.1:3000"), true);
});

test("the car's ACK record carries the request id the page is waiting for", () => {
  // TEL_TYPE_ACK 0x11, request 7, result COMPLETED(1): the firmware's golden record
  const ack = decodeRobotFrame(Buffer.from("110720", "hex"));
  assert.equal(ack.kind, "ack");
  assert.equal(ack.reqId, 7);
  assert.equal(ack.resultName, "COMPLETED");
});

const connectPacket = id => {
  const idBuf = Buffer.from(id);
  const body = Buffer.concat([Buffer.from([0, 4, 0x4d, 0x51, 0x54, 0x54, 4, 0x02, 0, 30, 0, idBuf.length]), idBuf]);
  return Buffer.concat([Buffer.from([0x10, body.length]), body]);
};
const subscribePacket = (id, filter) => {
  const f = Buffer.from(filter);
  const body = Buffer.concat([Buffer.from([id >> 8, id & 255, f.length >> 8, f.length & 255]), f, Buffer.from([0])]);
  return Buffer.concat([Buffer.from([0x82, body.length]), body]);
};
const collect = socket => {
  const chunks = [];
  socket.on("data", c => chunks.push(c));
  return { all: () => Buffer.concat(chunks), until: async n => { for (let i = 0; i < 200 && Buffer.concat(chunks).length < n; i++) await new Promise(r => setTimeout(r, 10)); return Buffer.concat(chunks); } };
};

test("START and STOP from the page reach a connected car as non-retained commands; with no car nothing is sent", async () => {
  const web = 39171, brokerPort = 39184;
  const child = spawn(process.execPath, ["server/index.js"], { env: { ...process.env, PORT: String(web), BROKER_PORT: String(brokerPort), BROKER_HOST: "127.0.0.1" }, stdio: "ignore" });
  try {
    let up = false;
    for (let i = 0; i < 100 && !up; i++) {
      up = await fetch(`http://127.0.0.1:${web}/`).then(r => r.ok, () => false);
      if (!up) await new Promise(r => setTimeout(r, 50));
    }
    assert.ok(up, "the dashboard started");
    await new Promise(r => setTimeout(r, 200));

    const results = [];
    const ws = new WebSocket(`ws://127.0.0.1:${web}/live`);
    ws.onmessage = e => { const m = JSON.parse(e.data); if (m.type === "command-result") results.push(m); };
    await new Promise(r => { ws.onopen = r; });
    const waitResult = async n => { for (let i = 0; i < 200 && results.length < n; i++) await new Promise(r => setTimeout(r, 10)); return results[n - 1]; };

    // nobody subscribed: refused, not queued
    ws.send(JSON.stringify({ type: "send-command", command: "start" }));
    const refused = await waitResult(1);
    assert.equal(refused.ok, false);
    assert.match(refused.error, /not connected/);

    // the car connects and subscribes to the command topic
    const car = net.connect(brokerPort, "127.0.0.1");
    await new Promise(r => car.once("connect", r));
    const rx = collect(car);
    car.write(connectPacket("aai12-robot"));
    car.write(subscribePacket(1, "aai12/robot/cmd"));
    await rx.until(9);
    const base = rx.all().length;                                    // CONNACK + SUBACK; nothing retained was replayed

    ws.send(JSON.stringify({ type: "send-command", command: "start" }));
    const started = await waitResult(2);
    assert.equal(started.ok, true);
    assert.equal(started.command, "start");
    const bytes = await rx.until(base + 2 + 2 + 15 + 5);
    const pkt = bytes.subarray(base);
    assert.equal(pkt[0], 0x30, "PUBLISH at QoS 0 and NOT retained");
    assert.equal(pkt.subarray(4, 19).toString(), "aai12/robot/cmd");
    assert.equal(pkt.subarray(19).toString("hex"), encodeCommand("start", started.reqId).toString("hex"));

    ws.send(JSON.stringify({ type: "send-command", command: "stop" }));
    const stopped = await waitResult(3);
    assert.equal(stopped.ok, true);
    assert.equal(stopped.reqId, started.reqId + 1);
    const more = await rx.until(base + 2 * (2 + 2 + 15 + 5));
    assert.equal(more.subarray(base + 2 + 2 + 15 + 5 + 19).toString("hex"), encodeCommand("stop", stopped.reqId).toString("hex"));

    // anything else is refused
    ws.send(JSON.stringify({ type: "send-command", command: "calibrate" }));
    assert.equal((await waitResult(4)).ok, false);

    // a car that connects later is NOT given the old START
    const late = net.connect(brokerPort, "127.0.0.1");
    await new Promise(r => late.once("connect", r));
    const lateRx = collect(late);
    late.write(connectPacket("aai12-robot-2"));
    late.write(subscribePacket(1, "aai12/robot/cmd"));
    await lateRx.until(9);
    await new Promise(r => setTimeout(r, 150));
    assert.equal(lateRx.all().length, 9, "no replay of a command");
    ws.close(); car.destroy(); late.destroy();
  } finally { child.kill(); }
});
