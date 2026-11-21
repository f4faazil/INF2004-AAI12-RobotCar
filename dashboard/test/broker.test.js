import test from "node:test";
import assert from "node:assert/strict";
import net from "node:net";
import mqtt from "mqtt";
import { createBroker, topicMatches } from "../server/broker.js";

const connectPacket = (id, keepalive) => {
  const idBuf = Buffer.from(id);
  const body = Buffer.concat([Buffer.from([0, 4, 0x4d, 0x51, 0x54, 0x54, 4, 0x02, keepalive >> 8, keepalive & 255, 0, idBuf.length]), idBuf]);
  return Buffer.concat([Buffer.from([0x10, body.length]), body]);
};
const publishPacket = (topic, payload, qos = 0, packetId = 0) => {
  const t = Buffer.from(topic);
  const head = [0, t.length];
  const body = Buffer.concat([Buffer.from(head), t, qos ? Buffer.from([packetId >> 8, packetId & 255]) : Buffer.alloc(0), payload]);
  return Buffer.concat([Buffer.from([0x30 | (qos << 1), body.length]), body]);
};
const readN = (socket, n) => new Promise(resolve => {
  let got = Buffer.alloc(0);
  const on = chunk => { got = Buffer.concat([got, chunk]); if (got.length >= n) { socket.off("data", on); resolve(got); } };
  socket.on("data", on);
});

async function start() {
  const published = [], events = [];
  const broker = createBroker({ port: 0, host: "127.0.0.1", onPublish: (topic, payload, info) => published.push({ topic, payload, info }), onEvent: e => events.push(e) });
  const port = await broker.listen();
  return { broker, port, published, events };
}

test("topic filters match the way MQTT says", () => {
  assert.equal(topicMatches("aai12/#", "aai12/robot/telemetry"), true);
  assert.equal(topicMatches("aai12/robot/+", "aai12/robot/telemetry"), true);
  assert.equal(topicMatches("aai12/robot/+", "aai12/robot/a/b"), false);
  assert.equal(topicMatches("aai12/robot/telemetry", "aai12/robot/telemetry"), true);
  assert.equal(topicMatches("aai12/robot/telemetry", "aai12/robot"), false);
  assert.equal(topicMatches("#", "anything/at/all"), true);
});

test("a robot-style client connects, publishes at QoS 0 and 1, and pings", async () => {
  const { broker, port, published, events } = await start();
  const socket = net.connect(port, "127.0.0.1");
  await new Promise(resolve => socket.once("connect", resolve));
  socket.write(connectPacket("aai12-picow-dev", 30));
  assert.deepEqual([...(await readN(socket, 4))], [0x20, 0x02, 0x00, 0x00]);       // CONNACK, accepted

  const frame = Buffer.from("063039be90302010402d6903323bbd0cb49c", "hex");
  socket.write(publishPacket("aai12/robot/telemetry", frame));
  socket.write(publishPacket("aai12/test", Buffer.from("AAI12_PICO_W_ONLINE"), 1, 7));
  assert.deepEqual([...(await readN(socket, 4))], [0x40, 0x02, 0x00, 0x07]);        // PUBACK for packet 7
  socket.write(Buffer.from([0xc0, 0x00]));
  assert.deepEqual([...(await readN(socket, 2))], [0xd0, 0x00]);                    // PINGRESP

  assert.equal(published.length, 2);
  assert.equal(published[0].topic, "aai12/robot/telemetry");
  assert.deepEqual(published[0].payload, frame);
  assert.equal(published[0].info.clientId, "aai12-picow-dev");
  assert.equal(published[1].payload.toString(), "AAI12_PICO_W_ONLINE");
  assert.ok(events.some(e => e.kind === "connect" && e.clientId === "aai12-picow-dev" && e.keepaliveS === 30));
  assert.ok(events.some(e => e.kind === "pingreq"));
  assert.equal(broker.clientCount(), 1);
  socket.destroy();
  await broker.close();
});

test("a record split across TCP segments is still one publish", async () => {
  const { broker, port, published } = await start();
  const socket = net.connect(port, "127.0.0.1");
  await new Promise(resolve => socket.once("connect", resolve));
  socket.write(connectPacket("split", 0));
  await readN(socket, 4);
  const packet = publishPacket("aai12/robot/telemetry", Buffer.from("0100640000", "hex"));
  socket.write(packet.subarray(0, 5));
  await new Promise(resolve => setTimeout(resolve, 30));
  socket.write(packet.subarray(5));
  await new Promise(resolve => setTimeout(resolve, 30));
  assert.equal(published.length, 1);
  assert.equal(published[0].payload.toString("hex"), "0100640000");
  socket.destroy();
  await broker.close();
});

test("another MQTT client can subscribe and receive what the robot publishes", async () => {
  const { broker, port } = await start();
  const viewer = await mqtt.connectAsync(`mqtt://127.0.0.1:${port}`, { clientId: "viewer" });
  await viewer.subscribeAsync("aai12/#");
  const got = new Promise(resolve => viewer.once("message", (topic, payload) => resolve({ topic, payload })));
  const socket = net.connect(port, "127.0.0.1");
  await new Promise(resolve => socket.once("connect", resolve));
  socket.write(connectPacket("robot", 30));
  await readN(socket, 4);
  socket.write(publishPacket("aai12/robot/heartbeat", Buffer.from("v=1,b=00112233,seq=0,up=1000")));
  const message = await got;
  assert.equal(message.topic, "aai12/robot/heartbeat");
  assert.equal(message.payload.toString(), "v=1,b=00112233,seq=0,up=1000");
  socket.destroy();
  await viewer.endAsync();
  await broker.close();
});

test("a malformed packet drops that client and reports it", async () => {
  const { broker, port, events } = await start();
  const socket = net.connect(port, "127.0.0.1");
  await new Promise(resolve => socket.once("connect", resolve));
  socket.write(Buffer.from([0x10, 0x02, 0x00, 0x04]));                              // CONNECT too short to parse
  await new Promise(resolve => socket.once("close", resolve));
  assert.ok(events.some(e => e.kind === "protocol-error"));
  await broker.close();
});
