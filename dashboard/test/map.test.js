import test from "node:test";
import assert from "node:assert/strict";
import net from "node:net";
import { spawn } from "node:child_process";
import { createBroker } from "../server/broker.js";
import { parseMap, encodeMapBinary, decodeMapAck } from "../server/barcode-map.js";
import { routeMqttMessage } from "../server/message-router.js";
import { LatestPicoState } from "../server/latest-state.js";
import { loadConfig } from "../server/config.js";

const topics = loadConfig({}).mqtt.topics;

test("a letter map is checked with the robot's rules and sent in one canonical form", () => {
  assert.equal(parseMap("A=L,B=R,F=U").text, "A=L,B=R,F=U");
  assert.equal(parseMap("f=uturn; a:left\nB = R").ok, false);                 // "B = R" has spaces: not one entry
  assert.equal(parseMap("f=uturn; a:left\nB=right").text, "A=L,B=R,F=U");     // sorted, upper-cased, long words shortened
  assert.equal(parseMap("C=straight").text, "C=F");
  assert.equal(parseMap("C=S,D=FWD").text, "C=F,D=F");
  assert.equal(parseMap("clear").text, "CLEAR");
  assert.deepEqual(parseMap("A=L").entries, [{ letter: "A", action: "L" }]);
});

test("the 26-letters-in-order form is accepted only at exactly 26", () => {
  const ok = parseMap("LRFULRFULRFULRFULRFULRFULR");
  assert.equal(ok.ok, true);
  assert.equal(ok.entries.length, 26);
  assert.equal(ok.entries[3].letter, "D");
  assert.equal(ok.entries[3].action, "U");                                    // the 4th letter, D, takes the 4th action: U
  assert.equal(parseMap("LRFULRFU").ok, false);
});

test("a bad map is refused with a reason and nothing is sent", () => {
  for (const [input, why] of [["", /empty/], ["A", /letter=action/], ["A=X", /not an action/], ["1=L", /not a letter/],
                              ["A=L,A=R", /twice/], ["AA=L", /letter=action/], [42, /text/]]) {
    const r = parseMap(input);
    assert.equal(r.ok, false, String(input));
    assert.match(r.error, why);
  }
});

test("the robot's own acknowledgement text is routed to the page", () => {
  const ack = routeMqttMessage(topics.mapAck, Buffer.from("OK n=3"), topics, () => new Date("2026-11-20T12:00:00Z"));
  assert.equal(ack.type, "map-ack");
  assert.equal(ack.data, "OK n=3");
  const err = routeMqttMessage(topics.mapAck, Buffer.from("ERR syntax"), topics);
  assert.equal(err.data, "ERR syntax");
});

test("a page opened later is told the last map sent and what the car answered, and a new map forgets the old answer", () => {
  const state = new LatestPicoState();
  state.record({ type: "map-sent", receivedAt: "2026-11-20T12:00:00.000Z", data: { text: "A=L", entries: [], deliveredNow: 1 } });
  state.record({ type: "map-ack", receivedAt: "2026-11-20T12:00:01.000Z", data: "OK n=1" });
  assert.deepEqual(state.replay().map(m => m.type), ["map-sent", "map-ack"]);
  state.record({ type: "map-sent", receivedAt: "2026-11-20T12:05:00.000Z", data: { text: "F=U", entries: [], deliveredNow: 0 } });
  assert.deepEqual(state.replay().map(m => m.type), ["map-sent"]);
});

test("the map goes over MQTT as bytes: the same bytes the firmware's own tests use", () => {
  // golden bytes from the C side (tests/host/test_barcode_map.c): A=L,B=R,F=U is 12 01 0a 2c
  assert.equal(encodeMapBinary(parseMap("A=L,B=R,F=U").entries).toString("hex"), "12010a2c");
  assert.equal(encodeMapBinary(parseMap("f=u, a=left").entries).toString("hex"), "12012c");     // sorted by letter
  assert.equal(encodeMapBinary(parseMap("CLEAR").entries).toString("hex"), "12");               // the type byte alone clears
  assert.equal(encodeMapBinary(parseMap("Z=U").entries).toString("hex"), "12cc");              // 25 << 3 | 4
  const all = parseMap("LRFULRFULRFULRFULRFULRFULR");
  assert.equal(encodeMapBinary(all.entries).length, 27);                                        // 26 letters: 27 bytes, was up to 100 as text
  assert.equal(encodeMapBinary(parseMap("A=L,B=R,F=U").entries).length, 4);                     // 3 letters: 4 bytes, was 11 as text
});

test("the car's reply is one byte after the type: result in 3 bits, letters in 5", () => {
  assert.deepEqual(decodeMapAck(Buffer.from("1303", "hex")), { binary: true, ok: true, result: 0, count: 3, text: "OK n=3" });
  assert.equal(decodeMapAck(Buffer.from("131a", "hex")).text, "OK n=26");
  assert.equal(decodeMapAck(Buffer.from("13a0", "hex")).text, "ERR duplicate");
  assert.equal(decodeMapAck(Buffer.from("1340", "hex")).text, "ERR syntax");
  assert.equal(decodeMapAck(Buffer.from("1380", "hex")).text, "ERR action");
  assert.equal(decodeMapAck(Buffer.from("13e0", "hex")).text, "ERR unknown");
  // older firmware answers in text: still understood, not mistaken for the binary form
  assert.deepEqual(decodeMapAck(Buffer.from("OK n=3")), { binary: false, ok: true, text: "OK n=3" });
  assert.equal(decodeMapAck(Buffer.from("ERR syntax")).ok, false);
  assert.equal(decodeMapAck(Buffer.from("13", "hex")).binary, false);                           // a truncated binary reply is not trusted
});

// ---- the bytes the robot actually sends and expects ----------------------------------------------------------------
const connectPacket = id => {
  const idBuf = Buffer.from(id);
  const body = Buffer.concat([Buffer.from([0, 4, 0x4d, 0x51, 0x54, 0x54, 4, 0x02, 0, 30, 0, idBuf.length]), idBuf]);
  return Buffer.concat([Buffer.from([0x10, body.length]), body]);
};
// what mqtt_wire_subscribe() builds: packet id 1, one filter, requested QoS 0
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

test("a car that subscribes to the map topic gets a retained map right after the SUBACK, even if it was sent earlier", async () => {
  const broker = createBroker({ port: 0, host: "127.0.0.1" });
  const port = await broker.listen();
  try {
    assert.equal(broker.publish("aai12/robot/map", Buffer.from("A=L,F=U"), { retain: true }), 0);   // nobody connected yet
    const car = net.connect(port, "127.0.0.1");
    await new Promise(r => car.once("connect", r));
    const rx = collect(car);
    car.write(connectPacket("aai12-robot"));
    car.write(subscribePacket(1, "aai12/robot/map"));
    const bytes = await rx.until(4 + 5 + 2 + 2 + 15 + 7);
    assert.deepEqual([...bytes.subarray(0, 4)], [0x20, 0x02, 0x00, 0x00]);                              // CONNACK
    assert.deepEqual([...bytes.subarray(4, 9)], [0x90, 0x03, 0x00, 0x01, 0x00]);                        // SUBACK id 1, QoS 0
    const pub = bytes.subarray(9);
    assert.equal(pub[0], 0x31);                                                                         // PUBLISH, QoS 0, RETAIN
    assert.equal(pub[1], 2 + 15 + 7);
    assert.equal(pub.subarray(4, 19).toString(), "aai12/robot/map");
    assert.equal(pub.subarray(19).toString(), "A=L,F=U");

    // a map sent while the car is online is delivered at once (not as a retained redelivery)
    assert.equal(broker.publish("aai12/robot/map", Buffer.from("B=R"), { retain: true }), 1);
    const more = await rx.until(bytes.length + 2 + 2 + 15 + 3);
    assert.equal(more.subarray(bytes.length)[0], 0x30);
    assert.equal(more.subarray(bytes.length + 19).toString(), "B=R");

    // and the newest one is what a later subscriber would get
    assert.equal(broker.retained().get("aai12/robot/map").toString(), "B=R");
    broker.publish("aai12/robot/map", Buffer.alloc(0), { retain: true });                               // empty clears it
    assert.equal(broker.retained().has("aai12/robot/map"), false);
    car.destroy();
  } finally { await broker.close(); }
});

test("a client that does not subscribe to the map topic gets nothing", async () => {
  const broker = createBroker({ port: 0, host: "127.0.0.1" });
  const port = await broker.listen();
  try {
    broker.publish("aai12/robot/map", Buffer.from("A=L"), { retain: true });
    const other = net.connect(port, "127.0.0.1");
    await new Promise(r => other.once("connect", r));
    const rx = collect(other);
    other.write(connectPacket("someone-else"));
    other.write(subscribePacket(1, "aai12/robot/telemetry"));
    await rx.until(9);
    await new Promise(r => setTimeout(r, 100));
    assert.equal(rx.all().length, 9);                                                                   // CONNACK + SUBACK only
    other.destroy();
  } finally { await broker.close(); }
});

// ---- the whole path: HTTP POST -> broker -> a car socket, and the car's answer -> the page ---------------------------
test("POST /api/map validates, publishes retained to the car, and the car's map_ack reaches the browser", async () => {
  const web = 39170, brokerPort = 39183;
  const child = spawn(process.execPath, ["server/index.js"], { env: { ...process.env, PORT: String(web), BROKER_PORT: String(brokerPort), BROKER_HOST: "127.0.0.1" }, stdio: "ignore" });
  try {
    let up = false;
    for (let i = 0; i < 100 && !up; i++) {
      up = await fetch(`http://127.0.0.1:${web}/api/map`, { method: "GET" }).then(() => true, () => false);
      if (!up) await new Promise(r => setTimeout(r, 50));
    }
    assert.ok(up, "the dashboard started");
    await new Promise(r => setTimeout(r, 200));

    const car = net.connect(brokerPort, "127.0.0.1");
    await new Promise(r => car.once("connect", r));
    const rx = collect(car);
    car.write(connectPacket("aai12-robot"));
    car.write(subscribePacket(1, "aai12/robot/map"));
    await rx.until(9);

    const bad = await fetch(`http://127.0.0.1:${web}/api/map`, { method: "POST", body: "A=Q" });
    assert.equal(bad.status, 400);
    assert.match((await bad.json()).error, /not an action/);

    const good = await fetch(`http://127.0.0.1:${web}/api/map`, { method: "POST", body: JSON.stringify({ map: "f=u, a=left" }) });
    const result = await good.json();
    assert.equal(good.status, 200);
    assert.equal(result.text, "A=L,F=U");
    assert.equal(result.deliveredNow, 1);
    const bytes = await rx.until(9 + 2 + 2 + 15 + 3);
    assert.equal(bytes.subarray(9 + 19).toString("hex"), "12012c");                                     // exactly what the car's parser reads: A=L,F=U in 3 bytes

    // the car answers; the dashboard forwards it to the page
    const answer = Buffer.from("1302", "hex");                                                          // the car's binary reply: OK, 2 letters
    const t = Buffer.from("aai12/robot/map_ack");
    const body = Buffer.concat([Buffer.from([0, t.length]), t, answer]);
    const seen = new Promise(resolve => {
      const ws = new WebSocket(`ws://127.0.0.1:${web}/live`);
      ws.onmessage = e => { const m = JSON.parse(e.data); if (m.type === "map-ack") { ws.close(); resolve(m); } };
    });
    await new Promise(r => setTimeout(r, 100));
    car.write(Buffer.concat([Buffer.from([0x30, body.length]), body]));
    const ack = await seen;
    assert.equal(ack.data, "OK n=2");
    car.destroy();
  } finally { child.kill(); }
});
