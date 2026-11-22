import { createRequire } from "node:module";
const require = createRequire(import.meta.url);
const mqtt = require("mqtt");
const c = mqtt.connect("mqtt://127.0.0.1:1883", { clientId: "map-cases-test" });
const replies = [];
c.on("message", (t, p) => replies.push(p.toString("hex")));
await new Promise(r => c.on("connect", r));
await new Promise(r => c.subscribe("aai12/robot/map_ack", r));
const cases = [
  ["26 letters (LRFU x6 + LR)", Buffer.from([0x12, ...Array.from({length: 26}, (_, i) => (i << 3) | ((i % 4) + 1))])],
  ["clear (type byte only)", Buffer.from([0x12])],
  ["duplicate letter A twice", Buffer.from([0x12, 0x01, 0x02])],
  ["command 0 (none) for A", Buffer.from([0x12, 0x00])],
  ["command 7 for A", Buffer.from([0x12, 0x07])],
  ["27 letters (too many)", Buffer.from([0x12, ...Array.from({length: 27}, (_, i) => ((i % 26) << 3) | 1)])],
  ["valid again: A=L,B=R,F=U", Buffer.from("12010a2c", "hex")],
];
for (const [name, payload] of cases) {
  replies.length = 0;
  c.publish("aai12/robot/map", payload, { retain: false });
  await new Promise(r => setTimeout(r, 1500));
  console.log(name.padEnd(30), "sent", payload.toString("hex").slice(0, 16).padEnd(16), "-> reply", replies.join(",") || "(none)");
}
c.end();
