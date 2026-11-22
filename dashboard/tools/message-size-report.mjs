// Message size comparison for the Week 13 evidence: the same telemetry records in three representations.
//
//   packed   the firmware's minimum-bit records (bytes measured from the C encoders: the golden records in test/robot-frames.test.js)
//   natural  the same fields at their natural C widths (uint8/16/32, bool as a byte), no padding, plus the 1-byte type header.
//            The struct sizes are measured on the Cortex-M0+ with arm-none-eabi-gcc -fpack-struct=1 (see docs/evidence/message_size.md).
//   text     the same fields as compact JSON ("name":value), counted from the decoded records. Conservative: only the raw numeric
//            and boolean fields are counted (names for enumerations and derived values are left out), so real JSON would be larger.
//
//   node tools/message-size-report.mjs
import { decodeRobotFrame } from "../server/robot-frames.js";

// Golden records produced by the firmware's encoders (the same bytes the unit tests check).
const GOLDEN = {
  status: "010064a24f061685806e4c02088525a2c0",
  event: "02009608006038",
  hump: "0300c84968b0",
  obstacle: "0400fa8a02c17c800080",
  heartbeat: "0500000e100005021b",
  sensors: "063039be90302010402d6903323bbd0cb49c",
  command: "1007200000",
  ack: "110720"
};
// sizeof of the decode structs on the Cortex-M0+ with -fpack-struct=1 (no padding), then + 1 byte of message type.
const NATURAL_STRUCT = { status: 26, event: 6, hump: 5, obstacle: 15, heartbeat: 9, sensors: 54, command: 4, ack: 2 };
const TOPIC = "aai12/robot/telemetry";

const derived = /(Name|Names|Char)$|^(hex|kind|type|byteLength|rollDeg|highestHump|hump|ir|irValid)$/;
function rawFields(frame) {
  const out = {};
  for (const [k, v] of Object.entries(frame)) {
    if (derived.test(k)) continue;
    if (typeof v === "number" || typeof v === "boolean") out[k] = v;
  }
  return out;
}
const mqttOverhead = payload => 1 + (payload < 128 ? 1 : payload < 16384 ? 2 : 3) + 2 + TOPIC.length;   // fixed header + remaining length + topic

const rows = [];
for (const [name, hex] of Object.entries(GOLDEN)) {
  const bytes = Buffer.from(hex, "hex");
  let text;
  if (name === "command") text = JSON.stringify({ reqId: 7, cmd: 2, param: 0 });
  else text = JSON.stringify(rawFields(decodeRobotFrame(bytes)));
  rows.push({ name, packed: bytes.length, natural: NATURAL_STRUCT[name] + 1, text: Buffer.byteLength(text) });
}
const pct = (baseline, packed) => (baseline === packed ? "same" : `${Math.round(((baseline - packed) / baseline) * 100)} % smaller`);

console.log("| Record | Packed (bytes) | Natural-width binary | Compact text | Packed saves vs natural | Packed saves vs text | MQTT packet, packed | MQTT packet, text |");
console.log("|---|---|---|---|---|---|---|---|");
let tp = 0, tn = 0, tt = 0;
for (const r of rows) {
  tp += r.packed; tn += r.natural; tt += r.text;
  console.log(`| ${r.name} | ${r.packed} | ${r.natural} | ${r.text} | ${pct(r.natural, r.packed)} | ${pct(r.text, r.packed)} | ${r.packed + mqttOverhead(r.packed)} | ${r.text + mqttOverhead(r.text)} |`);
}
console.log(`| **all 8 records** | **${tp}** | **${tn}** | **${tt}** | **${pct(tn, tp)}** | **${pct(tt, tp)}** | | |`);
// what the car sends in one second at the current rates: status 10 Hz, sensors 2 Hz, heartbeat 1 Hz
const perSecond = key => rows.find(r => r.name === "status")[key] * 10 + rows.find(r => r.name === "sensors")[key] * 2 + rows.find(r => r.name === "heartbeat")[key];
console.log(`\nSteady stream (status 10 Hz + sensors 2 Hz + heartbeat 1 Hz), payload only: packed ${perSecond("packed")} B/s, natural ${perSecond("natural")} B/s, text ${perSecond("text")} B/s.`);
