// Summarise a recording made with record-feed.mjs: one lap report you can attach as evidence.
//
//   node tools/feed-summary.mjs lap1.jsonl
//
// It reports: duration and message rates, time spent in each vehicle state, the distance run and speeds, every barcode / junction / hump / obstacle
// / plan / fault event with its time, the barcode, hump and encoder counters, lift and IMU status, gaps in the feed (the car silent for more than 2 s),
// broker connections and acknowledgements.
//
// Limits: only what the car sends over WiFi. Task run times, tick overruns and the [stop] timing lines are in the serial log, not here, and the
// event times have 10 ms resolution (the car stamps them in 10 ms units).
import { readFileSync } from "node:fs";

const file = process.argv[2];
if (!file) { console.error("usage: node tools/feed-summary.mjs <recording.jsonl>"); process.exit(2); }
const rows = readFileSync(file, "utf8").split("\n").filter(Boolean).map(l => JSON.parse(l));
if (!rows.length) { console.error("the recording is empty"); process.exit(3); }

const t0 = rows[0].t, tEnd = rows.at(-1).t;
const at = t => `${(((t - t0) / 1000)).toFixed(1).padStart(6)} s`;
const frames = rows.filter(r => r.m.type === "robot-frame");
const kinds = {};
for (const r of frames) kinds[r.m.kind] = (kinds[r.m.kind] ?? 0) + 1;
const secs = (tEnd - t0) / 1000;

console.log(`# Lap report: ${file}`);
console.log(`\nRecorded ${new Date(t0).toLocaleString()} for ${secs.toFixed(1)} s, ${rows.length} messages (${frames.length} robot records).`);
console.log("Records per second: " + Object.entries(kinds).map(([k, n]) => `${k} ${(n / secs).toFixed(1)}`).join(", "));

// ---- states
const statuses = frames.filter(r => r.m.kind === "status");
const timeIn = {}, changes = [];
let prev = null;
for (let i = 0; i < statuses.length; i++) {
  const r = statuses[i], name = r.m.data.stateName;
  const next = statuses[i + 1]?.t ?? tEnd;
  timeIn[name] = (timeIn[name] ?? 0) + (next - r.t);
  if (name !== prev) { changes.push({ t: r.t, name, faults: r.m.data.faultNames }); prev = name; }
}
console.log(`\n## Vehicle states (${changes.length} changes)`);
console.log("| State | Time (s) | Share |\n|---|---|---|");
for (const [name, ms] of Object.entries(timeIn).sort((a, b) => b[1] - a[1])) console.log(`| ${name} | ${(ms / 1000).toFixed(1)} | ${Math.round((ms / (tEnd - t0)) * 100)} % |`);
console.log("\nTimeline: " + changes.map(c => `${at(c.t).trim()} ${c.name}`).join("  >  "));

// ---- motion
if (statuses.length) {
  const dist = statuses.map(r => r.m.data.distanceMm), speeds = statuses.flatMap(r => [Math.abs(r.m.data.leftMmS), Math.abs(r.m.data.rightMmS)]);
  console.log(`\n## Motion\nDistance run: ${(Math.max(...dist) - Math.min(...dist))} mm. Fastest wheel: ${Math.max(...speeds)} mm/s.`);
}

// ---- events
const events = frames.filter(r => r.m.kind === "event").map(r => ({ t: r.t, name: r.m.data.name, text: String(r.m.data.detail ?? r.m.data.arg ?? "") }));
const count = n => events.filter(e => e.name === n).length;
console.log(`\n## Events (${events.length})`);
const importantNames = ["BARCODE", "BARCODE REJECTED", "JUNCTION", "HUMP", "OBSTACLE", "PLAN", "RECOVERED", "FAULT", "FAULT CLEAR", "LINE LOST", "LINE FOUND", "LIFT"];
const byName = {};
for (const e of events) byName[e.name] = (byName[e.name] ?? 0) + 1;
console.log("Counts: " + Object.entries(byName).map(([k, n]) => `${k} ${n}`).join(", "));
const listed = events.filter(e => e.name !== "STATE" && !(e.name === "LINE LOST" || e.name === "LINE FOUND"));
console.log("\n| Time | Event | Detail |\n|---|---|---|");
for (const e of listed.slice(0, 80)) console.log(`| ${at(e.t).trim()} | ${e.name} | ${e.text} |`);
if (listed.length > 80) console.log(`| ... | ${listed.length - 80} more | |`);

// ---- barcode, hump, counters
const sensors = frames.filter(r => r.m.kind === "sensors").map(r => r.m.data);
const last = sensors.at(-1);
if (last) {
  console.log("\n## Barcode, encoders, IMU, sonar (last sensor record, and extremes)");
  console.log(`- Barcode: decoded ${last.decoded}, rejected ${last.rejected}, completed ${last.barsDone}, last letter ${last.lastLetterChar ?? "none"}, letters mapped ${last.mapEntries}, junctions ${last.junctions}, executed ${last.executed}`);
  console.log(`- Encoder bad transitions: left ${Math.max(...sensors.map(s => s.encBadLeft))}, right ${Math.max(...sensors.map(s => s.encBadRight))} (counters stop at 255); encoder fault seen: ${sensors.some(s => s.encoderFault) ? "YES" : "no"}`);
  console.log(`- IMU: ok in ${sensors.filter(s => s.imuOk).length}/${sensors.length} records; lift latched seen: ${sensors.some(s => s.liftLatched) ? "YES" : "no"}; largest sideways tilt ${Math.max(...sensors.map(s => Math.abs(s.rollDeg))).toFixed(1)} degrees`);
  const us = sensors.filter(s => s.usValid).map(s => s.usMm);
  console.log(`- Sonar: ${us.length}/${sensors.length} valid readings${us.length ? `, closest ${Math.min(...us)} mm` : ""}; sensor fault seen: ${sensors.some(s => s.usFault) ? "YES" : "no"}`);
  console.log(`- Map in the car: ${last.mapEntries} letters`);
}
const humps = frames.filter(r => r.m.kind === "hump").map(r => r.m.data);
if (humps.length) {
  console.log(`\n## Hump records (${humps.length})\n| Height (mm) | Validity | Highest valid so far |\n|---|---|---|`);
  for (const h of humps) console.log(`| ${h.hump.heightMm} | ${h.hump.validityName} | ${h.highestHump ?? "none"} |`);
}
const obstacles = frames.filter(r => r.m.kind === "obstacle");
if (obstacles.length) {
  console.log(`\n## Obstacle scans (${obstacles.length})\n| Time | Closest (cm) | Width (cm) | Centre (cm) | Plan | Reason |\n|---|---|---|---|---|---|`);
  for (const r of obstacles) { const d = r.m.data; console.log(`| ${at(r.t).trim()} | ${d.valid ? d.closestCm : "-"} | ${d.valid ? d.widthCm : "-"} | ${d.valid ? d.centreCm : "-"} | ${d.actionName} | ${d.reasonName} |`); }
}

// ---- faults
const faultSeen = {};
for (const r of statuses) for (const f of r.m.data.faultNames) faultSeen[f] = (faultSeen[f] ?? 0) + 1;
console.log("\n## Faults seen in status records");
console.log(Object.keys(faultSeen).length ? Object.entries(faultSeen).map(([f, n]) => `${f} (${n} records)`).join(", ") : "none");

// ---- feed health
const gaps = [];
for (let i = 1; i < frames.length; i++) if (frames[i].t - frames[i - 1].t > 2000) gaps.push({ from: frames[i - 1].t, secs: (frames[i].t - frames[i - 1].t) / 1000 });
const broker = rows.filter(r => r.m.type === "broker-event").map(r => ({ t: r.t, ...r.m.data }));
const hb = frames.filter(r => r.m.kind === "heartbeat").map(r => r.m.data);
const acks = frames.filter(r => r.m.kind === "ack").map(r => ({ t: r.t, id: r.m.data.reqId, result: r.m.data.resultName }));
console.log("\n## Feed health");
console.log(gaps.length ? `Gaps of more than 2 s with no robot data: ${gaps.map(g => `${at(g.from).trim()} for ${g.secs.toFixed(1)} s`).join(", ")}` : "No gap of more than 2 s in the robot data.");
console.log(`Broker events: ${broker.filter(b => b.kind === "connect").length} connects, ${broker.filter(b => b.kind === "disconnect").length} disconnects, ${broker.filter(b => b.kind === "keepalive-timeout").length} keep-alive timeouts`);
if (hb.length) console.log(`Telemetry records dropped by the car (counter): ${hb[0].dropped} -> ${hb.at(-1).dropped} (+${hb.at(-1).dropped - hb[0].dropped}); queue depth now ${hb.at(-1).queueDepth}`);
if (acks.length) console.log("Acknowledgements: " + acks.map(a => `${at(a.t).trim()} request ${a.id} ${a.result}`).join("; "));
console.log("\nNot in this report (serial log only): task run times, tick overruns, [stop] timing lines.");
