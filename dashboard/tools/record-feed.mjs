// Record the car's live feed from the dashboard to a file, for runs where a USB cable to the car is not possible.
//
//   node tools/record-feed.mjs [--out lap1.jsonl] [--url ws://localhost:3000/live] [--seconds 300]
//
// The dashboard must be running with the car connected. Every live message is saved, one JSON object per line with the time the
// laptop received it. Stop with Ctrl+C (or --seconds); the summary is printed straight away, and `node tools/feed-summary.mjs lap1.jsonl`
// reprints it later.
//
// What the car sends over WiFi, and so what this records: vehicle state and faults, line position, wheel speeds and distance, events (barcode, junction,
// hump, obstacle, plan, faults), IMU tilt and lift, sonar distance, barcode and encoder counters, acknowledgements, and the broker connections.
// What it does NOT contain: the serial log's task run times, tick overruns and the [stop] timing lines. Those need the USB cable.
import { createWriteStream } from "node:fs";
import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";

const arg = (name, fallback) => {
  const i = process.argv.indexOf(`--${name}`);
  return i > 0 ? process.argv[i + 1] : fallback;
};
const WS_URL = arg("url", "ws://localhost:3000/live");
const OUT = arg("out", `feed-${new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19)}.jsonl`);
const SECONDS = Number(arg("seconds", 0));
const stamp = () => new Date().toLocaleTimeString([], { hour12: false });

const file = createWriteStream(OUT);
let count = 0, lastState = "-", lastEvent = "-", lastFrameAt = Date.now(), started = null;
const ws = new WebSocket(WS_URL);
ws.onerror = () => { console.error(`Cannot reach the dashboard at ${WS_URL}: is it running?`); process.exit(2); };
ws.onopen = () => console.log(`${stamp()}  recording to ${OUT}  (Ctrl+C to stop)`);
ws.onmessage = e => {
  const m = JSON.parse(e.data);
  if (m.cached) return;                                   // a replay of what the page already had: not live
  started ??= Date.now();
  count++;
  file.write(JSON.stringify({ t: Date.now(), m }) + "\n");
  if (m.type === "robot-frame") {
    lastFrameAt = Date.now();
    if (m.kind === "status") lastState = m.data.stateName;
    if (m.kind === "event") lastEvent = `${m.data.name} ${m.data.detail ?? m.data.arg ?? ""}`.trim();
  }
};

setInterval(() => {
  const silent = (Date.now() - lastFrameAt) / 1000;
  console.log(`${stamp()}  ${count} messages  state=${lastState}  last event=${lastEvent}${silent > 3 ? `  (NO ROBOT DATA FOR ${silent.toFixed(0)} s)` : ""}`);
}, 5000);

let finishing = false;
function finish() {
  if (finishing) return;
  finishing = true;
  try { ws.close(); } catch { /* already closed */ }
  file.end(() => {
    console.log(`\nSaved ${count} messages to ${OUT}\n`);
    if (count > 0) {
      const summary = fileURLToPath(new URL("./feed-summary.mjs", import.meta.url));
      spawnSync(process.execPath, [summary, OUT], { stdio: "inherit" });
    }
    process.exit(0);
  });
}
process.on("SIGINT", finish);
if (SECONDS > 0) setTimeout(finish, SECONDS * 1000);
