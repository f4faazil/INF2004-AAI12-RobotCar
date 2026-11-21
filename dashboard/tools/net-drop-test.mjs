// WiFi / MQTT drop-and-recover test (Tribunal 1: "WiFi/Telemetry + Motion - timing").
//
//   node tools/net-drop-test.mjs [--trials 10] [--down 20] [--wait 60] [--port 1883] [--out net-drop.csv]
//
// This program REPLACES the dashboard for the length of the test: it runs the dashboard's own MQTT broker, and in each
// trial it takes the broker away (the car loses its connection, its telemetry buffer fills and records are dropped), brings it
// back after --down seconds and measures how long the car takes to
//   (a) connect again, and
//   (b) deliver its first telemetry record again.
// Success criterion from the Tribunal: reconnection within 30 s in at least 9 of 10 trials.
//
// What it cannot see: whether Motion kept its 5 ms deadlines while the network was down. That is in the car's serial log:
// run the console with --log in parallel and read the "ovr=" and "us(m/l/v/isr)=" fields (see the end of this file's output).
//
// Stop the dashboard first (both want port 1883). Run with the car switched on, ideally with START pressed and the wheels off the
// floor so Motion is under load.
import { writeFileSync, appendFileSync } from "node:fs";
import net from "node:net";
import { createBroker } from "../server/broker.js";
import { decodeRobotFrame } from "../server/robot-frames.js";

const arg = (name, fallback) => {
  const i = process.argv.indexOf(`--${name}`);
  return i > 0 ? process.argv[i + 1] : fallback;
};
const TRIALS = Number(arg("trials", 10));
const DOWN_S = Number(arg("down", 20));
const WAIT_S = Number(arg("wait", 60));
const PORT = Number(arg("port", 1883));
const LIMIT_S = 30;
const OUT = arg("out", `net-drop-${new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19)}.csv`);
const sleep = ms => new Promise(r => setTimeout(r, ms));
const now = () => Date.now();
const stamp = () => new Date().toLocaleTimeString([], { hour12: false });
const say = text => console.log(`${stamp()}  ${text}`);

async function portFree(port) {
  return new Promise(resolve => {
    const probe = net.createServer().once("error", () => resolve(false)).once("listening", () => probe.close(() => resolve(true)));
    probe.listen(port, "0.0.0.0");
  });
}

if (!(await portFree(PORT))) {
  console.error(`Port ${PORT} is in use: stop the dashboard (Ctrl+C in its terminal) and run this again.`);
  process.exit(2);
}

// State shared with the broker callbacks.
const seen = { connectAt: null, firstRecordAt: null, records: 0, dropped: null, faults: null, queueDepth: null, clientId: null };
let broker = null;

function startBroker() {
  Object.assign(seen, { connectAt: null, firstRecordAt: null, records: 0 });
  broker = createBroker({
    port: PORT, host: "0.0.0.0",
    onEvent: e => { if (e.kind === "connect") { seen.connectAt ??= now(); seen.clientId = e.clientId; } },
    onPublish: (topic, payload) => {
      if (!topic.endsWith("/telemetry")) return;
      seen.records++;
      seen.firstRecordAt ??= now();
      try {
        const f = decodeRobotFrame(payload);
        if (f.kind === "heartbeat") { seen.dropped = f.dropped; seen.queueDepth = f.queueDepth; }
        if (f.kind === "status") seen.faults = f.faultNames.join(";") || "none";
      } catch { /* a malformed record is not this test's concern */ }
    }
  });
  return broker.listen();
}

async function waitFor(predicate, seconds) {
  const end = now() + seconds * 1000;
  while (now() < end) { if (predicate()) return true; await sleep(100); }
  return predicate();
}

writeFileSync(OUT, "trial,down_s,reconnect_s,first_record_s,records_in_first_5s,heartbeat_dropped,queue_depth,faults_after,result\n");
say(`Net drop test: ${TRIALS} trials, ${DOWN_S} s down each, limit ${LIMIT_S} s. Log: ${OUT}`);

await startBroker();
say(`Broker up on port ${PORT}. Waiting up to 120 s for the car to connect and send telemetry...`);
if (!(await waitFor(() => seen.firstRecordAt !== null, 120))) {
  console.error("The car never connected. Check it is on, joined to the hotspot, and that the laptop address matches the firmware's broker address.");
  await broker.close();
  process.exit(3);
}
say(`Car connected as "${seen.clientId}" and is sending. Starting the trials.`);
await sleep(5000);

const rows = [];
for (let trial = 1; trial <= TRIALS; trial++) {
  say(`trial ${trial}/${TRIALS}: taking the broker down for ${DOWN_S} s (the car's buffer will fill and drop records)`);
  await broker.close();
  await sleep(DOWN_S * 1000);
  const restoredAt = now();
  await startBroker();
  say(`trial ${trial}: broker back; waiting up to ${WAIT_S} s for the car`);
  await waitFor(() => seen.firstRecordAt !== null, WAIT_S);
  const reconnect = seen.connectAt ? (seen.connectAt - restoredAt) / 1000 : null;
  const first = seen.firstRecordAt ? (seen.firstRecordAt - restoredAt) / 1000 : null;
  await sleep(5000);                                     // let heartbeat and status arrive for the drop counter
  const pass = first !== null && first <= LIMIT_S;
  rows.push({ trial, reconnect, first, pass });
  const line = [trial, DOWN_S, reconnect?.toFixed(1) ?? "", first?.toFixed(1) ?? "", seen.records, seen.dropped ?? "", seen.queueDepth ?? "", seen.faults ?? "", pass ? "PASS" : "FAIL"].join(",");
  appendFileSync(OUT, line + "\n");
  say(`trial ${trial}: reconnected after ${reconnect?.toFixed(1) ?? "never"} s, data flowing after ${first?.toFixed(1) ?? "never"} s, car reports dropped=${seen.dropped ?? "?"} -> ${pass ? "PASS" : "FAIL"}`);
}
await broker.close();

const passes = rows.filter(r => r.pass).length;
const needed = Math.ceil(TRIALS * 0.9);
console.log("\n--- summary ---");
console.log(`passed ${passes}/${TRIALS} (criterion: at least ${needed}/${TRIALS} within ${LIMIT_S} s)`);
const times = rows.filter(r => r.first !== null).map(r => r.first).sort((a, b) => a - b);
if (times.length) console.log(`time to data: min ${times[0].toFixed(1)} s, median ${times[Math.floor(times.length / 2)].toFixed(1)} s, max ${times.at(-1).toFixed(1)} s`);
console.log(passes >= needed ? "RESULT: PASS" : "RESULT: FAIL");
console.log(`\nNow check Motion in the serial log (python serial_console.py --log ...), for the same time window:`);
console.log(`  the highest "ovr=a/b/c" (tick overruns: motion/control/vehicle) and "us(m/l/v/isr)" (worst task times in microseconds).`);
console.log(`  Success: no new Motion overruns during the drops (a=0), and "drop=" in the [robot] line grows while the broker is down.`);
process.exit(passes >= needed ? 0 : 1);
