// Guided desk tests that use a ruler and the live dashboard feed (the dashboard must be running and the car connected).
//
//   node tools/desk-ruler.mjs sonar   [--distances 100,150,200,250,300,400,500] [--seconds 8] [--out sonar.csv]
//   node tools/desk-ruler.mjs encoder [--length 500] [--cycles 3] [--out encoder.csv]
//
// sonar:   ultrasonic accuracy against known distances (Tribunal 1, "GPIO ISR dispatcher": error within +-20 mm over 100 to 500 mm).
//          Put a flat board (at least 20 cm wide) square in front of the sonar and measure from the sonar's FRONT FACE to the board.
// encoder: wheel-encoder distance and direction against a ruler, with the sonar's ECHO interrupts running at the same time.
//          Roll the car by hand (motors off, car in IDLE or STOPPED) along a ruler: forward a known length, then back to the mark.
//
// It prints the numbers and a verdict, and writes a CSV you can attach as evidence. It does not change anything on the car.
import { createInterface } from "node:readline/promises";
import { writeFileSync, appendFileSync } from "node:fs";

const mode = process.argv[2];
const arg = (name, fallback) => {
  const i = process.argv.indexOf(`--${name}`);
  return i > 0 ? process.argv[i + 1] : fallback;
};
if (!["sonar", "encoder"].includes(mode)) {
  console.error("usage: node tools/desk-ruler.mjs sonar|encoder [options]   (see the top of this file)");
  process.exit(2);
}
const URL = arg("url", "ws://localhost:3000/live");
const OUT = arg("out", `${mode}-${new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19)}.csv`);
const SECONDS = Number(arg("seconds", 8));
const sleep = ms => new Promise(r => setTimeout(r, ms));

const latest = { status: null, sensors: null, sensorsAt: 0 };
let sonarSamples = null;                                    // while collecting
const ws = new WebSocket(URL);
await new Promise((resolve, reject) => { ws.onopen = resolve; ws.onerror = () => reject(new Error(`cannot reach the dashboard at ${URL}: is it running?`)); }).catch(e => { console.error(e.message); process.exit(3); });
ws.onmessage = e => {
  const m = JSON.parse(e.data);
  if (m.type !== "robot-frame" || m.cached) return;
  if (m.kind === "status") latest.status = m.data;
  if (m.kind === "sensors") {
    latest.sensors = m.data; latest.sensorsAt = Date.now();
    if (sonarSamples) sonarSamples.push({ valid: m.data.usValid, mm: m.data.usMm });
  }
};
const rl = createInterface({ input: process.stdin, output: process.stdout });
const ask = text => rl.question(`\n${text}\n  -> press Enter when ready: `);
const median = a => { const s = [...a].sort((x, y) => x - y); return s.length ? s[Math.floor(s.length / 2)] : null; };

console.log("Waiting for live data from the car...");
for (let i = 0; i < 100 && !(latest.sensors && latest.status); i++) await sleep(100);
if (!(latest.sensors && latest.status)) { console.error("No live data: is the car on, connected and the dashboard receiving it?"); process.exit(4); }

if (mode === "sonar") {
  const distances = arg("distances", "100,150,200,250,300,400,500").split(",").map(Number);
  writeFileSync(OUT, "true_mm,median_mm,error_mm,min_mm,max_mm,valid,total,within_20mm\n");
  console.log(`Sonar ruler test: ${distances.length} distances, ${SECONDS} s of readings at each (the dashboard's sonar value updates twice a second).`);
  console.log("Keep the car still, servo straight ahead, board flat and square to the sonar.");
  const rows = [];
  for (const d of distances) {
    await ask(`Place the board ${d} mm from the FRONT FACE of the sonar.`);
    await sleep(500);
    sonarSamples = [];
    await sleep(SECONDS * 1000);
    const all = sonarSamples; sonarSamples = null;
    const ok = all.filter(s => s.valid).map(s => s.mm);
    const med = median(ok);
    const err = med === null ? null : med - d;
    const pass = err !== null && Math.abs(err) <= 20;
    rows.push({ d, med, err, pass });
    appendFileSync(OUT, [d, med ?? "", err ?? "", ok.length ? Math.min(...ok) : "", ok.length ? Math.max(...ok) : "", ok.length, all.length, pass ? "yes" : "no"].join(",") + "\n");
    console.log(`  true ${d} mm: median ${med ?? "no echo"} mm, error ${err === null ? "-" : (err > 0 ? "+" : "") + err} mm, range ${ok.length ? Math.min(...ok) + ".." + Math.max(...ok) : "-"}, ${ok.length}/${all.length} valid  ${pass ? "OK" : "OUT OF +-20 mm"}`);
  }
  const bad = rows.filter(r => !r.pass);
  console.log("\n--- summary ---");
  console.log(`within +-20 mm: ${rows.length - bad.length}/${rows.length}`);
  if (bad.length) console.log(`outside: ${bad.map(r => `${r.d} mm (${r.err === null ? "no echo" : (r.err > 0 ? "+" : "") + r.err})`).join(", ")}`);
  console.log(bad.length === 0 ? "RESULT: PASS (error within +-20 mm over the tested range)" : "RESULT: FAIL for the distances above. A consistent over-read can be removed with a correction table (true vs median is in the CSV).");
} else {
  const length = Number(arg("length", 500));
  const cycles = Number(arg("cycles", 3));
  writeFileSync(OUT, "cycle,leg,expected_mm,measured_mm,error_mm,error_percent,direction_ok,bad_transitions_added\n");
  console.log(`Encoder ruler test: ${cycles} cycles of forward ${length} mm then back ${length} mm. Roll by hand, motors off, car stopped (IDLE or STOPPED).`);
  console.log("The sonar keeps pinging during the test, so the ECHO and encoder interrupts run together.");
  const legs = [];
  const bads = () => (latest.sensors.encBadLeft ?? 0) + (latest.sensors.encBadRight ?? 0);
  await ask("Put the car at the START mark on the ruler, wheels straight, then stop touching it.");
  await sleep(1500);
  let pos = latest.status.distanceMm, bad = bads();
  for (let c = 1; c <= cycles; c++) {
    for (const leg of ["forward", "back"]) {
      const expected = leg === "forward" ? length : -length;
      await ask(`Cycle ${c}/${cycles}: roll the car ${leg === "forward" ? "FORWARD" : "BACK"} ${length} mm along the ruler (${leg === "forward" ? "away from" : "back to"} the start mark), and let go.`);
      await sleep(1500);
      const now = latest.status.distanceMm, nbad = bads();
      const measured = now - pos;
      const err = Math.abs(measured) - length;               // + means the car reports MORE distance than the ruler, - means less
      const pct = (err / length) * 100;
      const dirOk = Math.sign(measured) === Math.sign(expected);
      legs.push({ err, pct, dirOk, added: nbad - bad });
      appendFileSync(OUT, [c, leg, expected, measured, err, pct.toFixed(1), dirOk ? "yes" : "no", nbad - bad].join(",") + "\n");
      console.log(`  ${leg}: dashboard says ${measured > 0 ? "+" : ""}${measured} mm (ruler ${expected > 0 ? "+" : ""}${expected}), error ${err > 0 ? "+" : ""}${err} mm (${pct > 0 ? "+" : ""}${pct.toFixed(1)} %, + = over-reads), direction ${dirOk ? "correct" : "WRONG"}, bad transitions +${nbad - bad}`);
      pos = now; bad = nbad;
    }
  }
  const worst = Math.max(...legs.map(l => Math.abs(l.pct)));
  const dirs = legs.every(l => l.dirOk);
  const added = legs.reduce((a, l) => a + l.added, 0);
  console.log("\n--- summary ---");
  console.log(`direction correct on ${legs.filter(l => l.dirOk).length}/${legs.length} legs; worst distance error ${worst.toFixed(1)} %; bad (illegal) transitions added: ${added}`);
  console.log(dirs && worst <= 2 ? "RESULT: PASS against the proposed limit of 2 % (ruler accuracy limits what this can show; it cannot prove an exact pulse count)" : "RESULT: REVIEW: direction wrong or distance error above the proposed 2 % limit (the limit is a proposal, not from the Tribunal text)");
}
ws.close(); rl.close(); process.exit(0);
