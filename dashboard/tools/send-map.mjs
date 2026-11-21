// Feed the robot its barcode instructions from a terminal (the dashboard must be running):
//
//   npm run map -- "A=L,B=R,F=U"        letters and actions: L left, R right, F straight, U U-turn
//   npm run map -- CLEAR                forget every mapping on the car
//
// The map goes to the dashboard (POST /api/map), which checks it with the robot's own rules and publishes it,
// retained, on aai12/robot/map.  This then waits for the car's answer on aai12/robot/map_ack.
import { WebSocket } from "ws";

const text = process.argv.slice(2).join(" ").trim();
const base = process.env.DASHBOARD_URL ?? "http://localhost:3000";
if (!text) {
  console.error('usage: npm run map -- "A=L,B=R,F=U"   (L left, R right, F straight, U U-turn, or CLEAR)');
  process.exit(2);
}

// listen for the answer BEFORE sending, so a quick reply from a connected car is not missed
const ws = new WebSocket(`${base.replace(/^http/, "ws")}/live`);
ws.on("error", () => {});
let answered = null;
ws.on("message", data => {
  const m = JSON.parse(data.toString());
  if (m.type === "map-ack" && !m.cached) { answered = m.data; ws.emit("answer"); }
});
await new Promise(resolve => { ws.once("open", resolve); ws.once("close", resolve); setTimeout(resolve, 3000); });

const response = await fetch(`${base}/api/map`, { method: "POST", body: text }).catch(error => {
  console.error(`Cannot reach the dashboard at ${base}: ${error.message}
Start it first: npm start`);
  process.exit(1);
});
const result = await response.json();
if (!result.ok) {
  console.error(`Refused: ${result.error}`);
  process.exit(1);
}
console.log(`Sent (retained): ${result.text}`);
console.log(result.deliveredNow ? "The robot is connected and has it." : "The robot is not connected right now: it gets the map when it connects.");

if (answered === null) {
  await new Promise(resolve => { ws.once("answer", resolve); setTimeout(resolve, 10000); });
}
if (answered === null) {
  console.log("No answer from the robot yet (it may be offline).");
  process.exit(0);
}
console.log(`Robot answered: ${answered}`);
process.exit(answered.startsWith("OK") ? 0 : 1);
