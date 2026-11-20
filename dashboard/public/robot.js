// The robot dashboard: draws what the backend decoded from the robot's records.  No framework; every value goes in
// with textContent, never as HTML.
const $ = id => document.getElementById(id);
const set = (id, text, cls) => { const el = $(id); el.textContent = text; if (cls !== undefined) el.className = cls; };
const yesno = (v, good = true) => [v ? "yes" : "no", (v === good) ? "ok" : "warn"];
const fmt = n => n.toLocaleString();
const clock = iso => new Date(iso).toLocaleTimeString([], { hour12: false });
const uptime = s => `${Math.floor(s / 3600)}h ${String(Math.floor(s / 60) % 60).padStart(2, "0")}m ${String(s % 60).padStart(2, "0")}s`;

const state = { lastFrameAt: 0, frames: 0, framesWindow: [], mode: null, demo: false, brokerPingGaps: [], lastScanAt: 0 };
const history = { l: [], r: [], us: [] };
const HISTORY = 120;

// --- the bars of a barcode: 15 cells
$("bars").replaceChildren(...Array.from({ length: 15 }, () => document.createElement("i")));

function chart(canvasId, series, range, colors) {
  const c = $(canvasId), g = c.getContext("2d"), w = c.width, h = c.height;
  g.clearRect(0, 0, w, h);
  g.strokeStyle = "#263450"; g.lineWidth = 1; g.beginPath();
  const zero = range[0] < 0 ? h * (range[1] / (range[1] - range[0])) : h - 0.5;
  g.moveTo(0, zero); g.lineTo(w, zero); g.stroke();
  series.forEach((points, k) => {
    g.strokeStyle = colors[k]; g.lineWidth = 2; g.beginPath();
    points.forEach((v, i) => {
      const x = w - ((points.length - 1 - i) / (HISTORY - 1)) * w, y = h - ((Math.min(range[1], Math.max(range[0], v)) - range[0]) / (range[1] - range[0])) * h;
      i ? g.lineTo(x, y) : g.moveTo(x, y);
    });
    g.stroke();
  });
}
const push = (arr, v) => { arr.push(v); if (arr.length > HISTORY) arr.shift(); };

function stateClass(d) {
  if (d.stateName === "FAULT" || d.stopFault) return "fault";
  if (["OBSTACLE STOP", "OBSTACLE SCAN", "BYPASS", "REVERSE", "RECOVER LINE", "SEARCH LINE", "REACQUIRE"].includes(d.stateName)) return "warn";
  if (["FOLLOW", "DONE"].includes(d.stateName)) return "ok";
  return "info";
}
const STATE_NOTE = {
  "BOOT": "Starting up", "IDLE": "Waiting for START", "CALIBRATE": "Calibrating the line sensors", "FOLLOW": "Following the black line",
  "JUNCTION ADVANCE": "Driving to the junction centre", "JUNCTION TURN": "Turning at the junction", "REACQUIRE": "Finding the line again after the hump",
  "SEARCH LINE": "Lost the line: sweeping to find it", "OBSTACLE STOP": "Stopped for an obstacle", "OBSTACLE SCAN": "Scanning the obstacle with the servo",
  "BYPASS": "Going round the obstacle", "REVERSE": "Backing away from the obstacle", "RECOVER LINE": "Driving back to the line",
  "STOPPED": "Stopped by the operator", "FAULT": "Stopped by a fault", "DONE": "Finished"
};

function onStatus(d) {
  set("state", d.stateName, `state ${stateClass(d)}`);
  set("stateNote", STATE_NOTE[d.stateName] ?? "");
  const chips = $("faults");
  chips.replaceChildren(...(d.faultNames.length ? d.faultNames : ["none"]).map(name => {
    const chip = document.createElement("span");
    const stop = name !== "none" && ["LINE_STALE", "LINE_CAL", "LINE_LOST", "ENCODER", "MOTION", "ULTRASONIC", "NO_SAFE_PATH", "RECOVERY_LIMIT", "IMU_LIFT"].includes(name);
    chip.className = `chip ${name === "none" ? "none" : stop ? "stop" : "info"}`;
    chip.textContent = name;
    return chip;
  }));
  set("distance", `${(d.distanceMm / 1000).toFixed(2)} m`);
  set("linePos", "", undefined); $("linePos").style.left = `${50 + d.linePosition / 20}%`;
  set("lineValue", String(d.linePosition));
  const lineCls = d.lineState === 0 ? "ok" : d.lineState === 2 ? "warn" : "bad";
  set("lineState", d.lineStateName, lineCls);
  set("junction", d.junctionName);
  set("navCmd", d.navCmdName);
  set("barcodeState", d.barcodeName, d.barcode === 2 ? "warn" : "");
  $("wl").style.height = `${Math.min(50, Math.abs(d.leftMmS) / 300 * 50)}%`; $("wl").className = d.leftMmS < 0 ? "rev" : "";
  $("wr").style.height = `${Math.min(50, Math.abs(d.rightMmS) / 300 * 50)}%`; $("wr").className = d.rightMmS < 0 ? "rev" : "";
  $("wl").style.bottom = d.leftMmS < 0 ? "auto" : "50%"; $("wl").style.top = d.leftMmS < 0 ? "50%" : "auto";
  $("wr").style.bottom = d.rightMmS < 0 ? "auto" : "50%"; $("wr").style.top = d.rightMmS < 0 ? "50%" : "auto";
  set("wlv", String(d.leftMmS)); set("wrv", String(d.rightMmS));
  push(history.l, d.leftMmS); push(history.r, d.rightMmS);
  chart("chartSpeed", [history.l, history.r], [-300, 300], ["#3ddc97", "#5cb4ff"]);
  if (d.obstacleValid) { push(history.us, d.obstacleCm * 10); chart("chartUs", [history.us], [0, 800], ["#f5b342"]); }
  if (d.highestHump != null) set("humpMax", `${d.highestHump} mm`);
  if (d.hump.heightMm > 0) set("humpLast", `${d.hump.heightMm} mm (${d.hump.validityName.toLowerCase()})`);
  set("humpNow", d.hump.statusName);
  $("liftBanner").classList.toggle("hidden", !d.faultNames.includes("IMU_LIFT"));
}

function onSensors(d) {
  for (const [id, key] of [["irLeft", "left"], ["irCentre", "centre"], ["irRight", "right"]]) {
    $(id).className = `ir${d.ir[key] ? " black" : ""}${d.irValid[key] ? "" : " untrusted"}`;
  }
  set("irValid", ["left", "centre", "right"].filter(k => d.irValid[k]).join(" · ") || "none");
  set("pair", ...yesno(d.pairOnLine, true).slice(0, 1), d.pairOnLine ? "ok" : "");
  set("armed", d.junctionArmed ? "yes: the junction is next" : "no", d.junctionArmed ? "warn" : "");
  [...$("bars").children].forEach((cell, i) => cell.classList.toggle("on", i < d.barsNow));
  set("barsNow", `${d.barsNow} / 15`);
  set("letter", d.lastLetterChar ?? "—");
  set("barsDone", fmt(d.barsDone)); set("decoded", `${d.decoded} / ${d.rejected}`, d.rejected ? "warn" : "");
  set("junctions", `${d.junctions} / ${d.executed}`); set("mapEntries", `${d.mapEntries} of 26`, d.mapEntries ? "" : "warn");
  set("motion", d.motionStateName); set("stopped", d.motionStopped ? "yes" : "no");
  set("encFault", d.encoderFault ? "FAULT" : "none", d.encoderFault ? "bad" : "ok");
  set("encBad", `${d.encBadLeft} / ${d.encBadRight}`, (d.encBadLeft > 50 || d.encBadRight > 50) ? "warn" : "");
  set("roll", `${d.rollDeg.toFixed(1)}°`);
  $("horizon").style.transform = `rotate(${-d.rollDeg}deg)`;
  set("imuOk", d.imuOk ? "answering, calibrated" : "not ready", d.imuOk ? "ok" : "warn");
  set("lift", d.liftLatched ? "LIFTED: motors stopped" : "on the floor", d.liftLatched ? "bad" : "ok");
  if (d.humpActive) set("humpNow", "crossing a hump");
  set("usMm", d.usValid ? String(d.usMm) : "—");
  $("usBar").style.width = d.usValid ? `${Math.min(100, d.usMm / 8)}%` : "0%";
  set("usValid", d.usValid ? "yes" : "no echo", d.usValid ? "ok" : "warn");
  set("usObstacle", d.usObstacle ? "OBSTACLE" : "clear", d.usObstacle ? "warn" : "ok");
  set("usFault", d.usFault ? "FAULT" : "none", d.usFault ? "bad" : "ok");
  set("aim", d.servoAimName.toLowerCase() + (d.servoAim ? " (looking at the box)" : ""));
  set("netLink", d.linkUp ? "up, has an address" : "down", d.linkUp ? "ok" : "bad");
  set("netMqtt", d.netStateName, d.netState === 3 ? "ok" : "warn");
  if (d.liftLatched) $("liftBanner").classList.remove("hidden");
}

function onHump(d) {
  if (d.hump.heightMm > 0) set("humpLast", `${d.hump.heightMm} mm (${d.hump.validityName.toLowerCase()})`);
  if (d.highestHump != null) set("humpMax", `${d.highestHump} mm`);
}

function onObstacle(d, receivedAt) {
  state.lastScanAt = Date.parse(receivedAt);
  set("scClosest", d.valid ? `${d.closestCm} cm` : "no valid reading");
  set("scWidth", d.valid ? `${d.widthCm} cm` : "—");
  set("scCentre", d.valid ? `${d.centreCm > 0 ? "+" : ""}${d.centreCm} cm` : "—");
  set("scClear", `${d.leftClearCm} / ${d.rightClearCm} cm`);
  set("scPlan", d.actionName, d.action === 4 ? "bad" : "ok"); set("scReason", d.reasonName);
  const ns = "http://www.w3.org/2000/svg", box = $("scanBox");
  box.replaceChildren();
  const rect = (x, y, w, h, cls) => { const r = document.createElementNS(ns, "rect"); for (const [k, v] of Object.entries({ x, y, width: w, height: h })) r.setAttribute(k, v); r.setAttribute("class", cls); box.appendChild(r); };
  if (d.valid) {
    const y = Math.max(8, 170 - d.closestCm);
    rect(d.centreCm - d.widthCm / 2, y - 12, d.widthCm, 12, "obs");
    if (d.leftClearCm > 0) rect(-150, y - 12, Math.min(150 + d.centreCm - d.widthCm / 2, Math.min(150, d.leftClearCm)), 12, "clearzone");
  }
}

function onHeartbeat(d) {
  set("uptime", uptime(d.uptimeS));
  set("netQueue", String(d.queueDepth)); set("netDropped", fmt(d.dropped), d.dropped ? "warn" : "");
  set("netReconnects", String(d.reconnects));
}

function logEvent(d, receivedAt) {
  const list = $("events");
  list.querySelector(".empty")?.remove();
  const li = document.createElement("li");
  const t = document.createElement("span"); t.className = "t"; t.textContent = clock(receivedAt);
  const n = document.createElement("span");
  n.className = `n ${d.name === "STATE" ? "st" : d.name === "BARCODE" ? "barcode" : d.name === "HUMP" ? "hump" : d.name.startsWith("FAULT") ? "fault" : ""}`;
  n.textContent = d.name;
  const x = document.createElement("span"); x.textContent = d.detail ?? (d.arg ? String(d.arg) : "");
  li.append(t, n, x);
  list.prepend(li);
  while (list.children.length > 200) list.lastChild.remove();
}

function onRobotFrame(m) {
  state.lastFrameAt = Date.now(); state.frames++;
  state.framesWindow.push(Date.now());
  const d = m.data;
  if (m.kind === "status") onStatus(d);
  else if (m.kind === "sensors") onSensors(d);
  else if (m.kind === "hump") onHump(d);
  else if (m.kind === "obstacle") onObstacle(d, m.receivedAt);
  else if (m.kind === "heartbeat") onHeartbeat(d);
  else if (m.kind === "event") logEvent(d, m.receivedAt);
  else if (m.kind === "ack") onAck(d);
}

function onBroker(e) {
  if (e.kind === "connect") { set("brokerClient", `${e.clientId} (keep-alive ${e.keepaliveS} s)`, "ok"); }
  else if (e.kind === "disconnect") { set("brokerClient", "disconnected", "warn"); }
  else if (e.kind === "keepalive-timeout") { set("brokerClient", "dropped: no packet for 1.5 × keep-alive", "bad"); }
  else if (e.kind === "pingreq" && e.gapMs != null) {
    state.brokerPingGaps.push(e.gapMs); if (state.brokerPingGaps.length > 5) state.brokerPingGaps.shift();
    set("brokerPing", `${(e.gapMs / 1000).toFixed(1)} s (last ${state.brokerPingGaps.length})`);
  }
}

// --- the barcode letter map: A-Z, each unmapped or Left / Right / Straight / U-turn, sent over MQTT
const ACTIONS = [["", "—"], ["L", "Left"], ["R", "Right"], ["F", "Straight"], ["U", "U-turn"]];
const mapState = { ws: null, sending: false };
const mapSelect = letter => $("mapGrid").querySelector(`select[data-letter="${letter}"]`);
for (let i = 0; i < 26; i++) {
  const letter = String.fromCharCode(65 + i);
  const cell = document.createElement("label"), name = document.createElement("b"), select = document.createElement("select");
  cell.className = "mapcell"; name.textContent = letter; select.dataset.letter = letter;
  select.setAttribute("aria-label", `Instruction for barcode ${letter}`);
  for (const [value, label] of ACTIONS) select.append(Object.assign(document.createElement("option"), { value, textContent: label }));
  select.addEventListener("change", refreshMapText);
  cell.append(name, select);
  $("mapGrid").append(cell);
}
function mapAsText() {
  return [...$("mapGrid").querySelectorAll("select")].filter(s => s.value).map(s => `${s.dataset.letter}=${s.value}`).join(",");
}
function refreshMapText() {
  for (const s of $("mapGrid").querySelectorAll("select")) s.parentElement.classList.toggle("set", Boolean(s.value));
  $("mapText").textContent = mapAsText() || "(nothing set)";
}
function loadMapText(text) {
  for (const s of $("mapGrid").querySelectorAll("select")) s.value = "";
  for (const pair of text.split(",")) { const [letter, action] = pair.split("="); if (mapSelect(letter)) mapSelect(letter).value = action; }
  refreshMapText();
}
for (const b of document.querySelectorAll("[data-preset]")) b.addEventListener("click", () => loadMapText(b.dataset.preset));
$("mapClear").addEventListener("click", () => loadMapText(""));
$("mapSend").addEventListener("click", () => {
  if (!mapState.ws || mapState.ws.readyState !== WebSocket.OPEN) { set("mapSent", "dashboard link is down: try again", "bad"); return; }
  const text = mapAsText();
  mapState.ws.send(JSON.stringify({ type: "send-map", map: text || "CLEAR" }));
  set("mapSent", "sending…", "warn"); set("mapAck", "waiting for the robot…", "warn");
});
function onMapSent(m) {
  const d = m.data;
  set("mapSent", `${d.text} · ${clock(m.receivedAt)}${d.deliveredNow === 0 ? " · robot offline: delivered when it connects" : ""}`, d.deliveredNow === 0 ? "warn" : "ok");
  if (!m.cached) set("mapAck", "waiting for the robot…", "warn");
  if (m.cached) loadMapText(d.text === "CLEAR" ? "" : d.text);   // a page opened later shows what was last sent
}
function onMapAck(m) {
  const ok = m.data.startsWith("OK");
  set("mapAck", `${m.data}${ok ? " (letters mapped on the car)" : " (the car refused it and kept its old map)"} · ${clock(m.receivedAt)}`, ok ? "ok" : "bad");
}

// --- START / STOP: sent over MQTT to the car, which answers with an ACK record carrying the same request id
const ACK_TEXT = {
  "RECEIVED": ["the car accepted it", "warn"],
  "COMPLETED": ["done: the vehicle controller acted on it", "ok"],
  "REJECTED RUNNING": ["REFUSED: the car is already running, or it is picked up", "bad"],
  "INVALID": ["REFUSED: the car did not understand it", "bad"]
};
const control = { pending: null };
$("armCheck").addEventListener("change", () => { $("btnStart").disabled = !$("armCheck").checked; });
function sendCommand(command) {
  if (!mapState.ws || mapState.ws.readyState !== WebSocket.OPEN) { set("cmdSent", "dashboard link is down: not sent", "bad"); return; }
  mapState.ws.send(JSON.stringify({ type: "send-command", command }));
  set("cmdSent", `${command.toUpperCase()}: sending…`, "warn"); set("cmdAck", "waiting for the car…", "warn");
}
$("btnStart").addEventListener("click", () => {
  if (!$("armCheck").checked) return;
  sendCommand("start");
  $("armCheck").checked = false; $("btnStart").disabled = true;      // armed for one START at a time
});
$("btnStop").addEventListener("click", () => sendCommand("stop"));
function onCommandResult(m) {
  if (!m.ok) { set("cmdSent", `not sent: ${m.error}`, "bad"); set("cmdAck", "—", ""); control.pending = null; return; }
  control.pending = m.reqId;
  set("cmdSent", `${m.command.toUpperCase()} · request ${m.reqId} · ${clock(new Date().toISOString())}`, "ok");
}
function onAck(d) {
  if (control.pending === null || d.reqId !== control.pending) return;
  const [text, cls] = ACK_TEXT[d.resultName] ?? [d.resultName, ""];
  set("cmdAck", `${d.resultName}: ${text}`, cls);
  if (d.resultName !== "RECEIVED") control.pending = null;
}

function setPill(id, cls, text, textId) { $(id).className = `pill ${cls}`; if (textId) $(textId).textContent = text; }

function connect() {
  const ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/live`);
  mapState.ws = ws;
  ws.onopen = () => setPill("pillLink", "ok", "connected", "linkText");
  ws.onclose = () => { setPill("pillLink", "bad", "reconnecting…", "linkText"); setTimeout(connect, 1500); };
  ws.onmessage = event => {
    const m = JSON.parse(event.data);
    if (m.type === "status") {
      state.demo = m.mode === "demo";
      $("demoBanner").classList.toggle("hidden", !state.demo);
      setPill("pillMode", state.demo ? "warn" : "ok", state.demo ? "SIMULATION" : "LIVE", "modeText");
    } else if (m.type === "robot-frame") onRobotFrame(m);
    else if (m.type === "broker-event") onBroker(m.data);
    else if (m.type === "map-sent") onMapSent(m);
    else if (m.type === "command-result") onCommandResult(m);
    else if (m.type === "map-ack") onMapAck(m);
    else if (m.type === "map-result" && !m.ok) { set("mapSent", `refused: ${m.error}`, "bad"); set("mapAck", "—", ""); }
    else if (m.type === "parse-error") logEvent({ name: "BAD RECORD", detail: `${m.topic}: ${m.data.message}` }, m.receivedAt);
  };
}

setInterval(() => {
  const now = Date.now();
  state.framesWindow = state.framesWindow.filter(t => now - t < 5000);
  set("rate", state.framesWindow.length ? (state.framesWindow.length / 5).toFixed(1) : "0");
  if (!state.lastFrameAt) setPill("pillRobot", "warn", "no data yet", "robotText");
  else {
    const age = (now - state.lastFrameAt) / 1000;
    setPill("pillRobot", age < 3 ? "ok" : age < 10 ? "warn" : "bad", age < 3 ? "sending" : `last record ${age.toFixed(0)} s ago`, "robotText");
  }
  if (state.lastScanAt) set("scanAge", `${Math.round((now - state.lastScanAt) / 1000)} s ago`);
}, 500);

loadMapText("A=L,B=R,F=U");                       // the default table: just press Send
$("clearLog").addEventListener("click", () => { $("events").replaceChildren(Object.assign(document.createElement("li"), { className: "empty", textContent: "No events yet." })); });
connect();
