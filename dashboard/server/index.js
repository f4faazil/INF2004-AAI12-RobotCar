import "./load-env.js";
import http from "node:http";
import { readFile } from "node:fs/promises";
import { extname, join, normalize } from "node:path";
import { fileURLToPath } from "node:url";
import mqtt from "mqtt";
import { WebSocketServer, WebSocket } from "ws";
import { loadConfig } from "./config.js";
import { createSimulator } from "./simulator.js";
import { createRobotSimulator } from "./robot-simulator.js";
import { createBroker } from "./broker.js";
import { routeMqttMessage } from "./message-router.js";
import { LatestPicoState } from "./latest-state.js";
import { parseMap } from "./barcode-map.js";

const config = loadConfig();
const publicDir = fileURLToPath(new URL("../public/", import.meta.url));
const mime = { ".html": "text/html; charset=utf-8", ".css": "text/css; charset=utf-8", ".js": "text/javascript; charset=utf-8", ".svg": "image/svg+xml" };
let mqttClient;
let mqttState = config.demoMode ? "demo" : "connecting";
const latestPicoState = new LatestPicoState();

// Feed the barcode letter map to the robot: validated here (same rules as the robot), published RETAINED on
// aai12/robot/map so the car gets it whenever it connects, and answered by the car on aai12/robot/map_ack.
function sendMap(input) {
  const parsed = parseMap(input);
  if (!parsed.ok) return parsed;
  const topic = config.mqtt.topics.map;
  let delivered = null;
  if (config.demoMode) {
    delivered = 0;
  } else if (broker) {
    delivered = broker.publish(topic, Buffer.from(parsed.text), { retain: true });
  } else if (mqttClient?.connected) {
    mqttClient.publish(topic, parsed.text, { qos: 0, retain: true });
  } else {
    return { ok: false, error: "the dashboard is not connected to a broker" };
  }
  publishPicoMessage({ type: "map-sent", receivedAt: new Date().toISOString(), data: { text: parsed.text, entries: parsed.entries, deliveredNow: delivered } });
  if (config.demoMode) setTimeout(() => publishPicoMessage({ type: "map-ack", receivedAt: new Date().toISOString(), data: parsed.text === "CLEAR" ? "OK n=0" : `OK n=${parsed.entries.length}` }), 600);
  return { ok: true, text: parsed.text, entries: parsed.entries, deliveredNow: delivered };
}

const readBody = req => new Promise((resolve, reject) => {
  let size = 0;
  const chunks = [];
  req.on("data", chunk => { size += chunk.length; if (size > 2048) { reject(new Error("too large")); req.destroy(); } else chunks.push(chunk); });
  req.on("end", () => resolve(Buffer.concat(chunks).toString("utf8")));
  req.on("error", reject);
});

const server = http.createServer(async (req, res) => {
  if (req.method === "POST" && req.url.split("?")[0] === "/api/map") {
    // body: the map as plain text ("A=L,B=R,F=U") or JSON {"map": "..."}
    try {
      const raw = await readBody(req);
      let text = raw;
      if (raw.trim().startsWith("{")) text = JSON.parse(raw).map;
      const result = sendMap(text);
      res.writeHead(result.ok ? 200 : 400, { "content-type": "application/json" });
      res.end(JSON.stringify(result));
    } catch (error) {
      res.writeHead(400, { "content-type": "application/json" });
      res.end(JSON.stringify({ ok: false, error: error.message }));
    }
    return;
  }
  // "/" is the robot dashboard; the WiFi / MQTT diagnostics page is at /network.
  const path = req.url.split("?")[0];
  const requested = path === "/" ? "robot.html" : path === "/network" ? "index.html" : path.slice(1);
  const safePath = normalize(requested).replace(/^(\.\.[/\\])+/, "");
  try {
    const body = await readFile(join(publicDir, safePath));
    res.writeHead(200, { "content-type": mime[extname(safePath)] ?? "application/octet-stream", "cache-control": "no-store" });
    res.end(body);
  } catch {
    res.writeHead(404, { "content-type": "text/plain" });
    res.end("Not found");
  }
});

const sockets = new WebSocketServer({ server, path: "/live", maxPayload: 4096 });
const broadcast = message => {
  const encoded = JSON.stringify(message);
  for (const socket of sockets.clients) if (socket.readyState === WebSocket.OPEN) socket.send(encoded);
};
const publishPicoMessage = message => {
  latestPicoState.record(message);
  broadcast(message);
};
const status = detail => broadcast({ type: "status", mode: config.demoMode ? "demo" : "live", mqtt: mqttState, detail, receivedAt: new Date().toISOString() });

sockets.on("connection", socket => {
  socket.send(JSON.stringify({ type: "status", mode: config.demoMode ? "demo" : "live", mqtt: mqttState, receivedAt: new Date().toISOString() }));
  for (const message of latestPicoState.replay()) socket.send(JSON.stringify(message));
  socket.on("message", data => {
    try {
      const request = JSON.parse(data.toString("utf8"));
      if (request?.type === "send-map") socket.send(JSON.stringify({ type: "map-result", ...sendMap(request.map) }));
    } catch {
      socket.send(JSON.stringify({ type: "map-result", ok: false, error: "unreadable request" }));
    }
  });
});

// One place turns an MQTT message (from the built-in broker or an external one) into what the browser gets.
function handleMqtt(topic, payload) {
  try {
    const message = routeMqttMessage(topic, payload, config.mqtt.topics);
    if (message) publishPicoMessage(message);
  } catch (error) {
    const base = { receivedAt: new Date().toISOString(), topic, source: "firmware-confirmed" };
    broadcast({ ...base, type: "parse-error", data: { message: error.message, preview: payload.toString("hex").slice(0, 80) } });
  }
}

let broker;
if (config.demoMode) {
  createSimulator(publishPicoMessage);
  createRobotSimulator(publishPicoMessage);
} else if (config.embeddedBroker) {
  mqttState = "connected";
  broker = createBroker({
    port: config.brokerPort, host: config.brokerHost,
    onPublish: handleMqtt,
    onEvent: event => broadcast({ type: "broker-event", receivedAt: new Date().toISOString(), data: event })
  });
  broker.listen().then(
    port => { console.log(`MQTT broker for the robot: port ${port} on this machine`); status(`Built-in broker listening on ${port}`); },
    error => { mqttState = "error"; console.error(`Cannot start the built-in broker on port ${config.brokerPort}: ${error.message}`); status(error.message); }
  );
} else {
  mqttClient = mqtt.connect(config.mqtt.url, {
    username: config.mqtt.username, password: config.mqtt.password,
    clientId: config.mqtt.clientId, reconnectPeriod: 2000, connectTimeout: 10000
  });
  mqttClient.on("connect", () => {
    mqttState = "connected";
    const topics = Object.entries(config.mqtt.topics).filter(([name, topic]) => topic && name !== "map").map(([, topic]) => topic);
    mqttClient.subscribe(topics, { qos: 0 }, error => status(error ? `Subscribe failed: ${error.message}` : `Subscribed to ${topics.length} topics`));
  });
  mqttClient.on("reconnect", () => { mqttState = "reconnecting"; status(); });
  mqttClient.on("offline", () => { mqttState = "offline"; status(); });
  mqttClient.on("error", error => { mqttState = "error"; status(error.message); });
  mqttClient.on("message", handleMqtt);
}

server.listen(config.port, () => console.log(`AAI12 dashboard: http://localhost:${config.port} (${config.demoMode ? "demo" : "live"} mode)`));

function shutdown() {
  mqttClient?.end(true);
  broker?.close();
  sockets.close();
  server.close(() => process.exit(0));
}
process.on("SIGINT", shutdown);
process.on("SIGTERM", shutdown);
