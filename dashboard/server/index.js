import "./load-env.js";
import http from "node:http";
import { readFile } from "node:fs/promises";
import { extname, join, normalize } from "node:path";
import { fileURLToPath } from "node:url";
import mqtt from "mqtt";
import { WebSocketServer, WebSocket } from "ws";
import { loadConfig } from "./config.js";
import { createSimulator } from "./simulator.js";
import { routeMqttMessage } from "./message-router.js";
import { LatestPicoState } from "./latest-state.js";

const config = loadConfig();
const publicDir = fileURLToPath(new URL("../public/", import.meta.url));
const mime = { ".html": "text/html; charset=utf-8", ".css": "text/css; charset=utf-8", ".js": "text/javascript; charset=utf-8", ".svg": "image/svg+xml" };
let mqttClient;
let mqttState = config.demoMode ? "demo" : "connecting";
const latestPicoState = new LatestPicoState();

const server = http.createServer(async (req, res) => {
  const requested = req.url === "/" ? "index.html" : req.url.split("?")[0].slice(1);
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

const sockets = new WebSocketServer({ server, path: "/live" });
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
});

if (config.demoMode) {
  createSimulator(publishPicoMessage);
} else {
  mqttClient = mqtt.connect(config.mqtt.url, {
    username: config.mqtt.username, password: config.mqtt.password,
    clientId: config.mqtt.clientId, reconnectPeriod: 2000, connectTimeout: 10000
  });
  mqttClient.on("connect", () => {
    mqttState = "connected";
    const topics = Object.values(config.mqtt.topics).filter(Boolean);
    mqttClient.subscribe(topics, { qos: 0 }, error => status(error ? `Subscribe failed: ${error.message}` : `Subscribed to ${topics.length} topics`));
  });
  mqttClient.on("reconnect", () => { mqttState = "reconnecting"; status(); });
  mqttClient.on("offline", () => { mqttState = "offline"; status(); });
  mqttClient.on("error", error => { mqttState = "error"; status(error.message); });
  mqttClient.on("message", (topic, payload) => {
    try {
      const message = routeMqttMessage(topic, payload, config.mqtt.topics);
      if (message) publishPicoMessage(message);
    } catch (error) {
      const base = { receivedAt: new Date().toISOString(), topic, source: "firmware-confirmed" };
      broadcast({ ...base, type: "parse-error", data: { message: error.message, preview: payload.toString("hex").slice(0, 80) } });
    }
  });
}

server.listen(config.port, () => console.log(`AAI12 dashboard: http://localhost:${config.port} (${config.demoMode ? "demo" : "live"} mode)`));

function shutdown() {
  mqttClient?.end(true);
  sockets.close();
  server.close(() => process.exit(0));
}
process.on("SIGINT", shutdown);
process.on("SIGTERM", shutdown);
