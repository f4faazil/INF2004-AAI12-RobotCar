// A small MQTT 3.1.1 broker for the dashboard, so the laptop needs no Mosquitto: the robot connects to this
// machine's address on the hotspot and the dashboard reads what it publishes.  Plain Node, no dependencies.
//
// It implements what the robot's session uses and what a monitoring tool needs: CONNECT, PUBLISH (QoS 0, 1, 2
// acknowledged), SUBSCRIBE (+ and # filters, delivered at QoS 0), UNSUBSCRIBE, PINGREQ and DISCONNECT.  Retained
// messages, wills, authentication, persistent sessions and TLS are NOT implemented: use a real broker (set
// EMBEDDED_BROKER=false and MQTT_URL) when you need any of them.
import net from "node:net";

const CONNACK_OK = Buffer.from([0x20, 0x02, 0x00, 0x00]);
const PINGRESP = Buffer.from([0xd0, 0x00]);

export function topicMatches(filter, topic) {
  const f = filter.split("/");
  const t = topic.split("/");
  for (let i = 0; i < f.length; i++) {
    if (f[i] === "#") return true;
    if (i >= t.length) return false;
    if (f[i] !== "+" && f[i] !== t[i]) return false;
  }
  return f.length === t.length;
}

function encodeLength(n) {
  const out = [];
  do {
    let digit = n % 128;
    n = Math.floor(n / 128);
    if (n > 0) digit |= 0x80;
    out.push(digit);
  } while (n > 0);
  return Buffer.from(out);
}

function publishPacket(topic, payload) {
  const t = Buffer.from(topic, "utf8");
  const body = Buffer.concat([Buffer.from([t.length >> 8, t.length & 255]), t, payload]);
  return Buffer.concat([Buffer.from([0x30]), encodeLength(body.length), body]);
}

export function createBroker({ port = 1883, host = "0.0.0.0", onPublish = () => {}, onEvent = () => {}, now = () => Date.now() } = {}) {
  const clients = new Set();
  const stats = { connects: 0, publishes: 0, pingreqs: 0 };

  const server = net.createServer(socket => {
    const client = { socket, id: null, keepaliveS: 0, lastPacketAt: now(), lastPingAt: null, subscriptions: [], buffer: Buffer.alloc(0),
      address: `${socket.remoteAddress}:${socket.remotePort}`, connected: false };
    clients.add(client);
    socket.setNoDelay(true);

    // MQTT 3.1.1: the broker drops a client that is silent for 1.5 x its keep-alive interval.
    const watchdog = setInterval(() => {
      if (client.keepaliveS > 0 && now() - client.lastPacketAt > client.keepaliveS * 1500) {
        onEvent({ kind: "keepalive-timeout", clientId: client.id, address: client.address, keepaliveS: client.keepaliveS });
        socket.destroy();
      }
    }, 1000);

    const send = bytes => { if (!socket.destroyed) socket.write(bytes); };

    function handle(type, flags, body) {
      if (type === 1) {                                              // CONNECT
        const nameLen = body.readUInt16BE(0);
        const level = body[2 + nameLen];
        const keepalive = body.readUInt16BE(4 + nameLen);
        const idLen = body.readUInt16BE(6 + nameLen);
        client.id = body.toString("utf8", 8 + nameLen, 8 + nameLen + idLen);
        client.keepaliveS = keepalive;
        client.connected = true;
        stats.connects++;
        send(level === 4 || level === 3 ? CONNACK_OK : Buffer.from([0x20, 0x02, 0x00, 0x01]));
        onEvent({ kind: "connect", clientId: client.id, address: client.address, keepaliveS: keepalive, protocolLevel: level });
      } else if (type === 3) {                                       // PUBLISH
        const qos = (flags >> 1) & 3;
        const topicLen = body.readUInt16BE(0);
        const topic = body.toString("utf8", 2, 2 + topicLen);
        let pos = 2 + topicLen;
        let packetId = null;
        if (qos > 0) { packetId = body.readUInt16BE(pos); pos += 2; }
        const payload = body.subarray(pos);
        if (qos === 1) send(Buffer.from([0x40, 0x02, packetId >> 8, packetId & 255]));
        if (qos === 2) send(Buffer.from([0x50, 0x02, packetId >> 8, packetId & 255]));
        stats.publishes++;
        onPublish(topic, Buffer.from(payload), { clientId: client.id, qos });
        for (const other of clients) {
          if (other !== client && other.subscriptions.some(f => topicMatches(f, topic))) {
            if (!other.socket.destroyed) other.socket.write(publishPacket(topic, payload));
          }
        }
      } else if (type === 6) {                                       // PUBREL (flags 2)
        send(Buffer.from([0x70, 0x02, body[0], body[1]]));
      } else if (type === 8) {                                       // SUBSCRIBE
        const packetId = body.readUInt16BE(0);
        const granted = [];
        for (let pos = 2; pos < body.length;) {
          const len = body.readUInt16BE(pos);
          client.subscriptions.push(body.toString("utf8", pos + 2, pos + 2 + len));
          granted.push(0);
          pos += 2 + len + 1;
        }
        send(Buffer.concat([Buffer.from([0x90, 2 + granted.length, packetId >> 8, packetId & 255]), Buffer.from(granted)]));
      } else if (type === 10) {                                      // UNSUBSCRIBE
        const packetId = body.readUInt16BE(0);
        for (let pos = 2; pos < body.length;) {
          const len = body.readUInt16BE(pos);
          const filter = body.toString("utf8", pos + 2, pos + 2 + len);
          client.subscriptions = client.subscriptions.filter(f => f !== filter);
          pos += 2 + len;
        }
        send(Buffer.from([0xb0, 0x02, packetId >> 8, packetId & 255]));
      } else if (type === 12) {                                      // PINGREQ
        const t = now();
        stats.pingreqs++;
        onEvent({ kind: "pingreq", clientId: client.id, address: client.address, gapMs: client.lastPingAt == null ? null : t - client.lastPingAt, count: stats.pingreqs });
        client.lastPingAt = t;
        send(PINGRESP);
      } else if (type === 14) {                                      // DISCONNECT
        client.connected = false;
        socket.end();
      }
    }

    socket.on("data", chunk => {
      client.lastPacketAt = now();
      client.buffer = Buffer.concat([client.buffer, chunk]);
      for (;;) {
        const b = client.buffer;
        if (b.length < 2) return;
        let len = 0, mult = 1, i = 1;
        for (;; i++) {
          if (i >= b.length) return;                                 // the length is not all here yet
          len += (b[i] & 127) * mult;
          mult *= 128;
          if (!(b[i] & 128)) break;
          if (i > 4) { socket.destroy(); return; }                   // malformed
        }
        if (b.length < i + 1 + len) return;                          // the body is not all here yet
        const type = b[0] >> 4, flags = b[0] & 15, body = b.subarray(i + 1, i + 1 + len);
        client.buffer = b.subarray(i + 1 + len);
        try { handle(type, flags, body); } catch (error) {
          onEvent({ kind: "protocol-error", clientId: client.id, address: client.address, message: error.message });
          socket.destroy();
          return;
        }
      }
    });
    socket.on("error", () => {});
    socket.on("close", () => {
      clearInterval(watchdog);
      clients.delete(client);
      if (client.id !== null) onEvent({ kind: "disconnect", clientId: client.id, address: client.address });
    });
  });

  return {
    stats,
    listen: () => new Promise((resolve, reject) => {
      server.once("error", reject);
      server.listen(port, host, () => { server.off("error", reject); resolve(server.address().port); });
    }),
    close: () => new Promise(resolve => { for (const c of clients) c.socket.destroy(); server.close(() => resolve()); }),
    clientCount: () => [...clients].filter(c => c.connected).length,
    // Deliver a message to subscribed clients (the dashboard can use this to send commands later).
    publish(topic, payload) {
      for (const c of clients) if (c.subscriptions.some(f => topicMatches(f, topic))) c.socket.write(publishPacket(topic, Buffer.from(payload)));
    }
  };
}
