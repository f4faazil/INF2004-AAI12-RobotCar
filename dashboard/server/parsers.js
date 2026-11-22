const DIAGNOSTIC_NAMES = Object.freeze({
  v: "schemaVersion", b: "bootNonce", rr: "resetReason", tk: "timerTickConfig",
  r: "failureReason", t: "failureAtMs", s: "mqttState", oc: "ownerCalls",
  og: "ownerMaxGapMs", mp: "mqttPolls", mg: "mqttMaxGapMs", lp: "lastPollMs",
  lt: "lastTxMs", pa: "pingAttempts", pq: "pingsQueued", pr: "pingResponses",
  wf: "tcpWriteFailures", of: "tcpOutputFailures", te: "tcpErrors",
  le: "lastTcpError", ui: "usbIrqCount", us: "usbServiceCount",
  ue: "usbEventCount", ul: "usbLinkState", wc: "wifiLinkChanges",
  wt: "lastWifiChangeMs", dc: "dhcpChanges", dt: "lastDhcpChangeMs",
  ja: "joinAttempts", jr: "joinRecoveries"
});

// ---- the binary forms the firmware sends (lib/libnet/lwip/mqtt_lwip.c, app_program/mqtt_heartbeat.h) -----------------------------------------
// Everything is big-endian.  Each decoder returns the same named fields and the same "k=v,k=v" text as the old text payloads did, so the page and
// the recordings keep working; text payloads from older firmware are still read below.
const DIAG_LAYOUT = {
  0x23: { length: 50, fields: [["b", 4], ["rr", 1], ["tk", 2], ["r", 1], ["t", 4], ["s", 1], ["oc", 4], ["og", 4], ["mp", 4], ["mg", 4], ["lp", 4], ["lt", 4], ["pa", 4], ["pq", 4], ["pr", 4]] },
  0x24: { length: 58, fields: [["b", 4], ["wf", 4], ["of", 4], ["te", 4], ["le", -4], ["ui", 4], ["us", 4], ["ue", 4], ["ul", 1], ["wc", 4], ["wt", 4], ["dc", 4], ["dt", 4], ["ja", 4], ["jr", 4]] }
};
const hex32 = value => value.toString(16).padStart(8, "0");

function decodeBinaryDiagnostic(buffer) {
  const layout = DIAG_LAYOUT[buffer[0]];
  if (!layout || buffer.length !== layout.length) return null;
  const fields = {}, parts = ["v=1"];
  let at = 1;
  for (const [key, size] of layout.fields) {
    const width = Math.abs(size);
    const value = size < 0 ? buffer.readIntBE(at, width) : buffer.readUIntBE(at, width);
    at += width;
    fields[DIAGNOSTIC_NAMES[key]] = key === "b" ? hex32(value) : value;
    parts.push(`${key}=${key === "b" ? hex32(value) : value}`);
  }
  return { raw: parts.join(","), fields: { schemaVersion: 1, ...fields } };
}

// The online marker is the single byte 0x22 (older firmware sent the text AAI12_PICO_W_ONLINE).
export function decodeOnlineMarker(payload) {
  if (Buffer.isBuffer(payload) && payload.length === 1 && payload[0] === 0x22) return "AAI12_PICO_W_ONLINE";
  return Buffer.isBuffer(payload) ? payload.toString("utf8") : String(payload);
}

export function parseDiagnostic(input) {
  if (Buffer.isBuffer(input)) {
    const binary = decodeBinaryDiagnostic(input);
    if (binary) return binary;
  }
  const text = Buffer.isBuffer(input) ? input.toString("utf8") : String(input);
  const fields = {};
  for (const token of text.split(",")) {
    const equal = token.indexOf("=");
    if (equal < 1) continue;
    const key = token.slice(0, equal).trim();
    const raw = token.slice(equal + 1).trim();
    const name = DIAGNOSTIC_NAMES[key] ?? key;
    if (key === "b") fields[name] = raw;
    else if (/^-?\d+$/.test(raw)) fields[name] = Number(raw);
    else fields[name] = raw;
  }
  return { raw: text, fields };
}

// Mirrors telemetry_hump_demo.c: LSB-first type(8), status(3), height(7), validity(2), fault(1).
export function parseProvisionalTelemetry(payload) {
  if (!Buffer.isBuffer(payload)) payload = Buffer.from(payload);
  if (payload.length !== 3) return { format: "unknown-binary", byteLength: payload.length, hex: payload.toString("hex") };
  const bits = payload[0] | (payload[1] << 8) | (payload[2] << 16);
  const type = bits & 0xff;
  const height = (bits >>> 11) & 0x7f;
  const validity = (bits >>> 18) & 0x03;
  if ((payload[2] & 0xe0) !== 0 || type !== 0x04 || !((validity === 0 && height <= 126) || (validity === 3 && height === 127))) {
    return { format: "invalid-provisional-frame", byteLength: payload.length, hex: payload.toString("hex") };
  }
  return {
    format: "provisional-hump-v0",
    type,
    status: (bits >>> 8) & 0x07,
    height,
    validity,
    outOfRange: validity === 3,
    imuFault: Boolean((bits >>> 20) & 0x01),
    confirmedDiagnosticOnly: true
  };
}

export function parseFutureJson(payload) {
  const value = JSON.parse(Buffer.isBuffer(payload) ? payload.toString("utf8") : String(payload));
  if (!value || typeof value !== "object" || Array.isArray(value)) throw new Error("JSON telemetry must be an object");
  return value;
}

export function parseHeartbeat(input) {
  // 13 bytes: 0x21, boot id u32, sequence u32, uptime ms u32
  if (Buffer.isBuffer(input) && input.length === 13 && input[0] === 0x21) {
    const fields = { schemaVersion: 1, bootId: hex32(input.readUInt32BE(1)), sequence: input.readUInt32BE(5), uptimeMs: input.readUInt32BE(9) };
    return { raw: `v=1,b=${fields.bootId},seq=${fields.sequence},up=${fields.uptimeMs}`, fields };
  }
  const text = Buffer.isBuffer(input) ? input.toString("utf8") : String(input);
  const fields = {};
  for (const token of text.split(",")) {
    const [key, raw, ...extra] = token.split("=");
    if (!key || raw === undefined || extra.length) throw new Error("Malformed heartbeat field");
    if (key === "b") fields.bootId = raw;
    else if (key === "v") fields.schemaVersion = Number(raw);
    else if (key === "seq") fields.sequence = Number(raw);
    else if (key === "up") fields.uptimeMs = Number(raw);
  }
  if (fields.schemaVersion !== 1 || !/^[0-9a-fA-F]{8}$/.test(fields.bootId ?? "") ||
      !Number.isInteger(fields.sequence) || fields.sequence < 0 || fields.sequence > 0xffffffff ||
      !Number.isInteger(fields.uptimeMs) || fields.uptimeMs < 0 || fields.uptimeMs > 0xffffffff) {
    throw new Error("Invalid heartbeat payload");
  }
  return { raw: text, fields };
}
