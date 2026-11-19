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

export function parseDiagnostic(input) {
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
