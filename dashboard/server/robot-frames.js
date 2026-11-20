// Decoder (and encoder, for the demo simulator and the tests) for the robot's bit-packed telemetry records.
//
// It follows app_program/telemetry.h exactly: a 1-byte type, then fields packed MSB first with the stated
// widths, zero-padded to a byte; signed fields are two's complement in their width.  tools/robot_telemetry.py is
// the same codec in Python.  test/robot-frames.test.js checks this file against bytes the C firmware produced.

export const TYPE = Object.freeze({ STATUS: 1, EVENT: 2, HUMP: 3, OBSTACLE: 4, HEARTBEAT: 5, SENSORS: 6, ACK: 0x11 });

export const STATE_NAMES = ["BOOT", "IDLE", "CALIBRATE", "FOLLOW", "JUNCTION ADVANCE", "JUNCTION TURN", "REACQUIRE",
  "SEARCH LINE", "OBSTACLE STOP", "OBSTACLE SCAN", "BYPASS", "REVERSE", "RECOVER LINE", "STOPPED", "FAULT", "DONE"];
export const EVENT_NAMES = { 1: "STATE", 2: "BARCODE", 3: "JUNCTION", 4: "NAV EXEC", 5: "HUMP", 6: "OBSTACLE", 7: "PLAN",
  8: "FAULT", 9: "FAULT CLEAR", 10: "LINE LOST", 11: "LINE FOUND", 12: "RECOVERED", 13: "DONE" };
export const FAULT_NAMES = { 0: "LINE_STALE", 1: "LINE_CAL", 2: "LINE_LOST", 3: "ENCODER", 4: "MOTION", 5: "IMU",
  6: "ULTRASONIC", 7: "NO_SAFE_PATH", 8: "RECOVERY_LIMIT", 9: "IMU_LIFT", 12: "BARCODE_REJECT", 13: "LINE_OVERFLOW",
  14: "TELEMETRY_DROP", 15: "NET_DOWN" };
// Faults that stop the car (fault_mask.h FAULT_STOP_MASK) versus information only.
export const STOP_FAULTS = new Set([0, 1, 2, 3, 4, 6, 7, 8, 9]);
export const NAV_CMD_NAMES = ["NONE", "LEFT", "RIGHT", "STRAIGHT", "U-TURN"];
export const LINE_STATE_NAMES = ["TRACKING", "LOST", "MASKED BY HUMP", "STALE"];
export const JUNCTION_NAMES = ["NONE", "LEFT", "RIGHT", "BOTH"];
export const BARCODE_STATE_NAMES = ["NONE", "INSTRUCTION PENDING", "REJECTED", "EXPIRED"];
export const HUMP_STATUS_NAMES = ["FLAT", "RISING", "ON HUMP", "DESCENDING"];
export const HUMP_VALIDITY_NAMES = ["VALID", "UNAVAILABLE", "LOW CONFIDENCE"];
export const PLAN_ACTION_NAMES = ["CONTINUE", "BYPASS LEFT", "BYPASS RIGHT", "REVERSE", "STOP"];
export const PLAN_REASON_NAMES = ["OK", "NO OBSTACLE", "INVALID SCAN", "NO CLEARANCE", "DETOUR TOO LARGE", "TOO CLOSE"];
export const MOTION_STATE_NAMES = ["IDLE", "RUNNING", "DONE", "?"];
export const SERVO_AIM_NAMES = ["AHEAD", "LEFT", "RIGHT", "?"];
export const MQTT_STATE_NAMES = ["WAIT NETWORK", "TCP CONNECTING", "WAIT CONNACK", "ONLINE", "BACKOFF"];

class BitReader {
  constructor(bytes) { this.bytes = bytes; this.pos = 0; }
  get(width) {
    let value = 0;
    for (let i = 0; i < width; i++) {
      const byte = this.bytes[this.pos >> 3];
      if (byte === undefined) throw new Error("record too short");
      value = value * 2 + ((byte >> (7 - (this.pos & 7))) & 1);
      this.pos++;
    }
    return value;
  }
  signed(width) { const v = this.get(width); return v >= 2 ** (width - 1) ? v - 2 ** width : v; }
}

class BitWriter {
  constructor() { this.bits = []; }
  put(value, width) {
    const v = ((value % 2 ** width) + 2 ** width) % 2 ** width;
    for (let i = width - 1; i >= 0; i--) this.bits.push(Math.floor(v / 2 ** i) % 2);
  }
  bytes() {
    const bits = [...this.bits];
    while (bits.length % 8) bits.push(0);
    const out = Buffer.alloc(bits.length / 8);
    for (let i = 0; i < bits.length; i++) out[i >> 3] = (out[i >> 3] << 1) | bits[i];
    return out;
  }
}

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const faultNames = faults => Object.entries(FAULT_NAMES).filter(([bit]) => faults & (1 << Number(bit))).map(([, n]) => n);
const name = (list, i) => list[i] ?? "?";

export function unpackHump(packed) {
  const status = (packed >> 10) & 7, heightCode = (packed >> 3) & 0x7f, validity = (packed >> 1) & 3;
  return {
    status, statusName: name(HUMP_STATUS_NAMES, status), heightCode,
    heightMm: heightCode === 127 ? null : heightCode,           // 127 = out of range / none
    validity, validityName: name(HUMP_VALIDITY_NAMES, validity), imuFault: Boolean(packed & 1)
  };
}

const DECODERS = {
  [TYPE.STATUS](r) {
    const d = { ts10: r.get(16), state: r.get(4), navCmd: r.get(3), lineState: r.get(2), junction: r.get(2), barcode: r.get(2),
      linePosition: r.signed(11), leftMmS: r.signed(10), rightMmS: r.signed(10), distanceMm: r.signed(16), faults: r.get(16),
      obstacleValid: Boolean(r.get(1)), obstacleCm: r.get(9), humpPacked: r.get(13), highestHumpMm: r.get(7), saturated: r.get(4) };
    return { ...d, stateName: name(STATE_NAMES, d.state), navCmdName: name(NAV_CMD_NAMES, d.navCmd),
      lineStateName: name(LINE_STATE_NAMES, d.lineState), junctionName: name(JUNCTION_NAMES, d.junction),
      barcodeName: name(BARCODE_STATE_NAMES, d.barcode), faultNames: faultNames(d.faults),
      stopFault: faultNames(d.faults).length > 0 && Object.entries(FAULT_NAMES).some(([b]) => (d.faults & (1 << b)) && STOP_FAULTS.has(Number(b))),
      hump: unpackHump(d.humpPacked), highestHump: d.highestHumpMm === 127 ? null : d.highestHumpMm };
  },
  [TYPE.EVENT](r) {
    const d = { ts10: r.get(16), code: r.get(5), arg: r.get(16), seq: r.get(8) };
    const out = { ...d, name: EVENT_NAMES[d.code] ?? "UNKNOWN" };
    if (d.code === 1) out.detail = name(STATE_NAMES, d.arg);                       // STATE: the new state
    else if (d.code === 2) out.detail = d.arg >= 65 && d.arg <= 90 ? String.fromCharCode(d.arg) : String(d.arg);
    else if (d.code === 3) out.detail = name(JUNCTION_NAMES, d.arg & 3);
    else if (d.code === 4) out.detail = name(NAV_CMD_NAMES, d.arg);
    else if (d.code === 5) { const h = unpackHump(d.arg); out.detail = h.heightMm == null ? h.statusName : `${h.statusName} ${h.heightMm} mm`; }
    else if (d.code === 6) out.detail = d.arg === 1 ? "during a bypass" : d.arg === 2 ? "box passed: leg shortened" : d.arg === 3 ? "still beside the box: going on" : `${d.arg}`;
    else if (d.code === 7) out.detail = name(PLAN_ACTION_NAMES, d.arg);
    else if (d.code === 8 || d.code === 9) out.detail = faultNames(d.arg).join(", ") || `0x${d.arg.toString(16)}`;
    return out;
  },
  [TYPE.HUMP](r) {
    const d = { ts10: r.get(16), humpPacked: r.get(13), highestHumpMm: r.get(7) };
    return { ...d, hump: unpackHump(d.humpPacked), highestHump: d.highestHumpMm === 127 ? null : d.highestHumpMm };
  },
  [TYPE.OBSTACLE](r) {
    const d = { ts10: r.get(16), valid: Boolean(r.get(1)), closestCm: r.get(9), widthCm: r.get(9), centreCm: r.signed(9),
      leftClearCm: r.get(9), rightClearCm: r.get(9), action: r.get(3), reason: r.get(3) };
    return { ...d, actionName: name(PLAN_ACTION_NAMES, d.action), reasonName: name(PLAN_REASON_NAMES, d.reason) };
  },
  [TYPE.HEARTBEAT](r) {
    const d = { uptimeS: r.get(32), dropped: r.get(16), reconnects: r.get(8), queueDepth: r.get(5), netState: r.get(3) };
    return { ...d, netStateName: name(MQTT_STATE_NAMES, d.netState) };
  },
  [TYPE.SENSORS](r) {
    const d = { ts10: r.get(16), irLevels: r.get(3), irValidMask: r.get(3), pairOnLine: Boolean(r.get(1)), junctionArmed: Boolean(r.get(1)),
      barsNow: r.get(4), barsDone: r.get(8), decoded: r.get(8), rejected: r.get(8), junctions: r.get(8), executed: r.get(8),
      mapEntries: r.get(5), lastLetter: r.get(5), motionState: r.get(2), motionStopped: Boolean(r.get(1)), encoderFault: Boolean(r.get(1)),
      encBadLeft: r.get(8), encBadRight: r.get(8), rollPermille: r.signed(11), imuOk: Boolean(r.get(1)), liftLatched: Boolean(r.get(1)),
      humpActive: Boolean(r.get(1)), usMm: r.get(13), usValid: Boolean(r.get(1)), usObstacle: Boolean(r.get(1)), usFault: Boolean(r.get(1)),
      servoAim: r.get(2), netState: r.get(3), linkUp: Boolean(r.get(1)), saturated: Boolean(r.get(1)) };
    return { ...d, ir: { left: Boolean(d.irLevels & 1), centre: Boolean(d.irLevels & 2), right: Boolean(d.irLevels & 4) },
      irValid: { left: Boolean(d.irValidMask & 1), centre: Boolean(d.irValidMask & 2), right: Boolean(d.irValidMask & 4) },
      lastLetterChar: d.lastLetter >= 1 && d.lastLetter <= 26 ? String.fromCharCode(64 + d.lastLetter) : null,
      motionStateName: name(MOTION_STATE_NAMES, d.motionState), servoAimName: name(SERVO_AIM_NAMES, d.servoAim),
      netStateName: name(MQTT_STATE_NAMES, d.netState),
      rollDeg: Math.round(Math.asin(clamp(d.rollPermille, -1000, 1000) / 1000) * 1800 / Math.PI) / 10 };
  },
  [TYPE.ACK](r) {
    const d = { reqId: r.get(8), result: r.get(3) };
    return { ...d, resultName: ["RECEIVED", "COMPLETED", "REJECTED RUNNING", "INVALID"][d.result] ?? "?" };
  }
};

export const KIND = { [TYPE.STATUS]: "status", [TYPE.EVENT]: "event", [TYPE.HUMP]: "hump", [TYPE.OBSTACLE]: "obstacle",
  [TYPE.HEARTBEAT]: "heartbeat", [TYPE.SENSORS]: "sensors", [TYPE.ACK]: "ack" };

// Decode one robot record.  Throws on an unknown type or a short record.
export function decodeRobotFrame(bytes) {
  const buf = Buffer.isBuffer(bytes) ? bytes : Buffer.from(bytes);
  const type = buf[0];
  const decode = DECODERS[type];
  if (!decode) throw new Error(`unknown robot record type 0x${(type ?? 0).toString(16)}`);
  const r = new BitReader(buf);
  r.get(8);
  return { kind: KIND[type], type, byteLength: buf.length, ...decode(r) };
}

// ---- encoders (the simulator and the tests; the firmware's are in telemetry.c) ----
const ENCODERS = {
  status(w, s) {
    w.put(TYPE.STATUS, 8); w.put(s.ts10, 16); w.put(s.state, 4); w.put(s.navCmd ?? 0, 3); w.put(s.lineState ?? 0, 2);
    w.put(s.junction ?? 0, 2); w.put(s.barcode ?? 0, 2); w.put(clamp(s.linePosition ?? 0, -1000, 1000), 11);
    w.put(clamp(s.leftMmS ?? 0, -511, 511), 10); w.put(clamp(s.rightMmS ?? 0, -511, 511), 10);
    w.put(clamp(s.distanceMm ?? 0, -32767, 32767), 16); w.put(s.faults ?? 0, 16); w.put(s.obstacleValid ? 1 : 0, 1);
    w.put(clamp(s.obstacleCm ?? 0, 0, 510), 9); w.put(s.humpPacked ?? 0, 13); w.put(s.highestHumpMm ?? 127, 7); w.put(0, 4);
  },
  event(w, e) { w.put(TYPE.EVENT, 8); w.put(e.ts10, 16); w.put(e.code, 5); w.put(e.arg ?? 0, 16); w.put(e.seq ?? 0, 8); },
  hump(w, h) { w.put(TYPE.HUMP, 8); w.put(h.ts10, 16); w.put(h.humpPacked, 13); w.put(h.highestHumpMm ?? 127, 7); },
  obstacle(w, o) {
    w.put(TYPE.OBSTACLE, 8); w.put(o.ts10, 16); w.put(o.valid ? 1 : 0, 1); w.put(clamp(o.closestCm, 0, 511), 9);
    w.put(clamp(o.widthCm, 0, 511), 9); w.put(clamp(o.centreCm, -255, 255), 9); w.put(clamp(o.leftClearCm, 0, 511), 9);
    w.put(clamp(o.rightClearCm, 0, 511), 9); w.put(o.action, 3); w.put(o.reason, 3);
  },
  heartbeat(w, h) {
    w.put(TYPE.HEARTBEAT, 8); w.put(h.uptimeS, 32); w.put(h.dropped ?? 0, 16); w.put(h.reconnects ?? 0, 8);
    w.put(h.queueDepth ?? 0, 5); w.put(h.netState ?? 3, 3);
  },
  sensors(w, s) {
    const c8 = v => clamp(v ?? 0, 0, 255);
    w.put(TYPE.SENSORS, 8); w.put(s.ts10, 16); w.put(s.irLevels ?? 0, 3); w.put(s.irValidMask ?? 7, 3);
    w.put(s.pairOnLine ? 1 : 0, 1); w.put(s.junctionArmed ? 1 : 0, 1); w.put(s.barsNow ?? 0, 4);
    for (const k of ["barsDone", "decoded", "rejected", "junctions", "executed"]) w.put(c8(s[k]), 8);
    w.put(s.mapEntries ?? 0, 5); w.put(s.lastLetter ?? 0, 5); w.put(s.motionState ?? 0, 2); w.put(s.motionStopped ? 1 : 0, 1);
    w.put(s.encoderFault ? 1 : 0, 1); w.put(c8(s.encBadLeft), 8); w.put(c8(s.encBadRight), 8);
    w.put(clamp(s.rollPermille ?? 0, -1000, 1000), 11); w.put(s.imuOk ? 1 : 0, 1); w.put(s.liftLatched ? 1 : 0, 1);
    w.put(s.humpActive ? 1 : 0, 1); w.put(clamp(s.usMm ?? 0, 0, 8190), 13); w.put(s.usValid ? 1 : 0, 1);
    w.put(s.usObstacle ? 1 : 0, 1); w.put(s.usFault ? 1 : 0, 1); w.put(s.servoAim ?? 0, 2); w.put(s.netState ?? 3, 3);
    w.put(s.linkUp ? 1 : 0, 1); w.put(0, 1);
  }
};

export function encodeRobotFrame(kind, fields) {
  const w = new BitWriter();
  ENCODERS[kind](w, fields);
  return w.bytes();
}
