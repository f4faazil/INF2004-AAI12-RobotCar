// Demo mode: a scripted lap of the robot (follow the line, meet a box, scan, go round it, rejoin, read a barcode,
// turn at the junction, cross the hump, get picked up).  Every record is built with the same encoder the tests check
// against the firmware's bytes and goes through the real router, so the page shows exactly what live data would.
import { encodeRobotFrame } from "./robot-frames.js";
import { routeMqttMessage } from "./message-router.js";
import { loadConfig } from "./config.js";

const PERIOD_S = 72;
const STATE = { IDLE: 1, CALIBRATE: 2, FOLLOW: 3, JUNCTION_ADVANCE: 4, JUNCTION_TURN: 5, SEARCH: 7, OBSTACLE_STOP: 8, OBSTACLE_SCAN: 9, BYPASS: 10, RECOVER: 12, FAULT: 14 };
const EVT = { STATE: 1, BARCODE: 2, JUNCTION: 3, NAV_EXEC: 4, HUMP: 5, OBSTACLE: 6, PLAN: 7, FAULT: 8, FAULT_CLEAR: 9, RECOVERED: 12, LINE_FOUND: 11 };
const FAULT_IMU_LIFT = 1 << 9;

export function scenario(t) {
  // Returns the robot's instantaneous condition at t seconds into the lap.
  const c = { state: STATE.FOLLOW, left: 150, right: 150, linePos: Math.round(60 * Math.sin(t * 1.3)), lineState: 0, us: 900, usValid: true, obstacle: false,
    aim: 0, ir: 2, bars: 0, barsDone: 0, decoded: 0, lastLetter: 0, roll: Math.round(25 * Math.sin(t * 2.1)), lift: false, hump: 0, humpActive: false,
    faults: 0, junction: 0, nav: 0, barcode: 0, armed: false, motion: 1, stopped: false };
  if (t < 2) Object.assign(c, { state: STATE.CALIBRATE, left: 0, right: 0, motion: 1, linePos: 0 });
  else if (t < 14) c.us = 900 - Math.max(0, t - 9) * 100;
  else if (t < 15.5) Object.assign(c, { state: STATE.OBSTACLE_STOP, left: 0, right: 0, us: 330, obstacle: true, motion: 0, stopped: true });
  else if (t < 19) Object.assign(c, { state: STATE.OBSTACLE_SCAN, left: 0, right: 0, us: 330, obstacle: true, motion: 0, stopped: true });
  else if (t < 31) {
    Object.assign(c, { state: STATE.BYPASS, left: 90, right: 90, us: 700, linePos: 0, lineState: 1, ir: 0 });
    if (t >= 19 && t < 20) Object.assign(c, { left: -60, right: 60 });                      // the turn away
    if (t >= 24 && t < 29.5) Object.assign(c, { aim: 2, us: 340 - (t - 24) * 8, left: 90, right: 90 });   // looking at the box while passing it
    if (t >= 29.5) c.us = 900;
  } else if (t < 34) Object.assign(c, { state: STATE.RECOVER, left: 100, right: 100, lineState: 1, linePos: 0, ir: t > 33 ? 2 : 0 });
  else if (t < 44) {
    c.us = 900;
    if (t >= 38) { c.bars = Math.min(15, Math.floor((t - 38) * 3)); c.left = c.right = 55; c.armed = t >= 43; }
    if (t >= 43) { c.lastLetter = 26; c.decoded = 1; c.barsDone = 1; c.nav = 4; c.barcode = 1; c.armed = true; }
  } else if (t < 45) Object.assign(c, { state: STATE.JUNCTION_ADVANCE, left: 55, right: 55, junction: 3, lastLetter: 26, decoded: 1, barsDone: 1, nav: 4, barcode: 1, ir: 7 });
  else if (t < 47.5) Object.assign(c, { state: STATE.JUNCTION_TURN, left: 60, right: -60, lastLetter: 26, decoded: 1, barsDone: 1, nav: 4, ir: 0 });
  else if (t < 56) {
    Object.assign(c, { lastLetter: 26, decoded: 1, barsDone: 1 });
    if (t >= 51 && t < 54) Object.assign(c, { hump: 2, humpActive: true, roll: Math.round(170 * Math.sin((t - 51) * Math.PI / 3)), left: 110, right: 110 });
  } else if (t < 64) { c.lastLetter = 26; c.decoded = 1; c.barsDone = 1; }
  else if (t < PERIOD_S - 2) Object.assign(c, { state: STATE.FAULT, left: 0, right: 0, lift: true, faults: FAULT_IMU_LIFT, roll: 900, motion: 0, stopped: true, lastLetter: 26 });
  else Object.assign(c, { state: STATE.IDLE, left: 0, right: 0, motion: 0, stopped: true });
  return c;
}

export function createRobotSimulator(emit, { intervalMs = 100, speed = 1 } = {}) {
  const topics = loadConfig({}).mqtt.topics;
  const started = Date.now();
  let tick = 0, odo = 0, eventSeq = 0, lastState = -1, lastEventsAt = {}, sentObstacle = false, sentHump = false, hbUptime = 0;
  const send = (kind, fields) => {
    const bytes = encodeRobotFrame(kind, fields);
    const message = routeMqttMessage(topics.telemetry, bytes, topics);
    emit({ ...message, source: "demo-simulated" });
  };
  const event = (code, arg, ts10) => send("event", { ts10, code, arg, seq: eventSeq++ & 255 });

  const timer = setInterval(() => {
    tick++;
    const t = (((Date.now() - started) / 1000) * speed) % PERIOD_S;
    const ms = Date.now() - started;
    const ts10 = Math.floor(ms / 10) & 0xffff;
    const c = scenario(t);
    odo += ((c.left + c.right) / 2) * (intervalMs / 1000) * speed;
    if (t < 0.3 * speed) { odo = 0; sentObstacle = false; sentHump = false; }

    if (c.state !== lastState) { event(EVT.STATE, c.state, ts10); if (c.state === STATE.OBSTACLE_STOP) event(EVT.OBSTACLE, 0, ts10); lastState = c.state; }
    if (t >= 19 && t < 19.3 && !sentObstacle) {
      sentObstacle = true;
      send("obstacle", { ts10, valid: true, closestCm: 28, widthCm: 22, centreCm: 7, leftClearCm: 400, rightClearCm: 0, action: 1, reason: 0 });
      event(EVT.PLAN, 1, ts10);
    }
    if (t >= 33.9 && t < 34.1 && !lastEventsAt.recovered) { lastEventsAt.recovered = true; event(EVT.RECOVERED, 0, ts10); }
    if (t < 1) lastEventsAt = {};
    if (t >= 43 && t < 43.2 && !lastEventsAt.barcode) { lastEventsAt.barcode = true; event(EVT.BARCODE, 90, ts10); }
    if (t >= 44 && t < 44.2 && !lastEventsAt.junction) { lastEventsAt.junction = true; event(EVT.JUNCTION, 3, ts10); event(EVT.NAV_EXEC, 4, ts10); }
    if (t >= 54 && t < 54.2 && !sentHump) {
      sentHump = true;
      const packed = 11 << 3;                                                // flat again, last hump 11 mm, valid
      send("hump", { ts10, humpPacked: packed, highestHumpMm: 11 });
      event(EVT.HUMP, packed, ts10);
    }
    if (t >= 64 && t < 64.2 && !lastEventsAt.lift) { lastEventsAt.lift = true; event(EVT.FAULT, FAULT_IMU_LIFT, ts10); }

    if (tick % 2 === 0) {
      // finished hump (height code 11 mm, valid) after the crossing; on the hump the height is not known yet (code 127)
      const humpPacked = t >= 54 ? 11 << 3 : c.hump ? (2 << 10) | (127 << 3) : 0;
      send("status", { ts10, state: c.state, navCmd: c.nav, lineState: c.lineState, junction: c.junction, barcode: c.barcode,
        linePosition: c.linePos, leftMmS: c.left, rightMmS: c.right, distanceMm: Math.round(odo), faults: c.faults,
        obstacleValid: c.usValid, obstacleCm: Math.round(c.us / 10), humpPacked, highestHumpMm: t >= 54 ? 11 : 127 });
    }
    if (tick % 5 === 0) {
      send("sensors", { ts10, irLevels: c.ir, irValidMask: 7, pairOnLine: c.ir === 7 || c.ir === 2, junctionArmed: c.armed, barsNow: c.bars,
        barsDone: c.barsDone, decoded: c.decoded, rejected: 0, junctions: t >= 44 ? 1 : 0, executed: t >= 47.5 ? 1 : 0, mapEntries: 26,
        lastLetter: c.lastLetter, motionState: c.motion, motionStopped: c.stopped, encoderFault: false, encBadLeft: Math.floor(odo / 900), encBadRight: Math.floor(odo / 700),
        rollPermille: c.roll, imuOk: true, liftLatched: c.lift, humpActive: c.humpActive, usMm: Math.round(c.us), usValid: c.usValid,
        usObstacle: c.obstacle, usFault: false, servoAim: c.aim, netState: 3, linkUp: true });
    }
    if (tick % 10 === 0) {
      hbUptime = Math.round(ms / 1000);
      send("heartbeat", { uptimeS: hbUptime, dropped: 0, reconnects: 0, queueDepth: 1, netState: 3 });
    }
  }, intervalMs);
  return () => clearInterval(timer);
}
