// START and STOP for the car, sent from the dashboard over MQTT (topic aai12/robot/cmd).
//
// The bytes are the robot's own command record (app_program/telemetry.h, TEL_TYPE_COMMAND = 0x10): type (8 bits),
// request id (8), command (4), parameter (16), packed MSB first.  The car answers with an ACK record that carries the
// same request id: RECEIVED when it was accepted, COMPLETED when the vehicle controller has acted on it, or
// REJECTED RUNNING / INVALID when its safety policy refused it (picked up, or START while already running).
//
// A command is NEVER published retained: a retained START would replay every time the car reconnects.

export const COMMANDS = Object.freeze({ stop: 1, start: 2 });

// The command record, byte for byte what tel_encode_command() produces.
export function encodeCommand(name, reqId) {
  const cmd = COMMANDS[name];
  if (cmd === undefined) throw new Error(`unknown command "${name}" (use start or stop)`);
  if (!Number.isInteger(reqId) || reqId < 0 || reqId > 255) throw new Error("request id must be 0 to 255");
  const param = 0;
  return Buffer.from([0x10, reqId, (cmd << 4) | (param >> 12), (param >> 4) & 0xff, (param & 0x0f) << 4]);
}

// A same-origin check for the page's WebSocket: a page served from some other site cannot drive the car.
export function sameOrigin(origin, host) {
  if (!origin) return true;                       // not a browser (a script on this network): allowed, like the map
  try { return new URL(origin).host === host; } catch { return false; }
}

// Request ids 1..255, wrapping; 0 is never used so "no request" stays distinguishable.
export function createRequestIds() {
  let next = 0;
  return () => { next = (next % 255) + 1; return next; };
}
