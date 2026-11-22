// The barcode letter -> movement mapping that the dashboard feeds to the robot over MQTT (topic aai12/robot/map).
// The rules mirror the robot's own parser (app_program/barcode_map.c), so a map accepted here is accepted by the car:
// a bad map is refused HERE, with a message, instead of being rejected by the robot with "ERR ...".

export const MAX_MAP_BYTES = 128;                 // limit on the text typed into the page; what goes to the car is the binary form (at most 27 bytes, the mailbox holds 32)

const ACTION_WORDS = {
  L: "L", LEFT: "L",
  R: "R", RIGHT: "R",
  F: "F", FWD: "F", FORWARD: "F", S: "F", STRAIGHT: "F",
  U: "U", UTURN: "U", "U-TURN": "U"
};
export const ACTION_NAMES = { L: "Left", R: "Right", F: "Straight", U: "U-turn" };

// ---- the binary form that goes over MQTT (app_program/barcode_map.h) -------------------------------------------------------------
// Map:   0x12, then ONE BYTE PER LETTER: the letter index (A = 0 to Z = 25) in bits 7 to 3 and the movement in bits 2 to 0 (left 1, right 2,
//        straight 3, u-turn 4).  A=L,B=R,F=U is the 4 bytes 12 01 0a 2c; the type byte alone clears the map.  This is also how the car stores it.
// Reply: 0x13, then ONE byte: the result in bits 7 to 5 (0 ok, 1 empty, 2 syntax, 3 letter, 4 action, 5 duplicate) and the number of letters in bits 4 to 0.
export const MAP_TYPE = 0x12;
export const MAP_ACK_TYPE = 0x13;
const COMMAND = { L: 1, R: 2, F: 3, U: 4 };
const RESULT_NAMES = ["OK", "empty", "syntax", "letter", "action", "duplicate"];

export function encodeMapBinary(entries) {
  const bytes = [MAP_TYPE];
  for (const e of entries) bytes.push(((e.letter.charCodeAt(0) - 65) << 3) | COMMAND[e.action]);
  return Buffer.from(bytes);
}

// The car's reply as text for the page: "OK n=3" or "ERR duplicate".  A car with older firmware answers in text; that is passed through.
export function decodeMapAck(payload) {
  if (payload.length === 2 && payload[0] === MAP_ACK_TYPE) {
    const result = payload[1] >> 5, count = payload[1] & 31;
    return { binary: true, ok: result === 0, result, count, text: result === 0 ? `OK n=${count}` : `ERR ${RESULT_NAMES[result] ?? "unknown"}` };
  }
  const text = payload.toString("utf8").slice(0, 40);
  return { binary: false, ok: text.startsWith("OK"), text };
}

// Parse a text the way the robot does.  Returns { ok: true, entries: [{letter, action}], text } where text is the canonical form shown on the
// page ("A=L,B=R,F=U", or "CLEAR"; what goes over MQTT is encodeMapBinary(entries)), or { ok: false, error }.
export function parseMap(input) {
  if (typeof input !== "string") return { ok: false, error: "the map must be text" };
  const tokens = input.toUpperCase().split(/[\s,;]+/).filter(Boolean);
  if (tokens.length === 0) return { ok: false, error: "the map is empty" };

  if (tokens.length === 1 && tokens[0] === "CLEAR") return { ok: true, entries: [], text: "CLEAR" };

  const entries = [];
  const add = (letter, action) => {
    if (entries.some(e => e.letter === letter)) return `letter ${letter} is given twice`;
    entries.push({ letter, action });
    return null;
  };

  // "LRFULRFU...": exactly 26 single-letter actions, for A to Z in order
  if (tokens.length === 1 && tokens[0].length > 3 && /^[LRFSU]+$/.test(tokens[0])) {
    if (tokens[0].length !== 26) return { ok: false, error: `a letter-per-action string needs exactly 26 letters (got ${tokens[0].length})` };
    [...tokens[0]].forEach((c, i) => add(String.fromCharCode(65 + i), ACTION_WORDS[c]));
  } else {
    for (const token of tokens) {
      const m = /^(.)[=:](.+)$/.exec(token);
      if (!m) return { ok: false, error: `"${token}" is not letter=action (like A=L)` };
      if (!/^[A-Z]$/.test(m[1])) return { ok: false, error: `"${m[1]}" is not a letter A to Z` };
      const action = ACTION_WORDS[m[2]];
      if (!action) return { ok: false, error: `"${m[2]}" is not an action (use L, R, F for straight, or U for a U-turn)` };
      const problem = add(m[1], action);
      if (problem) return { ok: false, error: problem };
    }
  }
  entries.sort((a, b) => a.letter.localeCompare(b.letter));
  const text = entries.map(e => `${e.letter}=${e.action}`).join(",");
  if (Buffer.byteLength(text) > MAX_MAP_BYTES) return { ok: false, error: "the map is too long for the robot" };
  return { ok: true, entries, text };
}
