// The barcode letter -> movement mapping that the dashboard feeds to the robot over MQTT (topic aai12/robot/map).
// The rules mirror the robot's own parser (app_program/barcode_map.c), so a map accepted here is accepted by the car:
// a bad map is refused HERE, with a message, instead of being rejected by the robot with "ERR ...".

export const MAX_MAP_BYTES = 128;                 // what the robot's mailbox can hold (MQTT_MAILBOX_PAYLOAD_BYTES)

const ACTION_WORDS = {
  L: "L", LEFT: "L",
  R: "R", RIGHT: "R",
  F: "F", FWD: "F", FORWARD: "F", S: "F", STRAIGHT: "F",
  U: "U", UTURN: "U", "U-TURN": "U"
};
export const ACTION_NAMES = { L: "Left", R: "Right", F: "Straight", U: "U-turn" };

// Parse a text the way the robot does.  Returns { ok: true, entries: [{letter, action}], text } where text is the
// canonical form sent on the wire ("A=L,B=R,F=U", or "CLEAR"), or { ok: false, error }.
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
