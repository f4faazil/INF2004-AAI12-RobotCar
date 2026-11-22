import test from "node:test";
import assert from "node:assert/strict";
import { decodeOnlineMarker, parseDiagnostic, parseFutureJson, parseHeartbeat, parseProvisionalTelemetry } from "../server/parsers.js";

test("parses both diagnostic record parts and preserves boot nonce", () => {
  const result = parseDiagnostic("v=1,b=00af10cc,r=6,t=1200,pr=4,le=-11");
  assert.deepEqual(result.fields, {
    schemaVersion: 1, bootNonce: "00af10cc", failureReason: 6,
    failureAtMs: 1200, pingResponses: 4, lastTcpError: -11
  });
});

test("decodes the firmware's provisional LSB-first three-byte frame", () => {
  const packed = 0x04 | (6 << 8) | (73 << 11) | (1 << 20);
  const result = parseProvisionalTelemetry(Buffer.from([packed & 255, (packed >>> 8) & 255, (packed >>> 16) & 255]));
  assert.equal(result.type, 0x04);
  assert.equal(result.status, 6);
  assert.equal(result.height, 73);
  assert.equal(result.imuFault, true);
  assert.equal(result.confirmedDiagnosticOnly, true);
});

test("does not mislabel unknown binary payloads", () => {
  assert.deepEqual(parseProvisionalTelemetry(Buffer.from([1, 2])), { format: "unknown-binary", byteLength: 2, hex: "0102" });
});

test("accepts object JSON and rejects other JSON shapes", () => {
  assert.deepEqual(parseFutureJson('{"movement":"STOPPED"}'), { movement: "STOPPED" });
  assert.throws(() => parseFutureJson("[]"), /must be an object/);
});

test("strictly parses the bounded firmware heartbeat", () => {
  assert.deepEqual(parseHeartbeat("v=1,b=12abcdef,seq=42,up=123456").fields,
    { schemaVersion: 1, bootId: "12abcdef", sequence: 42, uptimeMs: 123456 });
  assert.throws(() => parseHeartbeat("v=1,b=bad,seq=42,up=123456"), /Invalid heartbeat/);
  assert.throws(() => parseHeartbeat("v=1,b=12abcdef,seq=-1,up=123456"), /Invalid heartbeat/);
});

// ---- binary forms: the same bytes the firmware's own tests check (tests/test_mqtt_heartbeat.c, tests/test_mqtt_lwip.c) ----
test("the 13-byte binary heartbeat gives the same fields as the old text one", () => {
  const bytes = Buffer.from("2112abcdef00000000" + "00002774", "hex");          // type, boot id 12abcdef, seq 0, uptime 10100
  const r = parseHeartbeat(bytes);
  assert.deepEqual(r.fields, { schemaVersion: 1, bootId: "12abcdef", sequence: 0, uptimeMs: 10100 });
  assert.equal(r.raw, "v=1,b=12abcdef,seq=0,up=10100");
  assert.deepEqual(parseHeartbeat(Buffer.from("2112abcdef00000001" + "0000c350", "hex")).fields,
    { schemaVersion: 1, bootId: "12abcdef", sequence: 1, uptimeMs: 50000 });
  assert.equal(parseHeartbeat(Buffer.from("2100ffffff" + "ffffffff" + "ffffffff", "hex")).fields.uptimeMs, 0xffffffff);
  assert.throws(() => parseHeartbeat(Buffer.from("2112abcdef0000000000002774aa", "hex")), /Malformed|Invalid/);   // wrong length is not trusted
});

test("the binary diagnostic parts decode to the named fields and the old text form", () => {
  const part1 = Buffer.alloc(50);
  part1[0] = 0x23; part1.writeUInt32BE(0x00af10cc, 1); part1[5] = 2; part1.writeUInt16BE(0x1ff, 6); part1[8] = 6; part1.writeUInt32BE(1200, 9);
  part1[13] = 3; part1.writeUInt32BE(77, 14); part1.writeUInt32BE(250, 18); part1.writeUInt32BE(9, 22); part1.writeUInt32BE(40, 26);
  part1.writeUInt32BE(1100, 30); part1.writeUInt32BE(1150, 34); part1.writeUInt32BE(5, 38); part1.writeUInt32BE(4, 42); part1.writeUInt32BE(3, 46);
  const a = parseDiagnostic(part1);
  assert.equal(a.fields.bootNonce, "00af10cc");
  assert.equal(a.fields.failureReason, 6);
  assert.equal(a.fields.failureAtMs, 1200);
  assert.equal(a.fields.ownerMaxGapMs, 250);
  assert.equal(a.fields.timerTickConfig, 0x1ff);
  assert.equal(a.fields.pingResponses, 3);
  assert.match(a.raw, /^v=1,b=00af10cc,rr=2,tk=511,r=6,t=1200,s=3,oc=77,og=250,/);
  // text from older firmware still parses to the same names
  assert.equal(parseDiagnostic(a.raw).fields.ownerMaxGapMs, 250);

  const part2 = Buffer.alloc(58);
  part2[0] = 0x24; part2.writeUInt32BE(0x00af10cc, 1); part2.writeUInt32BE(1, 5); part2.writeUInt32BE(2, 9); part2.writeUInt32BE(1, 13);
  part2.writeInt32BE(-11, 17); part2.writeUInt32BE(3, 21); part2[33] = 1; part2.writeUInt32BE(8, 54);
  const b = parseDiagnostic(part2);
  assert.equal(b.fields.lastTcpError, -11);                 // signed
  assert.equal(b.fields.usbIrqCount, 3);
  assert.equal(b.fields.usbLinkState, 1);
  assert.equal(b.fields.joinRecoveries, 8);
  assert.match(b.raw, /,le=-11,ui=3,/);
  // a part of the wrong length is not decoded as binary
  assert.equal(parseDiagnostic(Buffer.from([0x23, 1, 2, 3])).fields.schemaVersion, undefined);
});

test("the online marker is one byte now, and the old text is still shown the same", () => {
  assert.equal(decodeOnlineMarker(Buffer.from([0x22])), "AAI12_PICO_W_ONLINE");
  assert.equal(decodeOnlineMarker(Buffer.from("AAI12_PICO_W_ONLINE")), "AAI12_PICO_W_ONLINE");
});
