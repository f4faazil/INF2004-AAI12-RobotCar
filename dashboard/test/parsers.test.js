import test from "node:test";
import assert from "node:assert/strict";
import { parseDiagnostic, parseFutureJson, parseHeartbeat, parseProvisionalTelemetry } from "../server/parsers.js";

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
