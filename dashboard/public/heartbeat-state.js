(function installHeartbeatFreshness(global) {
  const staleAfterMs = 30000;
  global.heartbeatFreshness = function heartbeatFreshness(receivedAt, nowMs = Date.now()) {
    const receivedMs = Date.parse(receivedAt);
    if (!Number.isFinite(receivedMs)) return { valid: false, online: false, ageMs: null };
    const ageMs = Math.max(0, nowMs - receivedMs);
    return { valid: true, online: ageMs < staleAfterMs, ageMs };
  };
})(globalThis);
