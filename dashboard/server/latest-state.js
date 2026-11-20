function diagnosticPart(message) {
  const data = message.data ?? {};
  if (data.timerTickConfig !== undefined || data.failureReason !== undefined || data.mqttState !== undefined) return "session";
  return "network";
}

export class LatestPicoState {
  constructor() {
    this.onlineMarker = null;
    this.diagnostics = new Map();
    this.diagnosticBootNonce = null;
    this.heartbeat = null;
    this.robot = new Map();             // the latest robot record of each kind
    this.robotEvents = [];              // and the last few events, so a new page can show what just happened
  }

  record(message) {
    if (message?.type === "online-marker") {
      this.onlineMarker = structuredClone(message);
      return;
    }
    if (message?.type === "heartbeat") {
      this.heartbeat = structuredClone(message);
      return;
    }
    if (message?.type === "robot-frame") {
      if (message.kind === "event") {
        this.robotEvents.push(structuredClone(message));
        if (this.robotEvents.length > 40) this.robotEvents.shift();
      } else {
        this.robot.set(message.kind, structuredClone(message));
      }
      return;
    }
    if (message?.type !== "diagnostic-snapshot") return;

    const bootNonce = message.data?.bootNonce;
    if (bootNonce && this.diagnosticBootNonce && bootNonce !== this.diagnosticBootNonce) {
      this.diagnostics.clear();
    }
    if (bootNonce && this.heartbeat?.data?.bootId &&
        bootNonce !== this.heartbeat.data.bootId) this.heartbeat = null;
    if (bootNonce) this.diagnosticBootNonce = bootNonce;
    this.diagnostics.set(diagnosticPart(message), structuredClone(message));
  }

  replay(now = new Date()) {
    const replayedAt = now.toISOString();
    return [this.onlineMarker, ...this.diagnostics.values(), this.heartbeat, ...this.robot.values(), ...this.robotEvents]
      .filter(Boolean)
      .sort((a, b) => Date.parse(a.receivedAt) - Date.parse(b.receivedAt))
      .map(message => ({ ...structuredClone(message), cached: true, replayedAt }));
  }
}
