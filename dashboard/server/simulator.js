export function createSimulator(emit, intervalMs = 500) {
  let tick = 0;
  const timer = setInterval(() => {
    tick++;
    const receivedAt = new Date().toISOString();
    const cycle = Math.floor((tick - 1) / 40);
    if (tick % 40 === 1) emit({ type: "online-marker", topic: "aai12/test", source: "demo-simulated", receivedAt, data: "AAI12_PICO_W_ONLINE" });
    if (tick % 40 === 2) emit({
      type: "diagnostic-snapshot", topic: "aai12/diagnostic", source: "demo-simulated", receivedAt,
      raw: `v=1,b=demo${cycle},rr=0,tk=524,r=0,t=${tick * 100},s=3,oc=${tick * 10},og=104,mp=${tick * 8},mg=102,lp=${tick * 100},lt=${tick * 100 - 20},pa=${cycle + 2},pq=${cycle + 2},pr=${cycle + 2}`,
      data: { schemaVersion: 1, bootNonce: `demo${cycle}`, resetReason: 0, timerTickConfig: 524, failureReason: 0, failureAtMs: tick * 100, mqttState: 3, ownerCalls: tick * 10, ownerMaxGapMs: 104, mqttPolls: tick * 8, mqttMaxGapMs: 102, lastPollMs: tick * 100, lastTxMs: tick * 100 - 20, pingAttempts: cycle + 2, pingsQueued: cycle + 2, pingResponses: cycle + 2 }
    });
    if (tick % 40 === 3) emit({
      type: "diagnostic-snapshot", topic: "aai12/diagnostic", source: "demo-simulated", receivedAt,
      raw: `v=1,b=demo${cycle},wf=0,of=0,te=0,le=0,wc=${cycle + 1},wt=${tick * 100 - 80},dc=${cycle + 1},dt=${tick * 100 - 60},ja=${cycle + 1},jr=${cycle}`,
      data: { schemaVersion: 1, bootNonce: `demo${cycle}`, tcpWriteFailures: 0, tcpOutputFailures: 0, tcpErrors: 0, lastTcpError: 0, wifiLinkChanges: cycle + 1, lastWifiChangeMs: tick * 100 - 80, dhcpChanges: cycle + 1, lastDhcpChangeMs: tick * 100 - 60, joinAttempts: cycle + 1, joinRecoveries: cycle }
    });
    if (tick % 20 === 0) emit({
      type: "heartbeat", topic: "aai12/robot/heartbeat", source: "demo-simulated", receivedAt,
      raw: `v=1,b=de000000,seq=${tick / 20 - 1},up=${tick * intervalMs}`,
      data: { schemaVersion: 1, bootId: "de000000", sequence: tick / 20 - 1, uptimeMs: tick * intervalMs }
    });
  }, intervalMs);
  return () => clearInterval(timer);
}
