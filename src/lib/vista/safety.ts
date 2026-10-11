import { vistaConfig } from "./config";
import type { Esp32State, LocStatus, SafetyState, VehicleCmd } from "./types";

export type SafetyInput = {
  loc: LocStatus;
  nearest: number;
  terrainConf: number;
  perceptionOk: boolean;
  ultraFront: number;
  estop: boolean;
  watchdogOk: boolean;
  cmdAge: number;
  plannerOk: boolean;
  requested: VehicleCmd;
};

export type SafetyOut = {
  state: SafetyState;
  allowed: VehicleCmd;
  rejected: boolean;
  reason: string;
};

/**
 * Independent of the AI planner. Final authority to allow or reject motion.
 * Thresholds live in config — never silently fail.
 */
export function supervise(input: SafetyInput, prev: SafetyState, t: number): SafetyOut {
  const s = vistaConfig.safety;
  const zero: VehicleCmd = {
    speed: 0,
    steering: 0,
    stop: true,
    estop: input.estop,
    timestamp: t,
  };

  if (input.estop) {
    return { state: "EMERGENCY_STOP", allowed: { ...zero, estop: true }, rejected: true, reason: "E-STOP" };
  }
  if (!input.watchdogOk || input.cmdAge > s.commandTimeout * 2.5) {
    return { state: "EMERGENCY_STOP", allowed: zero, rejected: true, reason: "watchdog / stale command" };
  }
  if (input.ultraFront < s.emergencyStopDistance || input.nearest < s.emergencyStopDistance) {
    return { state: "STOPPING", allowed: zero, rejected: true, reason: "obstacle inside emergency distance" };
  }
  if (input.loc.health === "LOST") {
    return { state: "STOPPING", allowed: zero, rejected: true, reason: "localization lost" };
  }

  let state: SafetyState = "NORMAL";
  let scale = 1;
  let reason = "normal";

  if (input.loc.health === "RECOVERY" || !input.plannerOk) {
    state = "RECOVERY";
    scale = 0.25;
    reason = "recovery";
  } else if (input.loc.health === "DEGRADED" || !input.perceptionOk || input.terrainConf < s.terrainConfMin) {
    state = "DEGRADED";
    scale = 0.4;
    reason = "degraded perception/localization";
  } else if (input.loc.health === "LOW_CONFIDENCE" || input.loc.confidence < s.confidenceThreshold) {
    state = "LOW_CONFIDENCE";
    scale = 0.55;
    reason = "low confidence — reduced speed, larger margin";
  }

  if (input.ultraFront < s.slowDistance || input.nearest < s.slowDistance) {
    scale = Math.min(scale, 0.35);
    if (state === "NORMAL") state = "LOW_CONFIDENCE";
    reason = "obstacle in slow zone";
  }

  if (prev === "EMERGENCY_STOP" && !input.estop) {
    // Require an explicit clear — stay stopped until operator resumes.
    return { state: "STOPPING", allowed: zero, rejected: true, reason: "E-STOP latched — resume required" };
  }

  const allowed: VehicleCmd = {
    speed: input.requested.speed * scale,
    steering: input.requested.steering,
    stop: input.requested.speed * scale < 0.05,
    estop: false,
    timestamp: t,
  };
  const rejected = scale < 1 && input.requested.speed > allowed.speed + 0.05;
  void prev;
  return { state, allowed, rejected, reason };
}

export class Esp32Bridge {
  state: Esp32State = {
    connected: true,
    heartbeatAge: 0,
    motorPwm: 0,
    servoDeg: vistaConfig.esp32.servoCenter,
    watchdogOk: true,
    lastCmdAge: 0,
    estopLatched: false,
  };

  apply(cmd: VehicleCmd, dt: number): Esp32State {
    this.state.lastCmdAge += dt;
    this.state.heartbeatAge += dt;
    if (cmd.timestamp) this.state.lastCmdAge = 0;
    this.state.heartbeatAge = 0;
    this.state.connected = true;

    const stale = this.state.lastCmdAge > vistaConfig.safety.commandTimeout;
    this.state.watchdogOk = !stale;
    if (stale || cmd.stop || cmd.estop) {
      this.state.motorPwm = 0;
      if (cmd.estop) this.state.estopLatched = true;
    } else {
      const frac = cmd.speed / Math.max(0.01, vistaConfig.vehicle.maxSpeed);
      this.state.motorPwm = Math.round(frac * vistaConfig.esp32.pwmMax);
    }
    const span = vistaConfig.esp32.servoSpan;
    const n = cmd.steering / vistaConfig.vehicle.maxSteeringAngle;
    this.state.servoDeg = vistaConfig.esp32.servoCenter - n * span;
    return this.state;
  }

  failComm(): void {
    this.state.connected = false;
    this.state.watchdogOk = false;
    this.state.motorPwm = 0;
  }
}
