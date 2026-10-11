import { ByteTracker } from "./algo/bytetrack";
import { DEFAULT_GOAL, vistaConfig } from "./config";
import { trackTrajectory } from "./control";
import { LocalCostmap } from "./costmap";
import { VistaVio } from "./localization";
import { RingBuffer, clamp, dist } from "./math";
import { classifyTerrain, sense } from "./perception";
import { GoalManager, planGlobal, planLocal } from "./planning";
import { depthToRgb, paintWorldMap } from "./render";
import { Esp32Bridge, supervise, type SafetyOut } from "./safety";
import type {
  LogEvent,
  MissionMode,
  PerfSample,
  SafetyState,
  Trajectory,
  VehicleCmd,
  VistaSnapshot,
  Waypoint,
} from "./types";
import { createVehicle, stepVehicle, type Vehicle } from "./vehicle";
import { createWorld, stepAgents, ultrasonicRange, type World } from "./world";

export type VistaCommand =
  | { type: "set-goal"; x: number; y: number }
  | { type: "start" }
  | { type: "pause" }
  | { type: "stop" }
  | { type: "estop" }
  | { type: "resume" }
  | { type: "reset" }
  | { type: "manual"; throttle: number; steer: number }
  | { type: "mode"; mode: "auto" | "manual" }
  | { type: "reset-map" }
  | { type: "algorithm"; algo: "astar" | "dijkstra" };

const CAM_W = vistaConfig.camera.width;
const CAM_H = vistaConfig.camera.height;
const MAP_W = 360;
const MAP_H = 250;

export class VistaRuntime {
  world: World;
  vehicle: Vehicle;
  vio = new VistaVio();
  tracker = new ByteTracker(
    vistaConfig.tracker.highThresh,
    vistaConfig.tracker.lowThresh,
    vistaConfig.tracker.matchIou,
    vistaConfig.tracker.maxAge,
    vistaConfig.tracker.minHits,
    vistaConfig.tracker.predictHorizon,
    vistaConfig.tracker.staticSpeed,
  );
  costmap = new LocalCostmap();
  goals = new GoalManager();
  esp = new Esp32Bridge();
  mission: MissionMode = "IDLE";
  safety: SafetyState = "NORMAL";
  path: Waypoint[] = [];
  local: Trajectory | null = null;
  candidates: Trajectory[] = [];
  logs = new RingBuffer<LogEvent>(80);
  time = 0;
  dropVotes = 0;
  lastPlan = -999;
  lastCmd: VehicleCmd = { speed: 0, steering: 0, stop: true, estop: false, timestamp: 0 };
  estop = false;
  manual = { throttle: 0, steer: 0 };
  driveMode: "auto" | "manual" = "auto";
  paused = false;
  arrived = false;
  fps = 20;
  lastSafety: SafetyOut | null = null;
  worldRgb = new Uint8ClampedArray(MAP_W * MAP_H * 4);
  depthVis = new Uint8ClampedArray(CAM_W * CAM_H * 4);
  lastRgb = new Uint8ClampedArray(CAM_W * CAM_H * 4);
  perf: PerfSample = {
    cameraFps: 20,
    depthMs: 0,
    detectMs: 0,
    trackMs: 0,
    slamHz: 20,
    costmapMs: 0,
    plannerMs: 0,
    controlMs: 0,
    e2eMs: 0,
    estopMs: 4,
    cpu: 0,
  };
  plannerAlgo: "astar" | "dijkstra" = "astar";
  lastSnap: VistaSnapshot | null = null;
  listeners = new Set<(s: VistaSnapshot) => void>();

  constructor() {
    this.world = createWorld();
    this.vehicle = createVehicle(0, 0, 0);
    paintWorldMap(this.world, this.worldRgb, MAP_W, MAP_H);
    this.costmap.reset();
    this.log("info", "system", "SLAM origin set: X=0 Y=0 Yaw=0 (local frame, not GPS)");
    this.log("info", "system", "MiDaS-Small contract: relative depth + ultrasonic scale — not native metric");
  }

  on(fn: (s: VistaSnapshot) => void): () => void {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  }

  log(level: LogEvent["level"], source: string, msg: string): void {
    this.logs.push({ t: this.time, level, source, msg });
  }

  dispatch(cmd: VistaCommand): void {
    switch (cmd.type) {
      case "set-goal":
        this.goals.setGoal(cmd.x, cmd.y);
        this.arrived = false;
        this.replan("operator goal");
        this.log("info", "goal", `Goal set to (${cmd.x.toFixed(1)}, ${cmd.y.toFixed(1)})`);
        break;
      case "start":
        if (!this.goals.goal) this.goals.setGoal(DEFAULT_GOAL.x, DEFAULT_GOAL.y);
        this.estop = false;
        this.paused = false;
        this.driveMode = "auto";
        this.mission = "AUTONOMOUS";
        this.esp.state.estopLatched = false;
        this.replan("start autonomy");
        this.log("info", "mission", "Autonomy started");
        break;
      case "pause":
        this.paused = true;
        this.mission = "PAUSED";
        this.log("warn", "mission", "Paused");
        break;
      case "stop":
        this.paused = true;
        this.mission = "STOPPED";
        this.lastCmd = { ...this.lastCmd, speed: 0, stop: true };
        this.log("warn", "mission", "Stop");
        break;
      case "estop":
        this.estop = true;
        this.mission = "EMERGENCY_STOP";
        this.lastCmd = { speed: 0, steering: 0, stop: true, estop: true, timestamp: this.time };
        this.log("error", "safety", "EMERGENCY STOP");
        break;
      case "resume":
        this.estop = false;
        this.esp.state.estopLatched = false;
        this.paused = false;
        this.mission = this.driveMode === "manual" ? "MANUAL" : "AUTONOMOUS";
        this.log("info", "mission", "Resumed");
        break;
      case "reset":
        this.reset();
        break;
      case "manual":
        this.manual.throttle = cmd.throttle;
        this.manual.steer = cmd.steer;
        break;
      case "mode":
        this.driveMode = cmd.mode;
        this.mission = cmd.mode === "manual" ? "MANUAL" : this.mission === "IDLE" ? "IDLE" : "AUTONOMOUS";
        break;
      case "reset-map":
        this.costmap.reset();
        this.log("info", "costmap", "Local map cleared");
        break;
      case "algorithm":
        this.plannerAlgo = cmd.algo;
        this.replan("algorithm change");
        break;
    }
  }

  reset(): void {
    this.world = createWorld();
    this.vehicle = createVehicle(0, 0, 0);
    this.vio.reset();
    this.tracker.reset();
    this.costmap.reset();
    this.goals.clear();
    this.path = [];
    this.local = null;
    this.time = 0;
    this.estop = false;
    this.paused = false;
    this.arrived = false;
    this.mission = "IDLE";
    this.safety = "NORMAL";
    this.driveMode = "auto";
    this.manual = { throttle: 0, steer: 0 };
    paintWorldMap(this.world, this.worldRgb, MAP_W, MAP_H);
    this.log("info", "system", "Reset. Origin (0,0,0). Local SLAM frame.");
  }

  replan(why: string): void {
    if (!this.goals.goal) return;
    const pose = this.vio.pose;
    this.path = planGlobal(this.world, this.costmap, this.lastSnap?.tracks ?? [], pose, this.goals.goal, this.plannerAlgo);
    this.lastPlan = this.time;
    this.log("info", "planner", `A* replan (${why}) — ${this.path.length} waypoints`);
  }

  step(dt: number): VistaSnapshot {
    const t0 = performance.now();
    dt = clamp(dt, 0, 0.08);
    this.time += dt;
    stepAgents(this.world, dt);

    const veh = this.vehicle;
    const yawL = veh.yaw + Math.PI / 2;
    const yawR = veh.yaw - Math.PI / 2;
    const ultra = {
      front: ultrasonicRange(this.world, veh.x, veh.y, veh.yaw),
      left: ultrasonicRange(this.world, veh.x, veh.y, yawL, 3.5),
      right: ultrasonicRange(this.world, veh.x, veh.y, yawR, 3.5),
    };

    const tDepth = performance.now();
    const sensed = sense(this.world, veh, this.time, ultra.front);
    this.lastRgb.set(sensed.sense.rgb);
    depthToRgb(sensed.sense.depthRel, CAM_W, CAM_H, this.depthVis);
    this.perf.depthMs = performance.now() - tDepth;

    const tDet = performance.now();
    const detections = sensed.sense.detections;
    this.perf.detectMs = performance.now() - tDet;

    const tTrk = performance.now();
    const tracks = this.tracker.step(detections, this.time, dt);
    this.perf.trackMs = performance.now() - tTrk;

    // Texture proxy: mean gradient of RGB — low texture degrades VIO.
    let tex = 0;
    const rgb = sensed.sense.rgb;
    for (let i = 4; i < rgb.length; i += 16) tex += Math.abs(rgb[i]! - rgb[i - 4]!);
    tex = clamp(tex / (rgb.length * 0.02), 0.15, 1);

    const pose = this.vio.step(veh, this.time, dt, detections.length * 12 + 40, tex);

    const aheadZ = sensed.sense.depthMetric[((CAM_H * 0.72) | 0) * CAM_W + ((CAM_W / 2) | 0)] ?? 10;
    const aheadDrop = aheadZ > 12 || sensed.plane === null;
    if (aheadDrop && ultra.front > 3) this.dropVotes = Math.min(12, this.dropVotes + 1);
    else this.dropVotes = Math.max(0, this.dropVotes - 1);
    const terrain = classifyTerrain(sensed.plane, sensed.sense.points, this.dropVotes);

    const tCm = performance.now();
    this.costmap.update(this.world, pose.x, pose.y, pose.yaw, tracks, dt, this.time, this.dropVotes >= 4);
    this.perf.costmapMs = performance.now() - tCm;

    const nearest = Math.min(
      sensed.sense.nearest,
      ultra.front,
      ...tracks.filter((tr) => !tr.static).map((tr) => Math.hypot(tr.worldX - pose.x, tr.worldY - pose.y) - 0.4),
      99,
    );

    let requested: VehicleCmd = { speed: 0, steering: 0, stop: true, estop: this.estop, timestamp: this.time };

    if (this.goals.goal && this.goals.arrived(pose.x, pose.y) && this.mission === "AUTONOMOUS") {
      this.arrived = true;
      this.mission = "COMPLETE";
      this.log("info", "mission", "Goal tolerance reached — vehicle stopped");
    }

    if (this.driveMode === "manual" && !this.estop && this.mission !== "COMPLETE") {
      requested = {
        speed: this.manual.throttle * vistaConfig.vehicle.maxSpeed,
        steering: this.manual.steer * vistaConfig.vehicle.maxSteeringAngle,
        stop: Math.abs(this.manual.throttle) < 0.02,
        estop: false,
        timestamp: this.time,
      };
    } else if (
      this.mission === "AUTONOMOUS" &&
      !this.paused &&
      !this.arrived &&
      this.goals.goal &&
      !this.estop
    ) {
      if (this.path.length < 2 || this.time - this.lastPlan > 2.5) this.replan("periodic");
      const tPl = performance.now();
      const local = planLocal(pose.x, pose.y, pose.yaw, veh.v, this.path, this.costmap, tracks);
      this.candidates = local.candidates;
      this.local = local.best;
      this.perf.plannerMs = performance.now() - tPl;
      const tCtl = performance.now();
      requested = trackTrajectory(pose.x, pose.y, pose.yaw, veh.v, this.local, this.path, this.time);
      if (!this.local) {
        requested.speed = 0;
        requested.stop = true;
        if (!this.lastSnap?.localTraj) this.log("warn", "planner", "No feasible Ackermann trajectory — holding");
      }
      this.perf.controlMs = performance.now() - tCtl;
    }

    const cmdAge = this.time - this.lastCmd.timestamp;
    const safety = supervise(
      {
        loc: this.vio.status,
        nearest,
        terrainConf: terrain.confidence,
        perceptionOk: sensed.depth.confidence[sensed.depth.confidence.length >> 1]! > 0.25,
        ultraFront: ultra.front,
        estop: this.estop,
        watchdogOk: this.esp.state.watchdogOk && this.esp.state.connected,
        cmdAge,
        plannerOk: this.driveMode === "manual" || !!this.local || this.mission !== "AUTONOMOUS",
        requested,
      },
      this.safety,
      this.time,
    );
    this.lastSafety = safety;
    this.safety = safety.state;
    if (safety.state === "EMERGENCY_STOP") this.mission = "EMERGENCY_STOP";
    else if (safety.state === "DEGRADED" && this.mission === "AUTONOMOUS") this.mission = "DEGRADED";
    else if (safety.state === "RECOVERY" && this.mission === "AUTONOMOUS") this.mission = "RECOVERING";
    else if (
      this.mission === "DEGRADED" ||
      this.mission === "RECOVERING"
    ) {
      if (safety.state === "NORMAL" && !this.paused && !this.arrived) this.mission = "AUTONOMOUS";
    }

    const allowed = safety.allowed;
    this.lastCmd = allowed;
    this.esp.apply(allowed, dt);

    const appliedSpeed = this.esp.state.motorPwm === 0 ? 0 : allowed.speed;
    const appliedSteer = allowed.steering;
    if (!this.paused || this.driveMode === "manual") {
      stepVehicle(veh, appliedSpeed, appliedSteer, dt, this.time, this.world);
    } else {
      stepVehicle(veh, 0, 0, dt, this.time, this.world);
    }

    this.perf.e2eMs = performance.now() - t0;
    this.perf.cameraFps = this.fps;
    this.perf.slamHz = this.vio.status.updateHz;
    this.perf.cpu = clamp(this.perf.e2eMs / 16, 0.05, 1);

    const distGoal = this.goals.goal ? dist(pose.x, pose.y, this.goals.goal.x, this.goals.goal.y) : 0;

    const snap: VistaSnapshot = {
      time: this.time,
      mission: this.mission,
      safety: this.safety,
      pose,
      gtPose: veh.gt,
      loc: this.vio.status,
      goal: this.goals.goal,
      distanceToGoal: distGoal,
      arrived: this.arrived,
      speed: veh.v,
      steering: veh.steering,
      cmd: allowed,
      esp32: { ...this.esp.state },
      detections,
      tracks,
      globalPath: this.path,
      localTraj: this.local,
      candidates: this.candidates.filter((c) => c.feasible).slice(0, 8),
      plane: sensed.plane,
      terrainSummary: terrain,
      nearestObstacle: nearest,
      ultrasonic: ultra,
      perceptionOk: true,
      fps: this.fps,
      perf: { ...this.perf },
      logs: this.logs.toArray(),
      camW: CAM_W,
      camH: CAM_H,
      rgb: this.lastRgb,
      depthVis: this.depthVis,
      mapW: MAP_W,
      mapH: MAP_H,
      costRgb: this.costmap.rgb,
      worldRgb: this.worldRgb,
      costStates: this.costmap.state,
      costRes: this.costmap.res,
      costOriginX: this.costmap.originX,
      costOriginY: this.costmap.originY,
      costSize: this.costmap.size,
      worldMinX: this.world.minX,
      worldMinY: this.world.minY,
      worldMaxX: this.world.maxX,
      worldMaxY: this.world.maxY,
      agents: this.world.agents.map((a) => ({ classId: a.classId, x: a.x, y: a.y, yaw: a.yaw })),
      props: this.world.props.map((p) => ({ kind: p.kind, x: p.x, y: p.y, w: p.w, d: p.d })),
      dropRegions: this.world.dropRegions,
    };
    this.lastSnap = snap;
    for (const fn of this.listeners) fn(snap);
    return snap;
  }
}

let singleton: VistaRuntime | null = null;
export function getRuntime(): VistaRuntime {
  if (!singleton) singleton = new VistaRuntime();
  return singleton;
}
