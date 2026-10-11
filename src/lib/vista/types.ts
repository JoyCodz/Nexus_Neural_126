export type MissionMode =
  | "IDLE"
  | "AUTONOMOUS"
  | "MANUAL"
  | "PAUSED"
  | "DEGRADED"
  | "RECOVERING"
  | "STOPPED"
  | "EMERGENCY_STOP"
  | "COMPLETE";

export type SafetyState =
  | "NORMAL"
  | "LOW_CONFIDENCE"
  | "DEGRADED"
  | "RECOVERY"
  | "STOPPING"
  | "EMERGENCY_STOP";

export type LocHealth = "GOOD" | "LOW_CONFIDENCE" | "DEGRADED" | "RECOVERY" | "LOST";

export type TerrainClass = "SAFE" | "OBSTACLE" | "DROP_OFF" | "UNKNOWN" | "ROUGH";

export type CostState =
  | "FREE"
  | "LOW_COST"
  | "ROUGH"
  | "HIGH_COST"
  | "OBSTACLE"
  | "DROP_OFF"
  | "UNKNOWN"
  | "PREDICTED_DYNAMIC_HAZARD";

export type ObjectClass = "person" | "vehicle" | "animal" | "obstacle" | "other";

export type DetectedObject = {
  id: number;
  classId: ObjectClass;
  x: number;
  y: number;
  w: number;
  h: number;
  confidence: number;
  timestamp: number;
  range: number;
  bearing: number;
  worldX: number;
  worldY: number;
};

export type Track = {
  id: number;
  classId: ObjectClass;
  worldX: number;
  worldY: number;
  vx: number;
  vy: number;
  speed: number;
  static: boolean;
  confidence: number;
  age: number;
  hits: number;
  lastSeen: number;
  predX: number;
  predY: number;
  bbox: { x: number; y: number; w: number; h: number };
};

export type Pose = {
  x: number;
  y: number;
  z: number;
  roll: number;
  pitch: number;
  yaw: number;
  timestamp: number;
};

export type LocStatus = {
  health: LocHealth;
  tracking: "OK" | "RECENTLY_LOST" | "LOST";
  confidence: number;
  featureCount: number;
  updateHz: number;
  imuConsistent: boolean;
  jumpDetected: boolean;
};

export type Waypoint = { x: number; y: number };

export type Trajectory = {
  poses: { x: number; y: number; yaw: number; t: number }[];
  steering: number;
  speed: number;
  cost: number;
  feasible: boolean;
  reason: string;
};

export type VehicleCmd = {
  speed: number;
  steering: number;
  stop: boolean;
  estop: boolean;
  timestamp: number;
};

export type Esp32State = {
  connected: boolean;
  heartbeatAge: number;
  motorPwm: number;
  servoDeg: number;
  watchdogOk: boolean;
  lastCmdAge: number;
  estopLatched: boolean;
};

export type DepthFrame = {
  width: number;
  height: number;
  relative: Float32Array;
  metric: Float32Array;
  confidence: Float32Array;
  timestamp: number;
  scale: number;
  source: "relative+calibrated";
};

export type TerrainCell = {
  cost: number;
  slope: number;
  roughness: number;
  obstacle: boolean;
  dropOff: number;
  unknown: boolean;
  confidence: number;
  klass: TerrainClass;
};

export type GroundPlane = {
  a: number;
  b: number;
  c: number;
  d: number;
  inliers: number;
  slopeDeg: number;
  confidence: number;
};

export type LogEvent = {
  t: number;
  level: "info" | "warn" | "error";
  source: string;
  msg: string;
};

export type PerfSample = {
  cameraFps: number;
  depthMs: number;
  detectMs: number;
  trackMs: number;
  slamHz: number;
  costmapMs: number;
  plannerMs: number;
  controlMs: number;
  e2eMs: number;
  estopMs: number;
  cpu: number;
};

export type VistaSnapshot = {
  time: number;
  mission: MissionMode;
  safety: SafetyState;
  pose: Pose;
  gtPose: Pose;
  loc: LocStatus;
  goal: Waypoint | null;
  distanceToGoal: number;
  arrived: boolean;
  speed: number;
  steering: number;
  cmd: VehicleCmd;
  esp32: Esp32State;
  detections: DetectedObject[];
  tracks: Track[];
  globalPath: Waypoint[];
  localTraj: Trajectory | null;
  candidates: Trajectory[];
  plane: GroundPlane | null;
  terrainSummary: {
    slope: number;
    roughness: number;
    dropRisk: number;
    confidence: number;
    klass: TerrainClass;
  };
  nearestObstacle: number;
  ultrasonic: { front: number; left: number; right: number };
  perceptionOk: boolean;
  fps: number;
  perf: PerfSample;
  logs: LogEvent[];
  camW: number;
  camH: number;
  rgb: Uint8ClampedArray;
  depthVis: Uint8ClampedArray;
  mapW: number;
  mapH: number;
  costRgb: Uint8ClampedArray;
  worldRgb: Uint8ClampedArray;
  costStates: Uint8Array;
  costRes: number;
  costOriginX: number;
  costOriginY: number;
  costSize: number;
  worldMinX: number;
  worldMinY: number;
  worldMaxX: number;
  worldMaxY: number;
  agents: { classId: ObjectClass; x: number; y: number; yaw: number }[];
  props: { kind: string; x: number; y: number; w: number; d: number }[];
  dropRegions: { x0: number; y0: number; x1: number; y1: number }[];
};
