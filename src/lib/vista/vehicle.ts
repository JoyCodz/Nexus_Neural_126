import { bicycleStep, footprintCorners } from "./algo/ackermann";
import { vistaConfig } from "./config";
import { clamp } from "./math";
import { occAt, type World } from "./world";
import type { Pose } from "./types";

export type Vehicle = {
  x: number;
  y: number;
  z: number;
  yaw: number;
  roll: number;
  pitch: number;
  v: number;
  steering: number;
  gt: Pose;
};

export function createVehicle(x = 0, y = 0, yaw = 0): Vehicle {
  return {
    x,
    y,
    z: 0,
    yaw,
    roll: 0,
    pitch: 0,
    v: 0,
    steering: 0,
    gt: { x, y, z: 0, roll: 0, pitch: 0, yaw, timestamp: 0 },
  };
}

export function stepVehicle(
  v: Vehicle,
  cmdSpeed: number,
  cmdSteer: number,
  dt: number,
  t: number,
  world: World,
): void {
  const cfg = vistaConfig.vehicle;
  const steerRate = cfg.maxSteeringRate;
  const steerErr = clamp(cmdSteer, -cfg.maxSteeringAngle, cfg.maxSteeringAngle) - v.steering;
  v.steering += clamp(steerErr, -steerRate * dt, steerRate * dt);

  const accel =
    cmdSpeed > v.v
      ? clamp((cmdSpeed - v.v) / Math.max(dt, 1e-3), -cfg.maxDecel, cfg.maxAccel)
      : clamp((cmdSpeed - v.v) / Math.max(dt, 1e-3), -cfg.maxDecel, cfg.maxAccel);

  const next = bicycleStep(
    { x: v.x, y: v.y, yaw: v.yaw, v: v.v },
    v.steering,
    accel,
    cfg.wheelbase,
    dt,
    cfg.maxSteeringAngle,
    cfg.maxSpeed,
  );

  const corners = footprintCorners(next.x, next.y, next.yaw, cfg.length, cfg.width);
  let blocked = false;
  for (const [cx, cy] of corners) {
    if (occAt(world, cx, cy)) {
      blocked = true;
      break;
    }
  }
  if (blocked && next.v > 0.05) {
    v.v = 0;
    return;
  }

  v.x = next.x;
  v.y = next.y;
  v.yaw = next.yaw;
  v.v = next.v;
  v.z = 0;
  v.gt = {
    x: v.x,
    y: v.y,
    z: v.z,
    roll: v.roll,
    pitch: v.pitch,
    yaw: v.yaw,
    timestamp: t,
  };
}

export function vehiclePose(v: Vehicle, t: number): Pose {
  return { x: v.x, y: v.y, z: v.z, roll: v.roll, pitch: v.pitch, yaw: v.yaw, timestamp: t };
}
