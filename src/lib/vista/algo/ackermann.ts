import { clamp, wrapAngle } from "../math";
import type { Trajectory } from "../types";

export type BicycleState = { x: number; y: number; yaw: number; v: number };

/** Bicycle/Ackermann integration. Pose is the rear-axle / base_link. */
export function bicycleStep(
  s: BicycleState,
  steering: number,
  accel: number,
  wheelbase: number,
  dt: number,
  maxSteer: number,
  maxSpeed: number,
): BicycleState {
  const delta = clamp(steering, -maxSteer, maxSteer);
  const v = clamp(s.v + accel * dt, -maxSpeed * 0.25, maxSpeed);
  const yaw = wrapAngle(s.yaw + ((v * Math.tan(delta)) / wheelbase) * dt);
  return {
    x: s.x + v * Math.cos(s.yaw) * dt,
    y: s.y + v * Math.sin(s.yaw) * dt,
    yaw,
    v,
  };
}

export function footprintCorners(
  x: number,
  y: number,
  yaw: number,
  length: number,
  width: number,
  rearOverhang = 0.35,
): [number, number][] {
  const rear = -rearOverhang;
  const front = length - rearOverhang;
  const hy = width / 2;
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  const local: [number, number][] = [
    [front, hy],
    [front, -hy],
    [rear, -hy],
    [rear, hy],
    [(front + rear) / 2, 0],
    [front * 0.6, hy],
    [front * 0.6, -hy],
  ];
  return local.map(([fx, ly]) => [x + c * fx - s * ly, y + s * fx + c * ly]);
}

export function rollout(
  start: BicycleState,
  steering: number,
  speed: number,
  wheelbase: number,
  maxSteer: number,
  horizon: number,
  dt: number,
): Trajectory {
  const poses: Trajectory["poses"] = [];
  let s = { ...start, v: speed };
  let t = 0;
  poses.push({ x: s.x, y: s.y, yaw: s.yaw, t: 0 });
  while (t < horizon - 1e-6) {
    s = bicycleStep(s, steering, 0, wheelbase, dt, maxSteer, Math.max(speed, 0.1) + 0.4);
    t += dt;
    poses.push({ x: s.x, y: s.y, yaw: s.yaw, t });
  }
  return {
    poses,
    steering,
    speed,
    cost: 0,
    feasible: true,
    reason: "ok",
  };
}
