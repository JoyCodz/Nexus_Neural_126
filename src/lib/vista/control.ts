import { purePursuit, speedDependentLookahead } from "./algo/purePursuit";
import { vistaConfig } from "./config";
import type { Trajectory, VehicleCmd, Waypoint } from "./types";

export function trackTrajectory(
  x: number,
  y: number,
  yaw: number,
  speed: number,
  traj: Trajectory | null,
  fallback: Waypoint[],
  t: number,
): VehicleCmd {
  const cfg = vistaConfig.purePursuit;
  const veh = vistaConfig.vehicle;
  const pts = traj?.poses ?? fallback;
  if (!pts.length) {
    return { speed: 0, steering: 0, stop: true, estop: false, timestamp: t };
  }
  const ld = speedDependentLookahead(speed, cfg.lookaheadMin, cfg.lookaheadK, cfg.lookaheadMax);
  const pp = purePursuit(x, y, yaw, pts, veh.wheelbase, ld, veh.maxSteeringAngle);
  const targetSpeed = traj ? traj.speed : Math.min(veh.maxSpeed * 0.7, 1.1);
  return {
    speed: targetSpeed,
    steering: pp.steering,
    stop: false,
    estop: false,
    timestamp: t,
  };
}
