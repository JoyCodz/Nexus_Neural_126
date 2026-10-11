import { clamp, wrapAngle } from "../math";

export type PursuitPoint = { x: number; y: number };

/**
 * Pure Pursuit for an Ackermann vehicle.
 * curvature = 2 * sin(alpha) / Ld
 * steering = atan(L * curvature)
 */
export function purePursuit(
  x: number,
  y: number,
  yaw: number,
  path: PursuitPoint[],
  wheelbase: number,
  lookahead: number,
  maxSteer: number,
): { steering: number; target: PursuitPoint | null; alpha: number } {
  if (path.length < 2) return { steering: 0, target: null, alpha: 0 };

  let bestI = 0;
  let bestD = Infinity;
  for (let i = 0; i < path.length; i++) {
    const p = path[i]!;
    const d = (p.x - x) ** 2 + (p.y - y) ** 2;
    if (d < bestD) {
      bestD = d;
      bestI = i;
    }
  }

  let target = path[path.length - 1]!;
  for (let i = bestI; i < path.length; i++) {
    const p = path[i]!;
    if (Math.hypot(p.x - x, p.y - y) >= lookahead) {
      target = p;
      break;
    }
  }

  const dx = target.x - x;
  const dy = target.y - y;
  const alpha = wrapAngle(Math.atan2(dy, dx) - yaw);
  const curv = (2 * Math.sin(alpha)) / Math.max(lookahead, 0.4);
  const steering = clamp(Math.atan(wheelbase * curv), -maxSteer, maxSteer);
  return { steering, target, alpha };
}

export function speedDependentLookahead(
  speed: number,
  min: number,
  k: number,
  max: number,
): number {
  return clamp(min + k * Math.abs(speed), min, max);
}
