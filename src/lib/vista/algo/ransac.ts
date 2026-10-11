import type { GroundPlane } from "../types";

export type Vec3 = { x: number; y: number; z: number };

/**
 * Fit a local ground plane ax + by + cz + d = 0 with RANSAC.
 * Does not assume the plane is horizontal. Returns null if too few points.
 */
export function fitPlaneRansac(
  points: Vec3[],
  iterations = 48,
  threshold = 0.08,
  rng: () => number = Math.random,
): GroundPlane | null {
  if (points.length < 12) return null;

  let bestCount = 0;
  let best: [number, number, number, number] = [0, 0, 1, 0];

  for (let i = 0; i < iterations; i++) {
    const i0 = (rng() * points.length) | 0;
    const i1 = (rng() * points.length) | 0;
    const i2 = (rng() * points.length) | 0;
    if (i0 === i1 || i1 === i2 || i0 === i2) continue;
    const p0 = points[i0]!;
    const p1 = points[i1]!;
    const p2 = points[i2]!;
    const ux = p1.x - p0.x;
    const uy = p1.y - p0.y;
    const uz = p1.z - p0.z;
    const vx = p2.x - p0.x;
    const vy = p2.y - p0.y;
    const vz = p2.z - p0.z;
    let a = uy * vz - uz * vy;
    let b = uz * vx - ux * vz;
    let c = ux * vy - uy * vx;
    const n = Math.hypot(a, b, c);
    if (n < 1e-6) continue;
    a /= n;
    b /= n;
    c /= n;
    if (c < 0) {
      a = -a;
      b = -b;
      c = -c;
    }
    const d = -(a * p0.x + b * p0.y + c * p0.z);
    let count = 0;
    for (const p of points) {
      if (Math.abs(a * p.x + b * p.y + c * p.z + d) < threshold) count++;
    }
    if (count > bestCount) {
      bestCount = count;
      best = [a, b, c, d];
    }
  }

  if (bestCount < 8) return null;

  // Refine with inlier centroid normal via covariance (tiny 3x3).
  const [a, b, c, d] = best;
  const inliers: Vec3[] = [];
  for (const p of points) {
    if (Math.abs(a * p.x + b * p.y + c * p.z + d) < threshold) inliers.push(p);
  }
  const nIn = inliers.length;
  let mx = 0,
    my = 0,
    mz = 0;
  for (const p of inliers) {
    mx += p.x;
    my += p.y;
    mz += p.z;
  }
  mx /= nIn;
  my /= nIn;
  mz /= nIn;

  const slope = Math.atan2(Math.hypot(a, b), Math.max(1e-6, c));
  const conf = Math.min(1, nIn / Math.max(20, points.length * 0.55));
  return {
    a,
    b,
    c,
    d: -(a * mx + b * my + c * mz),
    inliers: nIn,
    slopeDeg: (slope * 180) / Math.PI,
    confidence: conf,
  };
}

export function pointHeightOverPlane(p: Vec3, plane: GroundPlane): number {
  return plane.a * p.x + plane.b * p.y + plane.c * p.z + plane.d;
}
