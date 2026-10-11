import { rollout } from "./algo/ackermann";
import { astar, dijkstra, simplifyPath } from "./algo/pathfind";
import { vistaConfig } from "./config";
import { dist } from "./math";
import type { Trajectory, Waypoint } from "./types";
import { buildGlobalGrid, type LocalCostmap } from "./costmap";
import type { Track } from "./types";
import type { World } from "./world";

export class GoalManager {
  goal: Waypoint | null = null;
  start: Waypoint = { x: 0, y: 0 };
  setGoal(x: number, y: number): void {
    this.goal = { x, y };
  }
  clear(): void {
    this.goal = null;
  }
  arrived(x: number, y: number): boolean {
    if (!this.goal) return false;
    return dist(x, y, this.goal.x, this.goal.y) <= vistaConfig.planner.goalTolerance;
  }
}

export function planGlobal(
  world: World,
  costmap: LocalCostmap,
  tracks: Track[],
  start: Waypoint,
  goal: Waypoint,
  algorithm: "astar" | "dijkstra" = "astar",
): Waypoint[] {
  const grid = buildGlobalGrid(world, costmap, tracks);
  const sc = clampInt(((start.x - world.minX) / world.cell) | 0, 0, grid.w - 1);
  const sr = clampInt(((start.y - world.minY) / world.cell) | 0, 0, grid.h - 1);
  const gc = clampInt(((goal.x - world.minX) / world.cell) | 0, 0, grid.w - 1);
  const gr = clampInt(((goal.y - world.minY) / world.cell) | 0, 0, grid.h - 1);

  // Nudge start/goal off inflated obstacles.
  const s = nearestFree(grid.costs, grid.w, grid.h, sc, sr);
  const g = nearestFree(grid.costs, grid.w, grid.h, gc, gr);
  const algo = algorithm;
  const cells =
    algo === "dijkstra"
      ? dijkstra(grid.costs, grid.w, grid.h, s, g)
      : astar(grid.costs, grid.w, grid.h, s, g, vistaConfig.planner.heuristicWeight);
  if (cells.length < 2) return [start, goal];
  const simple = simplifyPath(cells, grid.costs, grid.w);
  return simple.map((p) => ({
    x: world.minX + (p.c + 0.5) * world.cell,
    y: world.minY + (p.r + 0.5) * world.cell,
  }));
}

function clampInt(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

function nearestFree(
  costs: Uint8Array,
  w: number,
  h: number,
  c: number,
  r: number,
): { c: number; r: number } {
  if (costs[r * w + c]! < 200) return { c, r };
  for (let rad = 1; rad <= 12; rad++) {
    for (let dr = -rad; dr <= rad; dr++) {
      for (let dc = -rad; dc <= rad; dc++) {
        const cc = c + dc;
        const rr = r + dr;
        if (cc < 0 || rr < 0 || cc >= w || rr >= h) continue;
        if (costs[rr * w + cc]! < 200) return { c: cc, r: rr };
      }
    }
  }
  return { c, r };
}

export function planLocal(
  x: number,
  y: number,
  yaw: number,
  v: number,
  path: Waypoint[],
  costmap: LocalCostmap,
  tracks: Track[],
): { best: Trajectory | null; candidates: Trajectory[] } {
  const cfg = vistaConfig.localPlanner;
  const veh = vistaConfig.vehicle;
  const lookahead = 2.2;
  const target = lookAhead(path, x, y, lookahead) ?? path[path.length - 1];
  if (!target) return { best: null, candidates: [] };

  const candidates: Trajectory[] = [];
  const n = cfg.steerSamples;
  const speeds = [
    clampSpeed(v * 0.6, veh.maxSpeed),
    clampSpeed(Math.max(v, 0.45), veh.maxSpeed),
    clampSpeed(veh.maxSpeed * 0.85, veh.maxSpeed),
  ];

  for (const spd of speeds) {
    for (let i = 0; i < n; i++) {
      const steer = -veh.maxSteeringAngle + (2 * veh.maxSteeringAngle * i) / (n - 1);
      const traj = rollout(
        { x, y, yaw, v: spd },
        steer,
        spd,
        veh.wheelbase,
        veh.maxSteeringAngle,
        cfg.horizon,
        cfg.dt,
      );
      scoreTraj(traj, target, path, costmap, tracks, x, y);
      candidates.push(traj);
    }
  }

  let best: Trajectory | null = null;
  for (const c of candidates) {
    if (!c.feasible) continue;
    if (!best || c.cost < best.cost) best = c;
  }
  return { best, candidates };
}

function clampSpeed(v: number, max: number): number {
  return Math.min(max, Math.max(0.2, v));
}

function lookAhead(path: Waypoint[], x: number, y: number, ld: number): Waypoint | null {
  if (!path.length) return null;
  let bestI = 0;
  let bestD = Infinity;
  for (let i = 0; i < path.length; i++) {
    const d = dist(x, y, path[i]!.x, path[i]!.y);
    if (d < bestD) {
      bestD = d;
      bestI = i;
    }
  }
  for (let i = bestI; i < path.length; i++) {
    if (dist(x, y, path[i]!.x, path[i]!.y) >= ld) return path[i]!;
  }
  return path[path.length - 1]!;
}

function scoreTraj(
  traj: Trajectory,
  target: Waypoint,
  path: Waypoint[],
  costmap: LocalCostmap,
  tracks: Track[],
  x0: number,
  y0: number,
): void {
  const cfg = vistaConfig.localPlanner;
  const veh = vistaConfig.vehicle;
  let terrain = 0;
  let minClear = 8;
  let dyn = 0;
  const last = traj.poses[traj.poses.length - 1]!;

  for (const p of traj.poses) {
    if (costmap.footprintHits(p.x, p.y, p.yaw, veh.length, veh.width)) {
      traj.feasible = false;
      traj.reason = "footprint collision";
      traj.cost = 1e6;
      return;
    }
    const c = costmap.at(p.x, p.y);
    if (c >= 250) {
      traj.feasible = false;
      traj.reason = "lethal cost";
      traj.cost = 1e6;
      return;
    }
    terrain += c;
    minClear = Math.min(minClear, costmap.clearance(p.x, p.y));
    for (const tr of tracks) {
      const hx = tr.static ? tr.worldX : tr.worldX + tr.vx * p.t;
      const hy = tr.static ? tr.worldY : tr.worldY + tr.vy * p.t;
      const d = dist(p.x, p.y, hx, hy);
      const rad = tr.static ? 1.1 : 1.4;
      if (d < rad) {
        traj.feasible = false;
        traj.reason = "dynamic collision";
        traj.cost = 1e6;
        return;
      }
      if (d < rad + 1.2) dyn += (rad + 1.2 - d) / 1.2;
    }
  }

  if (minClear < cfg.minClearance) {
    traj.feasible = false;
    traj.reason = "clearance";
    traj.cost = 1e6;
    return;
  }

  const progress = dist(last.x, last.y, target.x, target.y);
  const startProg = dist(x0, y0, target.x, target.y);
  const along = Math.max(0, startProg - progress);
  const length = dist(last.x, last.y, x0, y0);
  const n = traj.poses.length;
  traj.cost =
    cfg.wProgress * (progress / Math.max(1, startProg)) +
    cfg.wClearance * Math.exp(-minClear) +
    cfg.wTerrain * (terrain / n / 32) +
    cfg.wLength * (length * 0.05) +
    cfg.wSteer * Math.abs(traj.steering) +
    cfg.wDynamic * dyn -
    along * 0.15;
  traj.reason = "ok";
  void path;
}
