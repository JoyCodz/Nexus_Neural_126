import { vistaConfig } from "./config";
import { clamp, seededRng } from "./math";
import type { ObjectClass } from "./types";

export type Prop = {
  kind: "rock" | "tree" | "crate" | "barrier";
  x: number;
  y: number;
  w: number;
  d: number;
  h: number;
};

export type Agent = {
  id: number;
  classId: ObjectClass;
  x: number;
  y: number;
  yaw: number;
  speed: number;
  radius: number;
  path: { x: number; y: number }[];
  i: number;
  dir: 1 | -1;
};

export type World = {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
  cell: number;
  cols: number;
  rows: number;
  height: Float32Array;
  occ: Uint8Array;
  terrain: Uint8Array; // 0 safe 1 rough 2 obstacle 3 drop
  props: Prop[];
  agents: Agent[];
  dropRegions: { x0: number; y0: number; x1: number; y1: number }[];
};

const T_SAFE = 0;
const T_ROUGH = 1;
const T_OBS = 2;
const T_DROP = 3;

export function worldIndex(w: World, x: number, y: number): number {
  const c = clamp(((x - w.minX) / w.cell) | 0, 0, w.cols - 1);
  const r = clamp(((y - w.minY) / w.cell) | 0, 0, w.rows - 1);
  return r * w.cols + c;
}

export function heightAt(w: World, x: number, y: number): number {
  if (x < w.minX || y < w.minY || x > w.maxX || y > w.maxY) return -2.4;
  return w.height[worldIndex(w, x, y)]!;
}

export function occAt(w: World, x: number, y: number): number {
  if (x < w.minX || y < w.minY || x > w.maxX || y > w.maxY) return 1;
  return w.occ[worldIndex(w, x, y)]!;
}

export function terrainAt(w: World, x: number, y: number): number {
  if (x < w.minX || y < w.minY || x > w.maxX || y > w.maxY) return T_DROP;
  return w.terrain[worldIndex(w, x, y)]!;
}

function stampRect(
  w: World,
  x0: number,
  y0: number,
  x1: number,
  y1: number,
  fn: (i: number, x: number, y: number) => void,
): void {
  const c0 = clamp(((Math.min(x0, x1) - w.minX) / w.cell) | 0, 0, w.cols - 1);
  const c1 = clamp(((Math.max(x0, x1) - w.minX) / w.cell) | 0, 0, w.cols - 1);
  const r0 = clamp(((Math.min(y0, y1) - w.minY) / w.cell) | 0, 0, w.rows - 1);
  const r1 = clamp(((Math.max(y0, y1) - w.minY) / w.cell) | 0, 0, w.rows - 1);
  for (let r = r0; r <= r1; r++) {
    for (let c = c0; c <= c1; c++) {
      const i = r * w.cols + c;
      fn(i, w.minX + (c + 0.5) * w.cell, w.minY + (r + 0.5) * w.cell);
    }
  }
}

function addProp(w: World, p: Prop): void {
  w.props.push(p);
  stampRect(w, p.x - p.w / 2, p.y - p.d / 2, p.x + p.w / 2, p.y + p.d / 2, (i) => {
    w.occ[i] = 1;
    w.terrain[i] = T_OBS;
    w.height[i] = Math.max(w.height[i]!, p.h * 0.45);
  });
}

/**
 * Outdoor course: origin is the SLAM local frame at startup.
 * Goal (15, 8) is reachable around a barrier, past a pedestrian, with a
 * drop-off south of the start and a narrow gap through rocks.
 */
export function createWorld(seed = 7): World {
  const cfg = vistaConfig.world;
  const cell = cfg.cell;
  const cols = Math.round((cfg.maxX - cfg.minX) / cell);
  const rows = Math.round((cfg.maxY - cfg.minY) / cell);
  const n = cols * rows;
  const w: World = {
    minX: cfg.minX,
    minY: cfg.minY,
    maxX: cfg.maxX,
    maxY: cfg.maxY,
    cell,
    cols,
    rows,
    height: new Float32Array(n),
    occ: new Uint8Array(n),
    terrain: new Uint8Array(n),
    props: [],
    agents: [],
    dropRegions: [],
  };
  const rng = seededRng(seed);

  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const i = r * cols + c;
      const x = cfg.minX + (c + 0.5) * cell;
      const y = cfg.minY + (r + 0.5) * cell;
      const hill = 0.11 * Math.sin(x * 0.35) * Math.cos(y * 0.28) + 0.05 * Math.sin(x * 0.9 + y * 0.4);
      w.height[i] = hill;
      w.terrain[i] = T_SAFE;
    }
  }

  // Ravine / drop-off along the south edge — must not be classified from one frame.
  const drop = { x0: -3, y0: -5, x1: 28, y1: -2.15 };
  w.dropRegions.push(drop);
  stampRect(w, drop.x0, drop.y0, drop.x1, drop.y1, (i) => {
    w.height[i] = -2.6;
    w.terrain[i] = T_DROP;
    w.occ[i] = 1;
  });

  // Rough patch north of start.
  stampRect(w, 3.5, 11, 8.5, 16, (i) => {
    w.terrain[i] = T_ROUGH;
    w.height[i] += 0.08 + rng() * 0.12;
  });

  // Barrier that blocks the straight line (0,0)→(15,8) with a northern gap.
  for (let y = -1.6; y <= 5.4; y += 0.7) {
    addProp(w, {
      kind: "rock",
      x: 7.4 + (rng() - 0.5) * 0.3,
      y,
      w: 1.1 + rng() * 0.4,
      d: 0.9,
      h: 0.7 + rng() * 0.4,
    });
  }
  addProp(w, { kind: "barrier", x: 7.6, y: 2.2, w: 0.6, d: 6.4, h: 0.9 });

  // Southern rock cluster — forces the northern gap.
  addProp(w, { kind: "rock", x: 11.2, y: 2.1, w: 1.6, d: 1.4, h: 0.85 });
  addProp(w, { kind: "rock", x: 12.4, y: 0.6, w: 1.3, d: 1.2, h: 0.7 });
  addProp(w, { kind: "crate", x: 4.8, y: -0.4, w: 0.8, d: 0.7, h: 0.6 });

  // Trees
  for (const [x, y] of [
    [2.2, 13.5],
    [18.5, 3.2],
    [21.4, 14.8],
    [9.8, 17.2],
    [25.2, 7.1],
  ] as const) {
    addProp(w, { kind: "tree", x, y, w: 0.7, d: 0.7, h: 2.4 });
  }

  // Parked vehicle-shaped obstacle past the gap.
  addProp(w, { kind: "crate", x: 19.2, y: 11.6, w: 2.2, d: 1.2, h: 1.1 });

  // Scatter rocks
  for (let k = 0; k < 10; k++) {
    const x = 1 + rng() * 24;
    const y = -1 + rng() * 18;
    if (Math.hypot(x, y) < 2.4) continue;
    if (Math.hypot(x - 15, y - 8) < 1.6) continue;
    addProp(w, {
      kind: "rock",
      x,
      y,
      w: 0.6 + rng() * 0.7,
      d: 0.5 + rng() * 0.6,
      h: 0.35 + rng() * 0.4,
    });
  }

  // Pedestrian looping across the planned corridor.
  w.agents.push({
    id: 1,
    classId: "person",
    x: 12.5,
    y: 9.2,
    yaw: 0,
    speed: 0.85,
    radius: 0.35,
    path: [
      { x: 11.2, y: 6.6 },
      { x: 13.8, y: 10.4 },
      { x: 12.0, y: 12.2 },
      { x: 10.4, y: 8.4 },
    ],
    i: 0,
    dir: 1,
  });
  // Second pedestrian further along.
  w.agents.push({
    id: 2,
    classId: "person",
    x: 17.5,
    y: 6.2,
    yaw: Math.PI / 2,
    speed: 0.7,
    radius: 0.32,
    path: [
      { x: 16.6, y: 5.2 },
      { x: 18.8, y: 7.8 },
    ],
    i: 0,
    dir: 1,
  });
  // Slow vehicle crossing near the goal.
  w.agents.push({
    id: 3,
    classId: "vehicle",
    x: 22.4,
    y: 2.5,
    yaw: Math.PI / 2,
    speed: 1.4,
    radius: 1.1,
    path: [
      { x: 22.4, y: 2.2 },
      { x: 22.4, y: 16.5 },
    ],
    i: 0,
    dir: 1,
  });

  return w;
}

export function stepAgents(w: World, dt: number): void {
  for (const a of w.agents) {
    if (a.path.length < 2) continue;
    const tgt = a.path[a.i]!;
    const dx = tgt.x - a.x;
    const dy = tgt.y - a.y;
    const d = Math.hypot(dx, dy);
    if (d < 0.25) {
      a.i += a.dir;
      if (a.i >= a.path.length || a.i < 0) {
        a.dir = a.dir === 1 ? -1 : 1;
        a.i = clamp(a.i, 0, a.path.length - 1);
      }
      continue;
    }
    a.yaw = Math.atan2(dy, dx);
    const step = Math.min(a.speed * dt, d);
    a.x += (dx / d) * step;
    a.y += (dy / d) * step;
  }
}

export function dynamicHit(w: World, x: number, y: number, radius: number): Agent | null {
  for (const a of w.agents) {
    if (Math.hypot(a.x - x, a.y - y) < a.radius + radius) return a;
  }
  return null;
}

export function ultrasonicRange(w: World, x: number, y: number, yaw: number, max = 5): number {
  const step = 0.08;
  for (let t = 0.2; t < max; t += step) {
    const px = x + Math.cos(yaw) * t;
    const py = y + Math.sin(yaw) * t;
    if (occAt(w, px, py) || dynamicHit(w, px, py, 0.12)) return t;
  }
  return max;
}
