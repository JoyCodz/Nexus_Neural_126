import { footprintCorners } from "./algo/ackermann";
import { vistaConfig } from "./config";
import { clamp } from "./math";
import type { CostState, Track } from "./types";
import { heightAt, occAt, terrainAt, type World } from "./world";

export const COST_CODE: Record<CostState, number> = {
  FREE: 0,
  LOW_COST: 1,
  ROUGH: 2,
  HIGH_COST: 3,
  UNKNOWN: 4,
  PREDICTED_DYNAMIC_HAZARD: 5,
  OBSTACLE: 6,
  DROP_OFF: 7,
};

export class LocalCostmap {
  size: number;
  res: number;
  originX = 0;
  originY = 0;
  cost: Uint8Array;
  state: Uint8Array;
  logOdds: Float32Array;
  seen: Float32Array;
  dropVotes: Uint8Array;
  rgb: Uint8ClampedArray;

  constructor() {
    this.size = vistaConfig.costmap.size;
    this.res = vistaConfig.costmap.resolution;
    const n = this.size * this.size;
    this.cost = new Uint8Array(n);
    this.state = new Uint8Array(n);
    this.logOdds = new Float32Array(n);
    this.seen = new Float32Array(n);
    this.dropVotes = new Uint8Array(n);
    this.rgb = new Uint8ClampedArray(n * 4);
  }

  reset(): void {
    this.cost.fill(vistaConfig.costmap.unknownCost);
    this.state.fill(COST_CODE.UNKNOWN);
    this.logOdds.fill(0);
    this.seen.fill(0);
    this.dropVotes.fill(0);
  }

  worldToCell(x: number, y: number): { c: number; r: number } | null {
    const c = ((x - this.originX) / this.res) | 0;
    const r = ((y - this.originY) / this.res) | 0;
    if (c < 0 || r < 0 || c >= this.size || r >= this.size) return null;
    return { c, r };
  }

  cellWorld(c: number, r: number): [number, number] {
    return [this.originX + (c + 0.5) * this.res, this.originY + (r + 0.5) * this.res];
  }

  at(x: number, y: number): number {
    const cell = this.worldToCell(x, y);
    if (!cell) return vistaConfig.costmap.obstacleCost;
    return this.cost[cell.r * this.size + cell.c]!;
  }

  blocked(x: number, y: number): boolean {
    return this.at(x, y) >= 200;
  }

  update(
    world: World,
    x: number,
    y: number,
    yaw: number,
    tracks: Track[],
    dt: number,
    t: number,
    dropHint: boolean,
  ): void {
    const half = (this.size * this.res) / 2;
    this.originX = x - half;
    this.originY = y - half;
    const n = this.size * this.size;
    const decay = Math.exp(-dt / vistaConfig.costmap.decayTau);
    const infl = vistaConfig.costmap.inflation;
    const dynInfl = vistaConfig.costmap.dynamicInflation;

    for (let i = 0; i < n; i++) {
      this.logOdds[i]! *= decay;
      if (this.seen[i]! > 0) this.seen[i]! = Math.max(0, this.seen[i]! - dt * 0.15);
    }

    const fov = (vistaConfig.camera.fovDeg * Math.PI) / 180;
    for (let r = 0; r < this.size; r++) {
      for (let c = 0; c < this.size; c++) {
        const i = r * this.size + c;
        const [wx, wy] = this.cellWorld(c, r);
        const dx = wx - x;
        const dy = wy - y;
        const fwd = Math.cos(yaw) * dx + Math.sin(yaw) * dy;
        const left = -Math.sin(yaw) * dx + Math.cos(yaw) * dy;
        const bearing = Math.atan2(left, fwd);
        const inFov = fwd > 0.2 && fwd < 14 && Math.abs(bearing) < fov * 0.52;
        const terr = terrainAt(world, wx, wy);
        const occ = occAt(world, wx, wy);
        const h = heightAt(world, wx, wy);

        if (inFov) this.seen[i] = 1;

        if (terr === 3 || h < -1.2) {
          if (inFov || dropHint) this.dropVotes[i] = Math.min(12, this.dropVotes[i]! + 1);
        } else if (inFov) {
          this.dropVotes[i] = Math.max(0, this.dropVotes[i]! - 1);
        }

        if (this.dropVotes[i]! >= vistaConfig.terrain.dropConfirm) {
          this.logOdds[i] = 12;
        } else if (occ) {
          this.logOdds[i] = Math.max(this.logOdds[i]!, 8);
        } else if (inFov && !occ) {
          this.logOdds[i] = Math.max(-4, this.logOdds[i]! - 0.6);
        }
      }
    }

    // Rasterize inflated static + dynamic hazards into cost/state.
    for (let r = 0; r < this.size; r++) {
      for (let c = 0; c < this.size; c++) {
        const i = r * this.size + c;
        const [wx, wy] = this.cellWorld(c, r);
        const terr = terrainAt(world, wx, wy);
        const seen = this.seen[i]! > 0.2;
        let st: CostState = seen ? "FREE" : "UNKNOWN";
        let cost = seen ? 1 : vistaConfig.costmap.unknownCost;

        if (this.dropVotes[i]! >= vistaConfig.terrain.dropConfirm) {
          st = "DROP_OFF";
          cost = 255;
        } else if (this.logOdds[i]! > 3 || occAt(world, wx, wy)) {
          st = "OBSTACLE";
          cost = 255;
        } else if (terr === 1) {
          st = "ROUGH";
          cost = vistaConfig.costmap.roughCost;
        }

        this.state[i] = COST_CODE[st];
        this.cost[i] = cost;
      }
    }

    this.inflate(infl, 180);
    this.paintDynamics(tracks, dynInfl);
    this.paintRgb();
  }

  inflate(radius: number, occThresh: number): void {
    const cells = Math.ceil(radius / this.res);
    const copy = this.cost.slice();
    for (let r = 0; r < this.size; r++) {
      for (let c = 0; c < this.size; c++) {
        const i = r * this.size + c;
        if (copy[i]! < occThresh) continue;
        for (let dr = -cells; dr <= cells; dr++) {
          for (let dc = -cells; dc <= cells; dc++) {
            const rr = r + dr;
            const cc = c + dc;
            if (rr < 0 || cc < 0 || rr >= this.size || cc >= this.size) continue;
            const d = Math.hypot(dr, dc) * this.res;
            if (d > radius) continue;
            const j = rr * this.size + cc;
            const inf = 40 + ((radius - d) / radius) * 80;
            if (this.cost[j]! < inf && this.cost[j]! < 250) {
              this.cost[j] = inf;
              if (this.state[j] === COST_CODE.FREE || this.state[j] === COST_CODE.LOW_COST) {
                this.state[j] = COST_CODE.HIGH_COST;
              }
            }
          }
        }
      }
    }
  }

  paintDynamics(tracks: Track[], infl: number): void {
    const horizon = vistaConfig.tracker.predictHorizon;
    for (const tr of tracks) {
      const steps = 6;
      for (let s = 0; s <= steps; s++) {
        const u = s / steps;
        const px = tr.worldX + (tr.predX - tr.worldX) * u;
        const py = tr.worldY + (tr.predY - tr.worldY) * u;
        const rad = (tr.static ? infl * 0.7 : infl) * (tr.static ? 1 : 1 + u * 0.4);
        this.stampCircle(px, py, rad, tr.static ? 220 : 200, COST_CODE.PREDICTED_DYNAMIC_HAZARD);
      }
      void horizon;
    }
  }

  stampCircle(x: number, y: number, radius: number, cost: number, state: number): void {
    const cells = Math.ceil(radius / this.res);
    const c0 = ((x - this.originX) / this.res) | 0;
    const r0 = ((y - this.originY) / this.res) | 0;
    for (let dr = -cells; dr <= cells; dr++) {
      for (let dc = -cells; dc <= cells; dc++) {
        const rr = r0 + dr;
        const cc = c0 + dc;
        if (rr < 0 || cc < 0 || rr >= this.size || cc >= this.size) continue;
        if (Math.hypot(dr, dc) * this.res > radius) continue;
        const i = rr * this.size + cc;
        if (this.cost[i]! < cost) this.cost[i] = cost;
        if (state === COST_CODE.PREDICTED_DYNAMIC_HAZARD && this.state[i]! < COST_CODE.OBSTACLE) {
          this.state[i] = state;
        }
      }
    }
  }

  footprintHits(x: number, y: number, yaw: number, length: number, width: number): boolean {
    const pts = footprintCorners(x, y, yaw, length, width);
    for (const [px, py] of pts) {
      if (this.at(px, py) >= 200) return true;
    }
    return false;
  }

  clearance(x: number, y: number): number {
    let min = 8;
    const probe = 8;
    for (let r = -probe; r <= probe; r++) {
      for (let c = -probe; c <= probe; c++) {
        const cc = ((x - this.originX) / this.res + c) | 0;
        const rr = ((y - this.originY) / this.res + r) | 0;
        if (cc < 0 || rr < 0 || cc >= this.size || rr >= this.size) continue;
        if (this.cost[rr * this.size + cc]! >= 200) {
          min = Math.min(min, Math.hypot(c, r) * this.res);
        }
      }
    }
    return min;
  }

  paintRgb(): void {
    const n = this.size * this.size;
    for (let i = 0; i < n; i++) {
      const st = this.state[i]!;
      let r = 28,
        g = 30,
        b = 32;
      if (st === COST_CODE.FREE) {
        r = 42;
        g = 58;
        b = 48;
      } else if (st === COST_CODE.LOW_COST) {
        r = 58;
        g = 78;
        b = 58;
      } else if (st === COST_CODE.ROUGH) {
        r = 110;
        g = 92;
        b = 48;
      } else if (st === COST_CODE.HIGH_COST) {
        r = 120;
        g = 72;
        b = 44;
      } else if (st === COST_CODE.UNKNOWN) {
        r = 48;
        g = 50;
        b = 54;
      } else if (st === COST_CODE.PREDICTED_DYNAMIC_HAZARD) {
        r = 140;
        g = 86;
        b = 52;
      } else if (st === COST_CODE.OBSTACLE) {
        r = 150;
        g = 58;
        b = 48;
      } else if (st === COST_CODE.DROP_OFF) {
        r = 18;
        g = 16;
        b = 20;
      }
      const pi = i * 4;
      this.rgb[pi] = r;
      this.rgb[pi + 1] = g;
      this.rgb[pi + 2] = b;
      this.rgb[pi + 3] = 255;
    }
  }

  /** Flattened global-planning costs in world grid space — used by A*. */
  sampleCost(x: number, y: number): number {
    return this.at(x, y);
  }
}

export function buildGlobalGrid(
  world: World,
  costmap: LocalCostmap,
  tracks: Track[],
): { costs: Uint8Array; w: number; h: number; cell: number } {
  const cell = world.cell;
  const costs = new Uint8Array(world.cols * world.rows);
  const infl = vistaConfig.costmap.inflation;
  const inflC = Math.ceil(infl / cell);

  for (let r = 0; r < world.rows; r++) {
    for (let c = 0; c < world.cols; c++) {
      const i = r * world.cols + c;
      const terr = world.terrain[i]!;
      const occ = world.occ[i]!;
      if (terr === 3 || occ) costs[i] = 255;
      else if (terr === 1) costs[i] = 40;
      else costs[i] = 4;
    }
  }
  // Inflate obstacles on the global grid by vehicle footprint radius.
  const copy = costs.slice();
  for (let r = 0; r < world.rows; r++) {
    for (let c = 0; c < world.cols; c++) {
      if (copy[r * world.cols + c]! < 250) continue;
      for (let dr = -inflC; dr <= inflC; dr++) {
        for (let dc = -inflC; dc <= inflC; dc++) {
          const rr = r + dr;
          const cc = c + dc;
          if (rr < 0 || cc < 0 || rr >= world.rows || cc >= world.cols) continue;
          if (Math.hypot(dr, dc) * cell > infl) continue;
          const j = rr * world.cols + cc;
          if (costs[j]! < 250) costs[j] = 180;
        }
      }
    }
  }
  for (const tr of tracks) {
    stampWorld(world, costs, tr.worldX, tr.worldY, tr.static ? infl : infl + 0.4);
    if (!tr.static) stampWorld(world, costs, tr.predX, tr.predY, infl + 0.5);
  }
  void costmap;
  return { costs, w: world.cols, h: world.rows, cell };
}

function stampWorld(world: World, costs: Uint8Array, x: number, y: number, rad: number): void {
  const c0 = ((x - world.minX) / world.cell) | 0;
  const r0 = ((y - world.minY) / world.cell) | 0;
  const n = Math.ceil(rad / world.cell);
  for (let dr = -n; dr <= n; dr++) {
    for (let dc = -n; dc <= n; dc++) {
      const rr = r0 + dr;
      const cc = c0 + dc;
      if (rr < 0 || cc < 0 || rr >= world.rows || cc >= world.cols) continue;
      if (Math.hypot(dc, dr) * world.cell > rad) continue;
      const j = rr * world.cols + cc;
      if (costs[j]! < 250) costs[j] = 200;
    }
  }
}
