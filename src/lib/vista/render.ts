import { clamp } from "./math";
import type { DetectedObject, Track, Trajectory, Waypoint } from "./types";
import type { Vehicle } from "./vehicle";
import type { World } from "./world";

const MAP_PAL = {
  safe: [72, 86, 64],
  rough: [112, 96, 58],
  obs: [58, 54, 50],
  drop: [16, 14, 16],
  fog: [32, 34, 36],
};

export function paintWorldMap(world: World, dest: Uint8ClampedArray, dw: number, dh: number): void {
  for (let j = 0; j < dh; j++) {
    for (let i = 0; i < dw; i++) {
      const x = world.minX + (i / dw) * (world.maxX - world.minX);
      const y = world.maxY - (j / dh) * (world.maxY - world.minY);
      const c = ((x - world.minX) / world.cell) | 0;
      const r = ((y - world.minY) / world.cell) | 0;
      const ii = clamp(r, 0, world.rows - 1) * world.cols + clamp(c, 0, world.cols - 1);
      const terr = world.terrain[ii]!;
      let col = MAP_PAL.safe;
      if (terr === 1) col = MAP_PAL.rough;
      else if (terr === 2) col = MAP_PAL.obs;
      else if (terr === 3) col = MAP_PAL.drop;
      const pi = (j * dw + i) * 4;
      const n = 0.9 + 0.08 * Math.sin(x * 2.2) * Math.cos(y * 1.7);
      dest[pi] = col[0]! * n;
      dest[pi + 1] = col[1]! * n;
      dest[pi + 2] = col[2]! * n;
      dest[pi + 3] = 255;
    }
  }
}

export function drawMapOverlay(
  ctx: CanvasRenderingContext2D,
  world: World,
  v: Vehicle,
  path: Waypoint[],
  traj: Trajectory | null,
  goal: Waypoint | null,
  tracks: Track[],
  w: number,
  h: number,
): void {
  const sx = w / (world.maxX - world.minX);
  const sy = h / (world.maxY - world.minY);
  const X = (x: number) => (x - world.minX) * sx;
  const Y = (y: number) => h - (y - world.minY) * sy;

  ctx.lineWidth = 2;
  ctx.strokeStyle = "rgba(185,196,206,0.85)";
  ctx.beginPath();
  path.forEach((p, i) => (i === 0 ? ctx.moveTo(X(p.x), Y(p.y)) : ctx.lineTo(X(p.x), Y(p.y))));
  ctx.stroke();

  if (traj && traj.poses.length) {
    ctx.strokeStyle = "rgba(196,165,116,0.95)";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    traj.poses.forEach((p, i) => (i === 0 ? ctx.moveTo(X(p.x), Y(p.y)) : ctx.lineTo(X(p.x), Y(p.y))));
    ctx.stroke();
  }

  for (const tr of tracks) {
    ctx.fillStyle = tr.static ? "rgba(180,180,180,0.7)" : "rgba(196,107,93,0.85)";
    ctx.beginPath();
    ctx.arc(X(tr.worldX), Y(tr.worldY), tr.classId === "vehicle" ? 6 : 4, 0, Math.PI * 2);
    ctx.fill();
    if (!tr.static) {
      ctx.strokeStyle = "rgba(196,107,93,0.7)";
      ctx.beginPath();
      ctx.moveTo(X(tr.worldX), Y(tr.worldY));
      ctx.lineTo(X(tr.predX), Y(tr.predY));
      ctx.stroke();
    }
  }

  if (goal) {
    ctx.strokeStyle = "#e6e3dc";
    ctx.fillStyle = "rgba(230,227,220,0.2)";
    ctx.beginPath();
    ctx.arc(X(goal.x), Y(goal.y), 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
  }

  const yaw = v.yaw;
  const px = X(v.x);
  const py = Y(v.y);
  ctx.save();
  ctx.translate(px, py);
  ctx.rotate(-yaw);
  ctx.fillStyle = "#e6e3dc";
  ctx.beginPath();
  ctx.moveTo(10, 0);
  ctx.lineTo(-8, 6);
  ctx.lineTo(-5, 0);
  ctx.lineTo(-8, -6);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

export function depthToRgb(rel: Float32Array, w: number, h: number, dest: Uint8ClampedArray): void {
  for (let i = 0; i < w * h; i++) {
    const t = clamp(rel[i]!, 0, 1);
    const c = turbo(t);
    dest[i * 4] = c[0];
    dest[i * 4 + 1] = c[1];
    dest[i * 4 + 2] = c[2];
    dest[i * 4 + 3] = 255;
  }
}

function turbo(t: number): [number, number, number] {
  const x = clamp(t, 0, 1);
  const r = 48 + 180 * x + 20 * Math.sin(x * 6);
  const g = 40 + 140 * Math.pow(x, 0.7);
  const b = 90 + 90 * (1 - x) * (1 - x);
  return [clamp(r, 0, 255), clamp(g, 0, 255), clamp(b, 0, 255)];
}

let scratch: HTMLCanvasElement | null = null;

export function blit(
  ctx: CanvasRenderingContext2D,
  data: Uint8ClampedArray,
  srcW: number,
  srcH: number,
): void {
  if (typeof document === "undefined") return;
  if (!scratch) scratch = document.createElement("canvas");
  if (scratch.width !== srcW || scratch.height !== srcH) {
    scratch.width = srcW;
    scratch.height = srcH;
  }
  const octx = scratch.getContext("2d");
  if (!octx) return;
  const img = octx.createImageData(srcW, srcH);
  img.data.set(data);
  octx.putImageData(img, 0, 0);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(scratch, 0, 0, ctx.canvas.width, ctx.canvas.height);
}

export function drawDetections(
  ctx: CanvasRenderingContext2D,
  dets: DetectedObject[],
  srcW: number,
  srcH: number,
): void {
  const sx = ctx.canvas.width / srcW;
  const sy = ctx.canvas.height / srcH;
  ctx.lineWidth = 1.5;
  ctx.font = "11px IBM Plex Mono, ui-monospace, monospace";
  for (const d of dets) {
    const color = d.classId === "person" ? "#c46b5d" : d.classId === "vehicle" ? "#c4a574" : "#b9c4ce";
    ctx.strokeStyle = color;
    ctx.strokeRect(d.x * sx, d.y * sy, d.w * sx, d.h * sy);
    ctx.fillStyle = color;
    ctx.fillText(`${d.classId} ${d.confidence.toFixed(2)}`, d.x * sx + 3, d.y * sy + 12);
  }
}

export function screenToWorld(
  world: World,
  canvas: HTMLCanvasElement,
  clientX: number,
  clientY: number,
): Waypoint {
  const r = canvas.getBoundingClientRect();
  const u = (clientX - r.left) / r.width;
  const v = (clientY - r.top) / r.height;
  return {
    x: world.minX + u * (world.maxX - world.minX),
    y: world.maxY - v * (world.maxY - world.minY),
  };
}
