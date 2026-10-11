import { fitPlaneRansac, type Vec3 } from "./algo/ransac";
import { vistaConfig } from "./config";
import { clamp, saturate } from "./math";
import type { DepthFrame, DetectedObject, GroundPlane, ObjectClass, TerrainClass } from "./types";
import { heightAt, occAt, terrainAt, type World } from "./world";
import type { Vehicle } from "./vehicle";

export type SenseResult = {
  rgb: Uint8ClampedArray;
  depthRel: Float32Array;
  depthMetric: Float32Array;
  depthConf: Float32Array;
  points: Vec3[];
  detections: DetectedObject[];
  nearest: number;
};

const COL_SKY_TOP = [92, 108, 122];
const COL_SKY_HOR = [186, 172, 148];
const COL_SAFE = [118, 124, 78];
const COL_ROUGH = [132, 108, 72];
const COL_ROCK = [86, 82, 76];
const COL_DROP = [28, 24, 22];
const COL_DIRT = [124, 106, 74];

function lerp3(a: number[], b: number[], t: number): [number, number, number] {
  return [a[0]! + (b[0]! - a[0]!) * t, a[1]! + (b[1]! - a[1]!) * t, a[2]! + (b[2]! - a[2]!) * t];
}

function shade(rgb: [number, number, number], dist: number, maxR: number): [number, number, number] {
  const h = saturate(dist / maxR);
  const fog = lerp3(rgb, COL_SKY_HOR, h * h * 0.72);
  return fog;
}

/**
 * Frame capture: synthetic RGB + geometric range.
 * MiDaS-Small contract: relative disparity, then optional metric scale
 * from ultrasonic / camera height. This is NOT claimed as native metric depth.
 */
export function sense(
  world: World,
  v: Vehicle,
  t: number,
  ultraFront: number,
): { sense: SenseResult; depth: DepthFrame; plane: GroundPlane | null } {
  const cam = vistaConfig.camera;
  const veh = vistaConfig.vehicle;
  const w = cam.width;
  const h = cam.height;
  const rgb = new Uint8ClampedArray(w * h * 4);
  const metric = new Float32Array(w * h);
  const rel = new Float32Array(w * h);
  const conf = new Float32Array(w * h);
  const fov = (cam.fovDeg * Math.PI) / 180;
  const fovV = fov * (h / w);
  const camX = v.x + Math.cos(v.yaw) * veh.cameraForward;
  const camY = v.y + Math.sin(v.yaw) * veh.cameraForward;
  const camH = veh.cameraHeight;
  const pitch = veh.cameraPitch;
  const points: Vec3[] = [];
  let nearest: number = cam.maxRange;

  for (let j = 0; j < h; j++) {
    const vn = j / (h - 1) - 0.5;
    const el = vn * fovV + pitch;
    for (let i = 0; i < w; i++) {
      const un = i / (w - 1) - 0.5;
      const az = un * fov;
      const pi = (j * w + i) * 4;
      if (el <= 0.012) {
        const sky = lerp3(COL_SKY_TOP, COL_SKY_HOR, saturate((vn + 0.5) * 1.4));
        rgb[pi] = sky[0]!;
        rgb[pi + 1] = sky[1]!;
        rgb[pi + 2] = sky[2]!;
        rgb[pi + 3] = 255;
        metric[j * w + i] = cam.maxRange;
        conf[j * w + i] = 0.15;
        continue;
      }
      const rayYaw = v.yaw + az;
      let hitD: number = cam.maxRange;
      let col: [number, number, number] = [COL_SKY_HOR[0]!, COL_SKY_HOR[1]!, COL_SKY_HOR[2]!];
      let hitConf = 0.2;
      let hitTerr = 0;
      const step = 0.18;
      for (let d = 0.35; d < cam.maxRange; d += step) {
        const px = camX + Math.cos(rayYaw) * d;
        const py = camY + Math.sin(rayYaw) * d;
        const gz = heightAt(world, px, py);
        const expected = camH - Math.tan(el) * d;
        const terr = terrainAt(world, px, py);
        if (gz < -1.4 && expected < -0.4) {
          hitD = d;
          hitTerr = 3;
          col = [COL_DROP[0]!, COL_DROP[1]!, COL_DROP[2]!];
          hitConf = 0.45;
          break;
        }
        if (expected <= gz + 0.03) {
          hitD = d;
          hitTerr = terr;
          if (terr === 3) col = [COL_DROP[0]!, COL_DROP[1]!, COL_DROP[2]!];
          else if (terr === 2) col = [COL_ROCK[0]!, COL_ROCK[1]!, COL_ROCK[2]!];
          else if (terr === 1) col = [COL_ROUGH[0]!, COL_ROUGH[1]!, COL_ROUGH[2]!];
          else col = lerp3(COL_SAFE, COL_DIRT, saturate((Math.sin(px * 3) + 1) * 0.2));
          const n = 0.88 + 0.12 * Math.sin(px * 8 + py * 6);
          col = [col[0]! * n, col[1]! * n, col[2]! * n];
          hitConf = terr === 3 ? 0.5 : 0.86;
          if ((j & 3) === 0 && (i & 3) === 0 && d < 10) {
            points.push({
              x: Math.cos(az) * d,
              y: Math.sin(az) * d,
              z: camH - gz,
            });
          }
          break;
        }
        if (occAt(world, px, py) && terr === 2 && expected < 1.2) {
          hitD = d;
          hitTerr = 2;
          col = [COL_ROCK[0]!, COL_ROCK[1]!, COL_ROCK[2]!];
          hitConf = 0.8;
          break;
        }
      }
      col = shade(col, hitD, cam.maxRange);
      rgb[pi] = col[0]!;
      rgb[pi + 1] = col[1]!;
      rgb[pi + 2] = col[2]!;
      rgb[pi + 3] = 255;
      metric[j * w + i] = hitD;
      conf[j * w + i] = hitConf;
      if (hitD < nearest && el > 0.04 && (hitTerr === 2 || hitTerr === 3)) nearest = hitD;
    }
  }

  // Dynamic agents as billboards + detections.
  const detections: DetectedObject[] = [];
  let detId = 1;
  const aspect = w / h;
  for (const a of world.agents) {
    const dx = a.x - camX;
    const dy = a.y - camY;
    const fwd = Math.cos(v.yaw) * dx + Math.sin(v.yaw) * dy;
    const left = -Math.sin(v.yaw) * dx + Math.cos(v.yaw) * dy;
    if (fwd < 0.6 || fwd > cam.maxRange) continue;
    const az = Math.atan2(left, fwd);
    if (Math.abs(az) > fov * 0.52) continue;
    const height = a.classId === "vehicle" ? 1.4 : a.classId === "person" ? 1.7 : 0.7;
    const width = a.classId === "vehicle" ? 1.8 : 0.55;
    const sx = (0.5 - az / fov) * w;
    const elTop = Math.atan2(camH - height, fwd);
    const elBot = Math.atan2(camH, fwd);
    const sy0 = (0.5 + (elTop - pitch) / fovV) * h;
    const sy1 = (0.5 + (elBot - pitch) / fovV) * h;
    const bw = (width / fwd) * (w / (fov * aspect)) * 1.8;
    const bh = Math.max(4, sy1 - sy0);
    const x0 = clamp(sx - bw / 2, 0, w - 1);
    const y0 = clamp(Math.min(sy0, sy1), 0, h - 1);
    const x1 = clamp(sx + bw / 2, 0, w - 1);
    const y1 = clamp(Math.max(sy0, sy1), 0, h - 1);
    const col =
      a.classId === "person" ? [52, 48, 44] : a.classId === "vehicle" ? [48, 58, 72] : [70, 62, 48];
    for (let yy = y0 | 0; yy <= y1; yy++) {
      for (let xx = x0 | 0; xx <= x1; xx++) {
        const mi = yy * w + xx;
        if (fwd >= metric[mi]! - 0.05) {
          metric[mi] = fwd;
          const pi = mi * 4;
          const edge = xx === (x0 | 0) || xx === (x1 | 0) || yy === (y0 | 0);
          rgb[pi] = col[0]! + (edge ? 30 : 0);
          rgb[pi + 1] = col[1]!;
          rgb[pi + 2] = col[2]!;
        }
      }
    }
    const confDet = clamp(0.92 - fwd / 28, 0.4, 0.95);
    detections.push({
      id: detId++,
      classId: a.classId,
      x: x0,
      y: y0,
      w: x1 - x0,
      h: y1 - y0,
      confidence: confDet,
      timestamp: t,
      range: fwd,
      bearing: az,
      worldX: a.x,
      worldY: a.y,
    });
    nearest = Math.min(nearest, fwd - a.radius);
  }

  // Static props as blocks in the image.
  for (const p of world.props) {
    const dx = p.x - camX;
    const dy = p.y - camY;
    const fwd = Math.cos(v.yaw) * dx + Math.sin(v.yaw) * dy;
    const left = -Math.sin(v.yaw) * dx + Math.cos(v.yaw) * dy;
    if (fwd < 0.5 || fwd > cam.maxRange) continue;
    const az = Math.atan2(left, fwd);
    if (Math.abs(az) > fov * 0.55) continue;
    const sx = (0.5 - az / fov) * w;
    const elTop = Math.atan2(camH - p.h, fwd);
    const elBot = Math.atan2(camH - 0.02, fwd);
    const sy0 = (0.5 + (elTop - pitch) / fovV) * h;
    const sy1 = (0.5 + (elBot - pitch) / fovV) * h;
    const bw = (Math.max(p.w, p.d) / fwd) * (w / fov) * 0.9;
    const x0 = clamp(sx - bw / 2, 0, w - 1);
    const x1 = clamp(sx + bw / 2, 0, w - 1);
    const y0 = clamp(Math.min(sy0, sy1), 0, h - 1);
    const y1 = clamp(Math.max(sy0, sy1), 0, h - 1);
    const base = p.kind === "tree" ? [42, 58, 36] : p.kind === "crate" ? [96, 72, 48] : COL_ROCK;
    for (let yy = y0 | 0; yy <= y1; yy++) {
      for (let xx = x0 | 0; xx <= x1; xx++) {
        const mi = yy * w + xx;
        if (fwd < metric[mi]!) {
          metric[mi] = fwd;
          const pi = mi * 4;
          rgb[pi] = base[0]!;
          rgb[pi + 1] = base[1]!;
          rgb[pi + 2] = base[2]!;
        }
      }
    }
    if (fwd < 11 && p.kind !== "tree") {
      detections.push({
        id: detId++,
        classId: "obstacle",
        x: x0,
        y: y0,
        w: Math.max(2, x1 - x0),
        h: Math.max(2, y1 - y0),
        confidence: clamp(0.8 - fwd / 20, 0.36, 0.88),
        timestamp: t,
        range: fwd,
        bearing: az,
        worldX: p.x,
        worldY: p.y,
      });
    }
    nearest = Math.min(nearest, fwd - Math.max(p.w, p.d) * 0.4);
  }

  // Relative depth (MiDaS-Small contract): disparity-like, normalized, smoothed.
  let maxDisp = 1e-6;
  const disp = new Float32Array(w * h);
  for (let k = 0; k < w * h; k++) {
    disp[k] = 1 / (metric[k]! + 0.15);
    if (disp[k]! > maxDisp) maxDisp = disp[k]!;
  }
  const kSize = vistaConfig.midas.smoothKernel;
  for (let j = 0; j < h; j++) {
    for (let i = 0; i < w; i++) {
      let acc = 0;
      let n = 0;
      for (let dj = -kSize; dj <= kSize; dj++) {
        const yy = j + dj;
        if (yy < 0 || yy >= h) continue;
        acc += disp[yy * w + i]!;
        n++;
      }
      rel[j * w + i] = acc / n / maxDisp;
    }
  }

  // Metric scale from ultrasonic on the lower-center strip (calibrated layer).
  let scale = 1;
  if (vistaConfig.midas.scaleFromUltrasonic && ultraFront < 4.8) {
    let acc = 0;
    let n = 0;
    const y0 = (h * 0.62) | 0;
    const x0 = (w * 0.42) | 0;
    const x1 = (w * 0.58) | 0;
    for (let j = y0; j < h; j++) {
      for (let i = x0; i < x1; i++) {
        const r = rel[j * w + i]!;
        if (r > 0.08) {
          acc += 1 / r;
          n++;
        }
      }
    }
    if (n > 8) {
      const pred = acc / n;
      scale = clamp(ultraFront / (pred * 0.15), vistaConfig.midas.minScale, vistaConfig.midas.maxScale);
    }
  }

  const plane = fitPlaneRansac(points, vistaConfig.terrain.ransacIters, vistaConfig.terrain.ransacThresh);

  return {
    sense: {
      rgb,
      depthRel: rel,
      depthMetric: metric,
      depthConf: conf,
      points,
      detections,
      nearest,
    },
    depth: {
      width: w,
      height: h,
      relative: rel,
      metric,
      confidence: conf,
      timestamp: t,
      scale,
      source: "relative+calibrated",
    },
    plane,
  };
}

export function classifyTerrain(
  plane: GroundPlane | null,
  points: Vec3[],
  nearestDropVotes: number,
): {
  slope: number;
  roughness: number;
  dropRisk: number;
  confidence: number;
  klass: TerrainClass;
} {
  if (!plane) {
    return { slope: 0, roughness: 0, dropRisk: 0, confidence: 0.2, klass: "UNKNOWN" };
  }
  let varH = 0;
  for (const p of points) {
    const h = plane.a * p.x + plane.b * p.y + plane.c * p.z + plane.d;
    varH += h * h;
  }
  const roughness = points.length ? Math.sqrt(varH / points.length) : 0;
  const dropRisk = saturate(nearestDropVotes / vistaConfig.terrain.dropConfirm);
  let klass: TerrainClass = "SAFE";
  if (dropRisk > 0.8) klass = "DROP_OFF";
  else if (plane.slopeDeg > vistaConfig.terrain.slopeBlockDeg || roughness > 0.18) klass = "OBSTACLE";
  else if (plane.slopeDeg > vistaConfig.terrain.slopeWarnDeg || roughness > vistaConfig.terrain.roughnessWarn) {
    klass = "ROUGH";
  }
  if (plane.confidence < 0.35) klass = "UNKNOWN";
  return {
    slope: plane.slopeDeg,
    roughness,
    dropRisk,
    confidence: plane.confidence,
    klass,
  };
}

export const OBJECT_CLASSES: ObjectClass[] = ["person", "vehicle", "animal", "obstacle", "other"];
