import type { DetectedObject, ObjectClass, Track } from "../types";
import { PlaneKalman } from "./kalman";
import { dist } from "../math";

type Internal = {
  id: number;
  kf: PlaneKalman;
  classId: ObjectClass;
  conf: number;
  age: number;
  hits: number;
  lastSeen: number;
  bbox: { x: number; y: number; w: number; h: number };
  history: { t: number; x: number; y: number }[];
};

function iou(
  a: { x: number; y: number; w: number; h: number },
  b: { x: number; y: number; w: number; h: number },
): number {
  const x0 = Math.max(a.x, b.x);
  const y0 = Math.max(a.y, b.y);
  const x1 = Math.min(a.x + a.w, b.x + b.w);
  const y1 = Math.min(a.y + a.h, b.y + b.h);
  const inter = Math.max(0, x1 - x0) * Math.max(0, y1 - y0);
  const union = a.w * a.h + b.w * b.h - inter;
  return union <= 0 ? 0 : inter / union;
}

function greedyMatch(
  tracks: Internal[],
  dets: DetectedObject[],
  thresh: number,
  useIou: boolean,
): { matches: [number, number][]; uT: number[]; uD: number[] } {
  const pairs: { t: number; d: number; s: number }[] = [];
  for (let ti = 0; ti < tracks.length; ti++) {
    for (let di = 0; di < dets.length; di++) {
      const tr = tracks[ti]!;
      const det = dets[di]!;
      let s: number;
      if (useIou) {
        s = iou(tr.bbox, { x: det.x, y: det.y, w: det.w, h: det.h });
      } else {
        const d = dist(tr.kf.x, tr.kf.y, det.worldX, det.worldY);
        s = d < 3.5 ? 1 - d / 3.5 : 0;
      }
      if (s >= thresh) pairs.push({ t: ti, d: di, s });
    }
  }
  pairs.sort((a, b) => b.s - a.s);
  const usedT = new Set<number>();
  const usedD = new Set<number>();
  const matches: [number, number][] = [];
  for (const p of pairs) {
    if (usedT.has(p.t) || usedD.has(p.d)) continue;
    usedT.add(p.t);
    usedD.add(p.d);
    matches.push([p.t, p.d]);
  }
  const uT: number[] = [];
  const uD: number[] = [];
  for (let i = 0; i < tracks.length; i++) if (!usedT.has(i)) uT.push(i);
  for (let i = 0; i < dets.length; i++) if (!usedD.has(i)) uD.push(i);
  return { matches, uT, uD };
}

/**
 * ByteTrack-style two-stage association with a per-track Kalman filter.
 * High-confidence detections match first; leftovers try low-confidence.
 * Velocity is delta_position / delta_time from the filter, not a YOLO claim.
 */
export class ByteTracker {
  private tracks: Internal[] = [];
  private nextId = 1;
  constructor(
    private high = 0.55,
    private low = 0.18,
    private match = 0.3,
    private maxAge = 0.9,
    private minHits = 2,
    private horizon = 1.6,
    private staticSpeed = 0.28,
  ) {}

  reset(): void {
    this.tracks = [];
    this.nextId = 1;
  }

  step(dets: DetectedObject[], t: number, dt: number): Track[] {
    for (const tr of this.tracks) tr.kf.predict(dt);

    const high = dets.filter((d) => d.confidence >= this.high);
    const low = dets.filter((d) => d.confidence >= this.low && d.confidence < this.high);

    const m1 = greedyMatch(this.tracks, high, this.match, true);
    const leftoverTracks = m1.uT.map((i) => this.tracks[i]!);
    const m2 = greedyMatch(leftoverTracks, low, this.match * 0.75, false);

    const apply = (tr: Internal, det: DetectedObject) => {
      tr.kf.update(det.worldX, det.worldY);
      tr.classId = det.classId;
      tr.conf = det.confidence;
      tr.hits++;
      tr.age = 0;
      tr.lastSeen = t;
      tr.bbox = { x: det.x, y: det.y, w: det.w, h: det.h };
      tr.history.push({ t, x: det.worldX, y: det.worldY });
      if (tr.history.length > 24) tr.history.shift();
    };

    for (const [ti, di] of m1.matches) apply(this.tracks[ti]!, high[di]!);
    for (const [ti, di] of m2.matches) apply(leftoverTracks[ti]!, low[di]!);

    const claimed = new Set<DetectedObject>();
    for (const [, di] of m1.matches) claimed.add(high[di]!);
    for (const [, di] of m2.matches) claimed.add(low[di]!);

    for (const det of high) {
      if (claimed.has(det)) continue;
      const kf = new PlaneKalman();
      kf.reset(det.worldX, det.worldY);
      this.tracks.push({
        id: this.nextId++,
        kf,
        classId: det.classId,
        conf: det.confidence,
        age: 0,
        hits: 1,
        lastSeen: t,
        bbox: { x: det.x, y: det.y, w: det.w, h: det.h },
        history: [{ t, x: det.worldX, y: det.worldY }],
      });
    }

    const kept: Internal[] = [];
    for (const tr of this.tracks) {
      if (t - tr.lastSeen > 1e-6) tr.age += dt;
      if (tr.age <= this.maxAge) kept.push(tr);
    }
    this.tracks = kept;

    const out: Track[] = [];
    for (const tr of this.tracks) {
      if (tr.hits < this.minHits && tr.age > 0) continue;
      const speed = Math.hypot(tr.kf.vx, tr.kf.vy);
      const pred = tr.kf.predictAt(this.horizon);
      out.push({
        id: tr.id,
        classId: tr.classId,
        worldX: tr.kf.x,
        worldY: tr.kf.y,
        vx: tr.kf.vx,
        vy: tr.kf.vy,
        speed,
        static: speed < this.staticSpeed,
        confidence: tr.conf,
        age: tr.age,
        hits: tr.hits,
        lastSeen: tr.lastSeen,
        predX: pred.x,
        predY: pred.y,
        bbox: tr.bbox,
      });
    }
    return out;
  }
}
