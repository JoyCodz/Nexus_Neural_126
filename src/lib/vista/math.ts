export const TAU = Math.PI * 2;

export function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

export function wrapAngle(a: number): number {
  let x = a;
  while (x > Math.PI) x -= TAU;
  while (x < -Math.PI) x += TAU;
  return x;
}

export function angDiff(a: number, b: number): number {
  return wrapAngle(a - b);
}

export function hypot2(x: number, y: number): number {
  return x * x + y * y;
}

export function dist(ax: number, ay: number, bx: number, by: number): number {
  return Math.hypot(ax - bx, ay - by);
}

export function deg(rad: number): number {
  return (rad * 180) / Math.PI;
}

export function rad(deg: number): number {
  return (deg * Math.PI) / 180;
}

export function saturate(v: number): number {
  return clamp(v, 0, 1);
}

/** Rotate local (fwd, left) into world given yaw (0 faces +X, CCW). */
export function bodyToWorld(
  x: number,
  y: number,
  yaw: number,
  fwd: number,
  left: number,
): [number, number] {
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  return [x + c * fwd - s * left, y + s * fwd + c * left];
}

export function worldToBody(
  x: number,
  y: number,
  yaw: number,
  wx: number,
  wy: number,
): [number, number] {
  const dx = wx - x;
  const dy = wy - y;
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  const fwd = c * dx + s * dy;
  const left = -s * dx + c * dy;
  return [fwd, left];
}

export function seededRng(seed: number): () => number {
  let a = seed | 0;
  return () => {
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export class RingBuffer<T> {
  private buf: T[] = [];
  constructor(private readonly cap: number) {}
  push(item: T): void {
    this.buf.push(item);
    if (this.buf.length > this.cap) this.buf.shift();
  }
  toArray(): T[] {
    return this.buf.slice();
  }
  clear(): void {
    this.buf.length = 0;
  }
  get length(): number {
    return this.buf.length;
  }
}
