export type Cell = { c: number; r: number };

const DIRS: [number, number, number][] = [
  [1, 0, 1],
  [-1, 0, 1],
  [0, 1, 1],
  [0, -1, 1],
  [1, 1, Math.SQRT2],
  [1, -1, Math.SQRT2],
  [-1, 1, Math.SQRT2],
  [-1, -1, Math.SQRT2],
];

function idx(c: number, r: number, w: number): number {
  return r * w + c;
}

class MinHeap {
  data: { k: number; i: number }[] = [];
  push(k: number, i: number): void {
    const d = this.data;
    d.push({ k, i });
    let n = d.length - 1;
    while (n > 0) {
      const p = (n - 1) >> 1;
      if (d[p]!.k <= d[n]!.k) break;
      const tmp = d[p]!;
      d[p] = d[n]!;
      d[n] = tmp;
      n = p;
    }
  }
  pop(): { k: number; i: number } | undefined {
    const d = this.data;
    if (d.length === 0) return undefined;
    const top = d[0]!;
    const last = d.pop()!;
    if (d.length) {
      d[0] = last;
      let n = 0;
      for (;;) {
        const l = n * 2 + 1;
        const r = l + 1;
        let s = n;
        if (l < d.length && d[l]!.k < d[s]!.k) s = l;
        if (r < d.length && d[r]!.k < d[s]!.k) s = r;
        if (s === n) break;
        const tmp = d[n]!;
        d[n] = d[s]!;
        d[s] = tmp;
        n = s;
      }
    }
    return top;
  }
}

const BLOCKED = 250;

/**
 * 8-connected A* on a flattened cost grid. `costs[i] >= 250` is non-traversable.
 */
export function astar(
  costs: Uint8Array | number[],
  w: number,
  h: number,
  start: Cell,
  goal: Cell,
  heuristicWeight = 1.05,
): Cell[] {
  return search(costs, w, h, start, goal, heuristicWeight);
}

/**
 * Dijkstra is A* with a zero heuristic — optimal on non-negative costs,
 * slower. Exposed as a configurable alternative.
 */
export function dijkstra(
  costs: Uint8Array | number[],
  w: number,
  h: number,
  start: Cell,
  goal: Cell,
): Cell[] {
  return search(costs, w, h, start, goal, 0);
}

function search(
  costs: Uint8Array | number[],
  w: number,
  h: number,
  start: Cell,
  goal: Cell,
  hw: number,
): Cell[] {
  const n = w * h;
  if (
    start.c < 0 ||
    start.r < 0 ||
    start.c >= w ||
    start.r >= h ||
    goal.c < 0 ||
    goal.r < 0 ||
    goal.c >= w ||
    goal.r >= h
  ) {
    return [];
  }
  const s = idx(start.c, start.r, w);
  const g = idx(goal.c, goal.r, w);
  if (costs[s]! >= BLOCKED || costs[g]! >= BLOCKED) return [];

  const gScore = new Float32Array(n);
  gScore.fill(1e15);
  gScore[s] = 0;
  const parent = new Int32Array(n);
  parent.fill(-1);
  const closed = new Uint8Array(n);
  const heap = new MinHeap();
  const h0 = hw * Math.hypot(start.c - goal.c, start.r - goal.r);
  heap.push(h0, s);

  while (heap.data.length) {
    const cur = heap.pop()!;
    if (closed[cur.i]) continue;
    closed[cur.i] = 1;
    if (cur.i === g) break;
    const c = cur.i % w;
    const r = (cur.i / w) | 0;
    for (const [dc, dr, step] of DIRS) {
      const nc = c + dc;
      const nr = r + dr;
      if (nc < 0 || nr < 0 || nc >= w || nr >= h) continue;
      const ni = idx(nc, nr, w);
      if (closed[ni]) continue;
      const cellCost = costs[ni]!;
      if (cellCost >= BLOCKED) continue;
      const stepCost = step * (1 + cellCost / 32);
      const ng = gScore[cur.i]! + stepCost;
      if (ng < gScore[ni]!) {
        gScore[ni] = ng;
        parent[ni] = cur.i;
        const hCost = hw * Math.hypot(nc - goal.c, nr - goal.r);
        heap.push(ng + hCost, ni);
      }
    }
  }

  if (parent[g] < 0 && s !== g) return [];
  const path: Cell[] = [];
  let i = g;
  while (i >= 0) {
    path.push({ c: i % w, r: (i / w) | 0 });
    if (i === s) break;
    i = parent[i]!;
  }
  path.reverse();
  return path;
}

export function simplifyPath(path: Cell[], costs: Uint8Array | number[], w: number): Cell[] {
  if (path.length < 3) return path;
  const out: Cell[] = [path[0]!];
  let keep = 0;
  for (let i = 1; i < path.length - 1; i++) {
    const a = out[keep]!;
    const b = path[i + 1]!;
    if (!lineFree(a, b, costs, w)) {
      out.push(path[i]!);
      keep++;
    }
  }
  out.push(path[path.length - 1]!);
  return out;
}

function lineFree(a: Cell, b: Cell, costs: Uint8Array | number[], w: number): boolean {
  const n = Math.max(Math.abs(b.c - a.c), Math.abs(b.r - a.r));
  for (let i = 0; i <= n; i++) {
    const c = (a.c + ((b.c - a.c) * i) / n) | 0;
    const r = (a.r + ((b.r - a.r) * i) / n) | 0;
    if (costs[r * w + c]! >= BLOCKED) return false;
  }
  return true;
}
