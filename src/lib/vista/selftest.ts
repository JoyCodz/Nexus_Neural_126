import { bicycleStep, footprintCorners } from "./algo/ackermann";
import { astar, dijkstra } from "./algo/pathfind";
import { ByteTracker } from "./algo/bytetrack";
import { PlaneKalman } from "./algo/kalman";
import { purePursuit } from "./algo/purePursuit";
import { fitPlaneRansac } from "./algo/ransac";
import { supervise } from "./safety";
import type { LocStatus, VehicleCmd } from "./types";

export type TestResult = { name: string; pass: boolean; detail: string };

export function runSelfTests(): { passed: number; failed: number; results: TestResult[] } {
  const results: TestResult[] = [];
  const check = (name: string, pass: boolean, detail: string) => results.push({ name, pass, detail });

  // RANSAC recovers a tilted plane.
  {
    const pts = [];
    for (let i = 0; i < 80; i++) {
      const x = (i % 10) * 0.2;
      const y = Math.floor(i / 10) * 0.2;
      const z = 0.12 * x + 0.02 * y + (i % 7 === 0 ? 0.4 : 0);
      pts.push({ x, y, z });
    }
    const p = fitPlaneRansac(pts, 60, 0.08, () => Math.random());
    check("RANSAC ground plane", !!p && p.inliers > 40 && p.slopeDeg > 3 && p.slopeDeg < 20, p ? `inliers=${p.inliers} slope=${p.slopeDeg.toFixed(1)}` : "null");
  }

  // A* goes around a wall.
  {
    const w = 20;
    const h = 16;
    const costs = new Uint8Array(w * h);
    costs.fill(4);
    for (let r = 0; r < 12; r++) costs[r * w + 8] = 255;
    const path = astar(costs, w, h, { c: 2, r: 2 }, { c: 16, r: 2 });
    const crosses = path.some((p) => p.c === 8 && p.r < 12);
    check("A* avoids lethal cells", path.length > 5 && !crosses, `len=${path.length}`);
  }

  {
    const w = 12;
    const h = 8;
    const costs = new Uint8Array(w * h);
    costs.fill(4);
    const a = astar(costs, w, h, { c: 1, r: 1 }, { c: 10, r: 6 });
    const d = dijkstra(costs, w, h, { c: 1, r: 1 }, { c: 10, r: 6 });
    check("Dijkstra alternative", d.length > 2 && a.length > 2, `astar=${a.length} dijkstra=${d.length}`);
  }

  // Ackermann: positive steer at yaw=0 increases Y.
  {
    let s = { x: 0, y: 0, yaw: 0, v: 1 };
    for (let i = 0; i < 20; i++) s = bicycleStep(s, 0.4, 0, 1.05, 0.05, 0.5, 2);
    check("Ackermann left steer increases Y", s.y > 0.1 && s.yaw > 0, `y=${s.y.toFixed(2)} yaw=${s.yaw.toFixed(2)}`);
  }

  {
    const fp = footprintCorners(0, 0, 0, 1.6, 1.1);
    check("Footprint has 4+ corners", fp.length >= 4, `${fp.length} pts`);
  }

  // Pure Pursuit: target to the left → positive steer (CCW).
  {
    const r = purePursuit(0, 0, 0, [{ x: 2, y: 0 }, { x: 3, y: 1.2 }], 1.05, 1.4, 0.5);
    check("Pure Pursuit left target → +steer", r.steering > 0, `steer=${r.steering.toFixed(3)}`);
  }

  {
    const kf = new PlaneKalman();
    kf.reset(0, 0);
    for (let i = 0; i < 8; i++) {
      kf.predict(0.1);
      kf.update(i * 0.2, 0);
    }
    check("Kalman estimates +vx", kf.vx > 0.3, `vx=${kf.vx.toFixed(2)}`);
  }

  {
    const tr = new ByteTracker();
    const t0 = tr.step(
      [{ id: 1, classId: "person", x: 10, y: 10, w: 8, h: 16, confidence: 0.9, timestamp: 0, range: 4, bearing: 0, worldX: 1, worldY: 1 }],
      0,
      0.1,
    );
    const t1 = tr.step(
      [{ id: 2, classId: "person", x: 12, y: 10, w: 8, h: 16, confidence: 0.88, timestamp: 0.1, range: 4, bearing: 0, worldX: 1.15, worldY: 1.02 }],
      0.1,
      0.1,
    );
    const t2 = tr.step(
      [{ id: 3, classId: "person", x: 14, y: 10, w: 8, h: 16, confidence: 0.87, timestamp: 0.2, range: 4, bearing: 0, worldX: 1.3, worldY: 1.04 }],
      0.2,
      0.1,
    );
    const idStable = t2[0] && t1[0] && t2[0].id === t1[0].id;
    check("ByteTrack persistent ID", !!idStable && (t0.length === 0 || t0[0]!.id === t2[0]!.id || t2[0]!.hits >= 2), `id=${t2[0]?.id} hits=${t2[0]?.hits}`);
  }

  {
    const loc: LocStatus = {
      health: "GOOD",
      tracking: "OK",
      confidence: 0.9,
      featureCount: 80,
      updateHz: 20,
      imuConsistent: true,
      jumpDetected: false,
    };
    const req: VehicleCmd = { speed: 1, steering: 0, stop: false, estop: false, timestamp: 1 };
    const stop = supervise(
      {
        loc,
        nearest: 0.4,
        terrainConf: 0.8,
        perceptionOk: true,
        ultraFront: 0.4,
        estop: false,
        watchdogOk: true,
        cmdAge: 0.01,
        plannerOk: true,
        requested: req,
      },
      "NORMAL",
      1,
    );
    check("Safety stops inside emergency distance", stop.allowed.speed === 0 && stop.state === "STOPPING", stop.reason);

    const estop = supervise(
      { ...stop, loc, nearest: 5, terrainConf: 0.9, perceptionOk: true, ultraFront: 5, estop: true, watchdogOk: true, cmdAge: 0, plannerOk: true, requested: req } as never,
      "NORMAL",
      1,
    );
    // rebuild properly
    const estop2 = supervise(
      {
        loc,
        nearest: 8,
        terrainConf: 0.9,
        perceptionOk: true,
        ultraFront: 8,
        estop: true,
        watchdogOk: true,
        cmdAge: 0,
        plannerOk: true,
        requested: req,
      },
      "NORMAL",
      1,
    );
    check("E-stop has authority over planner", estop2.state === "EMERGENCY_STOP" && estop2.allowed.speed === 0, estop2.reason);
    void estop;
  }

  {
    const locLost: LocStatus = {
      health: "LOST",
      tracking: "LOST",
      confidence: 0.05,
      featureCount: 4,
      updateHz: 2,
      imuConsistent: false,
      jumpDetected: true,
    };
    const r = supervise(
      {
        loc: locLost,
        nearest: 6,
        terrainConf: 0.4,
        perceptionOk: false,
        ultraFront: 6,
        estop: false,
        watchdogOk: true,
        cmdAge: 0.01,
        plannerOk: true,
        requested: { speed: 1, steering: 0.2, stop: false, estop: false, timestamp: 2 },
      },
      "NORMAL",
      2,
    );
    check("Lost localization rejects motion", r.allowed.speed === 0, r.reason);
  }

  const passed = results.filter((r) => r.pass).length;
  const failed = results.length - passed;
  return { passed, failed, results };
}
