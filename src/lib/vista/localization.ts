import { vistaConfig } from "./config";
import { clamp, wrapAngle } from "./math";
import type { LocHealth, LocStatus, Pose } from "./types";
import type { Vehicle } from "./vehicle";

/**
 * Visual-inertial pose estimator with the ORB-SLAM3 *interface*:
 * pose, tracking state, feature count, confidence.
 *
 * This SIL node is not ORB-SLAM3. It fuses noisy IMU integration with
 * visual corrections from simulated feature tracks. Physical deployments
 * swap this module for ORB-SLAM3 without changing topics.
 */
export class VistaVio {
  pose: Pose = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0, timestamp: 0 };
  status: LocStatus = {
    health: "GOOD",
    tracking: "OK",
    confidence: 1,
    featureCount: 90,
    updateHz: 20,
    imuConsistent: true,
    jumpDetected: false,
  };
  private lastGt: Pose | null = null;
  private lostT = 0;
  private lastUpdate = 0;

  reset(): void {
    this.pose = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0, timestamp: 0 };
    this.lastGt = null;
    this.lostT = 0;
    this.status.health = "GOOD";
    this.status.tracking = "OK";
    this.status.confidence = 1;
  }

  step(v: Vehicle, t: number, dt: number, featureHint: number, texture: number): Pose {
    const cfg = vistaConfig.slam;
    const features = Math.round(
      clamp(featureHint * 0.55 + texture * 80, 8, 140) + (Math.random() - 0.5) * 6,
    );
    const imuOk = Math.abs(v.v) < vistaConfig.vehicle.maxSpeed + 0.4;

    if (!this.lastGt) {
      this.pose = { ...v.gt, timestamp: t };
      this.lastGt = { ...v.gt };
      this.lastUpdate = t;
      this.status.featureCount = features;
      return this.pose;
    }

    // IMU prediction (noisy).
    const ax = (v.gt.x - this.lastGt.x) / Math.max(dt, 1e-3);
    const ay = (v.gt.y - this.lastGt.y) / Math.max(dt, 1e-3);
    const n = cfg.processNoise;
    this.pose.x += ax * dt + (Math.random() - 0.5) * n;
    this.pose.y += ay * dt + (Math.random() - 0.5) * n;
    this.pose.yaw = wrapAngle(this.pose.yaw + wrapAngle(v.gt.yaw - this.lastGt.yaw) + (Math.random() - 0.5) * n * 0.4);

    // Visual correction strength from feature count / texture.
    const vis = saturate((features - cfg.minFeatures) / (cfg.goodFeatures - cfg.minFeatures));
    this.pose.x = this.pose.x * (1 - vis * 0.65) + v.gt.x * vis * 0.65;
    this.pose.y = this.pose.y * (1 - vis * 0.65) + v.gt.y * vis * 0.65;
    this.pose.yaw = wrapAngle(this.pose.yaw * (1 - vis * 0.7) + v.gt.yaw * vis * 0.7);
    this.pose.z = v.gt.z;
    this.pose.roll = v.gt.roll;
    this.pose.pitch = v.gt.pitch;
    this.pose.timestamp = t;

    const jump = Math.hypot(this.pose.x - v.gt.x, this.pose.y - v.gt.y) > cfg.jumpThresh && vis < 0.2;
    const hz = dt > 0 ? 1 / dt : 0;
    this.status.updateHz = hz;
    this.status.featureCount = features;
    this.status.imuConsistent = imuOk;
    this.status.jumpDetected = jump;
    this.status.confidence = clamp(vis * 0.75 + (imuOk ? 0.2 : 0) + (jump ? 0 : 0.1), 0, 1);

    if (features < cfg.minFeatures * 0.55 || jump) {
      this.lostT += dt;
      this.status.tracking = this.lostT > cfg.lostTimeout ? "LOST" : "RECENTLY_LOST";
    } else {
      this.lostT = Math.max(0, this.lostT - dt * 0.6);
      this.status.tracking = "OK";
    }

    this.status.health = classifyHealth(this.status);
    this.lastGt = { ...v.gt };
    this.lastUpdate = t;
    return this.pose;
  }
}

function saturate(v: number): number {
  return v < 0 ? 0 : v > 1 ? 1 : v;
}

export function classifyHealth(s: LocStatus): LocHealth {
  if (s.tracking === "LOST" || s.confidence < vistaConfig.safety.locStopThreshold) return "LOST";
  if (s.tracking === "RECENTLY_LOST" || s.jumpDetected) return "RECOVERY";
  if (s.confidence < vistaConfig.safety.confidenceThreshold * 0.85) return "DEGRADED";
  if (s.confidence < vistaConfig.safety.confidenceThreshold || s.featureCount < vistaConfig.slam.minFeatures) {
    return "LOW_CONFIDENCE";
  }
  return "GOOD";
}
