import { useEffect, useMemo, useState, type ReactNode } from "react";
import {
  Activity,
  BookOpen,
  Gauge,
  OctagonX,
  Pause,
  Play,
  RotateCcw,
  Settings2,
  Square,
  Waypoints,
} from "lucide-react";
import { Briefing } from "@/components/vista/briefing";
import { CameraView, CostmapView, DepthView, MapView } from "@/components/vista/canvases";
import { Button } from "@/components/ui/button";
import { vistaConfig } from "@/lib/vista/config";
import { getRuntime } from "@/lib/vista/runtime";
import { runSelfTests } from "@/lib/vista/selftest";
import { useVista } from "@/lib/vista/store";
import { cn } from "@/lib/utils";
import type { MissionMode, SafetyState, LogEvent } from "@/lib/vista/types";

const EMPTY_LOGS: LogEvent[] = [];

export function OperatorConsole() {
  const snap = useVista((s) => s.snap);
  const briefing = useVista((s) => s.briefing);
  const panel = useVista((s) => s.panel);
  const dispatch = useVista((s) => s.dispatch);
  const setBriefing = useVista((s) => s.setBriefing);
  const setPanel = useVista((s) => s.setPanel);
  const startLoop = useVista((s) => s.startLoop);
  const [gx, setGx] = useState("15");
  const [gy, setGy] = useState("8");

  useEffect(() => {
    startLoop();
  }, [startLoop]);

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) return;
      if (e.code === "KeyG" && !briefing) dispatch({ type: "start" });
      if (e.code === "KeyP") dispatch({ type: "pause" });
      if (e.code === "Escape") dispatch({ type: "stop" });
      if (e.code === "KeyX") dispatch({ type: "estop" });
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [briefing, dispatch]);

  const mission = snap?.mission ?? "IDLE";

  return (
    <div className="relative flex min-h-dvh flex-col bg-background text-foreground">
      {briefing && <Briefing onEnter={() => setBriefing(false)} />}

      <header className="flex flex-wrap items-center gap-3 border-b border-border px-3 py-2.5 sm:px-4">
        <div className="min-w-0">
          <p className="font-mono text-[10px] tracking-[0.22em] text-muted uppercase">UGV-VISTA</p>
          <p className="truncate text-sm font-medium">Operator console</p>
        </div>
        <MissionBadge mode={mission} safety={snap?.safety ?? "NORMAL"} />
        <div className="ml-auto flex flex-wrap items-center gap-2">
          <NavBtn active={panel === "ops"} onClick={() => setPanel("ops")} icon={<Gauge className="size-3.5" />} label="Ops" />
          <NavBtn active={panel === "pipeline"} onClick={() => setPanel("pipeline")} icon={<Activity className="size-3.5" />} label="Pipeline" />
          <NavBtn active={panel === "config"} onClick={() => setPanel("config")} icon={<Settings2 className="size-3.5" />} label="Config" />
          <NavBtn active={panel === "tests"} onClick={() => setPanel("tests")} icon={<Waypoints className="size-3.5" />} label="Tests" />
          <NavBtn active={panel === "docs"} onClick={() => setPanel("docs")} icon={<BookOpen className="size-3.5" />} label="Docs" />
        </div>
      </header>

      <div className="grid flex-1 grid-cols-1 gap-3 p-3 lg:grid-cols-12 lg:gap-3 lg:p-4">
        <section className="flex min-h-0 flex-col gap-3 lg:col-span-8">
          <div className="grid gap-3 md:grid-cols-2">
            <Feed title="Live camera" hint="YOLO-n interface overlay" live>
              <div className="scanline relative aspect-video overflow-hidden rounded-md bg-card-2">
                <CameraView />
              </div>
            </Feed>
            <Feed title="Local map" hint="Click to set Point B">
              <div className="relative aspect-video overflow-hidden rounded-md bg-card-2">
                <MapView />
              </div>
            </Feed>
          </div>
          <div className="grid grid-cols-2 gap-3">
            <Feed title="Relative depth" hint="MiDaS-Small contract">
              <div className="aspect-[16/7] overflow-hidden rounded-md bg-card-2">
                <DepthView />
              </div>
            </Feed>
            <Feed title="Local costmap" hint="Rolling · footprint inflation">
              <div className="aspect-[16/7] overflow-hidden rounded-md bg-card-2">
                <CostmapView />
              </div>
            </Feed>
          </div>
          {panel === "ops" && <LogRail />}
          {panel === "pipeline" && <PipelinePanel />}
          {panel === "config" && <ConfigPanel />}
          {panel === "tests" && <TestsPanel />}
          {panel === "docs" && <DocsPanel />}
        </section>

        <aside className="flex flex-col gap-3 lg:col-span-4">
          <div className="panel p-4">
            <p className="text-xs font-medium tracking-wide text-muted uppercase">Pose · local frame</p>
            <div className="mt-3 grid grid-cols-3 gap-2 font-mono text-sm tabular">
              <KV k="X" v={fmt(snap?.pose.x)} u="m" />
              <KV k="Y" v={fmt(snap?.pose.y)} u="m" />
              <KV k="Yaw" v={fmt(((snap?.pose.yaw ?? 0) * 180) / Math.PI, 1)} u="deg" />
              <KV k="Z" v={fmt(snap?.pose.z, 2)} u="m" />
              <KV k="Speed" v={fmt(snap?.speed, 2)} u="m/s" />
              <KV k="Steer" v={fmt(((snap?.steering ?? 0) * 180) / Math.PI, 1)} u="deg" />
            </div>
            <div className="mt-3 grid grid-cols-2 gap-2 font-mono text-xs tabular text-muted">
              <span>Dist to B {fmt(snap?.distanceToGoal, 2)} m</span>
              <span>Loc {snap?.loc.health ?? "—"}</span>
              <span>Features {snap?.loc.featureCount ?? 0}</span>
              <span>Conf {fmt((snap?.loc.confidence ?? 0) * 100, 0)}%</span>
            </div>
          </div>

          <div className="panel p-4">
            <p className="text-xs font-medium tracking-wide text-muted uppercase">Safety supervisor</p>
            <p className="mt-2 font-mono text-lg tabular">{snap?.safety ?? "NORMAL"}</p>
            <p className="mt-1 text-xs text-muted">
              Nearest {fmt(snap?.nearestObstacle, 2)} m · Ultra F/L/R {fmt(snap?.ultrasonic.front, 1)} /{" "}
              {fmt(snap?.ultrasonic.left, 1)} / {fmt(snap?.ultrasonic.right, 1)}
            </p>
            <p className="mt-2 text-xs text-muted">
              Terrain {snap?.terrainSummary.klass} · slope {fmt(snap?.terrainSummary.slope, 1)}° · drop risk{" "}
              {fmt((snap?.terrainSummary.dropRisk ?? 0) * 100, 0)}%
            </p>
            <p className="mt-2 text-xs text-faint">Independent of the planner. Rejects unsafe commands.</p>
          </div>

          <div className="panel p-4">
            <p className="text-xs font-medium tracking-wide text-muted uppercase">ESP32 interface</p>
            <div className="mt-3 grid grid-cols-2 gap-2 font-mono text-xs tabular">
              <span>PWM {snap?.esp32.motorPwm ?? 0}</span>
              <span>Servo {fmt(snap?.esp32.servoDeg, 0)}°</span>
              <span>HB {snap?.esp32.connected ? "ok" : "lost"}</span>
              <span>WD {snap?.esp32.watchdogOk ? "ok" : "fail"}</span>
            </div>
          </div>

          <div className="panel p-4">
            <p className="text-xs font-medium tracking-wide text-muted uppercase">Tracks</p>
            <ul className="mt-2 max-h-32 space-y-1 overflow-auto font-mono text-[11px]">
              {(snap?.tracks ?? []).length === 0 && <li className="text-faint">No tracks</li>}
              {(snap?.tracks ?? []).map((tr) => (
                <li key={tr.id} className="flex justify-between gap-2">
                  <span>
                    #{tr.id} {tr.classId} {tr.static ? "static" : "dyn"}
                  </span>
                  <span className="text-muted">
                    {tr.speed.toFixed(2)} m/s · ({tr.worldX.toFixed(1)}, {tr.worldY.toFixed(1)})
                  </span>
                </li>
              ))}
            </ul>
          </div>

          <div className="panel p-4">
            <p className="text-xs font-medium tracking-wide text-muted uppercase">Performance</p>
            <div className="mt-2 grid grid-cols-2 gap-x-3 gap-y-1 font-mono text-[11px] tabular text-muted">
              <span>FPS {fmt(snap?.fps, 0)}</span>
              <span>E2E {fmt(snap?.perf.e2eMs, 1)} ms</span>
              <span>Depth {fmt(snap?.perf.depthMs, 1)} ms</span>
              <span>Plan {fmt(snap?.perf.plannerMs, 1)} ms</span>
              <span>Cost {fmt(snap?.perf.costmapMs, 1)} ms</span>
              <span>SLAM {fmt(snap?.perf.slamHz, 0)} Hz</span>
            </div>
          </div>
        </aside>
      </div>

      <footer className="sticky bottom-0 z-20 border-t border-border bg-background/95 px-3 py-2.5 backdrop-blur-sm sm:px-4">
        <div className="flex flex-wrap items-center gap-2">
          <Button
            variant="go"
            className="min-h-11"
            onClick={() => {
              dispatch({ type: "set-goal", x: Number(gx) || 15, y: Number(gy) || 8 });
              dispatch({ type: "start" });
            }}
          >
            <Play className="size-4" />
            Start autonomy
          </Button>
          <Button variant="secondary" className="min-h-11" onClick={() => dispatch({ type: "pause" })}>
            <Pause className="size-4" />
            Pause
          </Button>
          <Button variant="outline" className="min-h-11" onClick={() => dispatch({ type: "stop" })}>
            <Square className="size-4" />
            Stop
          </Button>
          <Button variant="stop" className="min-h-11" onClick={() => dispatch({ type: "estop" })}>
            <OctagonX className="size-4" />
            E-stop
          </Button>
          <Button variant="ghost" className="min-h-11" onClick={() => dispatch({ type: "resume" })}>
            Resume
          </Button>
          <Button variant="ghost" className="min-h-11" onClick={() => dispatch({ type: "reset" })}>
            <RotateCcw className="size-4" />
            Reset
          </Button>
          <Button
            variant="ghost"
            className="min-h-11"
            onClick={() => {
              const rt = getRuntime();
              const blob = new Blob(
                [
                  JSON.stringify(
                    {
                      t: rt.time,
                      pose: rt.vehicle.gt,
                      goal: rt.goals.goal,
                      mission: rt.mission,
                      safety: rt.safety,
                      logs: rt.logs.toArray(),
                    },
                    null,
                    2,
                  ),
                ],
                { type: "application/json" },
              );
              const a = document.createElement("a");
              a.href = URL.createObjectURL(blob);
              a.download = `ugv-vista-${Date.now()}.json`;
              a.click();
            }}
          >
            Save log
          </Button>
          <div className="ml-auto flex flex-wrap items-center gap-2">
            <label className="flex items-center gap-1.5 font-mono text-[11px] text-muted">
              X
              <input
                value={gx}
                onChange={(e) => setGx(e.target.value)}
                className="h-10 w-16 rounded-sm border border-border bg-card-2 px-2 text-foreground"
                inputMode="decimal"
                aria-label="Goal X metres"
              />
            </label>
            <label className="flex items-center gap-1.5 font-mono text-[11px] text-muted">
              Y
              <input
                value={gy}
                onChange={(e) => setGy(e.target.value)}
                className="h-10 w-16 rounded-sm border border-border bg-card-2 px-2 text-foreground"
                inputMode="decimal"
                aria-label="Goal Y metres"
              />
            </label>
            <Button
              variant="outline"
              className="min-h-11"
              onClick={() => dispatch({ type: "set-goal", x: Number(gx) || 0, y: Number(gy) || 0 })}
            >
              Set goal
            </Button>
          </div>
        </div>
        <p className="mt-1.5 hidden text-[10px] text-faint sm:block">
          G start · P pause · Esc stop · X e-stop · WASD manual (A left / D right) · click map for Point B
        </p>
      </footer>
    </div>
  );
}

function fmt(n: number | undefined, d = 2): string {
  if (n === undefined || Number.isNaN(n)) return "—";
  return n.toFixed(d);
}

function MissionBadge({ mode, safety }: { mode: MissionMode; safety: SafetyState }) {
  const tone =
    mode === "EMERGENCY_STOP" || safety === "EMERGENCY_STOP"
      ? "bg-stop text-foreground"
      : mode === "COMPLETE" || mode === "AUTONOMOUS"
        ? "bg-go text-background"
        : mode === "DEGRADED" || mode === "RECOVERING" || safety === "DEGRADED"
          ? "bg-warn text-background"
          : "bg-card-2 text-foreground";
  return (
    <div className={cn("rounded-sm px-2.5 py-1 font-mono text-[11px] tracking-wide uppercase", tone)}>
      {mode.replaceAll("_", " ")}
    </div>
  );
}

function NavBtn({
  active,
  onClick,
  icon,
  label,
}: {
  active: boolean;
  onClick: () => void;
  icon: ReactNode;
  label: string;
}) {
  return (
    <button
      type="button"
      onClick={onClick}
      className={cn(
        "inline-flex min-h-10 items-center gap-1.5 rounded-sm px-2.5 text-xs",
        active ? "bg-card-2 text-foreground" : "text-muted hover:text-foreground",
      )}
    >
      {icon}
      {label}
    </button>
  );
}

function Feed({
  title,
  hint,
  live,
  children,
}: {
  title: string;
  hint: string;
  live?: boolean;
  children: React.ReactNode;
}) {
  return (
    <div className="panel p-2">
      <div className="mb-1.5 flex items-center justify-between px-1">
        <p className="text-xs font-medium text-foreground">{title}</p>
        <p className="flex items-center gap-1.5 font-mono text-[10px] text-faint">
          {live && <span className="inline-block size-1.5 rounded-full bg-go" />}
          {hint}
        </p>
      </div>
      {children}
    </div>
  );
}

function KV({ k, v, u }: { k: string; v: string; u: string }) {
  return (
    <div>
      <p className="text-[10px] text-faint">{k}</p>
      <p>
        {v}
        <span className="ml-0.5 text-[10px] text-muted">{u}</span>
      </p>
    </div>
  );
}

function LogRail() {
  const logs = useVista((s) => s.snap?.logs ?? EMPTY_LOGS);
  const tail = logs.slice(-12).reverse();
  return (
    <div className="panel max-h-48 overflow-auto p-3">
      <p className="text-xs font-medium tracking-wide text-muted uppercase">Mission log</p>
      <ul className="mt-2 space-y-1 font-mono text-[11px]">
        {tail.map((e, i) => (
          <li key={`${e.t}-${i}`} className="flex gap-2">
            <span className="text-faint">{e.t.toFixed(1)}</span>
            <span className={e.level === "error" ? "text-stop" : e.level === "warn" ? "text-warn" : "text-go"}>
              {e.source}
            </span>
            <span className="text-foreground">{e.msg}</span>
          </li>
        ))}
      </ul>
    </div>
  );
}

function PipelinePanel() {
  const snap = useVista((s) => s.snap);
  const rows = [
    ["Frame manager", "synced RGB + IMU", "ok"],
    ["MiDaS-Small interface", `scale ${fmt(1, 2)} · relative`, snap?.perceptionOk ? "ok" : "degraded"],
    ["RANSAC terrain", `${snap?.terrainSummary.klass} · ${fmt(snap?.terrainSummary.slope, 1)}°`, snap?.plane ? "ok" : "low"],
    ["YOLO-n interface", `${snap?.detections.length ?? 0} boxes`, "ok"],
    ["ByteTrack + Kalman", `${snap?.tracks.length ?? 0} tracks`, "ok"],
    ["VIO / ORB-SLAM3 iface", `${snap?.loc.tracking} · ${snap?.loc.featureCount} feat`, snap?.loc.health ?? "—"],
    ["Costmap", `${snap?.costSize}² @ ${snap?.costRes} m`, "ok"],
    ["Global planner", `${snap?.globalPath.length ?? 0} wpts`, snap?.globalPath.length ? "ok" : "idle"],
    ["Ackermann local", snap?.localTraj ? `cost ${fmt(snap.localTraj.cost, 2)}` : "none", snap?.localTraj ? "ok" : "hold"],
    ["Pure Pursuit", `δ ${fmt(((snap?.steering ?? 0) * 180) / Math.PI, 1)}°`, "ok"],
    ["Safety supervisor", snap?.safety ?? "—", snap?.safety === "NORMAL" ? "ok" : "active"],
    ["ESP32", `PWM ${snap?.esp32.motorPwm ?? 0}`, snap?.esp32.watchdogOk ? "ok" : "fail"],
  ];
  return (
    <div className="panel overflow-auto p-3">
      <p className="text-xs font-medium tracking-wide text-muted uppercase">Module health</p>
      <table className="mt-2 w-full text-left font-mono text-[11px]">
        <tbody>
          {rows.map(([n, d, s]) => (
            <tr key={n} className="border-b border-border/60">
              <td className="py-1.5 text-foreground">{n}</td>
              <td className="text-muted">{d}</td>
              <td className="text-right text-go">{s}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

function ConfigPanel() {
  const dispatch = useVista((s) => s.dispatch);
  const overlay = useVista((s) => s.costOverlay);
  const setOverlay = useVista((s) => s.setCostOverlay);
  const v = vistaConfig;
  return (
    <div className="panel p-4 text-xs leading-relaxed text-muted">
      <p className="font-medium text-foreground">Runtime configuration</p>
      <pre className="mt-3 overflow-auto rounded-md bg-card-2 p-3 font-mono text-[11px] text-foreground">
        {`vehicle:
  wheelbase: ${v.vehicle.wheelbase} m
  width: ${v.vehicle.width} m
  length: ${v.vehicle.length} m
  max_speed: ${v.vehicle.maxSpeed} m/s
  max_steering_angle: ${((v.vehicle.maxSteeringAngle * 180) / Math.PI).toFixed(1)} deg
planner:
  algorithm: A* (Dijkstra selectable)
  goal_tolerance: ${v.planner.goalTolerance} m
safety:
  emergency_stop_distance: ${v.safety.emergencyStopDistance} m
  command_timeout: ${v.safety.commandTimeout} s
  confidence_threshold: ${v.safety.confidenceThreshold}`}
      </pre>
      <div className="mt-3 flex flex-wrap gap-2">
        <Button size="sm" variant="outline" onClick={() => dispatch({ type: "algorithm", algo: "astar" })}>
          A*
        </Button>
        <Button size="sm" variant="outline" onClick={() => dispatch({ type: "algorithm", algo: "dijkstra" })}>
          Dijkstra
        </Button>
        <Button size="sm" variant="outline" onClick={() => dispatch({ type: "reset-map" })}>
          Reset local map
        </Button>
        <Button size="sm" variant="outline" onClick={() => setOverlay(!overlay)}>
          {overlay ? "Hide" : "Show"} cost overlay
        </Button>
      </div>
    </div>
  );
}

function TestsPanel() {
  const results = useMemo(() => runSelfTests(), []);
  return (
    <div className="panel p-4">
      <p className="text-xs font-medium tracking-wide text-muted uppercase">
        Unit checks · {results.passed}/{results.passed + results.failed} passed
      </p>
      <ul className="mt-3 space-y-1.5 font-mono text-[11px]">
        {results.results.map((r) => (
          <li key={r.name}>
            <span className={r.pass ? "text-go" : "text-stop"}>{r.pass ? "PASS" : "FAIL"}</span>{" "}
            <span className="text-foreground">{r.name}</span>
            <span className="text-faint"> — {r.detail}</span>
          </li>
        ))}
      </ul>
    </div>
  );
}

function DocsPanel() {
  return (
    <div className="panel space-y-3 p-4 text-sm leading-relaxed text-muted">
      <p className="font-medium text-foreground">Architecture</p>
      <p>
        Navigation is performed in a local SLAM coordinate frame. GPS-free does not mean globally
        geolocated. MiDaS provides monocular relative depth, with optional calibrated scale from ultrasonic
        range. ORB-SLAM3 is not guaranteed — this console exposes tracking confidence and recovery. RANSAC
        does not prove a drop-off; votes, discontinuity, and ultrasonics must agree. YOLO does not predict
        motion; ByteTrack + Kalman does. The system is not universally safe; it is instrumented so failures
        are visible.
      </p>
      <p className="font-medium text-foreground">SIL vs physical</p>
      <p>
        Same topic contract: /camera/image_raw, /imu/data, /perception/depth, /perception/terrain,
        /perception/objects, /perception/tracks, /localization/pose, /costmap/local, /planner/global_path,
        /planner/local_trajectory, /controller/cmd, /safety/state, /vehicle/cmd. Frames: map, base_link,
        camera_link, imu_link. Physical UGV swaps the VIO node for ORB-SLAM3, the detector for YOLO-n
        weights, and the ESP32 bridge for serial — planners and safety stay.
      </p>
    </div>
  );
}
