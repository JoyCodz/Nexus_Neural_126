import { useEffect, useState } from "react";
import { ArrowRight, Shield, Waypoints } from "lucide-react";
import { Button } from "@/components/ui/button";
import { runSelfTests, type TestResult } from "@/lib/vista/selftest";
import { useVista } from "@/lib/vista/store";

const PIPE = [
  "RGB + IMU",
  "MiDaS relative depth",
  "RANSAC terrain",
  "YOLO-n + ByteTrack",
  "VIO pose",
  "Dynamic costmap",
  "A* + Ackermann",
  "Pure Pursuit",
  "Safety supervisor",
  "ESP32",
];

export function Briefing({ onEnter }: { onEnter: () => void }) {
  const [tests, setTests] = useState<{ passed: number; failed: number; results: TestResult[] } | null>(null);

  useEffect(() => {
    setTests(runSelfTests());
  }, []);

  return (
    <div className="absolute inset-0 z-30 flex items-center justify-center overflow-y-auto bg-background/95 p-4 sm:p-8">
      <div className="mx-auto w-full max-w-3xl py-6">
        <p className="font-mono text-xs tracking-[0.28em] text-muted uppercase">Field robotics · local frame</p>
        <h1 className="mt-3 font-sans text-4xl font-medium tracking-tight text-foreground sm:text-5xl">
          UGV-VISTA
        </h1>
        <p className="mt-2 max-w-xl text-base text-muted">
          Vision Intelligence for Safe Terrain Autonomy. GPS-independent A→B navigation in a local SLAM
          frame — perception proposes, geometry reasons, the safety supervisor decides.
        </p>

        <div className="mt-8 grid gap-3 sm:grid-cols-3">
          <Note title="Not GPS" body="Origin is X=0, Y=0, Yaw=0 at startup. Goals are metres in that frame, never lat/lon." />
          <Note
            title="Relative depth"
            body="MiDaS-Small contract: relative disparity with optional ultrasonic scale. Not native metric depth."
          />
          <Note
            title="Fail closed"
            body="Lost localization, stale commands, or E-stop reject motion. No silent failures."
          />
        </div>

        <div className="mt-8 panel p-4 sm:p-5">
          <p className="text-xs font-medium tracking-wide text-muted uppercase">Pipeline</p>
          <div className="mt-3 flex flex-wrap gap-1.5">
            {PIPE.map((p) => (
              <span
                key={p}
                className="rounded-sm border border-border bg-card-2 px-2 py-1 font-mono text-[11px] text-foreground"
              >
                {p}
              </span>
            ))}
          </div>
        </div>

        <div className="mt-4 panel p-4 sm:p-5">
          <div className="flex items-center justify-between gap-3">
            <p className="text-xs font-medium tracking-wide text-muted uppercase">Self-tests</p>
            {tests && (
              <span className={tests.failed === 0 ? "font-mono text-xs text-go" : "font-mono text-xs text-stop"}>
                {tests.passed}/{tests.passed + tests.failed} passed
              </span>
            )}
          </div>
          <ul className="mt-3 grid gap-1 sm:grid-cols-2">
            {(tests?.results ?? []).map((r) => (
              <li key={r.name} className="flex gap-2 font-mono text-[11px]">
                <span className={r.pass ? "text-go" : "text-stop"}>{r.pass ? "PASS" : "FAIL"}</span>
                <span className="text-foreground">{r.name}</span>
              </li>
            ))}
          </ul>
        </div>

        <div className="mt-8 flex flex-col gap-3 sm:flex-row sm:items-center">
          <Button size="lg" onClick={onEnter} className="min-h-12">
            Initialize SLAM
            <ArrowRight className="size-4" />
          </Button>
          <Button
            size="lg"
            variant="outline"
            className="min-h-12"
            onClick={() => {
              onEnter();
              useVista.getState().dispatch({ type: "set-goal", x: 15, y: 8 });
              useVista.getState().dispatch({ type: "start" });
            }}
          >
            <Waypoints className="size-4" />
            Run A→B demonstration
          </Button>
        </div>
        <p className="mt-4 flex items-start gap-2 text-xs text-faint">
          <Shield className="mt-0.5 size-3.5 shrink-0" />
          Software-in-the-loop console. Geometric modules run as specified. Neural nodes (MiDaS-Small,
          YOLO-n, ORB-SLAM3) use the same interfaces with documented SIL substitutes — they are not the
          trained weights.
        </p>
      </div>
    </div>
  );
}

function Note({ title, body }: { title: string; body: string }) {
  return (
    <div className="panel p-4">
      <p className="text-sm font-medium text-foreground">{title}</p>
      <p className="mt-1 text-xs leading-relaxed text-muted">{body}</p>
    </div>
  );
}
