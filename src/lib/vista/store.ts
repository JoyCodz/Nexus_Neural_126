import { create } from "zustand";
import type { VistaSnapshot } from "./types";
import { getRuntime, type VistaCommand } from "./runtime";

type VistaStore = {
  snap: VistaSnapshot | null;
  briefing: boolean;
  panel: "ops" | "pipeline" | "config" | "tests" | "docs";
  costOverlay: boolean;
  ready: boolean;
  dispatch: (c: VistaCommand) => void;
  setBriefing: (v: boolean) => void;
  setPanel: (p: VistaStore["panel"]) => void;
  setCostOverlay: (v: boolean) => void;
  startLoop: () => void;
};

let loopStarted = false;
let acc = 0;
let last = 0;
let frames = 0;
let fpsT = 0;

export const useVista = create<VistaStore>((set, get) => ({
  snap: null,
  briefing: true,
  panel: "ops",
  costOverlay: true,
  ready: false,
  dispatch: (c) => getRuntime().dispatch(c),
  setBriefing: (v) => set({ briefing: v }),
  setPanel: (p) => set({ panel: p }),
  setCostOverlay: (v) => set({ costOverlay: v }),
  startLoop: () => {
    if (loopStarted || typeof window === "undefined") return;
    loopStarted = true;
    const rt = getRuntime();
    try {
      set({ ready: true, snap: rt.lastSnap ?? rt.step(0) });
    } catch (err) {
      console.error("[vista] initial step failed", err);
      set({ ready: true });
    }
    last = performance.now();
    let lastUi = 0;
    const tick = (now: number) => {
      const dt = Math.min(0.05, (now - last) / 1000);
      last = now;
      acc += dt;
      frames++;
      fpsT += dt;
      if (fpsT >= 0.5) {
        rt.fps = frames / fpsT;
        frames = 0;
        fpsT = 0;
      }
      const step = 1 / 20;
      while (acc >= step) {
        acc -= step;
        try {
          rt.step(step);
        } catch (err) {
          console.error("[vista] step failed", err);
          acc = 0;
          break;
        }
      }
      if (now - lastUi > 90) {
        lastUi = now;
        if (rt.lastSnap) set({ snap: rt.lastSnap });
      }
      requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
    wireQa(rt);
  },
}));

function wireQa(rt: ReturnType<typeof getRuntime>): void {
  const w = window as Window & {
    __controlsTest?: {
      getYaw: () => number;
      getSpeed: () => number;
      setKeys: (codes: string[]) => void;
      setSteer?: (v: number) => void;
    };
    __vistaTest?: {
      startAutonomy: () => void;
      setGoal: (x: number, y: number) => void;
      getPose: () => { x: number; y: number; yaw: number };
      getMission: () => string;
      getSafety: () => string;
      eStop: () => void;
    };
  };
  let keys = new Set<string>();
  const applyKeys = () => {
    let throttle = 0;
    let steer = 0;
    if (keys.has("KeyW") || keys.has("ArrowUp")) throttle += 1;
    if (keys.has("KeyS") || keys.has("ArrowDown")) throttle -= 1;
    if (keys.has("KeyA") || keys.has("ArrowLeft")) steer += 1;
    if (keys.has("KeyD") || keys.has("ArrowRight")) steer -= 1;
    rt.dispatch({ type: "mode", mode: "manual" });
    rt.dispatch({ type: "manual", throttle, steer });
  };
  w.__controlsTest = {
    getYaw: () => rt.vehicle.yaw,
    getSpeed: () => rt.vehicle.v,
    setKeys: (codes: string[]) => {
      keys = new Set(codes);
      applyKeys();
    },
    setSteer: (v: number) => {
      rt.dispatch({ type: "mode", mode: "manual" });
      rt.dispatch({ type: "manual", throttle: 1, steer: v });
    },
  };
  w.__vistaTest = {
    startAutonomy: () => rt.dispatch({ type: "start" }),
    setGoal: (x, y) => rt.dispatch({ type: "set-goal", x, y }),
    getPose: () => ({ x: rt.vehicle.x, y: rt.vehicle.y, yaw: rt.vehicle.yaw }),
    getMission: () => rt.mission,
    getSafety: () => rt.safety,
    eStop: () => rt.dispatch({ type: "estop" }),
  };
}
