import { useEffect, useRef } from "react";
import { blit, drawDetections, drawMapOverlay, screenToWorld } from "@/lib/vista/render";
import { getRuntime } from "@/lib/vista/runtime";
import { useVista } from "@/lib/vista/store";
import { cn } from "@/lib/utils";

let costScratch: HTMLCanvasElement | null = null;

function fitCanvas(canvas: HTMLCanvasElement) {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const r = canvas.getBoundingClientRect();
  const w = Math.max(1, Math.round(r.width * dpr));
  const h = Math.max(1, Math.round(r.height * dpr));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
}

export function CameraView({ className }: { className?: string }) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    const canvas = ref.current;
    if (!canvas) return;
    let id = 0;
    const loop = () => {
      const snap = getRuntime().lastSnap;
      const ctx = canvas.getContext("2d");
      if (ctx && snap) {
        fitCanvas(canvas);
        blit(ctx, snap.rgb, snap.camW, snap.camH);
        drawDetections(ctx, snap.detections, snap.camW, snap.camH);
      }
      id = requestAnimationFrame(loop);
    };
    id = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(id);
  }, []);
  return <canvas ref={ref} className={cn("h-full w-full", className)} aria-label="Live camera" />;
}

export function DepthView({ className }: { className?: string }) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    const canvas = ref.current;
    if (!canvas) return;
    let id = 0;
    const loop = () => {
      const snap = getRuntime().lastSnap;
      const ctx = canvas.getContext("2d");
      if (ctx && snap) {
        fitCanvas(canvas);
        blit(ctx, snap.depthVis, snap.camW, snap.camH);
      }
      id = requestAnimationFrame(loop);
    };
    id = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(id);
  }, []);
  return <canvas ref={ref} className={cn("h-full w-full", className)} aria-label="Relative depth" />;
}

export function CostmapView({ className }: { className?: string }) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    const canvas = ref.current;
    if (!canvas) return;
    let id = 0;
    const loop = () => {
      const snap = getRuntime().lastSnap;
      const ctx = canvas.getContext("2d");
      if (ctx && snap) {
        fitCanvas(canvas);
        blit(ctx, snap.costRgb, snap.costSize, snap.costSize);
      }
      id = requestAnimationFrame(loop);
    };
    id = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(id);
  }, []);
  return <canvas ref={ref} className={cn("h-full w-full", className)} aria-label="Local costmap" />;
}

export function MapView({ className }: { className?: string }) {
  const ref = useRef<HTMLCanvasElement>(null);
  const overlay = useVista((s) => s.costOverlay);
  const dispatch = useVista((s) => s.dispatch);

  useEffect(() => {
    const canvas = ref.current;
    if (!canvas) return;
    let id = 0;
    const loop = () => {
      const rt = getRuntime();
      const snap = rt.lastSnap;
      const ctx = canvas.getContext("2d");
      if (ctx && snap) {
        fitCanvas(canvas);
        blit(ctx, snap.worldRgb, snap.mapW, snap.mapH);
        if (overlay) {
          ctx.globalAlpha = 0.42;
          const x0 =
            ((snap.costOriginX - snap.worldMinX) / (snap.worldMaxX - snap.worldMinX)) * canvas.width;
          const y1 =
            (1 - (snap.costOriginY - snap.worldMinY) / (snap.worldMaxY - snap.worldMinY)) * canvas.height;
          const spanX = (snap.costSize * snap.costRes) / (snap.worldMaxX - snap.worldMinX);
          const spanY = (snap.costSize * snap.costRes) / (snap.worldMaxY - snap.worldMinY);
          const w = spanX * canvas.width;
          const h = spanY * canvas.height;
          if (!costScratch) costScratch = document.createElement("canvas");
          costScratch.width = snap.costSize;
          costScratch.height = snap.costSize;
          const tctx = costScratch.getContext("2d");
          if (tctx) {
            const img = tctx.createImageData(snap.costSize, snap.costSize);
            img.data.set(snap.costRgb);
            tctx.putImageData(img, 0, 0);
            ctx.drawImage(costScratch, x0, y1 - h, w, h);
          }
          ctx.globalAlpha = 1;
        }
        drawMapOverlay(
          ctx,
          rt.world,
          rt.vehicle,
          snap.globalPath,
          snap.localTraj,
          snap.goal,
          snap.tracks,
          canvas.width,
          canvas.height,
        );
      }
      id = requestAnimationFrame(loop);
    };
    id = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(id);
  }, [overlay, dispatch]);

  return (
    <canvas
      ref={ref}
      className={cn("h-full w-full cursor-crosshair", className)}
      aria-label="Local map. Click to set goal."
      onClick={(e) => {
        const canvas = ref.current;
        if (!canvas) return;
        const rt = getRuntime();
        const p = screenToWorld(rt.world, canvas, e.clientX, e.clientY);
        dispatch({ type: "set-goal", x: p.x, y: p.y });
      }}
    />
  );
}
