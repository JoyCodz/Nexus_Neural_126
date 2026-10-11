# UGV-VISTA

**Vision Intelligence for Safe Terrain Autonomy**

[![Live Demo](https://img.shields.io/badge/Live%20Demo-ugv--vista.cappace.me-brightgreen?style=flat-square)](https://ugv-vista.cappace.me/)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg?style=flat-square)](LICENSE)

> 🌐 **Live Operator Console:** [https://ugv-vista.cappace.me/](https://ugv-vista.cappace.me/)

GPS-independent A→B navigation for an Ackermann unmanned ground vehicle.
This repository ships a **software-in-the-loop (SIL) operator console**: the geometric stack
(RANSAC, traversability, A\*/Dijkstra, Ackermann local planner, Pure Pursuit, ByteTrack/Kalman,
dynamic costmap, safety supervisor, ESP32 watchdog) runs as specified.
Neural nodes are exposed through the same contracts as MiDaS-Small, YOLO-n, and ORB-SLAM3,
with documented SIL substitutes — they are not trained weights.

Navigation is performed in a **local SLAM coordinate frame**.
Origin at startup is `X=0, Y=0, Yaw=0`. Goals are metres in that frame, never latitude/longitude.

---

## Operator Console

Access the live console at **[ugv-vista.cappace.me](https://ugv-vista.cappace.me/)** or run it locally.
Initialise SLAM or run the A→B demonstration:

- Click the local map to set Point B, or enter X/Y coordinates directly.
- Start autonomy — the safety supervisor can reject any command.
- E-stop is independent of the planner and always available.
- WASD is manual teleop (A turns left, D turns right), still gated by the safety supervisor.

---

## Tech Stack

| Layer | Technology |
|---|---|
| UI framework | React 19 + TanStack Start (SSR) |
| Routing | TanStack Router |
| Server state | TanStack Query |
| Language | TypeScript 5 |
| Styling | Tailwind CSS v4 |
| Build | Vite 8 + Nitro (Vercel preset) |
| DB (optional) | PGLite / PostgreSQL via Kysely |

---

## Getting Started

**Prerequisites:** Node 22+

```bash
git clone https://github.com/nobodydev-gh/Nexus_Neural_126.git
cd Nexus_Neural_126
npm install
npm run dev       # dev server → http://localhost:8080
```

### Build

```bash
npm run build        # production bundle
npm run typecheck    # TypeScript type checks
npm run lint         # ESLint
```

---

## Architecture

```
Operator Console (React/TanStack)
│
├── Local Map — click-to-set goal, costmap overlay
├── SLAM Panel — VIO interface, confidence, recovery state
├── Planner — A*/Dijkstra → Ackermann local planner → Pure Pursuit
├── Perception
│   ├── Depth  — MiDaS-Small contract (SIL: relative depth + ultrasonic scale)
│   ├── SLAM   — ORB-SLAM3 contract (SIL: VIO stub)
│   └── Objects— YOLO-n detection → ByteTrack + Kalman prediction
├── Safety Supervisor — rejects commands; fail-closed
└── ESP32 Watchdog — hardware E-stop bridge
```

---

## Honesty

| Claim | Actual |
|---|---|
| GPS-free | Local SLAM frame — not global geolocation |
| MiDaS-Small | Relative depth + optional ultrasonic scale |
| ORB-SLAM3 | VIO interface with confidence and recovery |
| RANSAC drop-off | Temporal votes + ultrasonics — never one frame |
| YOLO motion | Detection only; ByteTrack + Kalman predict |

The system is not universally safe. Failures are visible, logged, and fail closed.

---

## License

[Apache 2.0](LICENSE) © 2026 UGV-VISTA Contributors
