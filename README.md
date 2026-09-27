# ReckonX — Intelligent Dead Reckoning & Sensor Fusion Navigation

[![React](https://img.shields.io/badge/React-19.x-blue.svg)](https://react.dev/)
[![TypeScript](https://img.shields.io/badge/TypeScript-6.x-blue.svg)](https://www.typescriptlang.org/)
[![Vite](https://img.shields.io/badge/Vite-5.x-646CFF.svg)](https://vitejs.dev/)
[![TailwindCSS](https://img.shields.io/badge/TailwindCSS-v4-38B2AC.svg)](https://tailwindcss.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

**ReckonX** is an edge-first, intelligent Dead Reckoning (DR) navigation system engineered specifically for GPS-denied environments: tunnels, underground corridors, multi-level parkings, and urban canyons. Built as a high-performance React + TypeScript SPA, it combines browser hardware sensor streaming (up to 100 Hz) with an advanced 15-state Error-State Extended Kalman Filter (ES-EKF).

---

## Key Features

- **15-State Error-State EKF Fusion Engine (Default):**
  - Strapdown Inertial Navigation System (INS) mechanization in local East-North-Up (ENU) coordinates.
  - 15-state Kalman Filter tracking position error ($\delta\mathbf{p}$), velocity error ($\delta\mathbf{v}$), attitude error ($\delta\boldsymbol{\theta}$), accelerometer bias ($\mathbf{b}_a$), and gyroscope bias ($\mathbf{b}_g$).
  - Integrated **Zero Velocity Updates (ZUPT)** and **Non-Holonomic Constraints (NHC)** to arrest drift during vehicle stops and turns.
  - Powered by pure TypeScript matrix mathematics (`ml-matrix`).

- **GNSS Quality State Machine & Live Badging:**
  - Continuous 4-state GNSS signal classifier: `GOOD (<10m)`, `DEGRADED (10-25m)`, `WEAK`, and `LOST (>25m / no fix)` with hysteresis counters.
  - Real-time status indicators in the Navigation HUD and Telemetry inspector.

- **A/B Switchable Fusion Modes:**
  - Defaulting to `ekf` with a selectable runtime toggle to `legacy` (kinematic stepping + heading fusion) via Profile settings for real-world validation.

- **Real-Time Hardware Sensor Streaming:**
  - High-frequency ingestion of 3-axis Accelerometer, 3-axis Gyroscope, Magnetometer, and Device Orientation via W3C `DeviceMotionEvent`, `DeviceOrientationEvent`, and Generic Sensor APIs with iOS WebKit permission workflows.
  - **Strict No Fake Data Standard:** In Live Device Mode, values strictly reflect real hardware sensors without synthetic jitter or `Math.random()` mocks.

- **Vehicle-Specific Multi-Profile Routing:**
  - Dedicated OSRM routing profiles for **Car**, **Bike**, and **Walking** with profile-aware caching and `AbortController` concurrency protection.

- **Offline Tile Caching & 4-Scenario Matrix:**
  - IndexedDB map tile cache (`IDR_Tile_Cache_DB`) for full offline functionality.
  - Auto-detection across 4 operational scenarios:
    1. `[GPS ON + Net ON]` Standard Online Navigation
    2. `[GPS OFF + Net ON]` Tunnel / GNSS Outage Dead Reckoning
    3. `[GPS ON + Net OFF]` Offline Satellite Navigation
    4. `[GPS OFF + Net OFF]` Pure Offline Dead Reckoning

---

## Repository Structure

```text
reckonx/
├── src/
│   ├── components/                 # UI components (MapView, TelemetryChart, BottomNav, TopHeader, Toast)
│   ├── config/                     # Centralized API & Map tile configurations
│   ├── context/                    # React Context (NavigationContext - Central state provider)
│   ├── pages/                      # Page views (Explore, RouteSetup, NavigationHud, Telemetry, Profile, Solution, Summary)
│   ├── services/
│   │   ├── api/                    # REST API abstraction layer (Auth, Tracking, Telemetry, Profile)
│   │   ├── fusion/                 # 15-State Error-State EKF Fusion Subsystem
│   │   │   ├── __tests__/          # Fusion unit test suite (Vitest)
│   │   │   ├── AiMotionModel.ts    # AI motion error model runtime interface & normalization
│   │   │   ├── EkfCore.ts          # 15-state ES-EKF matrix math (ml-matrix)
│   │   │   ├── FusionAdapter.ts    # High-level bridge adapting ReckonX types to EKF runtime
│   │   │   ├── FusionRuntime.ts    # EKF orchestrator (INS, ZUPT, NHC & GNSS updates)
│   │   │   ├── GnssQualityStateMachine.ts # 4-state GNSS signal quality classifier
│   │   │   ├── InsMechanization.ts # Strapdown INS position/velocity/attitude integration
│   │   │   └── OutputStabilizer.ts # Smoothing & outlier rejection for fused outputs
│   │   ├── deadReckoningEngine.ts  # Legacy kinematic stepping & ZUPT filter (fallback)
│   │   ├── locationService.ts      # Geocoding & GPS watcher
│   │   ├── OrientationService.ts   # Multi-source heading fusion (fallback)
│   │   ├── routeService.ts         # Multi-profile routing engine
│   │   ├── sensorService.ts        # Browser motion/orientation listeners & WebKit permissions
│   │   └── tileCacheService.ts     # IndexedDB tile cache store
│   ├── types/                      # TypeScript domain types (navigation.ts)
│   └── utils/                      # Utilities (distanceFormatter, routeProgress)
├── brain.md                        # Central architectural brain & knowledge map
└── package.json
```

---

## State of the AI Correction Layer

The 15-state Error-State EKF core, strapdown INS mechanization, ZUPT/NHC constraints, and GNSS quality state machine run **100% in pure TypeScript** directly in the browser via `ml-matrix`. 

In the upstream reference design (`IDR_PRO`), a CNN-LSTM residual correction model (`ins_error_model_fused.tflite`, 134 KB) takes a `[1, 20, 6]` IMU temporal sliding window (accel/gyro normalized against pre-computed scale factors) and a `[1, 2]` INS state vector to predict residual velocity error corrections $[\delta v_N, \delta v_E]$. That model executes via `react-native-fast-tflite` (a native C++/JSI module for iOS/Android). In ReckonX, `src/services/fusion/AiMotionModel.ts` encapsulates this exact feature windowing and normalization pipeline behind a clean modular contract. Until client-side TensorFlow.js graph weights or an ONNX Web runtime model is compiled and deployed, it operates in safe zero-residual pass-through mode without impeding the EKF filter convergence.

---

## Getting Started

### Prerequisites
- Node.js 18+ / 20+
- npm 9+

### Installation & Running

```bash
# Install dependencies
npm install

# Start local development server
npm run dev

# Run unit tests
npm test

# Run Oxlint static analysis
npm run lint

# Build production bundle
npm run build

# Preview production build
npm run preview
```

---

## Verification & Testing

- **EKF Fusion Unit Tests:** Run `npm test` to execute Vitest verification of `FusionAdapter` initialization, IMU prediction updates, and GNSS correction cycles.
- **A/B Testing in HUD:** Toggle between **EKF** and **Legacy** under `Profile -> Dead Reckoning & Fusion Engine` to compare trajectory tracking performance during GNSS outages.
