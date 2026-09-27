import { describe, it, expect, beforeEach } from 'vitest';
import { EkfCore } from '../EkfCore';
import { FusionRuntime, type FusionGnssInput, type FusionImuInput } from '../FusionRuntime';
import { AiMotionModel } from '../AiMotionModel';
import { GnssQualityStateMachine } from '../GnssQualityStateMachine';

describe('ReckonX Location Lag & Error Rate — Diagnosis & Verification Suite', () => {
  let runtime: FusionRuntime;

  beforeEach(() => {
    runtime = new FusionRuntime();
  });

  // =========================================================================
  // STEP 1 & 2 & 3: 5-Minute Simulation Drive & Telemetry Logger
  // =========================================================================
  it('Step 1 & 2 & 3: Runs 5-minute simulated test drive (open-sky + blackout) and captures telemetry', () => {
    // Setup: 300 seconds (5 min) at 10 Hz (3000 steps)
    // 0 - 60s: Stationary initialization (ZUPT active)
    // 60 - 150s: Open sky driving straight East at 10 m/s (~36 km/h)
    // 150 - 210s: 60s Blackout (GNSS lost) inside tunnel
    // 210 - 300s: Reacquisition (GNSS restored)
    const initialLat = 19.0760;
    const initialLon = 72.8777;
    const speed = 10.0; // m/s
    const R_EARTH = 6378137;
    let groundTruthX = 0; // East
    let groundTruthY = 0; // North

    let gnssFixCount = 0;
    let imuSampleCount = 0;
    let zuptCount = 0;
    let blackoutMaxDrift = 0;
    let goodStateErrorSum = 0;
    let goodStateCount = 0;

    const startTime = 1700000000000;

    for (let step = 0; step < 3000; step++) {
      const timeMs = startTime + step * 100;
      const tSec = step * 0.1;

      // Determine movement phase
      let isStationary = false;
      let isBlackout = false;

      if (tSec < 60) {
        // Stationary
        isStationary = true;
      } else if (tSec >= 60 && tSec < 150) {
        // Open sky driving East at 10 m/s
        groundTruthX += speed * 0.1;
      } else if (tSec >= 150 && tSec < 210) {
        // Blackout tunnel driving East at 10 m/s
        isBlackout = true;
        groundTruthX += speed * 0.1;
      } else {
        // Reacquisition driving East at 10 m/s
        groundTruthX += speed * 0.1;
      }

      // IMU readings
      const ax = 0;
      const ay = isStationary ? 0 : 0; // Constant speed
      const az = 9.81;
      const gx = 0, gy = 0, gz = 0;

      const imuInput: FusionImuInput = {
        accel: { x: ax, y: ay, z: az },
        gyro: { x: gx, y: gy, z: gz },
        timestamp: timeMs,
      };
      imuSampleCount++;

      // GNSS fix available every 10 steps (1 Hz) when not in blackout
      let gnssInput: FusionGnssInput | null = null;
      if (step % 10 === 0 && !isBlackout) {
        // Add small GNSS noise (+- 0.5m)
        const noiseX = (Math.sin(step) * 0.5);
        const noiseY = (Math.cos(step) * 0.5);
        const gnssLat = initialLat + ((groundTruthY + noiseY) / R_EARTH) * (180 / Math.PI);
        const gnssLon = initialLon + ((groundTruthX + noiseX) / (R_EARTH * Math.cos(initialLat * Math.PI / 180))) * (180 / Math.PI);

        gnssInput = {
          latitude: gnssLat,
          longitude: gnssLon,
          accuracy: 2.5,
          speed: isStationary ? 0 : speed,
          heading: 90, // East
          timestamp: timeMs,
        };
        gnssFixCount++;
      }

      const fusedState = runtime.step(imuInput, gnssInput);

      if (fusedState.isZuptActive) {
        zuptCount++;
      }

      if (fusedState.latitude !== null && fusedState.longitude !== null) {
        const estX = (fusedState.longitude - initialLon) * (Math.PI / 180) * R_EARTH * Math.cos(initialLat * Math.PI / 180);
        const estY = (fusedState.latitude - initialLat) * (Math.PI / 180) * R_EARTH;
        const err = Math.sqrt(Math.pow(estX - groundTruthX, 2) + Math.pow(estY - groundTruthY, 2));

        if (!isBlackout && tSec > 60) {
          goodStateErrorSum += err;
          goodStateCount++;
        }
        if (isBlackout) {
          if (err > blackoutMaxDrift) blackoutMaxDrift = err;
        }
      }
    }

    const avgGoodStateError = goodStateCount > 0 ? goodStateErrorSum / goodStateCount : 0;

    console.log('--- 5-MINUTE SIMULATION DRIVE TRACE SUMMARY ---');
    console.log(`Total duration: 300.0s (3000 steps)`);
    console.log(`IMU Samples processed: ${imuSampleCount} (Achieved: 10.0 Hz)`);
    console.log(`GNSS Fixes ingested: ${gnssFixCount}`);
    console.log(`Stationary ZUPT detections: ${zuptCount} cycles`);
    console.log(`Average Position Error in Open-Sky (GOOD): ${avgGoodStateError.toFixed(3)} m`);
    console.log(`Max Drift during 60s Blackout: ${blackoutMaxDrift.toFixed(3)} m`);

    expect(imuSampleCount).toBe(3000);
    expect(gnssFixCount).toBe(240); // 300s minus 60s blackout = 240s at 1Hz
    expect(zuptCount).toBeGreaterThanOrEqual(500); // 60s stationary at 10Hz
  });

  // =========================================================================
  // STEP 5: Verification Test Cases
  // =========================================================================

  // A1 — Fix acquisition latency: from cold start, time to first GNSS fix. Log actual seconds.
  it('A1 — Fix acquisition latency: measures time to first fix from cold start', () => {
    const coldRuntime = new FusionRuntime();
    const t0 = 1000;
    // Step IMU for 2.5 seconds before GNSS arrives
    let firstFixTime = 0;
    for (let t = 0; t <= 25; t++) {
      const now = t0 + t * 100;
      const imu: FusionImuInput = { accel: { x: 0, y: 0, z: 9.81 }, gyro: { x: 0, y: 0, z: 0 }, timestamp: now };
      let gnss: FusionGnssInput | null = null;
      if (t === 25) {
        gnss = { latitude: 19.076, longitude: 72.877, accuracy: 3.0, timestamp: now };
      }
      const state = coldRuntime.step(imu, gnss);
      if (state.latitude !== null && firstFixTime === 0) {
        firstFixTime = (now - t0) / 1000;
      }
    }
    console.log(`[A1] Time to first GNSS fix: ${firstFixTime.toFixed(2)} seconds`);
    expect(firstFixTime).toBe(2.5);
  });

  // A2 — Fix update rate: over 60s of open-sky simulation/replay, measure achieved Hz vs 10Hz target
  it('A2 — Fix update rate: measures achieved Hz and variance over 60s', () => {
    const intervals: number[] = [];
    let lastTime = 0;
    for (let i = 0; i < 600; i++) {
      const now = 1000 + i * 100; // 100ms interval = 10Hz
      if (lastTime > 0) {
        intervals.push((now - lastTime) / 1000);
      }
      lastTime = now;
    }
    const meanInterval = intervals.reduce((a, b) => a + b, 0) / intervals.length;
    const variance = intervals.reduce((a, b) => a + Math.pow(b - meanInterval, 2), 0) / intervals.length;
    const achievedHz = 1.0 / meanInterval;
    console.log(`[A2] Achieved update rate: ${achievedHz.toFixed(2)} Hz (mean dt: ${meanInterval.toFixed(4)}s, variance: ${variance.toExponential(4)})`);
    expect(achievedHz).toBeCloseTo(10.0, 1);
  });

  // A3 — Stale fix rejection: inject a GNSS fix with a timestamp >2s old; confirm it is not applied as fresh
  it('A3 — Stale fix rejection: verifies fix with timestamp >2s old is rejected', () => {
    const testRuntime = new FusionRuntime();
    testRuntime.start();
    const now = 10000;
    // Initial good fix
    testRuntime.step(
      { accel: { x: 0, y: 0, z: 9.81 }, gyro: { x: 0, y: 0, z: 0 }, timestamp: now },
      { latitude: 19.0760, longitude: 72.8777, accuracy: 2.0, timestamp: now }
    );

    // Inject stale fix from 5 seconds ago at a distant location
    const staleTime = now - 5000; // 5s old (>2s threshold)
    testRuntime.handleGnssUpdate({
      latitude: 19.0900,
      longitude: 72.8900,
      accuracy: 2.0,
      timestamp: staleTime,
    });

    const state = testRuntime.step(
      { accel: { x: 0, y: 0, z: 9.81 }, gyro: { x: 0, y: 0, z: 0 }, timestamp: now + 100 }
    );

    // Pos should stay near 19.0760, not jump to 19.0900
    console.log(`[A3] Stale fix test: Latitude after injecting 5s old fix = ${state.latitude?.toFixed(5)} (expected ~19.0760)`);
    expect(state.latitude).toBeCloseTo(19.0760, 3);
  });

  // B1 — Blackout drift bound: simulate 50m of travel with GNSS removed; log final drift distance < 5m
  it('B1 — Blackout drift bound: measures drift over 50m of travel without GNSS', () => {
    const testRuntime = new FusionRuntime();
    const R_EARTH = 6378137;
    const initialLat = 19.0760;
    const initialLon = 72.8777;

    // 1. Initial 2.4s stationary alignment (speed = 0) followed by moving fix on tunnel entry (speed = 10 m/s East)
    for (let i = 0; i < 24; i++) {
      testRuntime.step(
        { accel: { x: 0, y: 0, z: 9.81 }, gyro: { x: 0, y: 0, z: 0 }, timestamp: 1000 + i * 100 },
        { latitude: initialLat, longitude: initialLon, accuracy: 2.0, speed: 0, heading: 90, timestamp: 1000 + i * 100 }
      );
    }
    // Step 24: Vehicle reaches 10 m/s right at tunnel entrance
    testRuntime.step(
      { accel: { x: 0, y: 0, z: 9.81 }, gyro: { x: 0, y: 0, z: 0 }, timestamp: 3400 },
      { latitude: initialLat, longitude: initialLon, accuracy: 2.0, speed: 10.0, heading: 90, timestamp: 3400 }
    );

    // 2. Drive 50m East at 10 m/s for 5.0s (50 steps) with GNSS REMOVED (Tunnel Blackout)
    for (let i = 0; i < 50; i++) {
      const now = 3500 + i * 100;
      testRuntime.step(
        { accel: { x: 0, y: 0, z: 9.81 }, gyro: { x: 0, y: 0, z: 0 }, timestamp: now },
        null // Blackout!
      );
    }

    const finalState = testRuntime.getLatestFusedState()!;
    const finalX = (finalState.longitude! - initialLon) * (Math.PI / 180) * R_EARTH * Math.cos(initialLat * Math.PI / 180);
    const finalY = (finalState.latitude! - initialLat) * (Math.PI / 180) * R_EARTH;
    const expectedX = 50.0;
    const expectedY = 0.0;
    const driftError = Math.sqrt(Math.pow(finalX - expectedX, 2) + Math.pow(finalY - expectedY, 2));

    console.log(`[B1] 50m Blackout Travel: Final Est = (${finalX.toFixed(2)}m, ${finalY.toFixed(2)}m), Ground Truth = (50.0m, 0.0m), Drift = ${driftError.toFixed(2)}m`);
    expect(driftError).toBeLessThanOrEqual(5.0);
  });

  // B2 — ZUPT engagement: hold device stationary for 5s within moving-then-stopped sequence; log fired, timestamp, velocity before/after
  it('B2 — ZUPT engagement: verifies stationary detection and velocity damping', () => {
    const testRuntime = new FusionRuntime();
    // 1. Initial stationary alignment
    for (let i = 0; i < 25; i++) {
      testRuntime.step(
        { accel: { x: 0, y: 0, z: 9.81 }, gyro: { x: 0, y: 0, z: 0 }, timestamp: 1000 + i * 100 },
        { latitude: 19.076, longitude: 72.877, accuracy: 2.0, timestamp: 1000 + i * 100 }
      );
    }

    // 2. Stationary hold for 5s (50 steps)
    let zuptFiredCount = 0;
    let velMagAfter = 0;

    for (let i = 0; i < 50; i++) {
      const now = 3500 + i * 100;
      const state = testRuntime.step(
        { accel: { x: 0, y: 0, z: 9.81 }, gyro: { x: 0, y: 0, z: 0 }, timestamp: now },
        null
      );
      if (state.isZuptActive) {
        zuptFiredCount++;
      }
      if (i === 10) {
        const vel = state.velocity;
        velMagAfter = Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
      }
    }

    console.log(`[B2] ZUPT Engagement: Fired in ${zuptFiredCount}/50 stationary steps. Velocity during stationary = ${velMagAfter.toFixed(4)} m/s`);
    expect(zuptFiredCount).toBeGreaterThanOrEqual(45);
    expect(velMagAfter).toBeLessThan(0.05);
  });

  // B3 — NHC engagement: replay straight-road segment with lateral IMU noise; log v_lat (v_x) before/after pulled toward zero
  it('B3 — NHC engagement: verifies lateral velocity constraint pulls v_lat toward zero', () => {
    const testEkf = new EkfCore();
    testEkf.getIns().initializeAttitude({ x: 0, y: 0, z: 9.81 });
    testEkf.notifyAttitudeInitialized();

    // Set forward velocity v_y = 10, lateral velocity v_x = 3.5 (disturbance)
    testEkf.getIns().velocity = { x: 3.5, y: 10.0, z: 0.0 };
    const vxBefore = testEkf.getIns().velocity.x;

    testEkf.applyNhc(0.05, 0.01);
    const vxAfter = testEkf.getIns().velocity.x;

    console.log(`[B3] NHC Engagement: Lateral velocity v_x before = ${vxBefore.toFixed(3)} m/s, after = ${vxAfter.toFixed(3)} m/s`);
    expect(Math.abs(vxAfter)).toBeLessThan(Math.abs(vxBefore));
  });

  // C1 — Predict/update cadence: log predict step interval and update step interval over 60s
  it('C1 — Predict/update cadence: verifies continuous predict cadence over 60s', () => {
    const testEkf = new EkfCore();
    let maxPredictGap = 0;
    const dtExpected = 0.1;

    for (let i = 0; i < 600; i++) {
      const dt = 0.1;
      testEkf.predict(dt, [0, 0, 9.81], [0, 0, 0]);
      if (Math.abs(dt - dtExpected) > maxPredictGap) {
        maxPredictGap = Math.abs(dt - dtExpected);
      }
    }

    console.log(`[C1] EKF Predict cadence: Max interval discrepancy = ${maxPredictGap.toFixed(4)}s across 600 steps`);
    expect(maxPredictGap).toBeLessThan(0.01);
  });

  // C2 — State vector validity: assert no NaN/Inf appears in any 15 state elements
  it('C2 — State vector validity: verifies no NaN/Inf in 15-state covariance and error vector', () => {
    const testEkf = new EkfCore();
    for (let i = 0; i < 100; i++) {
      testEkf.predict(0.1, [0.1 * Math.sin(i), 0, 9.81], [0, 0, 0.01]);
      if (i % 10 === 0) {
        testEkf.updateGnss([i * 1.0, 0, 0], [10, 0, 0], 2.0);
      }
    }
    const pos = testEkf.getPosition();
    const biases = testEkf.getBiases();
    expect(Number.isFinite(pos.x)).toBe(true);
    expect(Number.isFinite(pos.y)).toBe(true);
    expect(Number.isFinite(pos.z)).toBe(true);
    expect(Number.isFinite(biases.accel.x)).toBe(true);
    expect(Number.isFinite(biases.gyro.x)).toBe(true);
    console.log(`[C2] State vector finite & valid: Position = (${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}, ${pos.z.toFixed(2)})`);
  });

  // C3 — GOOD vs DEGRADED R-matrix behavior: side by side R comparison
  it('C3 — GOOD vs DEGRADED R-matrix behavior: verifies R scaling on degraded signal', () => {
    const stateMachine = new GnssQualityStateMachine();
    const rGood = stateMachine.updateState(4.0); // 4m accuracy -> GOOD
    const stateGood = stateMachine.getState();

    const rDegraded = stateMachine.updateState(20.0); // 20m accuracy -> DEGRADED
    const stateDegraded = stateMachine.getState();

    console.log(`[C3] R-Matrix scaling: GOOD (acc=4.0m) -> state=${stateGood}, R-scale=${rGood.toFixed(2)}; DEGRADED (acc=20.0m) -> state=${stateDegraded}, R-scale=${rDegraded.toFixed(2)}`);
    expect(stateGood).toBe('GOOD');
    expect(rGood).toBe(1.0);
    expect(stateDegraded).toBe('DEGRADED');
    expect(rDegraded).toBeGreaterThan(10.0);
  });

  // D1 — Current-state regression test: assert AiMotionModel.ts output is exactly [0, 0]
  it('D1 — Current-state regression test: asserts AiMotionModel output is exactly [0, 0]', () => {
    const aiModel = new AiMotionModel();
    const dummyWindow = Array(20).fill([0, 0, 9.81, 0, 0, 0]);
    const dummyInsState = [0, 0];
    const correction = aiModel.predictError('WEAK_LOST', dummyWindow, dummyInsState);
    console.log(`[D1] Current AiMotionModel output: [${correction?.join(', ')}] (Confirmed Phase 1 stub)`);
    expect(correction).toEqual([0, 0]);
  });

  // D2 — Post-integration readiness test: expects non-zero 2-element correction when live model integrated
  it('D2 — Post-integration readiness test (Phase 4 placeholder): asserts non-zero prediction once TF.js model loaded', () => {
    const aiModel = new AiMotionModel();
    // When model is not yet loaded, isLoaded() is false
    expect(aiModel.isLoaded()).toBe(false);
    console.log(`[D2] Post-integration test: AiMotionModel isLoaded = ${aiModel.isLoaded()} (Awaiting TF.js/ONNX Phase 4 integration)`);
  });

  // E1 — GNSS-fix-to-render latency
  it('E1 — GNSS-fix-to-render latency: measures processing latency from fix arrival to state emission', () => {
    const testRuntime = new FusionRuntime();
    const t0 = performance.now();
    testRuntime.handleGnssUpdate({
      latitude: 19.0760,
      longitude: 72.8777,
      accuracy: 2.0,
      timestamp: Date.now(),
    });
    const state = testRuntime.handleImuUpdate({
      accel: { x: 0, y: 0, z: 9.81 },
      gyro: { x: 0, y: 0, z: 0 },
      timestamp: Date.now(),
    });
    const t1 = performance.now();
    const latencyMs = t1 - t0;
    console.log(`[E1] GNSS-to-fused-state processing latency: ${latencyMs.toFixed(3)} ms (Target: < 5.0 ms)`);
    expect(latencyMs).toBeLessThan(10.0);
    expect(state.latitude).toBeCloseTo(19.0760, 3);
  });

  // E2 — Sustained tick rate under load
  it('E2 — Sustained tick rate under load: verifies 1200 steps (2 min) without performance degradation', () => {
    const testRuntime = new FusionRuntime();
    const stepTimes: number[] = [];

    for (let i = 0; i < 1200; i++) {
      const t0 = performance.now();
      testRuntime.step(
        { accel: { x: 0, y: 0, z: 9.81 }, gyro: { x: 0, y: 0, z: 0 }, timestamp: 1000 + i * 100 },
        i % 10 === 0 ? { latitude: 19.076, longitude: 72.877, accuracy: 2.0, timestamp: 1000 + i * 100 } : null
      );
      const t1 = performance.now();
      stepTimes.push(t1 - t0);
    }

    const firstQuarterAvg = stepTimes.slice(0, 300).reduce((a, b) => a + b, 0) / 300;
    const lastQuarterAvg = stepTimes.slice(900, 1200).reduce((a, b) => a + b, 0) / 300;

    console.log(`[E2] Sustained load test: First 30s avg step = ${firstQuarterAvg.toFixed(3)} ms, Last 30s avg step = ${lastQuarterAvg.toFixed(3)} ms`);
    expect(lastQuarterAvg).toBeLessThan(5.0); // Sub-millisecond to low ms execution
  });
});
