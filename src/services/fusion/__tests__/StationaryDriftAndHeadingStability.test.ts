import { describe, it, expect, beforeEach } from 'vitest';
import { FusionRuntime } from '../FusionRuntime';
import { OrientationService } from '../../OrientationService';
import { DeadReckoningEngine } from '../../deadReckoningEngine';

describe('Stationary Drift and Heading Stability Verification Suite', () => {
  let runtime: FusionRuntime;

  beforeEach(() => {
    runtime = new FusionRuntime();
  });

  it('STEP 1 & 2: 60-Second Stationary Test with GNSS GOOD & Tilted Device (ZUPT Verification)', () => {
    const originLat = 19.0760;
    const originLon = 72.8777;
    runtime.seedInitialPosition(originLat, originLon);

    // Simulate 60 seconds at 10 Hz (600 steps)
    // Phone tilted at 15 degrees (gravity projected into ax and az)
    const tiltRad = (15 * Math.PI) / 180;
    const ax = 9.81 * Math.sin(tiltRad); // ~2.54 m/s²
    const az = 9.81 * Math.cos(tiltRad); // ~9.47 m/s²

    let zuptTriggerCount = 0;
    let maxDriftMeters = 0;
    let maxVelocityMagnitude = 0;
    const now = 1700000000000;

    for (let step = 0; step < 600; step++) {
      const timestamp = now + step * 100;
      
      // GNSS fix at 1 Hz with realistic micro-jitter (±0.5m) and GOOD accuracy (3.0m)
      let gnssInput: any = null;
      if (step % 10 === 0) {
        const jitterLat = (Math.sin(step) * 0.000004); // ~0.4m
        const jitterLon = (Math.cos(step) * 0.000004); // ~0.4m
        gnssInput = {
          latitude: originLat + jitterLat,
          longitude: originLon + jitterLon,
          accuracy: 3.0,
          speed: 0.05, // GPS speed noise < 0.2 km/h
          heading: (step * 37) % 360, // Random noisy course-over-ground bearing at standstill
          timestamp,
        };
      }

      // Realistic IMU sensor noise on stationary table
      const noiseAx = (Math.random() - 0.5) * 0.02;
      const noiseAy = (Math.random() - 0.5) * 0.02;
      const noiseAz = (Math.random() - 0.5) * 0.02;
      const noiseGx = (Math.random() - 0.5) * 0.005;
      const noiseGy = (Math.random() - 0.5) * 0.005;
      const noiseGz = (Math.random() - 0.5) * 0.005;

      const imuInput = {
        accel: { x: ax + noiseAx, y: noiseAy, z: az + noiseAz },
        gyro: { x: noiseGx, y: noiseGy, z: noiseGz },
        timestamp,
      };

      const fused = runtime.step(imuInput, gnssInput);

      if (fused.isZuptActive) {
        zuptTriggerCount++;
      }

      const velMag = Math.sqrt(
        fused.velocity.x ** 2 + fused.velocity.y ** 2 + fused.velocity.z ** 2
      );
      if (velMag > maxVelocityMagnitude) {
        maxVelocityMagnitude = velMag;
      }

      if (fused.latitude !== null && fused.longitude !== null) {
        const dLat = (fused.latitude - originLat) * 111111;
        const dLon = (fused.longitude - originLon) * 111111 * Math.cos((originLat * Math.PI) / 180);
        const dist = Math.sqrt(dLat * dLat + dLon * dLon);
        if (dist > maxDriftMeters) {
          maxDriftMeters = dist;
        }
      }
    }

    console.log(`[Stationary Test] ZUPT Triggers: ${zuptTriggerCount}/600 cycles`);
    console.log(`[Stationary Test] Max Velocity Magnitude: ${maxVelocityMagnitude.toFixed(4)} m/s`);
    console.log(`[Stationary Test] Max Position Drift: ${maxDriftMeters.toFixed(3)} m`);

    // Confirm ZUPT engages consistently (>90% of samples)
    expect(zuptTriggerCount).toBeGreaterThan(540);
    // Confirm velocity remains clamped near zero (< 0.10 m/s / < 0.36 km/h)
    expect(maxVelocityMagnitude).toBeLessThan(0.10);
    // Confirm position remains strictly pinned within 1 meter of origin
    expect(maxDriftMeters).toBeLessThan(1.0);
  });

  it('STEP 4: Heading Stability at Standstill (< 1.5 km/h) prevents erratic trajectory bearing swings', () => {
    let currentHeading = 326.0;

    // Simulate 30 seconds of stationary standstill with random GPS multipath coordinate jitter
    // creating random 360-degree trajectory bearings
    const initialPos: [number, number] = [19.0760, 72.8777];
    let prevPos = [...initialPos] as [number, number];

    const headingsObserved: number[] = [];

    for (let step = 0; step < 300; step++) {
      const dt = 0.1;
      // GPS multipath random noise
      const currPos: [number, number] = [
        initialPos[0] + (Math.random() - 0.5) * 0.00003,
        initialPos[1] + (Math.random() - 0.5) * 0.00003,
      ];
      
      const noisyTrajBearing = OrientationService.calculateBearing(prevPos, currPos);

      // Raw magnetometer reading with tiny ±0.5° sensor noise around 326°
      const compassMag = 326.0 + (Math.random() - 0.5) * 1.0;

      // Speed is < 0.5 km/h
      const speedKmH = 0.2;

      currentHeading = OrientationService.fuseHeading({
        magnetometerHeading: compassMag,
        gnssTrackBearing: (step * 73) % 360, // Noisy GPS course
        trajectoryBearing: noisyTrajBearing, // Noisy trajectory bearing
        gyroZRate: (Math.random() - 0.5) * 0.01,
        speedKmH,
        isGnssAvailable: true,
        deltaTimeSec: dt,
        previousHeading: currentHeading,
      });

      headingsObserved.push(currentHeading);
      prevPos = currPos;
    }

    const minHeading = Math.min(...headingsObserved);
    const maxHeading = Math.max(...headingsObserved);
    const headingSpan = maxHeading - minHeading;

    console.log(`[Heading Stability Test] Heading Span at Standstill: ${headingSpan.toFixed(2)}° (Min: ${minHeading.toFixed(1)}°, Max: ${maxHeading.toFixed(1)}°)`);

    // Heading must stay rock-solid within a couple degrees of 326°, never swinging 50°+
    expect(headingSpan).toBeLessThan(3.5);
    expect(minHeading).toBeGreaterThan(323.0);
    expect(maxHeading).toBeLessThan(329.0);
  });

  it('STEP 5: DeadReckoningEngine kinematic step holds position when stationary', () => {
    const origin: [number, number] = [19.0760, 72.8777];
    let pos = [...origin] as [number, number];
    let drift = 0;

    for (let i = 0; i < 100; i++) {
      const res = DeadReckoningEngine.stepKinematics(
        pos,
        0.0, // 0 km/h speed
        0.05, // residual accel noise
        326.0,
        0.1,
        drift
      );
      pos = res.position;
      drift = res.driftErrorMeters;
      expect(res.isZuptActive).toBe(true);
      expect(res.velocitySpeedKmH).toBe(0);
    }

    expect(pos[0]).toBeCloseTo(origin[0], 6);
    expect(pos[1]).toBeCloseTo(origin[1], 6);
    expect(drift).toBe(0);
  });
});
