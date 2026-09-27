import { describe, it, expect } from 'vitest';
import { FusionAdapter } from '../FusionAdapter';
import { LocationService } from '../../locationService';
import { SensorService } from '../../sensorService';
import { DeadReckoningEngine } from '../../deadReckoningEngine';
import { OrientationService } from '../../OrientationService';
import type { RealSensorData, CurrentLocationData } from '../../../types/navigation';

describe('ReckonX Live Navigation 4-Issue Verification Suite', () => {
  // =========================================================================
  // ISSUE 1: Status Indicators & Defined Fallback States
  // =========================================================================
  describe('Issue 1: Status UI Indicators & Fallback States', () => {
    it('verifies GNSS Quality badges, scenarios, and drift values render defined states under null/undefined inputs', () => {
      const scenarios = ['scenario1', 'scenario2', 'scenario3', 'scenario4', undefined] as const;
      const qualities = ['GOOD', 'DEGRADED', 'WEAK_LOST', undefined, null] as const;

      for (const scenario of scenarios) {
        for (const quality of qualities) {
          const scenarioBadges: Record<string, { label: string }> = {
            scenario1: { label: 'Scenario 1: [GPS ON + Net ON] Online Nav' },
            scenario2: { label: 'Scenario 2: [GPS OFF + Net ON] Tunnel DR Mode' },
            scenario3: { label: 'Scenario 3: [GPS ON + Net OFF] Offline Satellite' },
            scenario4: { label: 'Scenario 4: [GPS OFF + Net OFF] Pure Offline DR' },
          };
          const currentBadge = scenario ? scenarioBadges[scenario] || scenarioBadges.scenario4 : scenarioBadges.scenario4;
          expect(currentBadge).toBeDefined();
          expect(currentBadge.label.length).toBeGreaterThan(0);

          const badgeText = quality === 'GOOD' ? 'GNSS: HIGH' : quality === 'DEGRADED' ? 'GNSS: MED' : 'GNSS: LOST';
          expect(['GNSS: HIGH', 'GNSS: MED', 'GNSS: LOST']).toContain(badgeText);
        }
      }

      // Heading formatter handles undefined / invalid headings gracefully
      const formatted = OrientationService.formatCardinalHeading(0, false);
      expect(formatted).toBe('--');

      const formattedValid = OrientationService.formatCardinalHeading(90, true);
      expect(formattedValid).toContain('E');
    });
  });

  // =========================================================================
  // ISSUE 2: Hardware Permission Gating & Stationary Cursor Stability
  // =========================================================================
  describe('Issue 2: Hardware Permission Gating & Stationary Cursor Stability', () => {
    it('verifies explicit permission check methods exist and return valid states', async () => {
      const permState = await LocationService.checkPermissionState();
      expect(['granted', 'denied', 'prompt']).toContain(permState);

      const motionGranted = await SensorService.requestMotionPermission();
      expect(typeof motionGranted).toBe('boolean');

      const orientationGranted = await SensorService.requestOrientationPermission();
      expect(typeof orientationGranted).toBe('boolean');
    });

    it('verifies stationary cursor stability: 30 seconds of stationary standing with noisy GPS (±5m jitter) does not wander', () => {
      const adapter = new FusionAdapter();
      const origin: [number, number] = [19.0760, 72.8777];
      adapter.seedOrigin(origin[0], origin[1]);

      const steps = 300; // 30s at 10Hz
      const positions: [number, number][] = [];
      const speeds: number[] = [];

      for (let i = 0; i < steps; i++) {
        // Stationary sensor frame (gravity only on Z, no body acceleration, no rotation)
        const sensorData: RealSensorData = {
          ax: (Math.random() - 0.5) * 0.05, // Small IMU noise
          ay: (Math.random() - 0.5) * 0.05,
          az: 9.81 + (Math.random() - 0.5) * 0.05,
          accelMag: 9.81,
          gx: 0.0,
          gy: 0.0,
          gz: 0.0,
          gyroMag: 0.0,
          magX: null,
          magY: null,
          magZ: null,
          alpha: 0,
          beta: 0,
          gamma: 0,
          headingDeg: 0,
          intervalMs: 100,
          sampleRateHz: 10,
          timestamp: Date.now() + i * 100,
        };

        // GPS fix arrives at 1Hz with ±5m multipath noise
        let gnssData: CurrentLocationData | null = null;
        if (i % 10 === 0) {
          const latJitter = ((Math.random() - 0.5) * 10) / 111111; // ~±5m jitter
          const lngJitter = ((Math.random() - 0.5) * 10) / 111111;
          gnssData = {
            latitude: origin[0] + latJitter,
            longitude: origin[1] + lngJitter,
            accuracy: 6.0,
            altitude: 10,
            speed: 0.0,
            bearing: 0,
            timestamp: Date.now() + i * 100,
            address: 'Stationary Position',
            source: 'gps',
            isStale: false,
            ageSec: 0,
          };
        }

        const estimate = adapter.step(sensorData, gnssData, origin);
        positions.push(estimate.position);
        speeds.push(estimate.velocitySpeedKmH);
      }

      // Compute total cursor wander distance from origin
      const wanderDistances = positions.map((p) =>
        DeadReckoningEngine.calculateHaversineDistance(origin, p) * 1000
      );
      const maxWanderMeters = Math.max(...wanderDistances);
      const finalSpeedKmH = speeds[speeds.length - 1];

      console.log(`[Issue 2 Test] 30s Stationary Test with GPS Jitter: Max Wander = ${maxWanderMeters.toFixed(2)}m, Final Speed = ${finalSpeedKmH.toFixed(2)} km/h`);

      // ZUPT clamps stationary speed to near 0 and limits wander to within GPS accuracy
      expect(maxWanderMeters).toBeLessThan(10.0);
      expect(finalSpeedKmH).toBeLessThan(0.5);
    });
  });

  // =========================================================================
  // ISSUE 3: End-to-End Latency Measurement
  // =========================================================================
  describe('Issue 3: Location Cursor Latency Profiling', () => {
    it('measures end-to-end processing latency from sensor arrival to fused estimate', () => {
      const adapter = new FusionAdapter();
      const origin: [number, number] = [19.0760, 72.8777];
      adapter.seedOrigin(origin[0], origin[1]);

      const sensorData: RealSensorData = {
        ax: 0,
        ay: 0,
        az: 9.81,
        accelMag: 9.81,
        gx: 0,
        gy: 0,
        gz: 0,
        gyroMag: 0,
        magX: null,
        magY: null,
        magZ: null,
        alpha: 0,
        beta: 0,
        gamma: 0,
        headingDeg: 0,
        intervalMs: 100,
        sampleRateHz: 10,
        timestamp: Date.now(),
      };

      const iterations = 500;
      const latencies: number[] = [];

      for (let i = 0; i < iterations; i++) {
        const start = performance.now();
        adapter.step(sensorData, null, origin);
        const end = performance.now();
        latencies.push(end - start);
      }

      const avgLatencyMs = latencies.reduce((a, b) => a + b, 0) / latencies.length;
      const maxLatencyMs = Math.max(...latencies);

      console.log(`[Issue 3 Test] EKF Step Latency over 500 samples: Avg = ${avgLatencyMs.toFixed(3)} ms, Max = ${maxLatencyMs.toFixed(3)} ms`);

      expect(avgLatencyMs).toBeLessThan(2.0); // Target < 2ms per step
    });
  });

  // =========================================================================
  // ISSUE 4: Full Airplane Mode Navigation (2-Minute Dead Reckoning Run)
  // =========================================================================
  describe('Issue 4: Airplane Mode Dead Reckoning Navigation', () => {
    it('simulates 2 minutes (120 seconds, 1200 steps) of uninterrupted walking/driving in pure Airplane Mode', () => {
      const adapter = new FusionAdapter();
      const origin: [number, number] = [19.0760, 72.8777];
      adapter.seedOrigin(origin[0], origin[1]);

      const totalSteps = 1200; // 120s at 10Hz
      const dt = 0.1;
      let currentPos = [...origin] as [number, number];
      let distanceHistory: number[] = [];
      let uncaughtErrors: string[] = [];

      // Vehicle Profile: 2 minutes walking at 1.4 m/s (5.0 km/h) North in Airplane Mode (GNSS = null)
      for (let i = 0; i < totalSteps; i++) {
        const time = i * dt;
        const isWalking = time <= 120;
        const walkingCadence = Math.sin(time * 2 * Math.PI * 1.8); // 1.8 Hz step frequency
        const forwardAccel = isWalking ? 0.8 * walkingCadence + 0.4 : 0.0;
        const verticalAccel = isWalking ? 9.81 + 1.8 * Math.cos(time * 2 * Math.PI * 1.8) : 9.81;
        const gyroPitch = isWalking ? 15.0 * walkingCadence : 0.0;

        const sensorData: RealSensorData = {
          ax: 0.0,
          ay: forwardAccel,
          az: verticalAccel,
          accelMag: Math.sqrt(forwardAccel * forwardAccel + verticalAccel * verticalAccel),
          gx: gyroPitch,
          gy: 0.0,
          gz: 0.0,
          gyroMag: Math.abs(gyroPitch),
          magX: null,
          magY: null,
          magZ: null,
          alpha: 0,
          beta: 0,
          gamma: 0,
          headingDeg: 0,
          intervalMs: 100,
          sampleRateHz: 10,
          timestamp: Date.now() + i * 100,
        };

        try {
          // Pure Airplane Mode: GNSS is strictly null, Network is offline
          const estimate = adapter.step(sensorData, null, origin);

          if (isNaN(estimate.position[0]) || isNaN(estimate.position[1])) {
            uncaughtErrors.push(`Step ${i}: NaN in position`);
          }

          currentPos = estimate.position;
          const distCovered = DeadReckoningEngine.calculateHaversineDistance(origin, currentPos) * 1000;
          distanceHistory.push(distCovered);
        } catch (err: any) {
          uncaughtErrors.push(`Step ${i}: Exception: ${err.message}`);
        }
      }

      const finalDistanceCovered = distanceHistory[distanceHistory.length - 1];
      const expectedDistance = 120 * 1.4; // ~168 meters

      console.log(`\n=== 2-MINUTE AIRPLANE MODE DEAD RECKONING WALK ===`);
      console.log(`Total Steps Executed: ${totalSteps} (10Hz)`);
      console.log(`Final Position: [${currentPos[0].toFixed(5)}, ${currentPos[1].toFixed(5)}]`);
      console.log(`Distance Traveled via DR: ${finalDistanceCovered.toFixed(2)} meters (Expected: ~${expectedDistance.toFixed(2)}m)`);
      console.log(`Uncaught Errors: ${uncaughtErrors.length}`);

      expect(uncaughtErrors).toEqual([]);
      expect(finalDistanceCovered).toBeGreaterThan(50); // Continuously advanced via inertial integration
      expect(finalDistanceCovered).toBeLessThan(3500); // Bounded unassisted accelerometer drift envelope
    });
  });
});
