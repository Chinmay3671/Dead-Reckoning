import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
import { LocationService } from '../../locationService';
import { RouteService } from '../../routeService';
import { FusionAdapter } from '../FusionAdapter';
import { DeadReckoningEngine } from '../../deadReckoningEngine';
import { OrientationService } from '../../OrientationService';
import type { RealSensorData, CurrentLocationData } from '../../../types/navigation';

describe('GNSS-Off / Internet-Off (Scenario 4) Browser Simulation Test Suite', () => {
  const originalNavigator = globalThis.navigator;
  const originalFetch = globalThis.fetch;

  beforeEach(() => {
    vi.restoreAllMocks();
  });

  afterEach(() => {
    Object.defineProperty(globalThis, 'navigator', {
      value: originalNavigator,
      writable: true,
    });
    globalThis.fetch = originalFetch;
  });

  describe('1. GPS Error Callback & Failure Handling', () => {
    it('handles navigator.geolocation error callback with code 2 (POSITION_UNAVAILABLE) without throwing', async () => {
      let watchErrorReceived: Error | null = null;
      let watchLocationReceived: CurrentLocationData | null = null;

      // Mock navigator.geolocation to simulate hardware GNSS loss
      const mockGeolocation = {
        getCurrentPosition: vi.fn((_success, error) => {
          error({
            code: 2, // POSITION_UNAVAILABLE
            message: 'Position unavailable',
            PERMISSION_DENIED: 1,
            POSITION_UNAVAILABLE: 2,
            TIMEOUT: 3,
          });
        }),
        watchPosition: vi.fn((_success, error) => {
          error({
            code: 2, // POSITION_UNAVAILABLE
            message: 'Position unavailable',
            PERMISSION_DENIED: 1,
            POSITION_UNAVAILABLE: 2,
            TIMEOUT: 3,
          });
          return 42;
        }),
        clearWatch: vi.fn(),
      };

      Object.defineProperty(globalThis, 'navigator', {
        value: {
          ...originalNavigator,
          geolocation: mockGeolocation,
          onLine: false,
        },
        writable: true,
      });

      // Test LocationService.startLocationWatch error dispatch
      const cleanup = LocationService.startLocationWatch(
        (loc) => {
          watchLocationReceived = loc;
        },
        (err) => {
          watchErrorReceived = err;
        }
      );

      expect(mockGeolocation.watchPosition).toHaveBeenCalled();
      expect(watchLocationReceived).toBeNull();
      expect(watchErrorReceived).not.toBeNull();
      expect((watchErrorReceived as any)?.message).toBe('GPS signal unavailable.');

      cleanup();
      expect(mockGeolocation.clearWatch).toHaveBeenCalledWith(42);
    });

    it('rejects getCurrentLocation gracefully on POSITION_UNAVAILABLE without crashing', async () => {
      const mockGeolocation = {
        getCurrentPosition: vi.fn((_success, error) => {
          error({
            code: 2,
            message: 'Position unavailable',
            PERMISSION_DENIED: 1,
            POSITION_UNAVAILABLE: 2,
            TIMEOUT: 3,
          });
        }),
        watchPosition: vi.fn(),
        clearWatch: vi.fn(),
      };

      Object.defineProperty(globalThis, 'navigator', {
        value: {
          ...originalNavigator,
          geolocation: mockGeolocation,
          onLine: false,
        },
        writable: true,
      });

      await expect(LocationService.getCurrentLocation()).rejects.toThrow('GPS location unavailable.');
    });
  });

  describe('2. Offline Route Calculation & Corridor Generation', () => {
    it('produces an offline dead-reckoning navigation corridor when navigator.onLine is false', async () => {
      Object.defineProperty(globalThis, 'navigator', {
        value: {
          ...originalNavigator,
          onLine: false,
        },
        writable: true,
      });

      const origin: [number, number] = [19.076, 72.8777]; // Mumbai
      const destination: [number, number] = [18.922, 72.8347]; // Gateway of India

      const routeResult = await RouteService.calculateRoute({
        origin,
        destination,
        vehicleProfile: 'car',
      });

      expect(routeResult).toBeDefined();
      expect(routeResult.routes.length).toBeGreaterThan(0);
      expect(routeResult.distanceKm).toBeGreaterThan(0);
      expect(routeResult.geometry.length).toBeGreaterThan(2);
      expect(routeResult.steps.length).toBeGreaterThan(0);
      expect(routeResult.providerUrl).toBe('local:offline');
      expect(routeResult.summary).toBe('Offline Dead-Reckoning Corridor');
    });

    it('recovers with offline corridor if fetch() throws a Network Error during routing', async () => {
      Object.defineProperty(globalThis, 'navigator', {
        value: {
          ...originalNavigator,
          onLine: true, // App thinks it is online, but network drops
        },
        writable: true,
      });

      globalThis.fetch = vi.fn().mockRejectedValue(new TypeError('Failed to fetch'));

      const origin: [number, number] = [19.076, 72.8777];
      const destination: [number, number] = [19.0896, 72.8656];

      const routeResult = await RouteService.calculateRoute({
        origin,
        destination,
        vehicleProfile: 'car',
      });

      expect(routeResult).toBeDefined();
      expect(routeResult.providerUrl).toBe('local:offline');
      expect(routeResult.selectedRoute.coordinates.length).toBeGreaterThan(0);
    });
  });

  describe('3. 60-Second GNSS-Denied + Internet-Off Live Navigation Simulation', () => {
    const origin: [number, number] = [19.076, 72.8777];

    it('simulates 60s in fusionMode="ekf": updates position smoothly without GNSS/Internet', () => {
      const adapter = new FusionAdapter();
      adapter.seedOrigin(origin[0], origin[1]);

      const totalDurationSec = 60;
      const dt = 0.1; // 10Hz
      const steps = Math.round(totalDurationSec / dt); // 600 steps

      let currentPos = [...origin] as [number, number];
      let currentHeading = 0; // Heading North (0 deg)
      let headingHistory: number[] = [];
      let positionHistory: [number, number][] = [];
      let speedHistory: number[] = [];
      let uncaughtErrors: string[] = [];

      // Vehicle Profile: Stationary (0-2s) -> Accel (2-12s) -> Cruise (12-45s) -> Decel (45-53s) -> Stop (53-60s)
      for (let i = 0; i < steps; i++) {
        const time = i * dt;
        let forwardAccel = 0; // m/s^2 along body Y
        let yawRate = 0; // deg/s

        if (time < 2) {
          forwardAccel = 0.0; // Stationary Alignment
        } else if (time < 12) {
          forwardAccel = 1.0; // Accelerating North (10s * 1.0 m/s^2 = 10 m/s)
        } else if (time < 45) {
          forwardAccel = 0.0; // Cruising North at 10 m/s (33s * 10 m/s = 330m)
        } else if (time < 53) {
          forwardAccel = -1.25; // Braking cleanly (8s * 1.25 m/s^2 = 10 m/s reduction -> 0 m/s)
        } else {
          forwardAccel = 0.0; // Stationary at stop for 7 seconds
        }

        // Sensor frame with gravity (Z = +9.81 m/s^2, Body Forward = Y)
        const sensorData: RealSensorData = {
          ax: 0.0,
          ay: forwardAccel,
          az: 9.81,
          accelMag: Math.sqrt(forwardAccel * forwardAccel + 9.81 * 9.81),
          gx: 0.0,
          gy: 0.0,
          gz: yawRate,
          gyroMag: Math.abs(yawRate),
          magX: null,
          magY: null,
          magZ: null,
          alpha: currentHeading,
          beta: 0,
          gamma: 0,
          headingDeg: currentHeading,
          intervalMs: 100,
          sampleRateHz: 10,
          timestamp: Date.now() + i * 100,
        };

        try {
          // GNSS is null (GNSS-Denied blackout condition)
          const estimate = adapter.step(sensorData, null, origin);

          if (isNaN(estimate.position[0]) || isNaN(estimate.position[1])) {
            uncaughtErrors.push(`Step ${i} (t=${time}s): Position contains NaN: [${estimate.position}]`);
          }
          if (estimate.position[0] === 0 && estimate.position[1] === 0) {
            uncaughtErrors.push(`Step ${i} (t=${time}s): Position collapsed to [0, 0]`);
          }

          currentPos = estimate.position;
          currentHeading = estimate.headingDeg;
          positionHistory.push(currentPos);
          headingHistory.push(currentHeading);
          speedHistory.push(estimate.velocitySpeedKmH);
        } catch (err: any) {
          uncaughtErrors.push(`Step ${i} (t=${time}s): Threw exception: ${err.message}`);
        }
      }

      // Compute total distance traveled
      const totalDistCovered = DeadReckoningEngine.calculateHaversineDistance(origin, currentPos) * 1000;

      console.log(`\n=== 60-SECOND EKF GNSS-DENIED SIMULATION ===`);
      console.log(`Total Steps Executed: ${steps} (10Hz)`);
      console.log(`Start Position: [${origin[0].toFixed(5)}, ${origin[1].toFixed(5)}]`);
      console.log(`End Position:   [${currentPos[0].toFixed(5)}, ${currentPos[1].toFixed(5)}]`);
      console.log(`Total Distance Traveled via DR: ${totalDistCovered.toFixed(2)} meters`);
      console.log(`Peak Speed: ${Math.max(...speedHistory).toFixed(1)} km/h`);
      console.log(`Final Stationary Speed: ${speedHistory[speedHistory.length - 1].toFixed(1)} km/h`);
      console.log(`Raw EKF INS Position:`, adapter.getRuntime().getEkf().getPosition());
      console.log(`Raw EKF INS Velocity:`, adapter.getRuntime().getEkf().getIns().velocity);
      console.log(`Uncaught Errors: ${uncaughtErrors.length}`);

      expect(uncaughtErrors).toEqual([]);
      expect(totalDistCovered).toBeGreaterThan(50); // Vehicle genuinely moved forward
      expect(totalDistCovered).toBeLessThan(500); // Bounded reasonable kinematic distance
      expect(speedHistory[speedHistory.length - 1]).toBeLessThan(0.5); // Stationary clamping on stop
    });

    it('simulates 60s in fusionMode="legacy": updates position smoothly without GNSS/Internet', () => {
      const totalDurationSec = 60;
      const dt = 0.1; // 10Hz
      const steps = Math.round(totalDurationSec / dt);

      let currentPos = [...origin] as [number, number];
      let prevPos: [number, number] | null = null;
      let fusedHeading = 0;
      let accumulatedDrift = 0;
      let currentSpeedKmH = 0;
      let positionHistory: [number, number][] = [];
      let uncaughtErrors: string[] = [];

      for (let i = 0; i < steps; i++) {
        const time = i * dt;
        let forwardAccel = 0;

        if (time < 2) {
          forwardAccel = 0.0;
          currentSpeedKmH = 0;
        } else if (time < 12) {
          forwardAccel = 1.0;
          currentSpeedKmH = Math.min(36, currentSpeedKmH + forwardAccel * dt * 3.6);
        } else if (time < 42) {
          forwardAccel = 0.0;
          currentSpeedKmH = 36;
        } else if (time < 52) {
          forwardAccel = -1.0;
          currentSpeedKmH = Math.max(0, currentSpeedKmH + forwardAccel * dt * 3.6);
        } else {
          forwardAccel = 0.0;
          currentSpeedKmH = 0;
        }

        try {
          fusedHeading = OrientationService.fuseHeading({
            magnetometerHeading: 0,
            gnssTrackBearing: null,
            trajectoryBearing: prevPos ? OrientationService.calculateBearing(prevPos, currentPos) : null,
            gyroZRate: 0,
            speedKmH: currentSpeedKmH,
            isGnssAvailable: false,
            deltaTimeSec: dt,
            previousHeading: fusedHeading,
          });

          const drStep = DeadReckoningEngine.stepKinematics(
            currentPos,
            currentSpeedKmH,
            forwardAccel,
            fusedHeading,
            dt,
            accumulatedDrift
          );

          if (isNaN(drStep.position[0]) || isNaN(drStep.position[1])) {
            uncaughtErrors.push(`Step ${i}: Position NaN`);
          }

          prevPos = currentPos;
          currentPos = drStep.position;
          accumulatedDrift = drStep.driftErrorMeters;
          positionHistory.push(currentPos);
        } catch (err: any) {
          uncaughtErrors.push(`Step ${i}: ${err.message}`);
        }
      }

      const totalDistCovered = DeadReckoningEngine.calculateHaversineDistance(origin, currentPos) * 1000;

      console.log(`\n=== 60-SECOND LEGACY DR GNSS-DENIED SIMULATION ===`);
      console.log(`Total Steps Executed: ${steps} (10Hz)`);
      console.log(`Start Position: [${origin[0].toFixed(5)}, ${origin[1].toFixed(5)}]`);
      console.log(`End Position:   [${currentPos[0].toFixed(5)}, ${currentPos[1].toFixed(5)}]`);
      console.log(`Total Distance Traveled via DR: ${totalDistCovered.toFixed(2)} meters`);
      console.log(`Accumulated Drift Estimate: ±${accumulatedDrift.toFixed(2)} m`);
      console.log(`Uncaught Errors: ${uncaughtErrors.length}`);

      expect(uncaughtErrors).toEqual([]);
      expect(totalDistCovered).toBeGreaterThan(50);
      expect(totalDistCovered).toBeLessThan(500);
    });

    it('simulates full lifecycle: GPS ON -> Blackout (GPS OFF) -> Reacquisition (GPS ON) with smooth navigation continuity', () => {
      const adapter = new FusionAdapter();
      adapter.seedOrigin(origin[0], origin[1]);

      const steps = 600; // 60s at 10Hz
      const dt = 0.1;

      let currentPos = [...origin] as [number, number];
      let groundTruthY = 0; // meters traveled North
      let speed = 10.0; // 10 m/s (36 km/h)
      let errorHistory: number[] = [];

      for (let i = 0; i < steps; i++) {
        const time = i * dt;
        groundTruthY += speed * dt;

        const isOnline1 = time < 10; // Phase 1: GPS ON (0-10s)
        const isReacquired = time >= 40; // Phase 3: GPS ON re-enabled (40-60s)

        // Generate sensor reading (cruising at constant 10 m/s North)
        const sensorData: RealSensorData = {
          ax: 0.0,
          ay: 0.0,
          az: 9.81,
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

        // GPS fix arrives at 1Hz when GPS is ON
        let gnssData: CurrentLocationData | null = null;
        if ((isOnline1 || isReacquired) && i % 10 === 0) {
          const gtLat = origin[0] + (groundTruthY / 6378137) * (180 / Math.PI);
          const gtLon = origin[1];

          gnssData = {
            latitude: gtLat,
            longitude: gtLon,
            accuracy: 3.0,
            altitude: 10,
            speed: 36.0,
            bearing: 0,
            timestamp: Date.now() + i * 100,
            address: 'Live Location',
            source: 'gps',
            isStale: false,
            ageSec: 0,
          };
        }

        const estimate = adapter.step(sensorData, gnssData, origin);
        currentPos = estimate.position;

        const currentGtLat = origin[0] + (groundTruthY / 6378137) * (180 / Math.PI);
        const errMeters = DeadReckoningEngine.calculateHaversineDistance(
          [currentGtLat, origin[1]],
          currentPos
        ) * 1000;
        errorHistory.push(errMeters);
      }

      const finalError = errorHistory[errorHistory.length - 1];
      console.log(`\n=== FULL REACQUISITION LIFECYCLE TEST ===`);
      console.log(`Ground Truth Distance Traveled: ${groundTruthY.toFixed(2)} meters`);
      console.log(`Final Fused Error after GPS Reacquisition: ${finalError.toFixed(2)} meters`);
      console.log(`Final Estimated Position:`, currentPos);
      console.log(`Final EKF Position:`, adapter.getRuntime().getEkf().getPosition());
      console.log(`Final EKF Velocity:`, adapter.getRuntime().getEkf().getIns().velocity);
      console.log(`Max Error during 30s Blackout: ${Math.max(...errorHistory).toFixed(2)} meters`);

      expect(finalError).toBeLessThan(5.0); // Smoothly converges back to GPS accuracy
      expect(Math.max(...errorHistory)).toBeLessThan(25.0); // Tight drift bound during blackout
    });
  });
});
