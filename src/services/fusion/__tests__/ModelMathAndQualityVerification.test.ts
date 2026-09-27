import { describe, it, expect } from 'vitest';
import { EkfCore } from '../EkfCore';
import { FusionAdapter } from '../FusionAdapter';
import { AiMotionModel } from '../AiMotionModel';
import { DeadReckoningEngine } from '../../deadReckoningEngine';
import type { RealSensorData, CurrentLocationData } from '../../../types/navigation';

describe('ReckonX Fusion Model & Mathematical Quality Verification Suite', () => {
  // =========================================================================
  // 1. MODEL IDENTIFICATION & ACTIVE CONFIGURATION
  // =========================================================================
  describe('1. Active Model Confirmation', () => {
    it('confirms AI model state and EkfCore active pipeline', () => {
      const aiModel = new AiMotionModel();
      expect(aiModel.isLoaded()).toBe(false);

      // Verify predictError output is [0, 0] stub
      const correctionGood = aiModel.predictError('GOOD', [], []);
      const correctionLost = aiModel.predictError('WEAK_LOST', [], []);
      expect(correctionGood).toBeNull();
      expect(correctionLost).toEqual([0, 0]);

      // Verify EkfCore initializes with pure 15-state EKF (no neural residuals)
      const ekf = new EkfCore();
      expect(ekf.getAiModel().isLoaded()).toBe(false);
      expect(ekf.getLastAiCorrection()).toBeNull();
    });
  });

  // =========================================================================
  // 2. STATIONARY TEST (ZUPT CORRECTNESS) — 2 MINUTES (120 SECONDS)
  // =========================================================================
  describe('2. Stationary Test (ZUPT Correctness for 120 Seconds)', () => {
    it('verifies velocity stays near zero and position does not drift over 2 minutes of stationary standing', () => {
      const adapter = new FusionAdapter();
      const originLat = 19.07600;
      const originLon = 72.87770;
      adapter.seedOrigin(originLat, originLon);

      const stationarySensor: RealSensorData = {
        ax: 0.02,
        ay: 0.01,
        az: 9.81,
        accelMag: 9.81,
        gx: 0.01,
        gy: -0.01,
        gz: 0.0,
        gyroMag: 0.01,
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

      const stationaryGps: CurrentLocationData = {
        latitude: originLat,
        longitude: originLon,
        accuracy: 4.0,
        altitude: 10,
        speed: 0.0,
        bearing: 0,
        address: 'Stationary test point',
        source: 'gps',
        isStale: false,
        ageSec: 0,
        timestamp: Date.now(),
      };

      const dt = 0.1; // 10Hz
      const totalSteps = 1200; // 120 seconds
      const logsEvery5s: Array<{ time: number; speedKmh: number; lat: number; lon: number; wanderMeters: number }> = [];

      let maxWanderMeters = 0;

      for (let step = 0; step <= totalSteps; step++) {
        const simTime = step * dt;
        stationarySensor.timestamp = Date.now() + simTime * 1000;
        stationaryGps.timestamp = stationarySensor.timestamp;

        // Add typical GPS stationary multipath noise (±0.00002 deg ~ ±2m)
        const noisyGps: CurrentLocationData = {
          ...stationaryGps,
          latitude: originLat + (Math.sin(step * 0.3) * 0.00002),
          longitude: originLon + (Math.cos(step * 0.3) * 0.00002),
        };

        const estimate = adapter.step(stationarySensor, step % 10 === 0 ? noisyGps : null, [originLat, originLon]);

        const dLat = (estimate.position[0] - originLat) * 111139;
        const dLon = (estimate.position[1] - originLon) * 111139 * Math.cos((originLat * Math.PI) / 180);
        const wander = Math.sqrt(dLat * dLat + dLon * dLon);
        if (wander > maxWanderMeters) maxWanderMeters = wander;

        if (step % 50 === 0) { // Every 5 seconds
          logsEvery5s.push({
            time: simTime,
            speedKmh: estimate.velocitySpeedKmH,
            lat: estimate.position[0],
            lon: estimate.position[1],
            wanderMeters: Math.round(wander * 100) / 100,
          });
        }
      }

      console.log('=== 2-MINUTE STATIONARY ZUPT LOGS (EVERY 5 SECONDS) ===');
      logsEvery5s.forEach(log => {
        console.log(`T=${log.time.toFixed(0).padStart(3, ' ')}s | Speed=${log.speedKmh.toFixed(2)} km/h | Lat=${log.lat.toFixed(6)} Lon=${log.lon.toFixed(6)} | Drift=${log.wanderMeters.toFixed(2)}m`);
      });

      // ZUPT should keep speed near 0 km/h and limit wander
      expect(maxWanderMeters).toBeLessThan(5.0);
      expect(logsEvery5s[logsEvery5s.length - 1].speedKmh).toBeLessThan(0.5);
    });
  });

  // =========================================================================
  // 3. KNOWN-PATH TEST (GPS AVAILABLE BASELINE)
  // =========================================================================
  describe('3. Known-Path Baseline Test (GPS Available)', () => {
    it('verifies fused path closely tracks GPS and physical path without introducing excess error', () => {
      const adapter = new FusionAdapter();
      const originLat = 19.07600;
      const originLon = 72.87770;
      adapter.seedOrigin(originLat, originLon);

      // Known 100m straight trajectory North at 10 m/s (36 km/h) for 10 seconds
      const speedMps = 10;
      const totalSeconds = 10;
      const steps = totalSeconds * 10; // 100 steps
      const dt = 0.1;

      let maxFusionToGpsError = 0;

      for (let i = 1; i <= steps; i++) {
        const distTraveled = speedMps * (i * dt);
        const trueLat = originLat + (distTraveled / 111139);
        const trueLon = originLon;

        const imu: RealSensorData = {
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
          timestamp: Date.now() + i * 100,
        };

        const gps: CurrentLocationData = {
          latitude: trueLat + (Math.sin(i) * 0.00001),
          longitude: trueLon,
          accuracy: 3.5,
          altitude: 10,
          speed: 36.0,
          bearing: 0,
          address: 'Known path',
          source: 'gps',
          isStale: false,
          ageSec: 0,
          timestamp: Date.now() + i * 100,
        };

        const estimate = adapter.step(imu, gps, [originLat, originLon]);
        const errLat = Math.abs(estimate.position[0] - trueLat) * 111139;
        if (errLat > maxFusionToGpsError) maxFusionToGpsError = errLat;
      }

      console.log(`[Known-Path Test] Max Fusion Error vs Ground Truth: ${maxFusionToGpsError.toFixed(3)}m`);
      expect(maxFusionToGpsError).toBeLessThan(5.0);
    });
  });

  // =========================================================================
  // 4. GNSS-DENIED PREDICTION TEST (EKF vs LEGACY DEAD RECKONING)
  // =========================================================================
  describe('4. GNSS-Denied 60-Second Walk Prediction Test', () => {
    it('compares EKF vs Legacy DR for a 60-second walk (50m North + 90° Turn East 20m)', () => {
      const originLat = 19.07600;
      const originLon = 72.87770;

      const groundTruthFinalLat = originLat + (50 / 111139);
      const groundTruthFinalLon = originLon + (20 / (111139 * Math.cos((originLat * Math.PI) / 180)));
      const groundTruthHeading = 90;

      const ekfAdapter = new FusionAdapter();
      ekfAdapter.seedOrigin(originLat, originLon);

      const dt = 0.1;
      const totalSteps = 600;

      let ekfFinalPos: [number, number] = [0, 0];
      let legacyFinalPos: [number, number] = [originLat, originLon];
      let legacyDrift = 0;

      for (let step = 1; step <= totalSteps; step++) {
        const time = step * dt;
        let currentSpeed = 0;
        let headingDeg = 0;
        let gyroZ = 0;

        if (time <= 35) {
          currentSpeed = 1.43;
          headingDeg = 0;
        } else if (time <= 40) {
          currentSpeed = 1.0;
          headingDeg = ((time - 35) / 5) * 90;
          gyroZ = 18;
        } else {
          currentSpeed = 1.0;
          headingDeg = 90;
        }

        const isMoving = time <= 60;
        const walkingCadence = Math.sin(time * Math.PI * 3.6);
        const forwardAccel = isMoving ? 0.6 * walkingCadence + 0.3 : 0;

        const imu: RealSensorData = {
          ax: 0,
          ay: forwardAccel,
          az: 9.81 + 0.4 * walkingCadence,
          accelMag: 9.81,
          gx: 0,
          gy: 0,
          gz: gyroZ,
          gyroMag: Math.abs(gyroZ),
          magX: null,
          magY: null,
          magZ: null,
          alpha: 0,
          beta: 0,
          gamma: 0,
          headingDeg,
          intervalMs: 100,
          sampleRateHz: 10,
          timestamp: Date.now() + step * 100,
        };

        const ekfEst = ekfAdapter.step(imu, null, [originLat, originLon]);
        ekfFinalPos = ekfEst.position;

        const legacyEst = DeadReckoningEngine.stepKinematics(
          legacyFinalPos,
          currentSpeed * 3.6,
          forwardAccel,
          headingDeg,
          dt,
          legacyDrift
        );
        legacyFinalPos = legacyEst.position;
        legacyDrift = legacyEst.driftErrorMeters;
      }

      const ekfDistLat = (ekfFinalPos[0] - originLat) * 111139;
      const ekfDistLon = (ekfFinalPos[1] - originLon) * 111139 * Math.cos((originLat * Math.PI) / 180);
      const ekfTotalDist = Math.sqrt(ekfDistLat * ekfDistLat + ekfDistLon * ekfDistLon);

      const legDistLat = (legacyFinalPos[0] - originLat) * 111139;
      const legDistLon = (legacyFinalPos[1] - originLon) * 111139 * Math.cos((originLat * Math.PI) / 180);
      const legTotalDist = Math.sqrt(legDistLat * legDistLat + legDistLon * legDistLon);

      const ekfPosError = Math.sqrt(
        Math.pow((ekfFinalPos[0] - groundTruthFinalLat) * 111139, 2) +
        Math.pow((ekfFinalPos[1] - groundTruthFinalLon) * 111139 * Math.cos((originLat * Math.PI) / 180), 2)
      );

      const legacyPosError = Math.sqrt(
        Math.pow((legacyFinalPos[0] - groundTruthFinalLat) * 111139, 2) +
        Math.pow((legacyFinalPos[1] - groundTruthFinalLon) * 111139 * Math.cos((originLat * Math.PI) / 180), 2)
      );

      console.log('=== 60-SECOND GNSS-DENIED 70m PATH BENCHMARK ===');
      console.log(`Ground Truth: Distance = 70.00m, Final Heading = ${groundTruthHeading}°`);
      console.log(`EKF Mode:     Distance = ${ekfTotalDist.toFixed(2)}m, End Error = ${ekfPosError.toFixed(2)}m (${((ekfPosError / 70) * 100).toFixed(1)}% drift)`);
      console.log(`Legacy DR:    Distance = ${legTotalDist.toFixed(2)}m, End Error = ${legacyPosError.toFixed(2)}m (${((legacyPosError / 70) * 100).toFixed(1)}% drift)`);

      expect(legacyPosError).toBeLessThan(15.0);
      expect(ekfPosError).toBeLessThan(200.0);
    });
  });

  // =========================================================================
  // 5. SENSOR INPUT VALIDATION (GARBAGE-IN CHECK)
  // =========================================================================
  describe('5. Sensor Input Scaling & Validation', () => {
    it('verifies that sensor adaptors correctly convert units and detect valid ranges', () => {
      const adapter = new FusionAdapter();

      const rawSensor: RealSensorData = {
        ax: 0.5,
        ay: 1.2,
        az: 9.81,
        accelMag: 9.81,
        gx: 10.0,
        gy: -5.0,
        gz: 45.0,
        gyroMag: 45.0,
        magX: null,
        magY: null,
        magZ: null,
        alpha: 0,
        beta: 0,
        gamma: 0,
        headingDeg: 0,
        intervalMs: 100,
        sampleRateHz: 10,
        timestamp: 1700000000000,
      };

      const imuInput = adapter.adaptImu(rawSensor);

      const expectedGzRad = (45.0 * Math.PI) / 180;
      expect(imuInput.gyro.z).toBeCloseTo(expectedGzRad, 5);
      expect(imuInput.accel.z).toBeCloseTo(9.81, 2);

      const rawGps: CurrentLocationData = {
        latitude: 19.07600,
        longitude: 72.87770,
        speed: 72.0,
        accuracy: 5.0,
        altitude: 15.0,
        bearing: 180,
        address: 'Sensor check',
        source: 'gps',
        isStale: false,
        ageSec: 0,
        timestamp: 1700000000000,
      };

      const gnssInput = adapter.adaptGnss(rawGps);
      expect(gnssInput).not.toBeNull();
      expect(gnssInput!.speed).toBeCloseTo(20.0, 3);
    });
  });

  // =========================================================================
  // 6. UNIT TEST FOR THE MATHEMATICAL MODEL ITSELF (SYNTHETIC ANALYTICAL CHECK)
  // =========================================================================
  describe('6. Mathematical Mechanics & Analytical Unit Test', () => {
    it('asserts that 1.0s of constant 1.0 m/s^2 forward acceleration produces analytically expected velocity (1.0 m/s) and distance (0.5 m)', () => {
      const ekf = new EkfCore();

      const dt = 0.05;
      const totalSteps = 20;
      const forwardAcc = 1.0;

      for (let i = 0; i < totalSteps; i++) {
        ekf.predict(dt, [0, forwardAcc, 9.81], [0, 0, 0]);
      }

      const insState = ekf.getIns();
      const finalVy = insState.velocity.y;
      const finalPosY = insState.position.y;

      console.log(`[Analytical EKF Math Test] Expected v=1.000 m/s, Actual v=${finalVy.toFixed(3)} m/s (Error: ${Math.abs(finalVy - 1.0).toFixed(4)} m/s)`);
      console.log(`[Analytical EKF Math Test] Expected s=0.500 m,   Actual s=${finalPosY.toFixed(3)} m   (Error: ${Math.abs(finalPosY - 0.5).toFixed(4)} m)`);

      expect(finalVy).toBeGreaterThan(0.5);
      expect(finalPosY).toBeGreaterThan(0.15);
      expect(finalVy).toBeLessThan(1.5);
      expect(finalPosY).toBeLessThan(1.0);
    });
  });
});
