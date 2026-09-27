import { describe, it, expect, beforeEach } from 'vitest';
import { FusionAdapter } from '../FusionAdapter';
import type { CurrentLocationData, RealSensorData } from '../../../types/navigation';

describe('FusionAdapter & EKF Fusion Engine Sanity Check', () => {
  let adapter: FusionAdapter;

  beforeEach(() => {
    adapter = new FusionAdapter();
    adapter.reset();
  });

  it('initializes and produces DRPositionEstimate from stationary IMU and GNSS fix', () => {
    const fixedLat = 19.0760;
    const fixedLng = 72.8777;

    const mockGnss: CurrentLocationData = {
      latitude: fixedLat,
      longitude: fixedLng,
      accuracy: 3.5,
      altitude: 15.0,
      speed: 0.0,
      bearing: 90.0,
      timestamp: 1000,
      address: 'Mumbai Central',
      source: 'gps',
      isStale: false,
      ageSec: 0,
    };

    const mockImuStationary: RealSensorData = {
      ax: 0.0,
      ay: 0.0,
      az: 9.81,
      accelMag: 9.81,
      gx: 0.0,
      gy: 0.0,
      gz: 0.0,
      gyroMag: 0.0,
      alpha: 90.0,
      beta: 0.0,
      gamma: 0.0,
      headingDeg: 90.0,
      magX: null,
      magY: null,
      magZ: null,
      timestamp: 1000,
      intervalMs: 100,
      sampleRateHz: 10,
    };

    // Feed 25 stationary samples to complete initial stationary alignment and window buffering
    let estimate = adapter.step(mockImuStationary, mockGnss);

    for (let i = 1; i <= 25; i++) {
      const imuSample: RealSensorData = {
        ...mockImuStationary,
        timestamp: 1000 + i * 100,
      };
      const gnssFix: CurrentLocationData = {
        ...mockGnss,
        timestamp: 1000 + i * 100,
      };
      estimate = adapter.step(imuSample, gnssFix);
    }

    expect(estimate).toBeDefined();
    expect(estimate.position).toBeDefined();
    expect(estimate.position[0]).toBeCloseTo(fixedLat, 3);
    expect(estimate.position[1]).toBeCloseTo(fixedLng, 3);
    expect(estimate.velocitySpeedKmH).toBeLessThanOrEqual(1.0);
    expect(estimate.isZuptActive).toBe(true);
  });
});
