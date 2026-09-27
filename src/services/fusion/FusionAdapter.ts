import type { CurrentLocationData, RealSensorData } from '../../types/navigation';
import type { DRPositionEstimate } from '../deadReckoningEngine';
import { FusionRuntime, type FusedState, type FusionGnssInput, type FusionImuInput } from './FusionRuntime';

export class FusionAdapter {
  private runtime: FusionRuntime;

  constructor(runtime?: FusionRuntime) {
    this.runtime = runtime ?? new FusionRuntime();
  }

  public getRuntime(): FusionRuntime {
    return this.runtime;
  }

  public reset(): void {
    this.runtime.reset();
  }

  /**
   * Converts ReckonX CurrentLocationData into FusionGnssInput.
   * Handles unit conversion:
   * - speed (km/h) -> speed (m/s)
   */
  public adaptGnss(location: CurrentLocationData | null): FusionGnssInput | null {
    if (!location || location.latitude === null || location.longitude === null) {
      return null;
    }

    const speedMps =
      location.speed !== null && !isNaN(location.speed)
        ? (location.speed * 1000) / 3600
        : null;

    return {
      latitude: location.latitude,
      longitude: location.longitude,
      accuracy: location.accuracy,
      altitude: location.altitude,
      speed: speedMps,
      heading: location.bearing,
      timestamp: location.timestamp ?? Date.now(),
    };
  }

  /**
   * Converts ReckonX RealSensorData into FusionImuInput.
   * Handles unit conversion:
   * - Gyroscope gx, gy, gz (deg/s) -> (rad/s)
   */
  public adaptImu(sensor: RealSensorData): FusionImuInput {
    const DEG_TO_RAD = Math.PI / 180;
    return {
      accel: {
        x: sensor.ax,
        y: sensor.ay,
        z: sensor.az,
      },
      gyro: {
        x: sensor.gx * DEG_TO_RAD,
        y: sensor.gy * DEG_TO_RAD,
        z: sensor.gz * DEG_TO_RAD,
      },
      timestamp: sensor.timestamp ?? Date.now(),
    };
  }

  /**
   * Steps the fusion engine with ReckonX sensor and optional location data,
   * returning ReckonX's standard DRPositionEstimate.
   */
  public step(
    sensor: RealSensorData,
    location?: CurrentLocationData | null
  ): DRPositionEstimate {
    const imuInput = this.adaptImu(sensor);
    const gnssInput = location ? this.adaptGnss(location) : null;

    const fusedState = this.runtime.step(imuInput, gnssInput);
    return this.toDrEstimate(fusedState, location);
  }

  /**
   * Converts internal FusedState to ReckonX DRPositionEstimate.
   */
  public toDrEstimate(
    fused: FusedState,
    fallbackLocation?: CurrentLocationData | null
  ): DRPositionEstimate {
    // 3D velocity to speed in km/h
    const vel = fused.velocity;
    const speedMps = Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
    const velocitySpeedKmH = Math.round(((speedMps * 3600) / 1000) * 10) / 10;

    let position: [number, number];
    if (fused.latitude !== null && fused.longitude !== null) {
      position = [fused.latitude, fused.longitude];
    } else if (
      fallbackLocation &&
      fallbackLocation.latitude !== null &&
      fallbackLocation.longitude !== null
    ) {
      position = [fallbackLocation.latitude, fallbackLocation.longitude];
    } else {
      position = [0, 0];
    }

    return {
      position,
      velocitySpeedKmH,
      driftErrorMeters: fused.driftMeters ?? 0,
      headingDeg: Math.round(fused.heading * 10) / 10,
      isZuptActive: fused.isZuptActive,
    };
  }
}

export const fusionAdapter = new FusionAdapter();
