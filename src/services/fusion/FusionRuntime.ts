import { EkfCore } from './EkfCore';
import { InsMechanization } from './InsMechanization';
import { OutputStabilizer, type StabilizerInput } from './OutputStabilizer';

export interface FusedState {
  latitude: number | null;
  longitude: number | null;
  velocity: { x: number; y: number; z: number };
  heading: number;
  timestamp: number;
  sourceMode: 'GNSS' | 'IDR';
  gnssState: string;
  pureInsLatitude: number | null;
  pureInsLongitude: number | null;
  accelBias: { x: number; y: number; z: number };
  gyroBias: { x: number; y: number; z: number };
  isAligned: boolean;
  isZuptActive?: boolean;
  driftMeters?: number;
  stabilization?: {
    wasClamped: boolean;
    wasSmoothed: boolean;
    clampEvent?: any;
  };
}

export interface FusionGnssInput {
  latitude: number | null;
  longitude: number | null;
  accuracy: number | null; // horizontal accuracy in meters
  altitude?: number | null;
  speed?: number | null; // in m/s
  heading?: number | null; // in degrees
  timestamp?: number | null;
}

export interface FusionImuInput {
  accel: { x: number; y: number; z: number }; // in m/s^2
  gyro: { x: number; y: number; z: number }; // in rad/s
  timestamp?: number;
}

export class FusionRuntime {
  private ekf: EkfCore;
  private pureIns: InsMechanization;
  private outputStabilizer: OutputStabilizer;
  private isRunning = false;

  private imuWindow: number[][] = [];
  private lastImuTimestamp: number = 0;
  private lastGnssTimestamp: number = 0;
  private lastGnssSpeed: number | null = null;

  private initialLat: number | null = null;
  private initialLon: number | null = null;
  private isAttitudeInitialized = false;

  // ZUPT state tracking
  private wasZuptActiveLastCycle = false;

  // Earth radius in meters
  private readonly R_EARTH = 6378137;

  private latestFusedState: FusedState | null = null;
  private onFusedDataCallback: ((state: FusedState) => void) | null = null;

  constructor() {
    this.ekf = new EkfCore();
    this.pureIns = new InsMechanization();
    this.outputStabilizer = new OutputStabilizer({ maxSpeedMps: 33.3, maxAccelMps2: 9.8 });
    this.reset();
  }

  public start(): void {
    this.isRunning = true;
  }

  public stop(): void {
    this.isRunning = false;
  }

  public isActive(): boolean {
    return this.isRunning;
  }

  public reset(): void {
    this.isRunning = false;
    this.ekf = new EkfCore();
    this.pureIns = new InsMechanization();
    this.outputStabilizer = new OutputStabilizer({ maxSpeedMps: 33.3, maxAccelMps2: 9.8 });
    this.imuWindow = [];
    for (let i = 0; i < 20; i++) {
      this.imuWindow.push([0, 0, 9.81, 0, 0, 0]);
    }
    this.lastImuTimestamp = 0;
    this.lastGnssTimestamp = 0;
    this.lastGnssSpeed = null;
    this.initialLat = null;
    this.initialLon = null;
    this.isAttitudeInitialized = false;
    this.wasZuptActiveLastCycle = false;
    this.latestFusedState = null;
  }

  public setOnFusedDataCallback(callback: (state: FusedState) => void) {
    this.onFusedDataCallback = callback;
  }

  public getLatestFusedState(): FusedState | null {
    return this.latestFusedState;
  }

  public seedInitialPosition(lat: number, lon: number): void {
    if (this.initialLat === null || this.initialLon === null) {
      this.initialLat = lat;
      this.initialLon = lon;
      this.pureIns.position = { x: 0, y: 0, z: 0 };
    }
  }

  public getEkf(): EkfCore {
    return this.ekf;
  }

  public handleGnssUpdate(gnss: FusionGnssInput): void {
    if (gnss.latitude === null || gnss.longitude === null) return;

    // A3 - Stale fix rejection: reject fixes older than 2.0s relative to fusion timeline
    if (gnss.timestamp) {
      const currentTimeline = Math.max(this.lastImuTimestamp, this.lastGnssTimestamp);
      if (currentTimeline > 0 && currentTimeline - gnss.timestamp > 2000) {
        return;
      }
    }

    if (gnss.speed != null && !isNaN(gnss.speed)) {
      this.lastGnssSpeed = gnss.speed;
    }

    // Estimate velocity from GNSS speed/heading if available (speed in m/s)
    let gnssVel: number[] | null = null;
    if (gnss.speed != null && gnss.heading != null && !isNaN(gnss.speed) && !isNaN(gnss.heading)) {
      gnssVel = [0, 0, 0];
      const hdgRad = gnss.heading * (Math.PI / 180);
      gnssVel[0] = gnss.speed * Math.sin(hdgRad); // East
      gnssVel[1] = gnss.speed * Math.cos(hdgRad); // North

      if (gnss.speed > 0.5) {
        this.ekf.getIns().attitude.yaw = -hdgRad;
        this.pureIns.attitude.yaw = -hdgRad;
        if (!this.isAttitudeInitialized) {
          this.ekf.getIns().initializeAttitude({ x: 0, y: 0, z: 9.81 });
          this.pureIns.initializeAttitude({ x: 0, y: 0, z: 9.81 });
          this.ekf.getIns().attitude.yaw = -hdgRad;
          this.pureIns.attitude.yaw = -hdgRad;
          this.ekf.notifyAttitudeInitialized();
          this.isAttitudeInitialized = true;
        }
      }
    }

    if (this.initialLat === null || this.initialLon === null) {
      this.initialLat = gnss.latitude;
      this.initialLon = gnss.longitude;
      this.pureIns.position = { x: 0, y: 0, z: 0 };
      this.pureIns.velocity = gnssVel ? { x: gnssVel[0], y: gnssVel[1], z: gnssVel[2] } : { x: 0, y: 0, z: 0 };
      if (gnssVel) {
        this.ekf.getIns().velocity = { x: gnssVel[0], y: gnssVel[1], z: gnssVel[2] };
      }
    } else if (gnssVel && gnss.speed && gnss.speed > 0.5) {
      const curSpeedSq = this.ekf.getIns().velocity.x ** 2 + this.ekf.getIns().velocity.y ** 2;
      if (curSpeedSq < 0.25) {
        this.ekf.getIns().velocity = { x: gnssVel[0], y: gnssVel[1], z: gnssVel[2] };
        this.pureIns.velocity = { x: gnssVel[0], y: gnssVel[1], z: gnssVel[2] };
      }
    }

    // Convert GNSS lat/lon to local ENU
    const latRad = this.initialLat * (Math.PI / 180);
    const dx =
      (gnss.longitude - this.initialLon) * (Math.PI / 180) * this.R_EARTH * Math.cos(latRad);
    const dy = (gnss.latitude - this.initialLat) * (Math.PI / 180) * this.R_EARTH;

    const gnssPos = [dx, dy, 0];

    if (gnssVel !== null && (gnssVel[0] !== 0 || gnssVel[1] !== 0)) {
      if (this.wasZuptActiveLastCycle) {
        this.ekf.notifyZuptReleased();
        this.wasZuptActiveLastCycle = false;
      }
    }

    this.lastGnssTimestamp = gnss.timestamp ?? Date.now();
    this.ekf.updateGnss(gnssPos, gnssVel, gnss.accuracy, this.imuWindow);
  }

  public handleImuUpdate(imu: FusionImuInput): FusedState {
    const now = imu.timestamp ?? Date.now();
    let dt = (now - this.lastImuTimestamp) / 1000.0;
    if (this.lastImuTimestamp === 0) {
      dt = 0.1; // Default 100ms on first run
    } else if (dt <= 0 || dt > 1.0) {
      dt = 0.1;
    }
    this.lastImuTimestamp = now;

    // DEBUG-INSTRUMENT: Raw IMU sample arrival (timestamp, accel, gyro, interval dt)
    // Sample vector [ax, ay, az, gx, gy, gz]
    const imuSample = [
      imu.accel.x,
      imu.accel.y,
      imu.accel.z,
      imu.gyro.x,
      imu.gyro.y,
      imu.gyro.z,
    ];

    this.imuWindow.push(imuSample);
    while (this.imuWindow.length > 20) {
      this.imuWindow.shift();
    }

    // Physical ZUPT / Stationary Detection
    // Thresholds: | |a| - 9.81 | < 0.25 m/s², |w| < 0.05 rad/s, speed < 0.5 m/s
    const accelMag = Math.sqrt(imu.accel.x * imu.accel.x + imu.accel.y * imu.accel.y + imu.accel.z * imu.accel.z);
    const gyroMag = Math.sqrt(imu.gyro.x * imu.gyro.x + imu.gyro.y * imu.gyro.y + imu.gyro.z * imu.gyro.z);
    const currentVel = this.ekf.getIns().velocity;
    const currentSpeed = Math.sqrt(currentVel.x * currentVel.x + currentVel.y * currentVel.y + currentVel.z * currentVel.z);

    // Initialize attitude on first valid sample
    if (!this.isAttitudeInitialized) {
      if (Math.abs(accelMag - 9.81) < 2.0) {
        const initialAccel = { x: imu.accel.x, y: imu.accel.y, z: imu.accel.z };
        this.ekf.getIns().initializeAttitude(initialAccel);
        this.pureIns.initializeAttitude(initialAccel);
        this.ekf.notifyAttitudeInitialized();
        this.isAttitudeInitialized = true;
      } else {
        return this.emitFusedState(false, now);
      }
    }

    // Leveled Navigation-Frame Horizontal Linear Acceleration
    const C_b_n = this.ekf.getIns().lastRotationMatrix;
    const a_nx = C_b_n[0][0] * imu.accel.x + C_b_n[0][1] * imu.accel.y + C_b_n[0][2] * imu.accel.z;
    const a_ny = C_b_n[1][0] * imu.accel.x + C_b_n[1][1] * imu.accel.y + C_b_n[1][2] * imu.accel.z;
    const horizLinearAccel = Math.sqrt(a_nx * a_nx + a_ny * a_ny);

    let isStationary = false;
    if (this.imuWindow.length >= 20) {
      const n = this.imuWindow.length;
      let sumAx = 0, sumAy = 0, sumAz = 0;
      let sumGx = 0, sumGy = 0, sumGz = 0;
      for (const s of this.imuWindow) {
        sumAx += s[0]; sumAy += s[1]; sumAz += s[2];
        sumGx += s[3]; sumGy += s[4]; sumGz += s[5];
      }
      const meanAx = sumAx / n, meanAy = sumAy / n, meanAz = sumAz / n;
      const meanGx = sumGx / n, meanGy = sumGy / n, meanGz = sumGz / n;

      let varA = 0, varG = 0;
      for (const s of this.imuWindow) {
        varA += Math.pow(s[0] - meanAx, 2) + Math.pow(s[1] - meanAy, 2) + Math.pow(s[2] - meanAz, 2);
        varG += Math.pow(s[3] - meanGx, 2) + Math.pow(s[4] - meanGy, 2) + Math.pow(s[5] - meanGz, 2);
      }
      varA /= n;
      varG /= n;

      // Physical stationary detection:
      // 1. Total acceleration magnitude close to 1g (9.81 m/s²), invariant to phone tilt/orientation
      // 2. Leveled horizontal linear acceleration in navigation frame < 0.35 m/s²
      // 3. Gyroscope rotation magnitude near zero (< 0.08 rad/s)
      // 4. Acceleration & angular rate variances over rolling 20-sample window are small (pure sensor noise)
      // 5. Vehicle is not currently moving according to fresh GNSS fixes
      const isLowVariance = varA < 0.35 && varG < 0.05;
      const isGravityMagnitude = Math.abs(accelMag - 9.81) < 0.50;
      const isHorizQuiet = horizLinearAccel < 0.35;
      const isGyroStationary = gyroMag < 0.08;
      const isLowVelocity = currentSpeed < 0.8;
      const isGnssMoving = this.lastGnssSpeed !== null && this.lastGnssSpeed > 0.8 && (now - this.lastGnssTimestamp < 2000);

      isStationary = isLowVariance && isGravityMagnitude && isHorizQuiet && isGyroStationary && isLowVelocity && !isGnssMoving;
    }

    // EKF Predict Step
    this.ekf.predict(
      dt,
      [imuSample[0], imuSample[1], imuSample[2]],
      [imuSample[3], imuSample[4], imuSample[5]]
    );

    // Pure INS Predict Step
    this.pureIns.predict(
      dt,
      { x: imuSample[0], y: imuSample[1], z: imuSample[2] },
      { x: imuSample[3], y: imuSample[4], z: imuSample[5] }
    );

    if (this.imuWindow.length >= 20) {
      if (isStationary) {
        // Apply ZUPT to EKF and zero pure INS velocity to prevent stationary integration creep
        this.ekf.updateZupt();
        this.pureIns.velocity = { x: 0, y: 0, z: 0 };
        this.wasZuptActiveLastCycle = true;
      } else {
        if (this.wasZuptActiveLastCycle) {
          this.ekf.notifyZuptReleased();
          this.wasZuptActiveLastCycle = false;
        }

        // Apply continuous NHC during dead reckoning / GNSS blackout at IMU rate
        const isGnssOutage = this.lastGnssTimestamp === 0 || (now - this.lastGnssTimestamp > 1000);
        if (isGnssOutage && this.isAttitudeInitialized) {
          this.ekf.applyNhc(0.05, 0.01);
        }
      }
    }

    // Trigger IDR mode state transition if GNSS is lost for > 2 seconds
    if (this.lastGnssTimestamp > 0 && now - this.lastGnssTimestamp > 2000) {
      this.ekf.updateGnss([0, 0, 0], [0, 0, 0], null, this.imuWindow);
      this.lastGnssTimestamp = now - 1000;
    }

    return this.emitFusedState(isStationary, now);
  }

  public step(
    imu: FusionImuInput,
    gnss?: FusionGnssInput | null
  ): FusedState {
    if (gnss && gnss.latitude !== null && gnss.longitude !== null) {
      this.handleGnssUpdate(gnss);
    }
    return this.handleImuUpdate(imu);
  }

  private emitFusedState(isZuptActive: boolean, timestamp: number): FusedState {
    const pos = this.ekf.getPosition();
    const vel = this.ekf.getIns().velocity;
    const att = this.ekf.getIns().attitude;

    let fusedLat: number | null = null;
    let fusedLon: number | null = null;
    let pureInsLat: number | null = null;
    let pureInsLon: number | null = null;

    if (this.initialLat !== null && this.initialLon !== null) {
      const latRad = this.initialLat * (Math.PI / 180);
      fusedLat = this.initialLat + (pos.y / this.R_EARTH) * (180 / Math.PI);
      fusedLon =
        this.initialLon + (pos.x / (this.R_EARTH * Math.cos(latRad))) * (180 / Math.PI);

      pureInsLat = this.initialLat + (this.pureIns.position.y / this.R_EARTH) * (180 / Math.PI);
      pureInsLon =
        this.initialLon +
        (this.pureIns.position.x / (this.R_EARTH * Math.cos(latRad))) * (180 / Math.PI);
    }

    const state = this.ekf.getGnssState().getState();
    const sourceMode = state === 'GOOD' || state === 'DEGRADED' ? 'GNSS' : 'IDR';

    let headingDeg = -att.yaw * (180 / Math.PI);
    headingDeg = ((headingDeg % 360) + 360) % 360;

    let stabilizationInfo = undefined;
    if (this.initialLat !== null && fusedLat !== null && fusedLon !== null) {
      const stabilizerInput: StabilizerInput = {
        lat: fusedLat,
        lon: fusedLon,
        timestamp,
        gnssState: state,
      };
      const stabilized = this.outputStabilizer.process(stabilizerInput);
      fusedLat = stabilized.lat;
      fusedLon = stabilized.lon;
      stabilizationInfo = {
        wasClamped: stabilized.wasClamped,
        wasSmoothed: stabilized.wasSmoothed,
        clampEvent: stabilized.clampEvent,
      };
    }

    const biases = this.ekf.getBiases
      ? this.ekf.getBiases()
      : { accel: { x: 0, y: 0, z: 0 }, gyro: { x: 0, y: 0, z: 0 } };

    // Calculate local drift relative to pure INS
    const driftMeters =
      pureInsLat !== null && pureInsLon !== null && fusedLat !== null && fusedLon !== null
        ? Math.sqrt(
            Math.pow(pos.x - this.pureIns.position.x, 2) +
              Math.pow(pos.y - this.pureIns.position.y, 2)
          )
        : 0;

    const fusedStateResult: FusedState = {
      latitude: fusedLat,
      longitude: fusedLon,
      velocity: vel,
      heading: headingDeg,
      timestamp,
      sourceMode,
      gnssState: state,
      pureInsLatitude: pureInsLat,
      pureInsLongitude: pureInsLon,
      accelBias: biases.accel,
      gyroBias: biases.gyro,
      isAligned: this.isAttitudeInitialized,
      isZuptActive,
      driftMeters: Math.round(driftMeters * 100) / 100,
      stabilization: stabilizationInfo,
    };

    this.latestFusedState = fusedStateResult;

    if (this.onFusedDataCallback) {
      this.onFusedDataCallback(fusedStateResult);
    }

    return fusedStateResult;
  }
}

export const fusionRuntime = new FusionRuntime();
