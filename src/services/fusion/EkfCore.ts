import { Matrix, inverse as mlInverse } from 'ml-matrix';
import { InsMechanization } from './InsMechanization';
import { GnssQualityStateMachine } from './GnssQualityStateMachine';
import { AiMotionModel } from './AiMotionModel';

export class EkfCore {
  // 15-state vector:
  // 0-2: Position (x,y,z)
  // 3-5: Velocity (x,y,z)
  // 6-8: Attitude errors (pitch, roll, yaw)
  // 9-11: Accel bias (x,y,z)
  // 12-14: Gyro bias (x,y,z)
  private x: Matrix;
  private P: Matrix;

  private ins: InsMechanization;
  private gnssState: GnssQualityStateMachine;
  private aiModel: AiMotionModel;

  constructor() {
    this.x = Matrix.zeros(15, 1);
    this.P = Matrix.zeros(15, 15);
    for (let i = 0; i < 3; i++) this.P.set(i, i, 5.0); // Position (5.0m variance allows rapid GNSS tracking)
    for (let i = 3; i < 6; i++) this.P.set(i, i, 1.0); // Velocity
    for (let i = 6; i < 9; i++) this.P.set(i, i, 0.001); // Attitude
    for (let i = 9; i < 12; i++) this.P.set(i, i, 0.0001); // Accel bias
    for (let i = 12; i < 15; i++) this.P.set(i, i, 0.00001); // Gyro bias

    this.ins = new InsMechanization();
    this.gnssState = new GnssQualityStateMachine();
    this.aiModel = new AiMotionModel();
  }

  // ZUPT state tracking for Q scheduling
  private wasZuptActive = false;
  private postZuptCooldown = 0;
  private readonly POST_ZUPT_COOLDOWN_CYCLES = 5; // ~0.5s at 10Hz
  private previousVelocity: { x: number; y: number; z: number } | null = null;
  private lastRawAccel: number[] = [0, 0, 9.81];
  private lastVelQ: number = 0.01;
  private lastAiCorrection: number[] | null = null;
  private isAttitudeInitialized = false;

  public getIns() {
    return this.ins;
  }

  public getGnssState() {
    return this.gnssState;
  }

  public getAiModel() {
    return this.aiModel;
  }

  public getLastRawAccel(): number[] {
    return this.lastRawAccel;
  }

  public getLastVelQ(): number {
    return this.lastVelQ;
  }

  public getLastAiCorrection(): number[] | null {
    return this.lastAiCorrection;
  }

  private currentTime: number = 0;

  /**
   * Predict step (runs constantly at IMU rate)
   */
  public predict(dt: number, accel: number[], gyro: number[]) {
    this.currentTime += dt;
    this.lastRawAccel = [...accel];

    if (this.ins.getFilteredAccel() === null) {
      this.ins.initializeAttitude({ x: 0, y: 0, z: 9.81 });
      this.isAttitudeInitialized = true;
    } else if (!this.isAttitudeInitialized) {
      this.isAttitudeInitialized = true;
    }

    // 0. Subtract estimated biases from raw IMU measurements
    const accX = accel[0] - this.x.get(9, 0);
    const accY = accel[1] - this.x.get(10, 0);
    const accZ = accel[2] - this.x.get(11, 0);

    const gyrX = gyro[0] - this.x.get(12, 0);
    const gyrY = gyro[1] - this.x.get(13, 0);
    const gyrZ = gyro[2] - this.x.get(14, 0);

    // 1. Advance INS Mechanization with bias-corrected inputs
    this.ins.predict(
      dt,
      { x: accX, y: accY, z: accZ },
      { x: gyrX, y: gyrY, z: gyrZ }
    );

    // 2. Propagate Covariance P = F * P * F^T + Q
    const F = Matrix.eye(15);

    // Position depends on velocity: p_new = p_old + v * dt
    F.set(0, 3, dt);
    F.set(1, 4, dt);
    F.set(2, 5, dt);

    // Rigorous F-matrix construction (Body to Navigation frame projection)
    const C_b_n = this.ins.lastRotationMatrix;
    const f_n = this.ins.lastSpecificForce;

    // Velocity error from Attitude error: -[f^n x] * dt
    F.set(3, 6, 0);
    F.set(3, 7, f_n.z * dt);
    F.set(3, 8, -f_n.y * dt);
    F.set(4, 6, -f_n.z * dt);
    F.set(4, 7, 0);
    F.set(4, 8, f_n.x * dt);
    F.set(5, 6, f_n.y * dt);
    F.set(5, 7, -f_n.x * dt);
    F.set(5, 8, 0);

    // Velocity error from Accelerometer bias: -C_b^n * dt
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        F.set(3 + r, 9 + c, -C_b_n[r][c] * dt);
      }
    }

    // Attitude error from Gyroscope bias: -C_b^n * dt
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        F.set(6 + r, 12 + c, -C_b_n[r][c] * dt);
      }
    }

    // Process noise Q
    const Q = Matrix.zeros(15, 15);
    for (let i = 0; i < 3; i++) Q.set(i, i, 0.1 * dt); // position

    let velQ = 0.1 * dt;
    if (this.postZuptCooldown > 0) {
      velQ = 2.0 * dt; // 2x inflation during transition (cooldown)
      this.postZuptCooldown--;
    }
    this.lastVelQ = velQ;
    for (let i = 3; i < 6; i++) Q.set(i, i, velQ);

    for (let i = 6; i < 9; i++) Q.set(i, i, 0.000001 * dt); // attitude (gyro noise ~1e-6)
    for (let i = 9; i < 12; i++) Q.set(i, i, 0.00001 * dt); // accel bias (slow drift)
    for (let i = 12; i < 15; i++) Q.set(i, i, 0.000001 * dt); // gyro bias (very slow drift)

    this.P = F.mmul(this.P).mmul(F.transpose()).add(Q);

    this.applyVelocityGuard();
  }

  /**
   * Applies a generalized Kalman measurement update in Joseph form with Mahalanobis gating.
   */
  public applyMeasurementUpdate(
    H: Matrix,
    z: Matrix,
    R: Matrix,
    chiSquareThreshold: number = 6.0,
    updateAttitude: boolean = true
  ): boolean {
    const S = H.mmul(this.P).mmul(H.transpose()).add(R);
    const S_inv = inverse(S);
    if (!S_inv) return false;

    const mahalanobisSq = z.transpose().mmul(S_inv).mmul(z).get(0, 0);
    if (mahalanobisSq >= chiSquareThreshold) {
      return false; // Outlier rejected
    }

    const K = this.P.mmul(H.transpose()).mmul(S_inv);
    const dx = K.mmul(z);
    this.x = this.x.add(dx);

    // Joseph-form covariance update
    const I = Matrix.eye(15);
    const IKH = I.sub(K.mmul(H));
    this.P = IKH.mmul(this.P).mmul(IKH.transpose()).add(K.mmul(R).mmul(K.transpose()));

    // Closed-loop state feedback to INS
    this.ins.position.x += this.x.get(0, 0);
    this.ins.position.y += this.x.get(1, 0);
    this.ins.position.z += this.x.get(2, 0);
    this.ins.velocity.x += this.x.get(3, 0);
    this.ins.velocity.y += this.x.get(4, 0);
    this.ins.velocity.z += this.x.get(5, 0);

    if (updateAttitude) {
      // Clamp attitude error correction to max ~3 degrees (0.05 rad)
      const maxAttJump = 0.05;
      const dPitch = Math.max(-maxAttJump, Math.min(maxAttJump, this.x.get(6, 0)));
      const dRoll = Math.max(-maxAttJump, Math.min(maxAttJump, this.x.get(7, 0)));
      const dYaw = Math.max(-maxAttJump, Math.min(maxAttJump, this.x.get(8, 0)));

      this.ins.attitude.pitch += dPitch;
      this.ins.attitude.roll += dRoll;
      this.ins.attitude.yaw += dYaw;
    }

    // Reset error state vector after closed-loop feedback
    for (let i = 0; i < 15; i++) {
      this.x.set(i, 0, 0);
    }
    return true;
  }

  /**
   * Non-Holonomic Constraints (NHC)
   */
  public applyNhc(R_lat: number = 0.05, R_vert: number = 0.01) {
    const C_b_n = this.ins.lastRotationMatrix;
    const vel = this.ins.velocity;

    // Body Frame velocity: v_b = (C_b^n)^T * v_n
    // Body X is Lateral (Transverse): Col 0 of C_b^n
    // Body Y is Longitudinal (Forward): Col 1 of C_b^n (UNCONSTRAINED)
    // Body Z is Vertical: Col 2 of C_b^n
    const v_lat = C_b_n[0][0] * vel.x + C_b_n[1][0] * vel.y + C_b_n[2][0] * vel.z;
    const v_fwd = C_b_n[0][1] * vel.x + C_b_n[1][1] * vel.y + C_b_n[2][1] * vel.z;
    const v_vert = C_b_n[0][2] * vel.x + C_b_n[1][2] * vel.y + C_b_n[2][2] * vel.z;

    const H = Matrix.zeros(2, 15);
    H.set(0, 3, C_b_n[0][0]);
    H.set(0, 4, C_b_n[1][0]);
    H.set(0, 5, C_b_n[2][0]);

    H.set(1, 3, C_b_n[0][2]);
    H.set(1, 4, C_b_n[1][2]);
    H.set(1, 5, C_b_n[2][2]);

    const z = new Matrix([[-v_lat], [-v_vert]]);

    const R = Matrix.zeros(2, 2);
    R.set(0, 0, R_lat);
    R.set(1, 1, R_vert);

    // Apply EKF covariance update for lateral/vertical states without corrupting pitch/roll attitude
    this.applyMeasurementUpdate(H, z, R, Infinity, false);

    // Explicitly project velocity to preserve longitudinal forward velocity in Body frame
    this.ins.velocity.x = C_b_n[0][1] * v_fwd;
    this.ins.velocity.y = C_b_n[1][1] * v_fwd;
    this.ins.velocity.z = C_b_n[2][1] * v_fwd;

    this.applyVelocityGuard();
  }

  private lastGnssPos: number[] | null = null;
  private lastGnssUpdateTime: number = 0;

  /**
   * Update step with GNSS or AI
   */
  public updateGnss(
    gnssPos: number[],
    gnssVel: number[] | null,
    accuracy: number | null,
    recentImuWindow: number[][] = []
  ) {
    const rScale = this.gnssState.updateState(accuracy);
    const state = this.gnssState.getState();

    let z: Matrix;
    let H: Matrix;
    let R: Matrix;
    let threshold = 6.0;

    let effectiveVel = gnssVel;
    if (
      effectiveVel === null &&
      this.lastGnssPos !== null &&
      (state === 'GOOD' || state === 'DEGRADED')
    ) {
      const dtGnss =
        this.lastGnssUpdateTime > 0
          ? Math.min(Math.max(this.currentTime - this.lastGnssUpdateTime, 0.1), 3.0)
          : 0;
      if (dtGnss >= 0.1 && dtGnss <= 3.0) {
        const vx = (gnssPos[0] - this.lastGnssPos[0]) / dtGnss;
        const vy = (gnssPos[1] - this.lastGnssPos[1]) / dtGnss;
        const vz = (gnssPos[2] - this.lastGnssPos[2]) / dtGnss;
        const speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
        if (speed < 40) {
          effectiveVel = [vx, vy, vz];
        }
      }
    }
    if (state === 'GOOD' || state === 'DEGRADED') {
      this.lastGnssPos = [...gnssPos];
      this.lastGnssUpdateTime = this.currentTime;
    }

    if (state === 'GOOD' || state === 'DEGRADED') {
      const basePosVar =
        accuracy !== null && accuracy > 0 ? Math.max(0.1, (accuracy * accuracy) / 9.0) : 1.0;

      if (effectiveVel !== null) {
        H = Matrix.zeros(6, 15);
        for (let i = 0; i < 6; i++) {
          H.set(i, i, 1);
        }

        R = Matrix.eye(6).mul(basePosVar);
        for (let i = 3; i < 6; i++) {
          R.set(i, i, gnssVel !== null ? 0.1 : 5.0);
        }
        if (state === 'DEGRADED') {
          R = R.mulColumnVector(Matrix.columnVector(Array(6).fill(rScale)));
        }

        z = new Matrix([
          [gnssPos[0] - this.ins.position.x],
          [gnssPos[1] - this.ins.position.y],
          [gnssPos[2] - this.ins.position.z],
          [effectiveVel[0] - this.ins.velocity.x],
          [effectiveVel[1] - this.ins.velocity.y],
          [effectiveVel[2] - this.ins.velocity.z],
        ]);
        threshold = Infinity;
      } else {
        H = Matrix.zeros(3, 15);
        for (let i = 0; i < 3; i++) {
          H.set(i, i, 1);
        }

        R = Matrix.eye(3).mul(basePosVar);
        if (state === 'DEGRADED') {
          R = R.mulColumnVector(Matrix.columnVector(Array(3).fill(rScale)));
        }

        z = new Matrix([
          [gnssPos[0] - this.ins.position.x],
          [gnssPos[1] - this.ins.position.y],
          [gnssPos[2] - this.ins.position.z],
        ]);
        threshold = Infinity;
      }

      this.applyMeasurementUpdate(H, z, R, threshold);
      if (state === 'GOOD') {
        const dX = Math.abs(gnssPos[0] - this.ins.position.x);
        const dY = Math.abs(gnssPos[1] - this.ins.position.y);
        if (dX > 10 || dY > 10) {
          this.ins.position.x = gnssPos[0];
          this.ins.position.y = gnssPos[1];
        }
      }
    } else {
      // WEAK_LOST State
      const aiCorrection = this.aiModel.predictError(
        state,
        recentImuWindow,
        [this.ins.velocity.x, this.ins.velocity.y]
      );
      this.lastAiCorrection = aiCorrection ? [...aiCorrection] : null;

      if (aiCorrection) {
        H = Matrix.zeros(2, 15);
        H.set(0, 3, 1);
        H.set(1, 4, 1);
        R = Matrix.eye(2).mul(0.5);
        z = new Matrix([[aiCorrection[0]], [aiCorrection[1]]]);
        this.applyMeasurementUpdate(H, z, R, 6.0);
      }
    }

    this.applyVelocityGuard();
  }

  /**
   * Zero Velocity Update (ZUPT)
   */
  public updateZupt() {
    const H = Matrix.zeros(3, 15);
    H.set(0, 3, 1);
    H.set(1, 4, 1);
    H.set(2, 5, 1);

    const R = Matrix.eye(3).mul(0.001);

    const z = new Matrix([
      [-this.ins.velocity.x],
      [-this.ins.velocity.y],
      [-this.ins.velocity.z],
    ]);

    this.applyMeasurementUpdate(H, z, R, Infinity, false);

    this.wasZuptActive = true;

    this.ins.velocity.x = 0;
    this.ins.velocity.y = 0;
    this.ins.velocity.z = 0;

    this.x.set(3, 0, 0);
    this.x.set(4, 0, 0);
    this.x.set(5, 0, 0);

    this.applyVelocityGuard();
  }

  public getPosition() {
    return this.ins.position;
  }

  public getBiases() {
    return {
      accel: { x: this.x.get(9, 0), y: this.x.get(10, 0), z: this.x.get(11, 0) },
      gyro: { x: this.x.get(12, 0), y: this.x.get(13, 0), z: this.x.get(14, 0) },
    };
  }

  private applyVelocityGuard() {
    const v = this.ins.velocity;

    if (this.previousVelocity === null) {
      this.previousVelocity = { x: v.x, y: v.y, z: v.z };
      return;
    }

    const MAX_VELOCITY_JUMP = 3.0;
    const dx = v.x - this.previousVelocity.x;
    const dy = v.y - this.previousVelocity.y;
    const dz = v.z - this.previousVelocity.z;
    const jumpMag = Math.sqrt(dx * dx + dy * dy + dz * dz);

    if (jumpMag > MAX_VELOCITY_JUMP) {
      // Per-step velocity-jump monitor
    }

    if (Math.abs(v.x) > 60 || Math.abs(v.y) > 60 || Math.abs(v.z) > 60) {
      v.x = Math.max(-60, Math.min(60, v.x));
      v.y = Math.max(-60, Math.min(60, v.y));
      v.z = Math.max(-60, Math.min(60, v.z));
    }

    this.previousVelocity = { x: v.x, y: v.y, z: v.z };
  }

  public notifyAttitudeInitialized() {
    this.isAttitudeInitialized = true;
  }

  public notifyZuptReleased() {
    if (this.wasZuptActive) {
      this.postZuptCooldown = this.POST_ZUPT_COOLDOWN_CYCLES;
      this.wasZuptActive = false;
      // Re-inflate velocity covariance so EKF immediately tracks vehicle dynamics
      for (let i = 3; i < 6; i++) {
        this.P.set(i, i, Math.max(this.P.get(i, i), 5.0));
      }
    }
  }
}

function inverse(m: Matrix): Matrix | null {
  try {
    return mlInverse(m);
  } catch {
    return null;
  }
}
