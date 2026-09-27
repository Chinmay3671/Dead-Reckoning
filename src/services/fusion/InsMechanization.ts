export class InsMechanization {
  // State variables
  public position: { x: number; y: number; z: number } = { x: 0, y: 0, z: 0 }; // ENU frame
  public velocity: { x: number; y: number; z: number } = { x: 0, y: 0, z: 0 }; // ENU frame
  public attitude: { roll: number; pitch: number; yaw: number } = { roll: 0, pitch: 0, yaw: 0 };

  // Exposed for rigorous EKF F-Matrix propagation
  public lastRotationMatrix: number[][] = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
  ];
  public lastSpecificForce: { x: number; y: number; z: number } = { x: 0, y: 0, z: 0 }; // f^n

  // Constants
  private gravity = 9.81;

  // 2nd-order Butterworth LPF state (cutoff: 2.0 Hz)
  private accelXHist: number[] = [0, 0];
  private accelYHist: number[] = [0, 0];
  private accelZHist: number[] = [0, 0];
  private filteredXHist: number[] = [0, 0];
  private filteredYHist: number[] = [0, 0];
  private filteredZHist: number[] = [0, 0];
  private filteredAccel: { x: number; y: number; z: number } | null = null;

  public setState(
    position: { x: number; y: number; z: number },
    velocity: { x: number; y: number; z: number },
    attitude: { roll: number; pitch: number; yaw: number }
  ) {
    this.position = { ...position };
    this.velocity = { ...velocity };
    this.attitude = { ...attitude };
  }

  public initializeAttitude(accel: { x: number; y: number; z: number }) {
    // Initialize roll and pitch using gravity vector to prevent gravity leakage into horizontal velocity
    this.attitude.pitch = Math.atan2(accel.y, accel.z);
    this.attitude.roll = Math.atan2(-accel.x, Math.sqrt(accel.y * accel.y + accel.z * accel.z));
    // Seed LPF history with initial reading to prevent startup transients
    this.accelXHist = [accel.x, accel.x];
    this.accelYHist = [accel.y, accel.y];
    this.accelZHist = [accel.z, accel.z];
    this.filteredXHist = [accel.x, accel.x];
    this.filteredYHist = [accel.y, accel.y];
    this.filteredZHist = [accel.z, accel.z];
    this.filteredAccel = { ...accel };
  }

  public getFilteredAccel(): { x: number; y: number; z: number } | null {
    return this.filteredAccel ? { ...this.filteredAccel } : null;
  }

  /**
   * Applies a 2nd-order Butterworth digital low-pass filter to accelerometer data.
   * Cutoff = 2.0Hz to attenuate heel-strike transients while preserving vehicle dynamics.
   * Coefficients are dynamically calculated based on dt via bilinear transform.
   */
  private filterAccel(dt: number, raw: { x: number; y: number; z: number }): { x: number; y: number; z: number } {
    if (this.filteredAccel === null) {
      this.accelXHist = [raw.x, raw.x];
      this.accelYHist = [raw.y, raw.y];
      this.accelZHist = [raw.z, raw.z];
      this.filteredXHist = [raw.x, raw.x];
      this.filteredYHist = [raw.y, raw.y];
      this.filteredZHist = [raw.z, raw.z];
      this.filteredAccel = { ...raw };
      return { ...raw };
    }

    const fc = 2.0; // Cutoff frequency (Hz)
    const safeDt = Math.max(dt, 0.001);
    const K = Math.tan(Math.PI * fc * safeDt);
    const K2 = K * K;
    const sqrt2K = Math.SQRT2 * K;
    const a0 = 1 + sqrt2K + K2;

    const b0 = K2 / a0;
    const b1 = (2 * K2) / a0;
    const b2 = K2 / a0;
    const a1 = (2 * (K2 - 1)) / a0;
    const a2 = (1 - sqrt2K + K2) / a0;

    const processAxis = (x: number, xHist: number[], yHist: number[]) => {
      const y = b0 * x + b1 * xHist[0] + b2 * xHist[1] - a1 * yHist[0] - a2 * yHist[1];
      xHist[1] = xHist[0];
      xHist[0] = x;
      yHist[1] = yHist[0];
      yHist[0] = y;
      return y;
    };

    this.filteredAccel = {
      x: processAxis(raw.x, this.accelXHist, this.filteredXHist),
      y: processAxis(raw.y, this.accelYHist, this.filteredYHist),
      z: processAxis(raw.z, this.accelZHist, this.filteredZHist),
    };
    return { ...this.filteredAccel };
  }

  /**
   * Predicts the next state given IMU readings.
   * @param dt Time step in seconds (e.g., 0.1 for 10Hz)
   * @param accel Accelerometer reading in body frame (m/s^2)
   * @param gyro Gyroscope reading in body frame (rad/s)
   */
  public predict(
    dt: number,
    accel: { x: number; y: number; z: number },
    gyro: { x: number; y: number; z: number }
  ) {
    // 0. Low-pass filter accelerometer to attenuate heel-strike impact transients
    const fAccel = this.filterAccel(dt, accel);

    // 1. Update attitude (Euler angles integration)
    this.attitude.roll += gyro.x * dt;
    this.attitude.pitch += gyro.y * dt;
    this.attitude.yaw += gyro.z * dt;

    // 2. Transform body frame acceleration to navigation frame (ENU)
    const cRoll = Math.cos(this.attitude.roll);
    const sRoll = Math.sin(this.attitude.roll);
    const cPitch = Math.cos(this.attitude.pitch);
    const sPitch = Math.sin(this.attitude.pitch);
    const cYaw = Math.cos(this.attitude.yaw);
    const sYaw = Math.sin(this.attitude.yaw);

    // Rotation Matrix R_b^n (Body Phone Frame to Nav ENU: Pitch around X, Roll around Y, Yaw around Z)
    // When phone is pitched nose-up (theta > 0), gravity measured along Body Y rotates onto Nav Z (Up)
    const R11 = cYaw * cRoll - sYaw * sPitch * sRoll;
    const R12 = -sYaw * cPitch;
    const R13 = cYaw * sRoll + sYaw * sPitch * cRoll;

    const R21 = sYaw * cRoll + cYaw * sPitch * sRoll;
    const R22 = cYaw * cPitch;
    const R23 = sYaw * sRoll - cYaw * sPitch * cRoll;

    const R31 = -cPitch * sRoll;
    const R32 = sPitch;
    const R33 = cPitch * cRoll;

    this.lastRotationMatrix = [
      [R11, R12, R13],
      [R21, R22, R23],
      [R31, R32, R33]
    ];

    const a_n_x = R11 * fAccel.x + R12 * fAccel.y + R13 * fAccel.z;
    const a_n_y = R21 * fAccel.x + R22 * fAccel.y + R23 * fAccel.z;
    const a_n_z = R31 * fAccel.x + R32 * fAccel.y + R33 * fAccel.z;

    this.lastSpecificForce = { x: a_n_x, y: a_n_y, z: a_n_z };

    // 3. Subtract gravity (assuming Z is up in ENU)
    const a_n_z_no_g = a_n_z - this.gravity;

    // 4. Integrate acceleration into velocity
    this.velocity.x += a_n_x * dt;
    this.velocity.y += a_n_y * dt;
    this.velocity.z += a_n_z_no_g * dt;

    // 5. Integrate velocity into position
    this.position.x += this.velocity.x * dt;
    this.position.y += this.velocity.y * dt;
    this.position.z += this.velocity.z * dt;
  }
}
