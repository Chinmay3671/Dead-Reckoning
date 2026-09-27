import type { GnssState } from './GnssQualityStateMachine';

/**
 * Normalization statistics extracted from normalization_stats.npz
 * for the CNN-LSTM error correction layer.
 */
export const AI_NORMALIZATION_STATS = {
  IMU_MEAN: [
    3.7883697e-3, -3.5776526e-2, 9.8567991e0, -1.7793525e-4, -5.1023387e-3, 3.7530807e-4,
  ],
  IMU_STD: [
    1.6299425, 1.4949728, 0.7497654, 0.11333029, 0.22307415, 0.13506341,
  ],
  STATE_MEAN: [-0.20375685, 0.56889135],
  STATE_STD: [7.3494987, 7.6384563],
  Y_MEAN: [-0.3668697, 1.0653404],
  Y_STD: [10.648473, 10.959547],
};

/**
 * AiMotionModel - AI Error Correction Interface
 * Phase 1: Pure-math placeholder (no native React Native dependencies).
 * Returns null or zero correction during Phase 1-3.
 * Full TF.js web runtime will be integrated in Phase 4.
 */
export class AiMotionModel {
  private isModelLoaded = false;

  public async loadModel(_pathOrUrl?: string): Promise<void> {
    // Model loading for web runtime deferred to Phase 4
    this.isModelLoaded = false;
  }

  public isLoaded(): boolean {
    return this.isModelLoaded;
  }

  public getNormalizationStats() {
    return AI_NORMALIZATION_STATS;
  }

  /**
   * Predicts velocity error correction if in ACTIVE mode.
   * @param gnssState Current GNSS quality state
   * @param _imuWindow Recent IMU samples [20, 6]
   * @param _insState Current INS state [2] (e.g. vel x, vel y)
   * @returns [error_x, error_y] or null if SLEEP / uninitialized mode
   */
  public predictError(
    gnssState: GnssState,
    _imuWindow: number[][],
    _insState: number[]
  ): number[] | null {
    if (gnssState === 'GOOD') {
      return null;
    }

    // In Phase 1 (pure math EKF), return zero error correction until Phase 4 AI layer
    return [0, 0];
  }
}
