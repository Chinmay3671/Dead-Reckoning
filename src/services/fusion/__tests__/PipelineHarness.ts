export interface DebugLogRecord {
  type: 'GNSS_RAW' | 'IMU_RAW' | 'EKF_PREDICT_ENTRY' | 'EKF_PREDICT_EXIT' | 'EKF_UPDATE_ENTRY' | 'EKF_UPDATE_EXIT' | 'AI_MODEL_OUTPUT' | 'UI_RENDER';
  timestamp: number;
  data: any;
}

export class PipelineHarness {
  public logs: DebugLogRecord[] = [];

  public log(type: DebugLogRecord['type'], data: any) {
    this.logs.push({
      type,
      timestamp: data.timestamp ?? Date.now(),
      data,
    });
  }
}
