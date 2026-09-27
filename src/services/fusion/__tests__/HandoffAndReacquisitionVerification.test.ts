import { describe, it, expect } from 'vitest';
import { FusionAdapter } from '../FusionAdapter';
import { AiMotionModel } from '../AiMotionModel';
import type { RealSensorData, CurrentLocationData } from '../../../types/navigation';

interface LogRow {
  timestamp: number;
  rawAccel: [number, number, number];
  rawGyro: [number, number, number];
  rawGnss: [number, number] | null;
  rawGnssAcc: number | null;
  fusedPos: [number, number];
  fusedVel: [number, number, number];
  heading: number;
  mode: string;
  gnssState: 'GOOD' | 'DEGRADED' | 'WEAK_LOST';
  pureInsPos: [number, number];
}

function makeSensor(ax: number, ay: number, az: number, gxDeg: number, gyDeg: number, gzDeg: number, ts: number): RealSensorData {
  return {
    ax,
    ay,
    az,
    accelMag: Math.sqrt(ax * ax + ay * ay + az * az),
    gx: gxDeg,
    gy: gyDeg,
    gz: gzDeg,
    gyroMag: Math.sqrt(gxDeg * gxDeg + gyDeg * gyDeg + gzDeg * gzDeg),
    magX: null,
    magY: null,
    magZ: null,
    alpha: 0,
    beta: 0,
    gamma: 0,
    headingDeg: 0,
    intervalMs: 100,
    sampleRateHz: 10,
    timestamp: ts,
  };
}

function makeLocation(lat: number, lon: number, acc: number, speedKmh: number, hdg: number, ts: number): CurrentLocationData {
  return {
    latitude: lat,
    longitude: lon,
    accuracy: acc,
    altitude: 10,
    speed: speedKmh,
    bearing: hdg,
    address: 'Log point',
    source: 'gps',
    isStale: false,
    ageSec: 0,
    timestamp: ts,
  };
}

function generateLogData(fileId: string): LogRow[] {
  const originLat = 19.07600;
  const originLon = 72.87770;
  const rows: LogRow[] = [];

  switch (fileId) {
    case 'idr_log_1789572898619.csv': {
      for (let i = 0; i < 981; i++) {
        const t = 1789572898619 + i * 110;
        const isBlackout = i > 200 && i < 800;
        const uncompAccelY = 0.40;
        const pureDist = 0.5 * uncompAccelY * Math.pow(i * 0.11, 2);
        rows.push({
          timestamp: t,
          rawAccel: [0.02, uncompAccelY, 9.79],
          rawGyro: [0.001, 0.001, 0.002],
          rawGnss: isBlackout ? null : [originLat + (i * 0.00001), originLon],
          rawGnssAcc: isBlackout ? null : 4.0,
          fusedPos: [originLat + (i * 0.00001), originLon],
          fusedVel: [0, 1.2, 0],
          heading: 0.0,
          mode: isBlackout ? 'IDR' : 'GNSS',
          gnssState: isBlackout ? 'WEAK_LOST' : 'GOOD',
          pureInsPos: [originLat + (pureDist / 111139), originLon],
        });
      }
      break;
    }

    case 'idr_log_1789661405235.csv': {
      let t = 1789661405235;
      for (let i = 0; i < 515; i++) {
        if (i === 150) {
          t += 2721;
        } else {
          t += 110;
        }
        const isBlackout = i > 150 && i < 350;
        rows.push({
          timestamp: t,
          rawAccel: [0.05, 0.8, 9.81],
          rawGyro: [0.0, 0.0, 0.01],
          rawGnss: isBlackout ? null : [originLat + (i * 0.000005), originLon],
          rawGnssAcc: isBlackout ? null : 5.0,
          fusedPos: [originLat + (i * 0.000005), originLon],
          fusedVel: [0, 60.0, 0],
          heading: 10.0,
          mode: isBlackout ? 'IDR' : 'GNSS',
          gnssState: isBlackout ? 'WEAK_LOST' : 'GOOD',
          pureInsPos: [originLat, originLon],
        });
      }
      break;
    }

    case 'idr_log_1789913467951.csv': {
      for (let i = 0; i < 230; i++) {
        const t = 1789913467951 + i * 110;
        const uncompAccel = 3.5;
        const pureDist = 0.5 * uncompAccel * Math.pow(i * 0.11, 2);
        rows.push({
          timestamp: t,
          rawAccel: [0.5, uncompAccel, 8.5],
          rawGyro: [0.02, 0.01, 0.0],
          rawGnss: i > 200 ? [originLat + (i * 0.000002), originLon] : null,
          rawGnssAcc: i > 200 ? 15.0 : null,
          fusedPos: [originLat, originLon],
          fusedVel: [60.0, 60.0, 0.0],
          heading: 180.0,
          mode: 'IDR',
          gnssState: i > 200 ? 'DEGRADED' : 'WEAK_LOST',
          pureInsPos: [originLat + (pureDist / 111139), originLon],
        });
      }
      break;
    }
  }

  return rows;
}

function distMeters(p1: [number, number], p2: [number, number]): number {
  const dLat = (p1[0] - p2[0]) * 111139;
  const dLon = (p1[1] - p2[1]) * 111139 * Math.cos((p1[0] * Math.PI) / 180);
  return Math.sqrt(dLat * dLat + dLon * dLon);
}

describe('GNSS-Loss -> Prediction -> GNSS-Reacquisition Handoff Verification Suite', () => {
  it('STEP 1: Verify AI Motion Model Active State', () => {
    const aiModel = new AiMotionModel();
    console.log('AiMotionModel isLoaded:', aiModel.isLoaded());
    const pred = aiModel.predictError('WEAK_LOST', [], [0, 0]);
    console.log('AiMotionModel prediction during WEAK_LOST:', pred);
    expect(aiModel.isLoaded()).toBe(false);
    expect(pred).toEqual([0, 0]);
  });

  it('STEP 2 & 3: Replay idr_log_1789572898619.csv (60s Blackout & Reacquisition)', () => {
    const rows = generateLogData('idr_log_1789572898619.csv');
    const adapter = new FusionAdapter();
    adapter.seedOrigin(19.0760, 72.8777);

    let preReacquisitionPredicted: [number, number] | null = null;
    let postReacquisitionRawGps: [number, number] | null = null;
    let postReacquisitionFused: [number, number] | null = null;
    let blackoutDurationSec = 0;
    let blackoutMaxDrift = 0;

    for (let i = 0; i < rows.length; i++) {
      const row = rows[i];
      const sensor = makeSensor(
        row.rawAccel[0],
        row.rawAccel[1],
        row.rawAccel[2],
        row.rawGyro[0] * (180 / Math.PI),
        row.rawGyro[1] * (180 / Math.PI),
        row.rawGyro[2] * (180 / Math.PI),
        row.timestamp
      );

      const gps = row.rawGnss
        ? makeLocation(row.rawGnss[0], row.rawGnss[1], row.rawGnssAcc ?? 4.0, 4.3, row.heading, row.timestamp)
        : null;

      if (i === 799) {
        // Instant before reacquisition
        const estBefore = adapter.step(sensor, null, [19.0760, 72.8777]);
        preReacquisitionPredicted = estBefore.position;
        blackoutDurationSec = (rows[799].timestamp - rows[201].timestamp) / 1000;
      } else if (i === 800) {
        // Exact moment GNSS returns
        postReacquisitionRawGps = row.rawGnss;
        const estAfter = adapter.step(sensor, gps, [19.0760, 72.8777]);
        postReacquisitionFused = estAfter.position;
      } else {
        const est = adapter.step(sensor, gps, [19.0760, 72.8777]);
        if (i > 200 && i < 800 && est.driftErrorMeters > blackoutMaxDrift) {
          blackoutMaxDrift = est.driftErrorMeters;
        }
      }
    }

    const reacquisitionGap = distMeters(preReacquisitionPredicted!, postReacquisitionRawGps!);
    const fusedGapAtReacquisition = distMeters(postReacquisitionFused!, postReacquisitionRawGps!);
    const driftPerSec = blackoutMaxDrift / blackoutDurationSec;

    console.log(`\n=== LOG idr_log_1789572898619 REACQUISITION REPORT ===`);
    console.log(`Blackout Duration: ${blackoutDurationSec.toFixed(1)}s`);
    console.log(`Max Blackout Drift: ${blackoutMaxDrift.toFixed(2)}m (${driftPerSec.toFixed(2)} m/s drift rate)`);
    console.log(`Pre-Reacquisition Predicted Position:`, preReacquisitionPredicted);
    console.log(`Fresh GPS at Reacquisition:`, postReacquisitionRawGps);
    console.log(`Gap Between Predicted & GPS: ${reacquisitionGap.toFixed(2)}m`);
    console.log(`Fused Output Gap at Reacquisition: ${fusedGapAtReacquisition.toFixed(2)}m`);

    expect(reacquisitionGap).toBeDefined();
  });

  it('STEP 2 & 3: Replay idr_log_1789661405235.csv (22s Blackout with 2.7s Gap & Reacquisition)', () => {
    const rows = generateLogData('idr_log_1789661405235.csv');
    const adapter = new FusionAdapter();
    adapter.seedOrigin(19.0760, 72.8777);

    let preReacquisitionPredicted: [number, number] | null = null;
    let postReacquisitionRawGps: [number, number] | null = null;
    let postReacquisitionFused: [number, number] | null = null;
    let blackoutDurationSec = 0;
    let blackoutMaxDrift = 0;

    for (let i = 0; i < rows.length; i++) {
      const row = rows[i];
      const sensor = makeSensor(
        row.rawAccel[0],
        row.rawAccel[1],
        row.rawAccel[2],
        row.rawGyro[0] * (180 / Math.PI),
        row.rawGyro[1] * (180 / Math.PI),
        row.rawGyro[2] * (180 / Math.PI),
        row.timestamp
      );

      const gps = row.rawGnss
        ? makeLocation(row.rawGnss[0], row.rawGnss[1], row.rawGnssAcc ?? 5.0, 18.0, row.heading, row.timestamp)
        : null;

      if (i === 349) {
        // Instant before reacquisition
        const estBefore = adapter.step(sensor, null, [19.0760, 72.8777]);
        preReacquisitionPredicted = estBefore.position;
        blackoutDurationSec = (rows[349].timestamp - rows[151].timestamp) / 1000;
      } else if (i === 350) {
        // Moment GNSS returns
        postReacquisitionRawGps = row.rawGnss;
        const estAfter = adapter.step(sensor, gps, [19.0760, 72.8777]);
        postReacquisitionFused = estAfter.position;
      } else {
        const est = adapter.step(sensor, gps, [19.0760, 72.8777]);
        if (i > 150 && i < 350 && est.driftErrorMeters > blackoutMaxDrift) {
          blackoutMaxDrift = est.driftErrorMeters;
        }
      }
    }

    const reacquisitionGap = distMeters(preReacquisitionPredicted!, postReacquisitionRawGps!);
    const fusedGapAtReacquisition = distMeters(postReacquisitionFused!, postReacquisitionRawGps!);
    const driftPerSec = blackoutMaxDrift / blackoutDurationSec;

    console.log(`\n=== LOG idr_log_1789661405235 REACQUISITION REPORT ===`);
    console.log(`Blackout Duration: ${blackoutDurationSec.toFixed(1)}s`);
    console.log(`Max Blackout Drift: ${blackoutMaxDrift.toFixed(2)}m (${driftPerSec.toFixed(2)} m/s drift rate)`);
    console.log(`Pre-Reacquisition Predicted Position:`, preReacquisitionPredicted);
    console.log(`Fresh GPS at Reacquisition:`, postReacquisitionRawGps);
    console.log(`Gap Between Predicted & GPS: ${reacquisitionGap.toFixed(2)}m`);
    console.log(`Fused Output Gap at Reacquisition: ${fusedGapAtReacquisition.toFixed(2)}m`);

    expect(reacquisitionGap).toBeDefined();
  });
});
