import { describe, expect, it } from 'vitest';
import { DataRecorder } from '../core/data-recorder';
import { exportDictToCsv, exportToCsv, sliceDictByTimeRange } from './csv-exporter';

function makeRecord(time: number, n = 2) {
  return {
    time,
    qpos: Array.from({ length: n }, (_, i) => time + i * 0.1),
    qvel: Array.from({ length: n }, () => time * 0.5),
    tau: Array.from({ length: n }, () => time * 0.2),
    q_desired: Array.from({ length: n }, (_, i) => time + i),
    qvel_desired: Array.from({ length: n }, () => 0),
    tau_commanded: Array.from({ length: n }, () => time),
    ee_pos: [time, time + 1, time + 2],
    ee_quat: [0, 0, 0, 1],
  };
}

describe('csv-exporter', () => {
  it('exports header in §5.8 column order', () => {
    const rec = new DataRecorder();
    rec.record({
      time: 0,
      qpos: [0.1, 0.2],
      qvel: [0.01, 0.02],
      tau: [1.0, 2.0],
      q_desired: [0.15, 0.25],
      qvel_desired: [0, 0],
      tau_commanded: [1.1, 2.1],
      ee_pos: [0, 0, 0],
      ee_quat: [0, 0, 0, 1],
    });

    const csv = exportToCsv(rec, { jointNames: ['j1', 'j2'] });
    const header = csv.split('\n')[0];
    expect(header).toBe(
      'time,j1_pos,j2_pos,j1_vel,j2_vel,j1_torque,j2_torque,j1_pos_desired,j2_pos_desired,j1_vel_desired,j2_vel_desired,j1_torque_cmd,j2_torque_cmd',
    );
  });

  it('exports data row with correct values', () => {
    const rec = new DataRecorder();
    rec.record({
      time: 0.002,
      qpos: [0.1, 0.2],
      qvel: [0.01, 0.02],
      tau: [1.0, 2.0],
      q_desired: [0.15, 0.25],
      qvel_desired: [0, 0],
      tau_commanded: [1.1, 2.1],
      ee_pos: [0, 0, 0],
      ee_quat: [0, 0, 0, 1],
    });

    const csv = exportToCsv(rec, { jointNames: ['j1', 'j2'] });
    const row = csv.split('\n')[1];
    expect(row).toBe(
      '0.002000,0.100000,0.200000,0.010000,0.020000,1.000000,2.000000,0.150000,0.250000,0.000000,0.000000,1.100000,2.100000',
    );
  });

  it('uses default joint names when not provided', () => {
    const rec = new DataRecorder();
    rec.record({
      time: 0,
      qpos: [0],
      qvel: [0],
      tau: [0],
      ee_pos: [0, 0, 0],
      ee_quat: [0, 0, 0, 1],
    });
    const csv = exportToCsv(rec);
    expect(csv.split('\n')[0]).toBe('time,joint1_pos,joint1_vel,joint1_torque');
  });

  it('omits desired columns when not recorded', () => {
    const rec = new DataRecorder();
    rec.record({
      time: 0,
      qpos: [0],
      qvel: [0],
      tau: [0],
      ee_pos: [0, 0, 0],
      ee_quat: [0, 0, 0, 1],
    });
    const csv = exportToCsv(rec);
    expect(csv).not.toContain('desired');
    expect(csv).not.toContain('torque_cmd');
  });

  it('exports only selected metrics and joints', () => {
    const rec = new DataRecorder();
    rec.record(makeRecord(0));
    rec.record(makeRecord(0.002));
    const dict = rec.toDict();

    const csv = exportDictToCsv(dict, {
      metrics: ['position', 'ee'],
      jointIndices: [1],
      jointNames: ['j1', 'j2'],
    });

    expect(csv.split('\n')[0]).toBe('time,j2_pos,j2_pos_desired,ee_x,ee_y,ee_z');
    expect(csv.split('\n')[1]).toBe('0.000000,0.100000,1.000000,0.000000,1.000000,2.000000');
  });

  it('slices recorder dict to recent window', () => {
    const rec = new DataRecorder();
    for (let t = 0; t <= 40; t += 10) {
      rec.record(makeRecord(t));
    }
    const dict = rec.toDict();
    const sliced = sliceDictByTimeRange(dict, { mode: 'window', windowSeconds: 30 });

    expect(sliced.time).toEqual([10, 20, 30, 40]);
  });
});
