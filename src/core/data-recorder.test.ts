import { describe, expect, it } from 'vitest';
import { DataRecorder } from './data-recorder';

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

describe('DataRecorder', () => {
  it('records and retrieves time series', () => {
    const rec = new DataRecorder();
    rec.record(makeRecord(0));
    rec.record(makeRecord(0.002));
    rec.record(makeRecord(0.004));

    expect(rec.getNumFrames()).toBe(3);
    expect(rec.getTimes()).toEqual([0, 0.002, 0.004]);
    expect(rec.getDuration()).toBeCloseTo(0.004, 6);
    expect(rec.getSamplingRate()).toBeCloseTo(500, 0);
  });

  it('deep-copies arrays on record', () => {
    const rec = new DataRecorder();
    const state = makeRecord(0);
    rec.record(state);
    state.qpos[0] = 999;
    expect(rec.getJointPositions()[0][0]).toBe(0);
  });

  it('exposes desired and commanded fields', () => {
    const rec = new DataRecorder();
    rec.record(makeRecord(0));
    rec.record(makeRecord(0.002));

    expect(rec.getDesiredPositions()![0]).toEqual([0, 1]);
    expect(rec.getDesiredVelocities()![1]).toEqual([0, 0]);
    expect(rec.getCommandedTorques()![1]).toEqual([0.002, 0.002]);
  });

  it('returns null for optional fields when absent', () => {
    const rec = new DataRecorder();
    rec.record({
      time: 0,
      qpos: [0],
      qvel: [0],
      tau: [0],
      ee_pos: [0, 0, 0],
      ee_quat: [0, 0, 0, 1],
    });
    expect(rec.getDesiredPositions()).toBeNull();
    expect(rec.getDesiredVelocities()).toBeNull();
    expect(rec.getCommandedTorques()).toBeNull();
  });

  it('toDict includes all present fields', () => {
    const rec = new DataRecorder();
    rec.record(makeRecord(0));
    const d = rec.toDict();
    expect(d.time).toHaveLength(1);
    expect(d.qpos).toHaveLength(1);
    expect(d.q_desired).toBeDefined();
    expect(d.qvel_desired).toBeDefined();
    expect(d.tau_commanded).toBeDefined();
    expect(d.ee_pos).toEqual([[0, 1, 2]]);
  });

  it('clear removes all records', () => {
    const rec = new DataRecorder();
    rec.record(makeRecord(0));
    rec.clear();
    expect(rec.getNumFrames()).toBe(0);
    expect(rec.getDuration()).toBe(0);
  });

  it('ring buffer drops frames older than maxDurationSec', () => {
    const rec = new DataRecorder();
    rec.maxDurationSec = 0.01;
    for (let i = 0; i <= 10; i++) {
      rec.record(makeRecord(i * 0.002));
    }
    expect(rec.getNumFrames()).toBeLessThan(11);
    expect(rec.getWindowStartTime()).toBeGreaterThanOrEqual(0.02 - 0.01 - 1e-9);
    expect(rec.getTimes()[0]).toBeGreaterThanOrEqual(0.01);
  });

  it('setMaxDurationSec trims existing records', () => {
    const rec = new DataRecorder();
    for (let i = 0; i <= 20; i++) {
      rec.record(makeRecord(i * 0.002));
    }
    expect(rec.getNumFrames()).toBe(21);
    rec.setMaxDurationSec(0.02);
    expect(rec.getNumFrames()).toBeLessThan(21);
    expect(rec.getDuration()).toBeLessThanOrEqual(0.02 + 1e-9);
  });

  it('maxDurationSec <= 0 disables trimming', () => {
    const rec = new DataRecorder();
    rec.setMaxDurationSec(0);
    for (let i = 0; i < 50; i++) {
      rec.record(makeRecord(i * 0.002));
    }
    expect(rec.getNumFrames()).toBe(50);
  });

  it('toDictForDisplay decimates large series but keeps endpoints', () => {
    const rec = new DataRecorder();
    rec.setMaxDurationSec(0);
    for (let i = 0; i < 5000; i++) {
      rec.record(makeRecord(i * 0.002));
    }
    expect(rec.getNumFrames()).toBe(5000);
    const d = rec.toDictForDisplay(200);
    expect(d.time.length).toBeLessThanOrEqual(201);
    expect(d.time[0]).toBeCloseTo(0, 6);
    expect(d.time[d.time.length - 1]).toBeCloseTo(4999 * 0.002, 4);
    expect(d.qpos).toHaveLength(d.time.length);
  });

  it('ring buffer advances lastTime after window is full', () => {
    const rec = new DataRecorder();
    rec.setMaxDurationSec(0.02);
    for (let i = 0; i < 100; i++) {
      rec.record(makeRecord(i * 0.002));
    }
    const tEnd = rec.getLastTime();
    const n = rec.getNumFrames();
    expect(n).toBeLessThan(100);
    expect(tEnd).toBeCloseTo(99 * 0.002, 6);
    expect(rec.getDuration()).toBeLessThanOrEqual(0.02 + 1e-9);
  });
});
