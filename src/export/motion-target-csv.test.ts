import { describe, expect, it } from 'vitest';
import { motionTargetsToCsv, parseMotionTargetsCsv } from './motion-target-csv';
import type { MotionTarget } from '../types/motion-target';

const jointNames = ['j1', 'j2'];

const sample: MotionTarget[] = [
  {
    id: 'mt-a',
    source: 'joint',
    jointPositions: [0.1, 0.2],
    eePosition: [1, 2, 3],
    eeQuaternion: [0, 0, 0.707, 0.707],
    eeSceneWorld: [4, 5, 6],
  },
];

describe('motion-target-csv', () => {
  it('exports joint-space columns only', () => {
    const csv = motionTargetsToCsv(sample, jointNames);
    expect(csv).toContain('index');
    expect(csv).toContain('q_j1');
    expect(csv).toContain('q_j2');
    expect(csv).not.toContain('ee_px');
    expect(csv).not.toContain('source');
    const lines = csv.split('\n');
    expect(lines[1]).toBe('1,0.1,0.2');
  });

  it('parses joint-space CSV and ignores legacy ee columns', () => {
    const jointOnly = motionTargetsToCsv(sample, jointNames);
    const { targets, warnings } = parseMotionTargetsCsv(jointOnly, jointNames);
    expect(warnings).toHaveLength(0);
    expect(targets).toHaveLength(1);
    expect(targets[0]!.jointPositions).toEqual([0.1, 0.2]);
    expect(targets[0]!.source).toBe('joint');

    const legacy =
      'index,source,ee_px,ee_py,ee_pz,ee_qw,ee_qx,ee_qy,ee_qz,scene_wx,scene_wy,scene_wz,q_j1,q_j2\n' +
      '1,joint,9,9,9,1,0,0,0,9,9,9,0.1,0.2';
    const legacyParsed = parseMotionTargetsCsv(legacy, jointNames);
    expect(legacyParsed.targets[0]!.jointPositions).toEqual([0.1, 0.2]);
    expect(legacyParsed.targets[0]!.eePosition).toEqual([0, 0, 0]);
  });
});
