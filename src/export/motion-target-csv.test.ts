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
  it('round-trips motion targets with quaternion column order', () => {
    const csv = motionTargetsToCsv(sample, jointNames);
    expect(csv).toContain('ee_qw');
    expect(csv).toContain('q_j1');
    const { targets, warnings } = parseMotionTargetsCsv(csv, jointNames);
    expect(warnings).toHaveLength(0);
    expect(targets).toHaveLength(1);
    expect(targets[0]!.jointPositions).toEqual([0.1, 0.2]);
    expect(targets[0]!.eePosition).toEqual([1, 2, 3]);
    expect(targets[0]!.eeQuaternion).toEqual([0, 0, 0.707, 0.707]);
    expect(targets[0]!.eeSceneWorld).toEqual([4, 5, 6]);
    expect(targets[0]!.source).toBe('joint');
  });
});
