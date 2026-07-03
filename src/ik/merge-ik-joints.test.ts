import { describe, expect, it } from 'vitest';
import { mergeIkJointsWithChain } from './merge-ik-joints';

describe('mergeIkJointsWithChain', () => {
  const jointNames = ['zarm_l1', 'zarm_l2', 'zarm_r1', 'zarm_r2'];
  const chain = ['zarm_l1', 'zarm_l2'];

  it('preserves off-chain joints from seed when solving left arm', () => {
    const seed = [0.1, 0.2, 1.0, 1.1];
    const solved = [0.5, 0.6, 9.0, 9.1];
    const merged = mergeIkJointsWithChain(jointNames, chain, seed, solved);
    expect(merged[0]).toBeCloseTo(0.5);
    expect(merged[1]).toBeCloseTo(0.6);
    expect(merged[2]).toBeCloseTo(1.0);
    expect(merged[3]).toBeCloseTo(1.1);
  });
});
