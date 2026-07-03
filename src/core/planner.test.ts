import { describe, expect, it } from 'vitest';
import { JointVelocityLimitPlanner } from './planner';

describe('JointVelocityLimitPlanner', () => {
  it('computes duration from slowest joint', () => {
    const planner = new JointVelocityLimitPlanner([0, 0], [1, 0.5], 0.5, 2);
    expect(planner.getDuration()).toBeCloseTo(2.0);
  });

  it('reaches qEnd at duration', () => {
    const planner = new JointVelocityLimitPlanner([0, 1], [2, 3], 1, 2);
    const end = planner.getDesired(planner.getDuration());
    expect(end.q_d[0]).toBeCloseTo(2);
    expect(end.q_d[1]).toBeCloseTo(3);
  });
});
