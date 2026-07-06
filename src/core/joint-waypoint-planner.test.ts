import { describe, expect, it } from 'vitest';
import { JointMultiWaypointPlanner } from './joint-waypoint-planner';

describe('JointMultiWaypointPlanner', () => {
  const waypoints = [
    new Float64Array([0, 0]),
    new Float64Array([1, 0.5]),
    new Float64Array([2, 1]),
  ];

  it('linear mode reaches final waypoint', () => {
    const planner = new JointMultiWaypointPlanner(waypoints, 1, 2, 'linear');
    const end = planner.getDesired(planner.getDuration());
    expect(end.q_d[0]).toBeCloseTo(2);
    expect(end.q_d[1]).toBeCloseTo(1);
  });

  it('cubic mode passes through middle knot', () => {
    const planner = new JointMultiWaypointPlanner(waypoints, 1, 2, 'cubic');
    const midT = planner.getDuration() * 0.5;
    const mid = planner.getDesired(midT);
    expect(mid.q_d[0]).toBeGreaterThan(0.2);
    expect(mid.q_d[0]).toBeLessThan(1.8);
  });

  it('cubic duration matches velocity-limited knot times', () => {
    const linear = new JointMultiWaypointPlanner(waypoints, 0.5, 2, 'linear');
    const cubic = new JointMultiWaypointPlanner(waypoints, 0.5, 2, 'cubic');
    expect(cubic.getDuration()).toBeCloseTo(linear.getDuration());
  });

  it('cubic mode accepts zero-displacement segments (mixed targets / duplicate knots)', () => {
    const withDupes = [
      new Float64Array([0, 0]),
      new Float64Array([0, 0]),
      new Float64Array([1, 0.5]),
      new Float64Array([1, 0.5]),
      new Float64Array([2, 1]),
    ];
    const planner = new JointMultiWaypointPlanner(withDupes, 1, 2, 'cubic');
    expect(planner.getDuration()).toBeGreaterThan(0);
    const end = planner.getDesired(planner.getDuration());
    expect(end.q_d[0]).toBeCloseTo(2);
    expect(end.q_d[1]).toBeCloseTo(1);
  });
});
