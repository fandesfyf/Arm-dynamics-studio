import { describe, expect, it } from 'vitest';
import { Trajectory, type Quat, type Vec3 } from './trajectory';

function quatNorm(q: Quat): number {
  return Math.hypot(q[0], q[1], q[2], q[3]);
}

function vecClose(a: number[], b: number[], tol = 1e-6): boolean {
  return a.every((v, i) => Math.abs(v - b[i]) < tol);
}

describe('Trajectory', () => {
  it('returns exact pose at waypoints', () => {
    const traj = new Trajectory();
    const wp: Array<{ t: number; pos: Vec3; quat: Quat }> = [
      { t: 0, pos: [0.5, 0, 0.5], quat: [0, 0, 0, 1] },
      { t: 1, pos: [0.4, 0.2, 0.6], quat: [0, 0, 0.383, 0.924] },
      { t: 2, pos: [0.3, -0.1, 0.5], quat: [0, 0, 0.707, 0.707] },
    ];
    for (const { t, pos, quat } of wp) {
      traj.addWaypoint(t, pos, quat);
    }

    expect(traj.getDuration()).toBe(2);

    for (const { t, pos, quat } of wp) {
      const s = traj.sample(t);
      expect(vecClose(s.ee_pos, pos)).toBe(true);
      expect(Math.abs(quatNorm(s.ee_quat) - 1)).toBeLessThan(1e-6);
      expect(vecClose(s.ee_quat, quat, 1e-3)).toBe(true);
    }
  });

  it('interpolates position smoothly between waypoints', () => {
    const traj = new Trajectory();
    traj.addWaypoint(0, [0, 0, 0], [0, 0, 0, 1]);
    traj.addWaypoint(1, [1, 0, 0], [0, 0, 0, 1]);

    const mid = traj.sample(0.5);
    expect(mid.ee_pos[0]).toBeGreaterThan(0);
    expect(mid.ee_pos[0]).toBeLessThan(1);
    expect(mid.ee_pos[1]).toBeCloseTo(0, 5);
    expect(mid.ee_pos[2]).toBeCloseTo(0, 5);
  });

  it('SLERP rotates quaternion at midpoint', () => {
    const traj = new Trajectory();
    traj.addWaypoint(0, [0, 0, 0], [0, 0, 0, 1]);
    // 绕 Z 轴 90°
    traj.addWaypoint(1, [0, 0, 0], [0, 0, 0.70710678, 0.70710678]);

    const mid = traj.sample(0.5);
    expect(Math.abs(quatNorm(mid.ee_quat) - 1)).toBeLessThan(1e-6);
    // 中点应接近绕 Z 45°
    expect(mid.ee_quat[2]).toBeGreaterThan(0.35);
    expect(mid.ee_quat[2]).toBeLessThan(0.45);
  });

  it('clips time to trajectory bounds', () => {
    const traj = new Trajectory();
    traj.addWaypoint(1, [1, 2, 3], [0, 0, 0, 1]);
    traj.addWaypoint(2, [4, 5, 6], [0, 0, 0, 1]);

    const before = traj.sample(0);
    expect(vecClose(before.ee_pos, [1, 2, 3])).toBe(true);

    const after = traj.sample(10);
    expect(vecClose(after.ee_pos, [4, 5, 6])).toBe(true);
  });

  it('single waypoint returns constant pose', () => {
    const traj = new Trajectory();
    traj.addWaypoint(0, [1, 2, 3], [0, 0, 0, 1]);
    const s = traj.sample(5);
    expect(vecClose(s.ee_pos, [1, 2, 3])).toBe(true);
  });

  it('sampleVelocity is non-zero along moving axis', () => {
    const traj = new Trajectory();
    traj.addWaypoint(0, [0, 0, 0], [0, 0, 0, 1]);
    traj.addWaypoint(2, [2, 0, 0], [0, 0, 0, 1]);

    const vel = traj.sampleVelocity(1);
    expect(vel[0]).toBeGreaterThan(0);
    expect(Math.abs(vel[1])).toBeLessThan(1e-3);
  });

  it('throws when no waypoints', () => {
    const traj = new Trajectory();
    expect(() => traj.sample(0)).toThrow();
  });
});
