import { describe, expect, it } from 'vitest';
import { solveInverseKinematics, type PinocchioModule } from './ik';

/** 单自由度平面臂 mock：末端位置 = [L cos(q), L sin(q), 0] */
function createPlanarArmMock(length = 1): {
  pin: PinocchioModule;
  model: { njoints: number; nv: number };
  data: Record<string, never>;
} {
  let q = new Float64Array(1);

  const pin: PinocchioModule = {
    forwardKinematics(_model, _data, newQ) {
      q = newQ;
    },
    updateFramePlacements() {},
    getJointPlacement() {
      return {
        translation: [length * Math.cos(q[0]), length * Math.sin(q[0]), 0],
      };
    },
    computeJointJacobians() {},
    getJointJacobian() {
      const j = new Float64Array(6);
      j[0] = -length * Math.sin(q[0]);
      j[1] = length * Math.cos(q[0]);
      j[2] = 0;
      return j;
    },
  };

  return { pin, model: { njoints: 2, nv: 1 }, data: {} };
}

describe('solveInverseKinematics (mock pin)', () => {
  it('收敛到可达目标位置', () => {
    const { pin, model, data } = createPlanarArmMock(1);
    const target = [0.6, 0.8, 0];
    const result = solveInverseKinematics(pin, model, data, target, [0.1], {
      endEffectorJointId: 1,
      maxIter: 200,
      tolerance: 1e-4,
      stepSize: 0.5,
    });

    expect(result.converged).toBe(true);
    expect(result.error).toBeLessThan(1e-3);
    expect(Math.cos(result.q[0])).toBeCloseTo(target[0], 2);
    expect(Math.sin(result.q[0])).toBeCloseTo(target[1], 2);
  });

  it('不可达目标时返回最小误差解', () => {
    const { pin, model, data } = createPlanarArmMock(0.5);
    const target = [2, 0, 0];
    const result = solveInverseKinematics(pin, model, data, target, [0], {
      endEffectorJointId: 1,
      maxIter: 50,
      tolerance: 1e-6,
    });

    expect(result.converged).toBe(false);
    expect(result.error).toBeGreaterThan(1.4);
    expect(result.error).toBeLessThan(1.6);
  });
});
