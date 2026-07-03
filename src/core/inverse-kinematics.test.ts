import { describe, expect, it } from 'vitest';
import { InverseKinematics, type JointMap } from './inverse-kinematics';
import type { JointMapping } from '../types/robot';
import type { PinocchioModule } from '../pinocchio/ik';

function createIdentityJointMap(nv: number, nq = nv): JointMap {
  const mappings: JointMapping[] = Array.from({ length: nv }, (_, i) => ({
    name: `joint_${i}`,
    mj_qposadr: i,
    mj_dofadr: i,
    pin_vidx: i,
  }));

  return {
    mappings,
    mjQposToPinQ(mjQpos) {
      const out = new Float64Array(nv);
      for (let i = 0; i < nv; i++) {
        out[i] = mjQpos[i];
      }
      return out;
    },
    pinQToMjQpos(pinQ, mjQpos) {
      const out = mjQpos ? new Float64Array(mjQpos) : new Float64Array(nq);
      for (let i = 0; i < nv; i++) {
        out[i] = pinQ[i];
      }
      return out;
    },
  };
}

function createPlanarArmPin(length = 1): PinocchioModule {
  let q = new Float64Array(1);
  return {
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
      return j;
    },
  };
}

describe('InverseKinematics', () => {
  it('solve 返回 MuJoCo qpos 并收敛', () => {
    const pin = createPlanarArmPin(1);
    const model = { njoints: 2, nv: 1 };
    const jointMap = createIdentityJointMap(1);
    const ik = new InverseKinematics(pin, model, {}, jointMap, ['joint_0']);
    ik.setEndEffector(1);

    const result = ik.solve([0.6, 0.8, 0], [0.2], {
      maxIterations: 200,
      tolerance: 1e-4,
    });

    expect(result.converged).toBe(true);
    expect(result.error).toBeLessThan(1e-2);
    expect(result.q).toBeInstanceOf(Float64Array);
    expect(result.q.length).toBe(1);
  });

  it('setEndEffector 支持关节名', () => {
    const jointMap = createIdentityJointMap(2);
    const ik = new InverseKinematics({} as never, { njoints: 3, nv: 2 }, {}, jointMap, [
      'j1',
      'j2',
    ]);
    ik.setEndEffector('j2');
    expect(ik.getEndEffectorJointId()).toBe(2);
  });

  it('未知关节名抛出错误', () => {
    const jointMap = createIdentityJointMap(1);
    const ik = new InverseKinematics({} as never, { njoints: 2, nv: 1 }, {}, jointMap, ['j1']);
    expect(() => ik.setEndEffector('missing')).toThrow(/未知末端关节/);
  });
});
