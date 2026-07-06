/**
 * @vitest-environment happy-dom
 */
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { describe, expect, it } from 'vitest';
import { angleDiff, ComputedTorqueController, gainsFromMassDiagonal } from './controller';
import { ConstantJointPlanner } from './planner';
import { CONTROL_DT, createSimulation } from './simulation';
import { loadMujocoRobot } from '../mujoco/loader';
import { vecGet } from '../types/mujoco';
import { SimulationEngine } from './simulation';
import { RobotSession } from './robot-session';
import { nvGainsToActuated } from '../utils/joint-qpos';
import { ensureFixedBase } from '../utils/urdf-base-fixture';

const urdfPath = resolve(__dirname, '../../public/robots/test_arm.urdf');
const urdfXml = readFileSync(urdfPath, 'utf-8');
const upperBodyPath = resolve(__dirname, '../../public/robots/biped_s70_upper_body.urdf');
const upperBodyXml = readFileSync(upperBodyPath, 'utf-8');

describe('T2 controller + simulation', () => {
  it('angleDiff wraps at 2π', () => {
    const d = angleDiff([0], [Math.PI * 1.9]);
    expect(Math.abs(d[0])).toBeLessThan(Math.PI / 2);
  });

  it('ComputedTorqueController massMatrixDiagonal yields valid Kp/Kd', async () => {
    const bundle = await loadMujocoRobot({
      urdfText: urdfXml,
      urdfFileName: 'test_arm.urdf',
      meshes: new Map(),
    });
    const ctrl = new ComputedTorqueController(
      bundle.mujoco,
      bundle.model,
      bundle.data,
      bundle.nv,
      bundle.effortLimits,
      vecGet(bundle.data.qpos, bundle.nq),
      bundle.nq,
    );
    const gains = gainsFromMassDiagonal(
      ctrl.massMatrixDiagonal(vecGet(bundle.data.qpos, bundle.nq)),
    );
    expect(gains.kp.length).toBe(bundle.nv);
    expect(gains.kd.length).toBe(bundle.nv);
    for (let i = 0; i < bundle.nv; i++) {
      expect(gains.kp[i]).toBeGreaterThanOrEqual(0.5);
      expect(gains.kd[i]).toBeGreaterThan(0);
    }
    const tau = ctrl.computeTorque(
      new Float64Array(bundle.nv),
      new Float64Array(bundle.nv),
      new Float64Array(bundle.nv),
      new Float64Array(bundle.nv),
      new Float64Array(bundle.nv),
    );
    expect(tau.every((v) => Number.isFinite(v))).toBe(true);
    bundle.data.delete();
    bundle.model.delete();
  });

  it('ConstantJointPlanner holds q_d with v_d=a_d=0', () => {
    const target = new Float64Array([0.5, -0.3, 0.1]);
    const planner = new ConstantJointPlanner(target, 3);
    const { q_d, v_d, a_d } = planner.getDesired(1.0);
    expect(Array.from(q_d)).toEqual([0.5, -0.3, 0.1]);
    expect(Array.from(v_d)).toEqual([0, 0, 0]);
    expect(Array.from(a_d)).toEqual([0, 0, 0]);
  });

  it('runToTarget single-joint step reduces error', async () => {
    const bundle = await loadMujocoRobot({
      urdfText: urdfXml,
      urdfFileName: 'test_arm.urdf',
      meshes: new Map(),
    });
    const ctrl = new ComputedTorqueController(
      bundle.mujoco,
      bundle.model,
      bundle.data,
      bundle.nv,
      bundle.effortLimits,
      vecGet(bundle.data.qpos, bundle.nq),
      bundle.nq,
    );
    const q0 = vecGet(bundle.data.qpos, bundle.nq);
    const gains = gainsFromMassDiagonal(ctrl.massMatrixDiagonal(q0));
    ctrl.setGains(gains.kp, gains.kd);
    const session = {
      mujoco: bundle.mujoco,
      model: bundle.model,
      data: bundle.data,
      nq: bundle.nq,
      nv: bundle.nv,
      nu: bundle.nu,
      physicsDt: bundle.model.opt.timestep,
      forwardKinematics: {
        compute: () => ({ pos: [0, 0, 0], quat: [0, 0, 0, 1] }),
      },
    };
    const engine = new SimulationEngine(session as never, ctrl);
    expect(engine.controlDt).toBe(CONTROL_DT);

    const nq = bundle.nq;
    const qInit = Array.from(vecGet(bundle.data.qpos, nq));
    const qTarget = qInit.slice();
    qTarget[0] = (qTarget[0] ?? 0) + 0.3;

    const steps: number[] = [];
    engine.runToTarget(qTarget, {
      maxTime: 2.0,
      tol: 0.05,
      stepCallback: (s) => steps.push(s.qpos[0] ?? 0),
    });

    expect(steps.length).toBeGreaterThan(0);
    const finalQ = vecGet(bundle.data.qpos, nq)[0] ?? 0;
    expect(Math.abs(finalQ - (qTarget[0] ?? 0))).toBeLessThan(0.1);
    bundle.data.delete();
    bundle.model.delete();
  });

  it('loads biped_s70 upper body with fixed base_link', async () => {
    const fixture = ensureFixedBase(upperBodyXml);
    expect(fixture.baseLink).toBe('base_link');
    const session = await RobotSession.create({
      urdfXml: fixture.urdfText,
      urdfFileName: 'biped_s70_upper_body.urdf',
    });
    expect(session.jointNames.length).toBeGreaterThan(10);
    expect(session.jointNames.some((n) => n.startsWith('zarm_l'))).toBe(true);
    expect(session.jointNames.some((n) => n.startsWith('leg_'))).toBe(false);
    session.dispose();
  });

  it('biped upper body auto gains initialize from mass matrix diagonal', async () => {
    const fixture = ensureFixedBase(upperBodyXml);
    const session = await RobotSession.create({
      urdfXml: fixture.urdfText,
      urdfFileName: 'biped_s70_upper_body.urdf',
    });
    const engine = createSimulation(session);
    engine.recomputeAutoGains(vecGet(session.data.qpos, session.nq));
    const gains = engine.getGains();
    const { kp, kd } = nvGainsToActuated(session, gains.kp, gains.kd);
    expect(kp.every((v) => v >= 0.5)).toBe(true);
    expect(kd.every((v) => v > 0)).toBe(true);
    session.dispose();
  });
});
