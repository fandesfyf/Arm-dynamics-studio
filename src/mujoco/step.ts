import type { MjData, MjModel } from '@mujoco/mujoco';
import type { MujocoModule } from '../types/robot';

/** 单步 mj_step */
export function mjStep(mujoco: MujocoModule, model: MjModel, data: MjData): void {
  mujoco.mj_step(model, data);
}

/** 连续步进 */
export function mjStepN(
  mujoco: MujocoModule,
  model: MjModel,
  data: MjData,
  steps: number,
): void {
  for (let i = 0; i < steps; i++) {
    mujoco.mj_step(model, data);
  }
}

/** 向 qfrc_applied 施加恒定力矩（无 actuator 时使用） */
export function applyConstantTorque(
  data: MjData,
  torques: number[] | Float64Array,
): void {
  const qfrc = data.qfrc_applied;
  const n = Math.min(qfrc.size(), torques.length);
  for (let i = 0; i < n; i++) {
    qfrc.set(i, torques[i] ?? 0);
  }
}

/** 清零 qfrc_applied（mj_inverse 前必须调用，避免正反馈） */
export function clearAppliedForces(data: MjData): void {
  const qfrc = data.qfrc_applied;
  for (let i = 0; i < qfrc.size(); i++) {
    qfrc.set(i, 0);
  }
}

export interface MjInverseResult {
  qfrc_inverse: Float64Array;
}

/**
 * 逆动力学：给定 q, v, a 计算 τ。
 * 调用前会清零 qfrc_applied。
 */
export function mjInverse(
  mujoco: MujocoModule,
  model: MjModel,
  data: MjData,
  qacc?: number[] | Float64Array,
): MjInverseResult {
  clearAppliedForces(data);

  if (qacc) {
    const acc = data.qacc;
    const n = Math.min(acc.size(), qacc.length);
    for (let i = 0; i < n; i++) {
      acc.set(i, qacc[i] ?? 0);
    }
  }

  mujoco.mj_forward(model, data);
  mujoco.mj_inverse(model, data);

  const qfrc = data.qfrc_inverse;
  const out = new Float64Array(qfrc.size());
  for (let i = 0; i < qfrc.size(); i++) {
    out[i] = qfrc.get(i) ?? 0;
  }
  return { qfrc_inverse: out };
}
