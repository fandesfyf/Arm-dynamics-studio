import type { MjData, MjModel } from '@mujoco/mujoco';
import type { MujocoModule } from '../types/robot';
import type { Wrench6 } from '../core/payload-editor';

function collectBodyNames(model: MjModel, limit = 12): string[] {
  const names: string[] = [];
  for (let i = 1; i < model.nbody && names.length < limit; i++) {
    try {
      const name = model.body(i).name;
      if (name) names.push(name);
    } catch {
      // ignore
    }
  }
  return names;
}

function setXfrcComponent(xfrc: { get?: (i: number) => number; set?: (i: number, v: number) => void }, index: number, value: number): void {
  if (typeof xfrc.set === 'function') {
    xfrc.set(index, value);
  } else {
    xfrc[index] = value;
  }
}

/**
 * 在 MuJoCo 步进前施加 link 6D 外力。
 *
 * 首选：data.body(bodyId).xfrc_applied（body 坐标系 wrench，索引 0-2 力、3-5 力矩）。
 * MuJoCo 在 mj_step 中将 xfrc_applied 计入动力学。
 *
 * 回退（body 未找到）：对每个 wrench 的力矩分量 [tx,ty,tz] 均分到
 * 前 min(3, nv) 个自由度作为 qfrc_applied 偏置。该近似忽略力臂与雅可比，仅用于
 * MVP 演示；完整实现应使用 J^T * f 或 Pinocchio rnea 外力项。
 */
export function applyExternalWrenches(
  mujoco: MujocoModule,
  model: MjModel,
  data: MjData,
  wrenches: Map<string, Wrench6>,
  nv: number,
): void {
  if (wrenches.size === 0) return;

  const mjOBJ_BODY = mujoco.mjtObj.mjOBJ_BODY.value;

  for (const [linkName, wrench] of wrenches) {
    if (wrench.every((v) => Math.abs(v) < 1e-12)) continue;

    const bodyId = mujoco.mj_name2id(model, mjOBJ_BODY, linkName);
    if (bodyId < 0) {
      const hint = collectBodyNames(model).join(', ');
      console.warn(
        `applyExternalWrenches: body "${linkName}" not found; falling back to qfrc bias. ` +
          `Sample body names: ${hint || '(none)'}`,
      );
      applyConstantTorqueBias(data, wrench, nv);
      continue;
    }

    try {
      const body = data.body(bodyId);
      const xfrc = body.xfrc_applied;
      for (let i = 0; i < 6; i++) {
        setXfrcComponent(xfrc, i, wrench[i] ?? 0);
      }
    } catch (err) {
      const hint = collectBodyNames(model).join(', ');
      console.warn(
        `applyExternalWrenches: failed to set xfrc on "${linkName}" (id=${bodyId}):`,
        err,
        `Sample body names: ${hint || '(none)'}`,
      );
      applyConstantTorqueBias(data, wrench, nv);
    }
  }
}

/** MVP 常值偏置力矩：将 tx,ty,tz 加到前几个 DOF（见模块顶注释） */
function applyConstantTorqueBias(data: MjData, wrench: Wrench6, nv: number): void {
  const qfrc = data.qfrc_applied;
  const torque = [wrench[3], wrench[4], wrench[5]];
  const n = Math.min(3, nv);
  for (let i = 0; i < n; i++) {
    const prev =
      typeof qfrc.get === 'function' ? (qfrc.get(i) ?? 0) : (qfrc[i] ?? 0);
    const next = prev + (torque[i] ?? 0);
    if (typeof qfrc.set === 'function') {
      qfrc.set(i, next);
    } else {
      qfrc[i] = next;
    }
  }
}

/** 清零所有 body 的 xfrc_applied（模型重载或步间重置时可选） */
export function clearExternalWrenches(mujoco: MujocoModule, model: MjModel, data: MjData): void {
  for (let i = 0; i < model.nbody; i++) {
    try {
      const body = data.body(i);
      const xfrc = body.xfrc_applied;
      for (let j = 0; j < 6; j++) {
        setXfrcComponent(xfrc, j, 0);
      }
    } catch {
      // ignore missing accessor
    }
  }
  void mujoco;
}
