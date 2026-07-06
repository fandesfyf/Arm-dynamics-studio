import type { MjData, MjModel } from '@mujoco/mujoco';
import type { MujocoModule } from '../types/robot';
import type { Wrench6 } from '../core/payload-editor';
import { vecZero } from '../types/mujoco';
import {
  type LinkBodyBinding,
  parseFixedChildToParentMap,
  resolveLinkBodyBinding,
  transformWrenchAlongFixedChain,
} from './link-body-map';

/**
 * 在 MuJoCo 步进前施加 link 6D 外力。
 * 使用 mj_applyFT 写入 qfrc_applied（WASM 下比 xfrc_applied 平坦数组更可靠）。
 * fixed 子 link 的力螺旋先变换到父 body 系再施加。
 */
export function applyExternalWrenches(
  mujoco: MujocoModule,
  model: MjModel,
  data: MjData,
  wrenches: Map<string, Wrench6>,
  nv: number,
  options?: {
    linkBodyBindings?: Map<string, LinkBodyBinding>;
    baseLink?: string;
    urdfText?: string;
    /** 有 actuator 时 qfrc_applied 仅用于外力，需先清零 */
    zeroQfrcBeforeApply?: boolean;
  },
): void {
  if (wrenches.size === 0) return;

  if (options?.zeroQfrcBeforeApply) {
    vecZero(data.qfrc_applied, nv);
  }

  const fixedParents = options?.urdfText
    ? parseFixedChildToParentMap(options.urdfText)
    : null;

  const byBody = new Map<number, Wrench6>();

  for (const [linkName, wrench] of wrenches) {
    if (wrench.every((v) => Math.abs(v) < 1e-12)) continue;

    const binding =
      options?.linkBodyBindings?.get(linkName) ??
      (fixedParents
        ? resolveLinkBodyBinding(mujoco, model, linkName, fixedParents, options?.baseLink)
        : null);

    if (!binding) {
      console.warn(`applyExternalWrenches: no MuJoCo body for URDF link "${linkName}"`);
      continue;
    }

    const bodyWrench = transformWrenchAlongFixedChain(wrench, binding.fixedChain);
    const prev = byBody.get(binding.bodyId);
    if (prev) {
      byBody.set(binding.bodyId, [
        prev[0] + bodyWrench[0],
        prev[1] + bodyWrench[1],
        prev[2] + bodyWrench[2],
        prev[3] + bodyWrench[3],
        prev[4] + bodyWrench[4],
        prev[5] + bodyWrench[5],
      ]);
    } else {
      byBody.set(binding.bodyId, bodyWrench);
    }
  }

  for (const [bodyId, bodyWrench] of byBody) {
    try {
      mujoco.mj_applyFT(
        model,
        data,
        [bodyWrench[0], bodyWrench[1], bodyWrench[2]],
        [bodyWrench[3], bodyWrench[4], bodyWrench[5]],
        [0, 0, 0],
        bodyId,
        data.qfrc_applied,
      );
    } catch (err) {
      console.warn(`applyExternalWrenches: mj_applyFT failed (bodyId=${bodyId}):`, err);
    }
  }
}

/** 步间清零 xfrc_applied（兼容旧路径；外力现走 qfrc） */
export function clearExternalWrenches(_mujoco: MujocoModule, _model: MjModel, _data: MjData): void {
  // no-op：外力通过 mj_applyFT 每步写入，不持久化在 xfrc
}
