import type { MjModel } from '@mujoco/mujoco';
import type { MujocoModule } from '../types/robot';
import type { Wrench6 } from '../core/payload-editor';
import { detectBaseLink, parseJointBlocks, parseLinkNames } from '../utils/urdf-base-fixture';

export interface FixedJointStep {
  xyz: [number, number, number];
  rpy: [number, number, number];
}

/** URDF link → MuJoCo body 绑定（fixed 子 link 沿父链合并到最近 body） */
export interface LinkBodyBinding {
  bodyId: number;
  /** 从 URDF link 系沿 fixed 关节向上到 body 系的变换步骤 */
  fixedChain: FixedJointStep[];
}

/** 枚举 MuJoCo 模型中全部 body 名称 */
export function collectMjBodyNames(_mujoco: MujocoModule, model: MjModel): string[] {
  const names: string[] = [];
  for (let i = 0; i < model.nbody; i++) {
    try {
      const name = model.body(i).name;
      if (name) names.push(name);
    } catch {
      // ignore
    }
  }
  return names;
}

function mjBodyIdByName(mujoco: MujocoModule, model: MjModel, name: string): number {
  const mjOBJ_BODY = mujoco.mjtObj.mjOBJ_BODY.value;
  return mujoco.mj_name2id(model, mjOBJ_BODY, name);
}

function parseOriginAttrs(block: string): FixedJointStep {
  const originTag = block.match(/<origin\b[^>]*\/?>/);
  const origin = originTag?.[0] ?? '';
  const xyzRaw = origin.match(/xyz="([^"]+)"/)?.[1] ?? '0 0 0';
  const rpyRaw = origin.match(/rpy="([^"]+)"/)?.[1] ?? '0 0 0';
  const xyz = xyzRaw.split(/\s+/).map(Number) as [number, number, number];
  const rpy = rpyRaw.split(/\s+/).map(Number) as [number, number, number];
  return {
    xyz: [xyz[0] || 0, xyz[1] || 0, xyz[2] || 0],
    rpy: [rpy[0] || 0, rpy[1] || 0, rpy[2] || 0],
  };
}

/** child link → fixed 父关节（仅 type=fixed） */
export function parseFixedChildToParentMap(urdfText: string): Map<string, { parent: string } & FixedJointStep> {
  const map = new Map<string, { parent: string } & FixedJointStep>();
  for (const joint of parseJointBlocks(urdfText)) {
    if (joint.type !== 'fixed') continue;
    const block =
      urdfText.match(new RegExp(`<joint\\s+name="${joint.name.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}"[\\s\\S]*?<\\/joint>`))?.[0] ??
      '';
    const origin = parseOriginAttrs(block);
    map.set(joint.child, { parent: joint.parent, ...origin });
  }
  return map;
}

/**
 * 将 URDF link 名解析为 MuJoCo body id（不含 fixed 父链回退）。
 */
export function resolveLinkToMjBodyId(
  mujoco: MujocoModule,
  model: MjModel,
  linkName: string,
  options?: { baseLink?: string },
): number {
  if (linkName === 'world') {
    const worldId = mjBodyIdByName(mujoco, model, 'world');
    return worldId >= 0 ? worldId : 0;
  }

  let id = mjBodyIdByName(mujoco, model, linkName);
  if (id >= 0) return id;

  if (linkName.endsWith('_link')) {
    id = mjBodyIdByName(mujoco, model, linkName.slice(0, -'_link'.length));
    if (id >= 0) return id;
  } else {
    id = mjBodyIdByName(mujoco, model, `${linkName}_link`);
    if (id >= 0) return id;
  }

  const baseLink = options?.baseLink;
  if (baseLink && linkName === baseLink) {
    id = mjBodyIdByName(mujoco, model, 'world');
    return id >= 0 ? id : 0;
  }

  return -1;
}

function rpyToRotMat(rpy: [number, number, number]): number[] {
  const [roll, pitch, yaw] = rpy;
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  return [
    cy * cp,
    cy * sp * sr - sy * cr,
    cy * sp * cr + sy * sr,
    sy * cp,
    sy * sp * sr + cy * cr,
    sy * sp * cr - cy * sr,
    -sp,
    cp * sr,
    cp * cr,
  ];
}

function mat3MulVec3(m: number[], v: [number, number, number]): [number, number, number] {
  return [
    m[0]! * v[0] + m[1]! * v[1] + m[2]! * v[2],
    m[3]! * v[0] + m[4]! * v[1] + m[5]! * v[2],
    m[6]! * v[0] + m[7]! * v[1] + m[8]! * v[2],
  ];
}

function cross3(a: [number, number, number], b: [number, number, number]): [number, number, number] {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

/** URDF 子 link 系 wrench → 父 link 系（joint origin 为 parent→child） */
export function transformWrenchChildToParent(
  wrench: Wrench6,
  step: FixedJointStep,
): Wrench6 {
  const R = rpyToRotMat(step.rpy);
  const f = mat3MulVec3(R, [wrench[0], wrench[1], wrench[2]]);
  const tau0 = mat3MulVec3(R, [wrench[3], wrench[4], wrench[5]]);
  const tauXf = cross3(step.xyz, f);
  return [f[0], f[1], f[2], tau0[0] + tauXf[0], tau0[1] + tauXf[1], tau0[2] + tauXf[2]];
}

export function transformWrenchAlongFixedChain(
  wrench: Wrench6,
  chain: FixedJointStep[],
): Wrench6 {
  let w = wrench;
  for (const step of chain) {
    w = transformWrenchChildToParent(w, step);
  }
  return w;
}

export function resolveLinkBodyBinding(
  mujoco: MujocoModule,
  model: MjModel,
  linkName: string,
  fixedParents: Map<string, { parent: string } & FixedJointStep>,
  baseLink?: string,
): LinkBodyBinding | null {
  const fixedChain: FixedJointStep[] = [];
  let cur = linkName;

  while (true) {
    const bodyId = resolveLinkToMjBodyId(mujoco, model, cur, { baseLink });
    if (bodyId >= 0) {
      return { bodyId, fixedChain };
    }

    const fj = fixedParents.get(cur);
    if (!fj) return null;
    fixedChain.push({ xyz: fj.xyz, rpy: fj.rpy });
    cur = fj.parent;
  }
}

/** 为 URDF 全部 link 建立 MuJoCo body 绑定 */
export function buildUrdfLinkBodyBindings(
  mujoco: MujocoModule,
  model: MjModel,
  urdfText: string,
  baseLink?: string,
): Map<string, LinkBodyBinding> {
  const fixedParents = parseFixedChildToParentMap(urdfText);
  const resolvedBase = baseLink ?? detectBaseLink(urdfText);
  const map = new Map<string, LinkBodyBinding>();

  for (const link of parseLinkNames(urdfText)) {
    const binding = resolveLinkBodyBinding(mujoco, model, link, fixedParents, resolvedBase);
    if (binding) map.set(link, binding);
  }
  return map;
}

/** @deprecated 使用 buildUrdfLinkBodyBindings */
export function buildUrdfLinkToMjBodyMap(
  mujoco: MujocoModule,
  model: MjModel,
  urdfLinkNames: string[],
  baseLink?: string,
): Map<string, number> {
  const map = new Map<string, number>();
  for (const link of urdfLinkNames) {
    const id = resolveLinkToMjBodyId(mujoco, model, link, { baseLink });
    if (id >= 0) map.set(link, id);
  }
  return map;
}
