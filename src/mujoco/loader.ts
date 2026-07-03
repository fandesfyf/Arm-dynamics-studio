import loadMujoco from '@mujoco/mujoco';
import type { MjData, MjModel } from '@mujoco/mujoco';
import mujocoWasmUrl from '@mujoco/mujoco/mujoco.wasm?url';
import type { MujocoLoadResult, MujocoModule, RobotAssetBundle } from '../types/robot';
import { finalizeUrdfForMujoco } from '../utils/urdf-sanitize';

const VFS_ROOT = '/robot';

let mujocoModule: MujocoModule | null = null;

const inBrowser = typeof window !== 'undefined' && import.meta.env.MODE !== 'test';

export async function getMujocoModule(): Promise<MujocoModule> {
  if (!mujocoModule) {
    mujocoModule = inBrowser
      ? await loadMujoco({
          locateFile: (path: string) =>
            path.endsWith('.wasm') ? mujocoWasmUrl : path,
        })
      : await loadMujoco();
  }
  return mujocoModule;
}

/** 将 package:// 或相对 mesh 路径规范化为 VFS 相对路径 */
export function normalizeAssetPath(rawPath: string): string {
  let path = rawPath.trim();
  if (path.startsWith('package://')) {
    const withoutScheme = path.slice('package://'.length);
    const slash = withoutScheme.indexOf('/');
    path = slash >= 0 ? withoutScheme.slice(slash + 1) : withoutScheme;
  }
  return path.replace(/^\.?\//, '').replace(/\\/g, '/');
}

function ensureVfsDir(mujoco: MujocoModule, dirPath: string): void {
  const parts = dirPath.split('/').filter(Boolean);
  let current = '';
  for (const part of parts) {
    current += `/${part}`;
    try {
      mujoco.FS.mkdir(current);
    } catch {
      // 目录已存在
    }
  }
}

function mountAssets(mujoco: MujocoModule, bundle: RobotAssetBundle): string {
  ensureVfsDir(mujoco, VFS_ROOT);

  const urdfPath = `${VFS_ROOT}/${bundle.urdfFileName}`;
  const urdfDir = urdfPath.substring(0, urdfPath.lastIndexOf('/'));
  if (urdfDir.length > VFS_ROOT.length) {
    ensureVfsDir(mujoco, urdfDir);
  }
  mujoco.FS.writeFile(urdfPath, new TextEncoder().encode(bundle.urdfText));

  for (const [relativePath, bytes] of bundle.meshes) {
    const normalized = normalizeAssetPath(relativePath);
    const fullPath = `${VFS_ROOT}/${normalized}`;
    const dir = fullPath.substring(0, fullPath.lastIndexOf('/'));
    if (dir.length > VFS_ROOT.length) {
      ensureVfsDir(mujoco, dir);
    }
    mujoco.FS.writeFile(fullPath, bytes);
  }

  return urdfPath;
}

function jointTypeValue(_mujoco: MujocoModule, type: unknown): number {
  if (typeof type === 'number') return type;
  if (type && typeof type === 'object' && 'value' in type) {
    return (type as { value: number }).value;
  }
  return Number(type);
}

/** 提取 MuJoCo 活动关节（hinge / slide）名称，按 jnt id 顺序 */
export function extractMujocoActuatedJointNames(
  mujoco: MujocoModule,
  model: MjModel,
): string[] {
  const hinge = mujoco.mjtJoint.mjJNT_HINGE.value;
  const slide = mujoco.mjtJoint.mjJNT_SLIDE.value;
  const names: string[] = [];

  for (let i = 0; i < model.njnt; i++) {
    const jnt = model.jnt(i);
    const type = jointTypeValue(mujoco, jnt.type);
    if (type === hinge || type === slide) {
      names.push(jnt.name);
    }
  }

  return names;
}

export interface MujocoJointAddress {
  name: string;
  qposadr: number;
  dofadr: number;
}

export function getMujocoJointAddresses(
  mujoco: MujocoModule,
  model: MjModel,
): MujocoJointAddress[] {
  const hinge = mujoco.mjtJoint.mjJNT_HINGE.value;
  const slide = mujoco.mjtJoint.mjJNT_SLIDE.value;
  const result: MujocoJointAddress[] = [];

  for (let i = 0; i < model.njnt; i++) {
    const jnt = model.jnt(i);
    const type = jointTypeValue(mujoco, jnt.type);
    if (type === hinge || type === slide) {
      result.push({
        name: jnt.name,
        qposadr: jnt.qposadr as number,
        dofadr: jnt.dofadr as number,
      });
    }
  }

  return result;
}

function parseUrdfEffortLimits(urdfText: string, jointNames: string[]): number[] {
  const parser = new DOMParser();
  const doc = parser.parseFromString(urdfText, 'text/xml');
  const effortByName = new Map<string, number>();

  for (const jointEl of Array.from(doc.getElementsByTagName('joint'))) {
    const name = jointEl.getAttribute('name');
    const limitEl = jointEl.getElementsByTagName('limit')[0];
    if (!name || !limitEl) continue;
    const effort = parseFloat(limitEl.getAttribute('effort') ?? '100');
    effortByName.set(name, Number.isFinite(effort) ? effort : 100);
  }

  return jointNames.map((name) => effortByName.get(name) ?? 100);
}

/**
 * 从 URDF 文本 + mesh 资源加载 MuJoCo 模型。
 * 关闭接触（mjDSBL_CONTACT），与旧版 Python 行为一致。
 */
export async function loadMujocoRobot(
  bundle: RobotAssetBundle,
): Promise<MujocoLoadResult> {
  const mujoco = await getMujocoModule();
  const urdfText = finalizeUrdfForMujoco(bundle.urdfText);
  const xmlPath = mountAssets(mujoco, { ...bundle, urdfText });

  let model: MjModel;
  try {
    model = mujoco.MjModel.from_xml_path(xmlPath);
  } catch (e) {
    const detail = e instanceof Error ? e.message : String(e);
    throw new Error(`MuJoCo 加载 URDF 失败: ${detail}`);
  }
  model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT.value;

  const data = new mujoco.MjData(model);
  mujoco.mj_forward(model, data);

  const jointNames = extractMujocoActuatedJointNames(mujoco, model);
  const effortLimits = parseUrdfEffortLimits(urdfText, jointNames);

  return {
    mujoco,
    model,
    data,
    jointNames,
    nq: model.nq,
    nv: model.nv,
    nu: model.nu,
    effortLimits,
  };
}

/** 释放 Embind 对象（模型重载时调用） */
export function disposeMujocoRobot(model: MjModel, data: MjData): void {
  data.delete();
  model.delete();
}
