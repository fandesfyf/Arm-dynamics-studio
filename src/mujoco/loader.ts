import loadMujoco from '@mujoco/mujoco';
import type { MjData, MjModel } from '@mujoco/mujoco';
import mujocoWasmUrl from '@mujoco/mujoco/mujoco.wasm?url';
import type { MujocoLoadResult, MujocoModule, RobotAssetBundle } from '../types/robot';
import {
  stripMeshVisualsForMujoco,
  urdfReferencesMeshFiles,
} from '../utils/urdf-mujoco-physics';
import { validateUrdfInertiaForMujoco, prepareUrdfForMujocoLoad } from '../utils/urdf-sanitize';
import {
  dumpFailedRobotBundle,
  formatUrdfLoadError,
  isUrdfDebugEnabled,
  logUrdfSnippet,
} from '../utils/urdf-debug';

const VFS_ROOT = '/robot';

const MUJOCO_MODULE_KEY = '__armSimMujocoModule';
const MUJOCO_LOAD_CHAIN_KEY = '__armSimMujocoLoadChain';
const ACTIVE_HANDLES_KEY = '__armSimMujocoActiveHandles';

const inBrowser = typeof window !== 'undefined' && import.meta.env.MODE !== 'test';

let mujocoModuleNode: MujocoModule | null = null;
let mujocoLoadChainNode: Promise<unknown> = Promise.resolve();
let activeMujocoHandlesNode: { model: MjModel; data: MjData } | null = null;

function getMujocoModuleRef(): MujocoModule | null {
  if (inBrowser && typeof window !== 'undefined') {
    return (window as unknown as Record<string, unknown>)[MUJOCO_MODULE_KEY] as
      | MujocoModule
      | null
      | undefined ?? null;
  }
  return mujocoModuleNode;
}

function setMujocoModuleRef(mod: MujocoModule | null): void {
  if (inBrowser && typeof window !== 'undefined') {
    (window as unknown as Record<string, unknown>)[MUJOCO_MODULE_KEY] = mod;
  } else {
    mujocoModuleNode = mod;
  }
}

function getMujocoLoadChain(): Promise<unknown> {
  if (inBrowser && typeof window !== 'undefined') {
    const w = window as unknown as Record<string, unknown>;
    if (!w[MUJOCO_LOAD_CHAIN_KEY]) {
      w[MUJOCO_LOAD_CHAIN_KEY] = Promise.resolve();
    }
    return w[MUJOCO_LOAD_CHAIN_KEY] as Promise<unknown>;
  }
  return mujocoLoadChainNode;
}

function setMujocoLoadChain(chain: Promise<unknown>): void {
  if (inBrowser && typeof window !== 'undefined') {
    (window as unknown as Record<string, unknown>)[MUJOCO_LOAD_CHAIN_KEY] = chain;
  } else {
    mujocoLoadChainNode = chain;
  }
}

function getActiveMujocoHandles(): { model: MjModel; data: MjData } | null {
  if (inBrowser && typeof window !== 'undefined') {
    return (window as unknown as Record<string, unknown>)[ACTIVE_HANDLES_KEY] as
      | { model: MjModel; data: MjData }
      | null
      | undefined ?? null;
  }
  return activeMujocoHandlesNode;
}

function setActiveMujocoHandles(handles: { model: MjModel; data: MjData } | null): void {
  if (inBrowser && typeof window !== 'undefined') {
    (window as unknown as Record<string, unknown>)[ACTIVE_HANDLES_KEY] = handles;
  } else {
    activeMujocoHandlesNode = handles;
  }
}

/** 释放当前 WASM 中唯一的活动 MuJoCo 模型（重载前必须调用） */
export function releaseActiveMujocoHandles(): void {
  const handles = getActiveMujocoHandles();
  if (!handles) return;
  setActiveMujocoHandles(null);
  try {
    handles.data.delete();
  } catch {
    // already freed
  }
  try {
    handles.model.delete();
  } catch {
    // already freed
  }
}

function registerActiveMujocoHandles(model: MjModel, data: MjData): void {
  setActiveMujocoHandles({ model, data });
}

function enqueueMujocoLoad<T>(task: () => Promise<T>): Promise<T> {
  const run = () => task();
  const chain = getMujocoLoadChain();
  const result = chain.then(run, run);
  setMujocoLoadChain(
    result.then(
      () => undefined,
      () => undefined,
    ),
  );
  return result;
}

export async function getMujocoModule(): Promise<MujocoModule> {
  let mod = getMujocoModuleRef();
  if (!mod) {
    mod = inBrowser
      ? await loadMujoco({
          locateFile: (path: string) =>
            path.endsWith('.wasm') ? mujocoWasmUrl : path,
        })
      : await loadMujoco();
    setMujocoModuleRef(mod);
  }
  return mod;
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

function extractMeshRefs(urdfText: string): string[] {
  const refs = new Set<string>();
  const re = /filename="([^"]+)"/gi;
  let match: RegExpExecArray | null;
  while ((match = re.exec(urdfText)) !== null) {
    const raw = match[1];
    if (raw && /\.(stl|dae|obj|ply)$/i.test(raw)) refs.add(raw);
  }
  return [...refs];
}

function resolveMeshBytes(
  ref: string,
  urdfFileName: string,
  meshes: Map<string, Uint8Array>,
): Uint8Array | null {
  const base = ref.split('/').pop() ?? ref;
  const urdfDir = urdfFileName.includes('/')
    ? urdfFileName.slice(0, urdfFileName.lastIndexOf('/'))
    : '';
  const candidates = new Set<string>([
    ref,
    normalizeAssetPath(ref),
    base,
    `meshes/${base}`,
    `../meshes/${base}`,
    ref.replace(/^\.\.\//, ''),
  ]);
  if (urdfDir) {
    candidates.add(`${urdfDir}/${ref}`.replace(/\/+/g, '/').replace(/^\.\//, ''));
  }
  for (const key of candidates) {
    const bytes = meshes.get(key);
    if (bytes) return bytes;
  }
  return null;
}

function buildMujocoVfs(
  mujoco: MujocoModule,
  bundle: RobotAssetBundle,
): InstanceType<typeof mujoco.MjVFS> | null {
  const vfs = new mujoco.MjVFS();
  const seen = new Set<string>();
  let count = 0;

  const add = (name: string, bytes: Uint8Array) => {
    if (seen.has(name)) return;
    seen.add(name);
    vfs.addBuffer(name, bytes);
    count += 1;
  };

  for (const ref of extractMeshRefs(bundle.urdfText)) {
    const bytes = resolveMeshBytes(ref, bundle.urdfFileName, bundle.meshes);
    if (!bytes) continue;
    const base = ref.split('/').pop() ?? ref;
    add(ref, bytes);
    add(normalizeAssetPath(ref), bytes);
    add(base, bytes);
    add(`meshes/${base}`, bytes);
    add(`../meshes/${base}`, bytes);
  }

  return count > 0 ? vfs : null;
}

function mountAssets(mujoco: MujocoModule, bundle: RobotAssetBundle): string {
  ensureVfsDir(mujoco, VFS_ROOT);

  const urdfPath = `${VFS_ROOT}/${bundle.urdfFileName}`;
  const urdfDir = urdfPath.substring(0, urdfPath.lastIndexOf('/'));
  if (urdfDir.length > VFS_ROOT.length) {
    ensureVfsDir(mujoco, urdfDir);
  }
  try {
    mujoco.FS.unlink(urdfPath);
  } catch {
    // 首次写入
  }
  mujoco.FS.writeFile(urdfPath, bundle.urdfText);

  for (const [relativePath, bytes] of bundle.meshes) {
    const normalized = normalizeAssetPath(relativePath);
    const fullPath = `${VFS_ROOT}/${normalized}`;
    const dir = fullPath.substring(0, fullPath.lastIndexOf('/'));
    if (dir.length > VFS_ROOT.length) {
      ensureVfsDir(mujoco, dir);
    }
    try {
      mujoco.FS.unlink(fullPath);
    } catch {
      // 首次写入
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
  return enqueueMujocoLoad(async () => {
  releaseActiveMujocoHandles();
  const mujoco = await getMujocoModule();
  const rawLen = bundle.urdfText.length;
  const urdfText = bundle.urdfPrepared
    ? bundle.urdfText
    : prepareUrdfForMujocoLoad(bundle.urdfText);
  if (!bundle.urdfPrepared) {
    validateUrdfInertiaForMujoco(urdfText);
  }
  if (isUrdfDebugEnabled()) {
    logUrdfSnippet(`loadMujocoRobot [${bundle.loadPhase ?? '?'}]`, urdfText);
  }

  const physicsUrdf = stripMeshVisualsForMujoco(urdfText);
  const vfs =
    urdfReferencesMeshFiles(physicsUrdf) && bundle.meshes.size > 0
      ? buildMujocoVfs(mujoco, { ...bundle, urdfText: physicsUrdf })
      : null;
  const xmlPath = inBrowser ? '' : mountAssets(mujoco, { ...bundle, urdfText: physicsUrdf });

  let model: MjModel;
  try {
    if (inBrowser) {
      model =
        vfs != null
          ? mujoco.MjModel.from_xml_string(physicsUrdf, vfs)
          : mujoco.MjModel.from_xml_string(physicsUrdf);
    } else {
      model =
        vfs != null
          ? mujoco.MjModel.from_xml_path(xmlPath, vfs)
          : mujoco.MjModel.from_xml_path(xmlPath);
    }
  } catch (e) {
    releaseActiveMujocoHandles();
    const detail = e instanceof Error ? e.message : String(e);
    const dumpPath = await dumpFailedRobotBundle({
      urdfText,
      urdfFileName: bundle.urdfFileName,
      meshes: bundle.meshes,
      detail,
      loadPhase: bundle.loadPhase,
      rawUrdfLength: rawLen,
    });
    throw new Error(formatUrdfLoadError(urdfText, detail, dumpPath));
  } finally {
    vfs?.delete?.();
  }
  model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT.value;

  const data = new mujoco.MjData(model);
  registerActiveMujocoHandles(model, data);
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
  });
}

/** 释放 Embind 对象（模型重载时调用） */
export function disposeMujocoRobot(model: MjModel, data: MjData): void {
  const active = getActiveMujocoHandles();
  if (active?.model === model) {
    releaseActiveMujocoHandles();
    return;
  }
  data.delete();
  model.delete();
}
