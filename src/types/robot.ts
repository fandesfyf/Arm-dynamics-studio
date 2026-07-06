import type { MjData, MjModel } from '@mujoco/mujoco';

/** 跨 MuJoCo / Pinocchio 的关节名对齐映射 */
export interface JointMapping {
  name: string;
  mj_qposadr: number;
  mj_dofadr: number;
  pin_vidx: number;
}

export interface RobotAssetBundle {
  urdfText: string;
  urdfFileName: string;
  meshes: Map<string, Uint8Array>;
  /** dev 调试：区分初次加载与负载重载 */
  loadPhase?: 'initial' | 'payload-reload' | 'manual';
  /** 已由 prepareUrdfForMujocoLoad 处理，跳过 loader 内二次清洗与校验 */
  urdfPrepared?: boolean;
}

export interface MujocoLoadResult {
  mujoco: MujocoModule;
  model: MjModel;
  data: MjData;
  jointNames: string[];
  nq: number;
  nv: number;
  nu: number;
  /** 各活动关节力矩上限（与 jointNames 同序，来自 URDF effort） */
  effortLimits: number[];
}

/** @mujoco/mujoco 运行时模块（loadMujoco 返回值） */
export type MujocoModule = Awaited<
  ReturnType<typeof import('@mujoco/mujoco').default>
>;

/** pinocchio-js WASM 模块 */
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type PinocchioModule = any;

export interface PinocchioLoadResult {
  pin: PinocchioModule;
  model: PinocchioModule['Model'];
  jointNames: string[];
  lowerLimits: number[];
  upperLimits: number[];
  neutralConfiguration: number[];
  nq: number;
  nv: number;
}

/** pinocchio-js 加载结果（T2 robot-session 别名） */
export type PinocchioRobotBundle = PinocchioLoadResult;

/** URDF 加载输入 */
export interface RobotLoadInput {
  urdfXml: string;
  urdfFileName?: string;
  meshes?: Map<string, Uint8Array>;
  endEffectorLink?: string;
  baseLink?: string;
  loadPhase?: 'initial' | 'payload-reload' | 'manual';
  urdfPrepared?: boolean;
}
