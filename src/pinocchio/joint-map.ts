import { getMujocoJointAddresses } from '../mujoco/loader';
import type {
  JointMapping,
  MujocoLoadResult,
  MujocoModule,
  PinocchioLoadResult,
} from '../types/robot';

/**
 * 按关节名构建 MuJoCo ↔ Pinocchio 映射表。
 * pin_vidx 为 Pinocchio 速度空间下标（单 DOF 关节递增）。
 */
export function buildJointMap(
  mujocoBundle: MujocoLoadResult,
  pinResult: PinocchioLoadResult,
): JointMapping[];
export function buildJointMap(
  mujoco: MujocoModule,
  model: import('@mujoco/mujoco').MjModel,
  pinResult: PinocchioLoadResult,
): JointMapping[];
export function buildJointMap(
  a: MujocoLoadResult | MujocoModule,
  b: PinocchioLoadResult | import('@mujoco/mujoco').MjModel,
  c?: PinocchioLoadResult,
): JointMapping[] {
  const mujoco = c ? (a as MujocoModule) : (a as MujocoLoadResult).mujoco;
  const model = c ? (b as import('@mujoco/mujoco').MjModel) : (a as MujocoLoadResult).model;
  const pinResult = (c ?? b) as PinocchioLoadResult;

  const mjJoints = getMujocoJointAddresses(mujoco, model);
  const mjByName = new Map(mjJoints.map((j) => [j.name, j]));

  const pinVelocityIndex = new Map<string, number>();
  pinResult.jointNames.forEach((name, idx) => {
    pinVelocityIndex.set(name, idx);
  });

  const mappings: JointMapping[] = [];

  for (const mj of mjJoints) {
    mappings.push({
      name: mj.name,
      mj_qposadr: mj.qposadr,
      mj_dofadr: mj.dofadr,
      pin_vidx: pinVelocityIndex.get(mj.name) ?? -1,
    });
  }

  // Pinocchio 有而 MuJoCo 无的关节（理论上不应出现在同 URDF）
  for (const pinName of pinResult.jointNames) {
    if (!mjByName.has(pinName)) {
      mappings.push({
        name: pinName,
        mj_qposadr: -1,
        mj_dofadr: -1,
        pin_vidx: pinVelocityIndex.get(pinName) ?? -1,
      });
    }
  }

  return mappings;
}

export function jointMapNames(mappings: JointMapping[]): string[] {
  return mappings
    .filter((m) => m.pin_vidx >= 0 && m.mj_dofadr >= 0)
    .map((m) => m.name);
}
