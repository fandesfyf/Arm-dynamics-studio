import type {
  JointMapping,
  MujocoLoadResult,
  PinocchioLoadResult,
  RobotLoadInput,
} from '../types/robot';
import { loadMujocoRobot } from '../mujoco/loader';
import { loadPinocchioFromUrdf } from '../pinocchio/loader';
import { buildJointMap } from '../pinocchio/joint-map';
import { createJointMapAdapter } from '../pinocchio/joint-map-adapter';
import {
  applyFixedOriginsToPlacement,
  collectFixedOriginsToLink,
} from '../utils/ee-frame-utils';
import { resolveEndEffectorJointName } from '../utils/urdf-base-fixture';
import { vecGet } from '../types/mujoco';
import { ComputedTorqueController } from './controller';

/** 末端正运动学（由 pinocchio / T4 层实现） */
export interface ForwardKinematics {
  compute(q: ArrayLike<number>): { pos: number[]; quat: number[] };
}

/**
 * 机器人会话：双引擎模型加载、JointMap、资源释放。
 * 对应计划 robot-session + robot_model 职责划分。
 */
export class RobotSession {
  readonly jointMap: JointMapping[];
  readonly mujocoBundle: MujocoLoadResult;
  readonly pinocchioBundle: PinocchioLoadResult;
  readonly forwardKinematics: ForwardKinematics;

  private disposed = false;

  private constructor(
    mujocoBundle: MujocoLoadResult,
    pinocchioBundle: PinocchioLoadResult,
    jointMap: JointMapping[],
    forwardKinematics: ForwardKinematics,
  ) {
    this.mujocoBundle = mujocoBundle;
    this.pinocchioBundle = pinocchioBundle;
    this.jointMap = jointMap;
    this.forwardKinematics = forwardKinematics;
  }

  get mujoco() {
    return this.mujocoBundle.mujoco;
  }

  static async create(input: RobotLoadInput): Promise<RobotSession> {
    const meshes = input.meshes ?? new Map<string, Uint8Array>();
    const mujocoBundle = await loadMujocoRobot({
      urdfText: input.urdfXml,
      urdfFileName: input.urdfFileName ?? 'robot.urdf',
      meshes,
    });
    const pinocchioBundle = await loadPinocchioFromUrdf(input.urdfXml);
    const jointMap = buildJointMap(
      mujocoBundle.mujoco,
      mujocoBundle.model,
      pinocchioBundle,
    );
    const forwardKinematics = createForwardKinematics(
      pinocchioBundle,
      jointMap,
      input.urdfXml,
      input.endEffectorLink,
    );
    return new RobotSession(mujocoBundle, pinocchioBundle, jointMap, forwardKinematics);
  }

  get model() {
    return this.mujocoBundle.model;
  }

  get data() {
    return this.mujocoBundle.data;
  }

  get nq(): number {
    return this.mujocoBundle.nq;
  }

  get nv(): number {
    return this.mujocoBundle.nv;
  }

  get nu(): number {
    return this.mujocoBundle.nu;
  }

  get jointNames(): string[] {
    return this.mujocoBundle.jointNames;
  }

  get physicsDt(): number {
    return this.model.opt.timestep;
  }

  createController(): ComputedTorqueController {
    return new ComputedTorqueController(
      this.mujoco,
      this.model,
      this.data,
      this.nv,
      this.mujocoBundle.effortLimits,
      vecGet(this.data.qpos, this.nq),
      this.nq,
    );
  }

  dispose(): void {
    if (this.disposed) return;
    this.mujocoBundle.data.delete();
    this.mujocoBundle.model.delete();
    this.disposed = true;
  }
}

/** 按 URDF child link 解析 Pinocchio 关节 id（fixed 关节向上追溯） */
function resolveEndEffectorJointId(
  urdfXml: string,
  jointNames: string[],
  endEffectorLink?: string,
): number {
  const defaultId = jointNames.length;
  if (!endEffectorLink) return defaultId;

  const blocks = urdfXml.match(/<joint[\s\S]*?<\/joint>/g) ?? [];
  for (const block of blocks) {
    const childMatch = block.match(/<child\s+link="([^"]+)"/);
    if (!childMatch || childMatch[1] !== endEffectorLink) continue;

    const nameMatch = block.match(/name="([^"]+)"/);
    if (!nameMatch) continue;

    const idx = jointNames.indexOf(nameMatch[1]);
    if (idx >= 0) return idx + 1;

    const parentMatch = block.match(/<parent\s+link="([^"]+)"/);
    if (parentMatch) {
      return resolveEndEffectorJointId(urdfXml, jointNames, parentMatch[1]);
    }
  }

  return defaultId;
}

export function createForwardKinematics(
  pinBundle: PinocchioLoadResult,
  jointMap: JointMapping[],
  urdfXml: string,
  endEffectorLink?: string,
): ForwardKinematics {
  const { pin, model } = pinBundle;
  const data = new pin.Data(model);
  const adapter = createJointMapAdapter(jointMap);
  const jointId = resolveEndEffectorJointId(
    urdfXml,
    pinBundle.jointNames,
    endEffectorLink,
  );
  const resolvedJointName = resolveEndEffectorJointName(
    urdfXml,
    pinBundle.jointNames,
    endEffectorLink,
  );
  const fixedOrigins =
    resolvedJointName && endEffectorLink
      ? collectFixedOriginsToLink(urdfXml, endEffectorLink, resolvedJointName)
      : [];

  return {
    compute(q: ArrayLike<number>) {
      const pinQ = adapter.mjQposToPinQ(
        q instanceof Float64Array ? q : Float64Array.from(q),
      );
      pin.forwardKinematics(model, data, pinQ);
      pin.updateFramePlacements(model, data);
      const placement = pin.getJointPlacement(data, jointId);
      const { pos, quat } = applyFixedOriginsToPlacement(
        Array.from(placement.translation as ArrayLike<number>) as [number, number, number],
        placement.rotation as ArrayLike<number>,
        fixedOrigins,
      );
      return { pos, quat };
    },
  };
}
