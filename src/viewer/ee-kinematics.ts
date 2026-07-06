import * as THREE from 'three';
import type { URDFRobot } from 'urdf-loader';
import type { Vec3 } from '../core/trajectory';
import { getUrdfLinkObject } from '../ik/ik-chain-utils';

/** URDF Z-up root → Three.js Y-up (matches UrdfModel rotation.x). */
export const Z_UP_TO_Y_UP = -Math.PI / 2;

/** Pinocchio / URDF FK frame (Z-up) → Three.js scene world (Y-up). */
export function urdfTargetToWorld([x, y, z]: Vec3): THREE.Vector3 {
  return new THREE.Vector3(x, z, -y);
}

/** Three.js scene world (Y-up) → Pinocchio / URDF FK frame (Z-up). */
export function worldToUrdfTarget(pos: THREE.Vector3 | [number, number, number]): Vec3 {
  const x = Array.isArray(pos) ? pos[0] : pos.x;
  const y = Array.isArray(pos) ? pos[1] : pos.y;
  const z = Array.isArray(pos) ? pos[2] : pos.z;
  return [x, -z, y];
}

/** @deprecated Use urdfTargetToWorld — kept for gradual migration. */
export function fkToScene(target: Vec3): [number, number, number] {
  const v = urdfTargetToWorld(target);
  return [v.x, v.y, v.z];
}

/** @deprecated Use worldToUrdfTarget — kept for gradual migration. */
export function sceneToFk(scene: [number, number, number]): Vec3 {
  return worldToUrdfTarget(scene);
}

/** Authoritative end-effector world pose from the Three.js URDF robot. */
export function readEeWorldPoseFromRobot(
  robot: URDFRobot,
  linkName: string,
): { position: THREE.Vector3; quaternion: THREE.Quaternion } | null {
  const link = getUrdfLinkObject(robot, linkName);
  if (!link) return null;
  link.updateWorldMatrix(true, false);
  const position = new THREE.Vector3();
  const quaternion = new THREE.Quaternion();
  link.getWorldPosition(position);
  link.getWorldQuaternion(quaternion);
  return { position, quaternion };
}

/** Authoritative end-effector world position from the Three.js URDF robot. */
export function readEeWorldFromRobot(robot: URDFRobot, linkName: string): THREE.Vector3 | null {
  const pose = readEeWorldPoseFromRobot(robot, linkName);
  return pose?.position ?? null;
}

export function applyAllJointAngles(
  robot: URDFRobot,
  jointNames: string[],
  angles: number[],
): void {
  for (let i = 0; i < jointNames.length; i++) {
    const name = jointNames[i]!;
    if (robot.joints[name]) {
      robot.setJointValue(name, angles[i] ?? 0);
    }
  }
  robot.updateMatrixWorld(true);
}

/** 临时设置关节角后读取末端 link 的 Three.js 世界坐标，再恢复原关节角。 */
export function readEeSceneWorldForJointAngles(
  robot: URDFRobot,
  jointNames: string[],
  angles: number[],
  endEffectorLink: string,
): [number, number, number] | null {
  const saved = jointNames.map((name) => robot.joints[name]?.angle ?? 0);
  applyAllJointAngles(robot, jointNames, angles);
  const world = readEeWorldFromRobot(robot, endEffectorLink);
  applyAllJointAngles(robot, jointNames, saved);
  if (!world) return null;
  return [world.x, world.y, world.z];
}
