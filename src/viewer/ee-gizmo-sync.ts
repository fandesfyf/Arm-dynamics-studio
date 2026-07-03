import * as THREE from 'three';
import type { URDFRobot } from 'urdf-loader';
import type { Quat, Vec3 } from '../core/trajectory';
import { readEeWorldPoseFromRobot, urdfTargetToWorld } from './ee-kinematics';

const _euler = new THREE.Euler(0, 0, 0, 'ZYX');
const _quat = new THREE.Quaternion();

/** URDF FK quaternion (x,y,z,w) → Three.js scene world quaternion. */
export function urdfQuatToWorld(quat: Quat): THREE.Quaternion {
  _quat.set(quat[0], quat[1], quat[2], quat[3]);
  const frame = new THREE.Quaternion().setFromEuler(_euler.set(-Math.PI / 2, 0, 0));
  return frame.multiply(_quat);
}

export interface GizmoWorldPoseOptions {
  /** When true, gizmo follows eeTarget / eeTargetQuat instead of main-robot FK. */
  preferTarget?: boolean;
}

export interface GizmoWorldPose {
  position: THREE.Vector3;
  quaternion: THREE.Quaternion;
}

/**
 * Gizmo world pose. Default: live FK from main URDF robot.
 * Use preferTarget only after explicit panel / IK command edit (eeTargetDirty).
 */
export function computeGizmoWorldPose(
  mainRobot: URDFRobot | null,
  endEffectorLink: string,
  eeTarget: Vec3,
  eeTargetQuat: Quat,
  options?: GizmoWorldPoseOptions,
): GizmoWorldPose | null {
  if (options?.preferTarget) {
    return {
      position: urdfTargetToWorld(eeTarget),
      quaternion: urdfQuatToWorld(eeTargetQuat),
    };
  }

  if (mainRobot) {
    const visual = readEeWorldPoseFromRobot(mainRobot, endEffectorLink);
    if (visual) {
      return visual;
    }
  }
  const fallbackPos = urdfTargetToWorld(eeTarget);
  return mainRobot
    ? null
    : {
        position: fallbackPos,
        quaternion: urdfQuatToWorld(eeTargetQuat),
      };
}

/** @deprecated Use computeGizmoWorldPose */
export function computeGizmoWorldPosition(
  mainRobot: URDFRobot | null,
  endEffectorLink: string,
  eeTarget: Vec3,
  options?: GizmoWorldPoseOptions,
): { x: number; y: number; z: number } | null {
  const pose = computeGizmoWorldPose(mainRobot, endEffectorLink, eeTarget, [0, 0, 0, 1], options);
  if (!pose) return null;
  return { x: pose.position.x, y: pose.position.y, z: pose.position.z };
}
