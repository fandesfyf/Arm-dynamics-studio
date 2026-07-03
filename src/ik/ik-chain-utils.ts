import type { Object3D } from 'three';
import type { URDFRobot } from 'urdf-loader';

const CHAIN_STOP_KEYWORDS = [
  'waist',
  'torso',
  'pelvis',
  'base_link',
  'base',
  'trunk',
  'lumbar',
];

const ROOT_LINK_NAMES = new Set(['base_link', 'world']);

export type ArmSide = 'left' | 'right';

function getObjectUrdfName(obj: Object3D): string {
  const named = obj as Object3D & { urdfName?: string; name: string };
  return named.urdfName || named.name || '';
}

/** Detect left/right arm from link or joint naming (biped zarm_l/r, left_/right_, l_arm/r_arm). */
export function detectArmSideFromName(name: string): ArmSide | undefined {
  const n = name.toLowerCase();
  if (
    n.startsWith('left_') ||
    n.startsWith('l_') ||
    n.includes('l_arm') ||
    n.includes('zarm_l') ||
    /^zarm_l/.test(n)
  ) {
    return 'left';
  }
  if (
    n.startsWith('right_') ||
    n.startsWith('r_') ||
    n.includes('r_arm') ||
    n.includes('zarm_r') ||
    /^zarm_r/.test(n)
  ) {
    return 'right';
  }
  return undefined;
}

export function isOppositeArm(name: string, side: ArmSide): boolean {
  const detected = detectArmSideFromName(name);
  return detected !== undefined && detected !== side;
}

function shouldStopIkChainWalk(jointName: string, side: ArmSide | undefined): boolean {
  const nameLower = jointName.toLowerCase();
  if (CHAIN_STOP_KEYWORDS.some((k) => nameLower.includes(k))) return true;
  if (side && isOppositeArm(jointName, side)) return true;
  return false;
}

export interface KinematicChainFrames {
  joints: Object3D[];
  links: Object3D[];
}

function isSharedTorsoName(name: string): boolean {
  const n = name.toLowerCase();
  return (
    ROOT_LINK_NAMES.has(n) ||
    n === 'base_link' ||
    n.includes('waist') ||
    n.includes('torso') ||
    n.includes('pelvis') ||
    n.includes('trunk') ||
    n.includes('lumbar')
  );
}

export interface FullVizChainOptions {
  rootLink?: string;
}

/**
 * Full visualization chain: end-effector → base_link (or world).
 * Includes all joints (fixed + actuated) and all links; does not stop at waist/torso.
 */
export function collectFullVizChainFrames(
  robot: URDFRobot,
  endEffectorLinkName: string,
  options?: FullVizChainOptions,
): KinematicChainFrames {
  const rootLink = options?.rootLink ?? 'base_link';
  const endLink = getUrdfLinkObject(robot, endEffectorLinkName);
  if (!endLink) return { joints: [], links: [] };

  const eeSide = detectArmSideFromName(endEffectorLinkName);
  const joints: Object3D[] = [];
  const links: Object3D[] = [endLink];
  let current: Object3D | null = endLink;

  while (current) {
    const linkName = getObjectUrdfName(current);
    if (ROOT_LINK_NAMES.has(linkName) || linkName === rootLink) break;

    const parentJoint = current.parent as Object3D & {
      isURDFJoint?: boolean;
      urdfName?: string;
      name: string;
      parent?: Object3D | null;
    };
    if (!parentJoint?.isURDFJoint) break;

    const jointName = getObjectUrdfName(parentJoint);
    const parentLink = parentJoint.parent;
    if (!parentLink) break;
    const parentLinkName = getObjectUrdfName(parentLink);

    if (
      eeSide &&
      !isSharedTorsoName(jointName) &&
      !isSharedTorsoName(parentLinkName) &&
      (isOppositeArm(jointName, eeSide) || isOppositeArm(parentLinkName, eeSide))
    ) {
      break;
    }

    joints.unshift(parentJoint);
    links.unshift(parentLink);
    current = parentLink;

    if (ROOT_LINK_NAMES.has(parentLinkName) || parentLinkName === rootLink) break;
  }

  return { joints, links };
}

/**
 * All links and actuated joints on the robot — for full-body reference pose TF overlay.
 * Unlike collectFullVizChainFrames, does not stop at waist or opposite arm.
 */
export function collectAllReferenceTfFrames(robot: URDFRobot): KinematicChainFrames {
  const links: Object3D[] = Object.values(robot.links);
  const joints: Object3D[] = [];
  for (const joint of Object.values(robot.joints)) {
    if (
      joint.jointType === 'revolute' ||
      joint.jointType === 'continuous' ||
      joint.jointType === 'prismatic'
    ) {
      joints.push(joint);
    }
  }
  return { joints, links };
}

/** @deprecated Use collectFullVizChainFrames for visualization */
export function collectKinematicChainFrames(
  robot: URDFRobot,
  endEffectorLinkName: string,
): KinematicChainFrames {
  return collectFullVizChainFrames(robot, endEffectorLinkName);
}

export function getUrdfLinkObject(robot: URDFRobot, linkName: string): Object3D | null {
  if (robot.links?.[linkName]) return robot.links[linkName]!;
  let found: Object3D | null = null;
  robot.traverse((obj) => {
    const urdfLink = obj as Object3D & { isURDFLink?: boolean; urdfName?: string };
    if (urdfLink.isURDFLink && (urdfLink.urdfName === linkName || urdfLink.name === linkName)) {
      found = obj;
    }
  });
  return found;
}

/** 从末端 link 向上收集运动链上的关节名（至 rootLink / 躯干 / 对侧肢体为止，供 IK 使用） */
export function inferChainJointNames(
  robot: URDFRobot,
  endEffectorLinkName: string,
  options?: { rootLink?: string },
): string[] {
  if (!endEffectorLinkName) return [];

  const rootLink = options?.rootLink ?? 'base_link';
  const endLink = getUrdfLinkObject(robot, endEffectorLinkName);
  if (!endLink) return [];

  const eeSide = detectArmSideFromName(endEffectorLinkName);
  const jointNames: string[] = [];
  let current: Object3D | null = endLink;

  while (current) {
    const joint = current as Object3D & {
      isURDFJoint?: boolean;
      urdfName?: string;
      name: string;
      jointType?: string;
      parent: Object3D | null;
    };
    if (joint.isURDFJoint) {
      const name = getObjectUrdfName(joint);
      const parentLink = joint.parent;
      const parentLinkName = parentLink ? getObjectUrdfName(parentLink) : '';

      if (shouldStopIkChainWalk(name, eeSide)) break;

      if (parentLinkName === rootLink || ROOT_LINK_NAMES.has(parentLinkName)) {
        if (joint.jointType !== 'fixed') {
          jointNames.unshift(name);
        }
        break;
      }

      if (joint.jointType !== 'fixed') {
        jointNames.unshift(name);
      }
    }
    current = current.parent;
  }

  return jointNames;
}

export function findIkLinkByName(
  ikRoot: { traverse: (fn: (c: IkTreeNode) => void) => void },
  linkName: string,
): IkTreeNode | null {
  let found: IkTreeNode | null = null;
  ikRoot.traverse((c) => {
    if (c.isLink && (c.name === linkName || c.urdfName === linkName)) {
      found = c;
    }
  });
  return found;
}

export interface IkTreeNode {
  isLink?: boolean;
  isJoint?: boolean;
  name?: string;
  urdfName?: string;
  parent?: IkTreeNode | null;
  dof?: unknown[];
  traverse?: (fn: (c: IkTreeNode) => void) => void;
  getDoFValue?: (dof: number) => number;
  setMinLimit?: (dof: number, v: number) => void;
  setMaxLimit?: (dof: number, v: number) => void;
  updateMatrixWorld?: (force?: boolean) => void;
  getWorldPosition?: (target: { set: (x: number, y: number, z: number) => void }) => void;
  getWorldQuaternion?: (target: {
    set: (x: number, y: number, z: number, w: number) => void;
  }) => void;
}
