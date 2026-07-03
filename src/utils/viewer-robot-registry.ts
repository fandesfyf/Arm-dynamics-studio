import type { URDFRobot } from 'urdf-loader';
import { readEeWorldFromRobot, worldToUrdfTarget } from '../viewer/ee-kinematics';

type RobotListener = (robot: URDFRobot | null) => void;

let mainRobot: URDFRobot | null = null;
let referenceRobot: URDFRobot | null = null;
const mainRobotListeners = new Set<RobotListener>();
const referenceRobotListeners = new Set<RobotListener>();

function notifyMain(): void {
  for (const listener of mainRobotListeners) {
    listener(mainRobot);
  }
}

function notifyReference(): void {
  for (const listener of referenceRobotListeners) {
    listener(referenceRobot);
  }
}

export function registerMainUrdfRobot(robot: URDFRobot | null): void {
  mainRobot = robot;
  notifyMain();
}

export function registerReferenceUrdfRobot(robot: URDFRobot | null): void {
  referenceRobot = robot;
  notifyReference();
}

export function getMainUrdfRobot(): URDFRobot | null {
  return mainRobot;
}

export function getReferenceUrdfRobot(): URDFRobot | null {
  return referenceRobot;
}

export function onMainUrdfRobotChange(listener: RobotListener): () => void {
  mainRobotListeners.add(listener);
  if (mainRobot) {
    listener(mainRobot);
  }
  return () => {
    mainRobotListeners.delete(listener);
  };
}

export function onReferenceUrdfRobotChange(listener: RobotListener): () => void {
  referenceRobotListeners.add(listener);
  if (referenceRobot) {
    listener(referenceRobot);
  }
  return () => {
    referenceRobotListeners.delete(listener);
  };
}

/** Prefer reference ghost robot for IK; fall back to main sim robot. */
export function getIkUrdfRobot(): URDFRobot | null {
  return referenceRobot ?? mainRobot;
}

/** Read end-effector link origin in Three.js world space (same frame as TransformControls). */
export function readEndEffectorSceneWorld(
  endEffectorLink: string,
): [number, number, number] | null {
  if (!mainRobot) return null;
  const world = readEeWorldFromRobot(mainRobot, endEffectorLink);
  if (!world) return null;
  return [world.x, world.y, world.z];
}

/** URDF-frame target derived from the visual main-robot end-effector pose. */
export function readEndEffectorUrdfTarget(endEffectorLink: string): [number, number, number] | null {
  if (!mainRobot) return null;
  const world = readEeWorldFromRobot(mainRobot, endEffectorLink);
  if (!world) return null;
  return worldToUrdfTarget(world);
}
