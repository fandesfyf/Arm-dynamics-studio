import type { URDFRobot } from 'urdf-loader';
import { IK_DOF } from './ik-dof';

// eslint-disable-next-line @typescript-eslint/no-explicit-any
type IkNode = any;

/**
 * urdfRobotToIKRoot 会把 URDF 关节当前旋转写进 IK 外层固定变换，
 * setIKFromUrdf 又会把同一角度写入内层 DoF，导致角度被重复计入。
 */
export function syncIkFixedJointOriginsFromUrdf(robot: URDFRobot, ikRoot: IkNode): void {
  if (!robot || !ikRoot?.traverse) return;

  ikRoot.traverse((c: IkNode) => {
    if (!c.isJoint || !c.name || c.name === '__world_joint__') return;

    const urdfJoint = robot.joints[c.name] as
      | {
          isURDFJoint?: boolean;
          jointType?: string;
          angle?: number;
          origPosition?: { x: number; y: number; z: number };
          origQuaternion?: { x: number; y: number; z: number; w: number };
          setJointValue: (v: number) => void;
        }
      | undefined;
    if (!urdfJoint?.isURDFJoint) return;
    if (
      urdfJoint.jointType !== 'revolute' &&
      urdfJoint.jointType !== 'continuous' &&
      urdfJoint.jointType !== 'prismatic'
    ) {
      return;
    }

    if (!urdfJoint.origPosition || !urdfJoint.origQuaternion) {
      urdfJoint.setJointValue(urdfJoint.angle ?? 0);
    }
    if (!urdfJoint.origPosition || !urdfJoint.origQuaternion) return;

    const linkParent = c.parent;
    const outer = linkParent?.parent;
    if (!outer?.isJoint || outer.name) return;

    const op = urdfJoint.origPosition;
    const oq = urdfJoint.origQuaternion;
    outer.setPosition(op.x, op.y, op.z);
    outer.setQuaternion(oq.x, oq.y, oq.z, oq.w);
    outer.setMatrixNeedsUpdate();
  });

  ikRoot.updateMatrixWorld(true);
}

export function applyAllIkJointsToUrdf(robot: URDFRobot, ikRoot: IkNode): void {
  if (!robot || !ikRoot?.traverse) return;
  ikRoot.updateMatrixWorld(true);
  ikRoot.traverse((c: IkNode) => {
    if (!c.isJoint || !c.name || c.name === '__world_joint__') return;
    const urdfJoint = robot.joints[c.name];
    if (!urdfJoint) return;
    if (urdfJoint.jointType === 'prismatic') {
      urdfJoint.setJointValue(c.getDoFValue(IK_DOF.Z));
    } else {
      urdfJoint.setJointValue(c.getDoFValue(IK_DOF.EZ));
    }
  });
  robot.updateMatrixWorld(true);
}
