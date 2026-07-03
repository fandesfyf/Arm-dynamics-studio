import * as THREE from 'three';
import type { URDFRobot } from 'urdf-loader';
import { Solver } from 'closed-chain-ik/src/core/Solver.js';
import { Goal } from 'closed-chain-ik/src/core/Goal.js';
import { setIKFromUrdf, urdfRobotToIKRoot } from 'closed-chain-ik/src/three/urdfHelpers.js';
import type { Vec3 } from '../core/trajectory';
import { urdfTargetToWorld } from '../viewer/ee-kinematics';
import { findIkLinkByName, inferChainJointNames } from './ik-chain-utils';
import { mergeIkJointsWithChain } from './merge-ik-joints';
import { applyAllIkJointsToUrdf, syncIkFixedJointOriginsFromUrdf } from './ik-sync-utils';
import { IK_DOF } from './ik-dof';
import {
  capIterationsForLiveDrag,
  getDragEndWeights,
  type PositionIkWeights,
} from './ik-weight-config';

const _pos = new THREE.Vector3();
const _quat = new THREE.Quaternion();
const _euler = new THREE.Euler(0, 0, 0, 'ZYX');

// eslint-disable-next-line @typescript-eslint/no-explicit-any
type IkNode = any;

export interface ClosedChainIkSolveResult {
  success: boolean;
  jointAngles: number[];
  message?: string;
}

export class ClosedChainIkBridge {
  private robot: URDFRobot | null = null;
  private restoreDisplayJoints = false;
  private ikRoot: IkNode | null = null;
  private solver: Solver | null = null;
  private goal: Goal | null = null;
  private closureLink: IkNode | null = null;
  private chainJointNames: string[] = [];
  private jointNames: string[] = [];
  private endEffectorLinkName = '';
  private rootLink = 'base_link';
  private worldJoint: IkNode | null = null;
  private disabled = false;

  isReady(): boolean {
    return !this.disabled && !!(this.robot && this.solver && this.goal && this.closureLink);
  }

  getChainJointNames(): string[] {
    return [...this.chainJointNames];
  }

  dispose(): void {
    this.robot = null;
    this.ikRoot = null;
    this.solver = null;
    this.goal = null;
    this.closureLink = null;
    this.worldJoint = null;
    this.disabled = false;
  }

  /** Attach IK to a live Three.js URDF robot (reference ghost preferred). */
  rebuild(
    robot: URDFRobot,
    endEffectorLink: string,
    jointNames: string[],
    options?: { restoreDisplayJoints?: boolean; rootLink?: string },
  ): boolean {
    this.dispose();

    try {
      this.robot = robot;
      this.restoreDisplayJoints = options?.restoreDisplayJoints ?? false;
      this.jointNames = [...jointNames];
      this.endEffectorLinkName = endEffectorLink;
      this.rootLink = options?.rootLink ?? 'base_link';
      this.chainJointNames = inferChainJointNames(robot, endEffectorLink, {
        rootLink: this.rootLink,
      });

      this.ikRoot = urdfRobotToIKRoot(robot, false) as IkNode;
      if (!this.ikRoot) return false;

      this.closureLink = findIkLinkByName(this.ikRoot, endEffectorLink) as IkNode | null;
      if (!this.closureLink) {
        console.warn('closed-chain-ik: link not found', endEffectorLink);
        return false;
      }

      this.goal = new Goal();
      this.goal.makeClosure(this.closureLink);

      this.solver = new Solver(this.ikRoot);
      this.solver.maxIterations = 48;
      this.solver.dampingFactor = 0.002;
      this.solver.translationFactor = 1;
      this.solver.rotationFactor = 0.012;
      this.solver.translationErrorClamp = 0.05;
      this.solver.divergeThreshold = 0.25;
      this.solver.translationConvergeThreshold = 0.004;
      this.solver.stallThreshold = 1e-4;

      this.worldJoint = this.findWorldJoint();
      this.syncRobotToIk();
      return true;
    } catch (err) {
      console.warn('closed-chain-ik bridge rebuild failed:', err);
      this.dispose();
      return false;
    }
  }

  private findWorldJoint(): IkNode | null {
    let found: IkNode | null = null;
    this.ikRoot?.traverse((c: IkNode) => {
      if (c.isJoint && c.name === '__world_joint__') found = c;
    });
    return found;
  }

  private setJointAngles(jointAngles: number[]): void {
    if (!this.robot) return;
    for (let i = 0; i < this.jointNames.length; i++) {
      const name = this.jointNames[i]!;
      const value = jointAngles[i] ?? 0;
      if (this.robot.joints[name]) {
        this.robot.setJointValue(name, value);
      }
    }
    this.robot.updateMatrixWorld(true);
  }

  private readJointAngles(): number[] {
    if (!this.robot) return [];
    return this.jointNames.map((name) => this.robot!.joints[name]?.angle ?? 0);
  }

  private syncRobotToIk(): void {
    if (!this.robot || !this.ikRoot) return;
    this.robot.updateMatrixWorld(true);
    syncIkFixedJointOriginsFromUrdf(this.robot, this.ikRoot);
    setIKFromUrdf(this.ikRoot, this.robot);

    if (this.worldJoint) {
      const wj = this.worldJoint;
      wj.setDoFValue(IK_DOF.X, this.robot.position.x);
      wj.setDoFValue(IK_DOF.Y, this.robot.position.y);
      wj.setDoFValue(IK_DOF.Z, this.robot.position.z);
      _euler.setFromQuaternion(this.robot.quaternion, 'ZYX');
      wj.setDoFValue(IK_DOF.EX, _euler.x);
      wj.setDoFValue(IK_DOF.EY, _euler.y);
      wj.setDoFValue(IK_DOF.EZ, _euler.z);
      wj.setMatrixDoFNeedsUpdate();
    }

    this.ikRoot.updateMatrixWorld(true);
  }

  private lockJointAtCurrent(joint: IkNode): void {
    if (!joint?.dof?.length) return;
    for (const dof of joint.dof as number[]) {
      const v = joint.getDoFValue(dof);
      joint.setMinLimit(dof, v);
      joint.setMaxLimit(dof, v);
    }
  }

  private prepareIkTreeForSolve(): void {
    const chainSet = new Set(this.chainJointNames);
    if (this.worldJoint) {
      this.lockJointAtCurrent(this.worldJoint);
    }
    this.ikRoot?.traverse((c: IkNode) => {
      if (!c.isJoint || c.name === '__world_joint__') return;
      if (!chainSet.has(c.name)) {
        this.lockJointAtCurrent(c);
      }
    });
  }

  private setGoalTarget(position: THREE.Vector3, quaternion: THREE.Quaternion, positionOnly: boolean): void {
    if (!this.goal) return;
    if (positionOnly) {
      this.goal.setGoalDoF(IK_DOF.X, IK_DOF.Y, IK_DOF.Z);
      this.goal.setWorldPosition(position.x, position.y, position.z);
    } else {
      this.goal.setGoalDoF(IK_DOF.X, IK_DOF.Y, IK_DOF.Z, IK_DOF.EX, IK_DOF.EY, IK_DOF.EZ);
      this.goal.setWorldPosition(position.x, position.y, position.z);
      (this.goal as IkNode).setWorldQuaternion(
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
      );
    }
    (this.goal as IkNode).updateMatrixWorld(true);
  }

  private applySolverWeights(weights: PositionIkWeights): void {
    if (!this.solver) return;
    this.solver.maxIterations = weights.maxIterations;
    this.solver.dampingFactor = weights.dampingFactor;
    this.solver.translationFactor = weights.translationFactor;
    this.solver.rotationFactor = weights.rotationFactor;
    this.solver.translationErrorClamp = weights.translationErrorClamp;
    this.solver.divergeThreshold = weights.divergeThreshold;
    this.solver.translationConvergeThreshold = weights.convergedPositionTolerance;
  }

  private estimateError(
    targetPos: THREE.Vector3,
    targetQuat: THREE.Quaternion,
    positionOnly: boolean,
  ): { position: number; rotation: number } {
    const link = this.robot?.links?.[this.endEffectorLinkName];
    if (!link) return { position: Infinity, rotation: Infinity };
    link.getWorldPosition(_pos);
    link.getWorldQuaternion(_quat);
    const position = _pos.distanceTo(targetPos);
    const rotation = positionOnly ? 0 : _quat.angleTo(targetQuat);
    return { position, rotation };
  }

  solveTarget(
    targetFk: Vec3,
    jointAngles: number[],
    options: {
      positionOnly?: boolean;
      liveDrag?: boolean;
      dragEnd?: boolean;
      weights?: PositionIkWeights;
      /** Three.js world position — authoritative when dragging gizmo. */
      targetSceneWorld?: [number, number, number];
      /** Three.js world quaternion (x,y,z,w) — authoritative when dragging gizmo. */
      targetSceneQuaternion?: [number, number, number, number];
    } = {},
  ): ClosedChainIkSolveResult {
    if (!this.isReady() || !this.robot || !this.solver) {
      return { success: false, jointAngles, message: 'closed-chain-ik 未初始化' };
    }

    const positionOnly = options.positionOnly ?? true;
    let weights = options.weights;
    if (weights) {
      if (options.dragEnd) {
        weights = getDragEndWeights(weights);
      } else if (options.liveDrag) {
        weights = capIterationsForLiveDrag(weights, true);
      }
      this.applySolverWeights(weights);
    }

    const displaySnapshot = this.restoreDisplayJoints ? this.readJointAngles() : null;

    this.setJointAngles(jointAngles);

    if (options.targetSceneWorld) {
      const [sx, sy, sz] = options.targetSceneWorld;
      _pos.set(sx, sy, sz);
    } else {
      _pos.copy(urdfTargetToWorld(targetFk));
    }

    const eeLink = this.robot.links[this.endEffectorLinkName];
    if (options.targetSceneQuaternion) {
      const [qx, qy, qz, qw] = options.targetSceneQuaternion;
      _quat.set(qx, qy, qz, qw);
    } else if (eeLink) {
      eeLink.getWorldQuaternion(_quat);
    } else {
      _quat.identity();
    }

    this.syncRobotToIk();
    this.prepareIkTreeForSolve();
    this.setGoalTarget(_pos, _quat, positionOnly);

    try {
      this.solver.solve();
    } catch (err) {
      console.warn('closed-chain-ik solve failed, disabling bridge:', err);
      this.disabled = true;
      return { success: false, jointAngles, message: 'closed-chain-ik 求解异常' };
    }

    this.ikRoot?.updateMatrixWorld(true);
    applyAllIkJointsToUrdf(this.robot, this.ikRoot!);

    const err = this.estimateError(_pos, _quat, positionOnly);
    const posTol = options.weights?.convergedPositionTolerance ?? 0.03;
    const success = err.position < posTol;
    const solvedRaw = this.readJointAngles();
    const solved = mergeIkJointsWithChain(
      this.jointNames,
      this.chainJointNames,
      jointAngles,
      solvedRaw,
    );

    if (this.restoreDisplayJoints && displaySnapshot) {
      this.setJointAngles(displaySnapshot);
    }

    return {
      success,
      jointAngles: solved,
      message: success ? undefined : `IK 位置误差 ${err.position.toFixed(4)} m`,
    };
  }
}
