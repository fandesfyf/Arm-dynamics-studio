import type { RobotSession } from '../core/robot-session';
import type { IKSolver, IkSolveOptions } from '../core/simulation';
import type { InverseKinematics } from '../core/inverse-kinematics';
import { actuatedJointsToQpos, qposToActuatedJoints } from '../utils/joint-qpos';
import type { Vec3 } from '../core/trajectory';
import { ClosedChainIkBridge } from './closed-chain-ik-bridge';
import { mergeIkJointsWithChain } from './merge-ik-joints';

function isPositionOnlyGoal(goalMode?: IkSolveOptions['goalMode']): boolean {
  return goalMode !== 'pose' && goalMode !== 'orientation';
}

export function createHybridIkSolver(
  session: RobotSession,
  pinocchioIk: InverseKinematics,
  closedChainBridge: ClosedChainIkBridge | null,
): IKSolver {
  return {
    solve(pos, quat, qInit, options) {
      const seedJoints = qposToActuatedJoints(session, qInit);
      const positionOnly = isPositionOnlyGoal(options?.goalMode);
      const chainJointNames = closedChainBridge?.getChainJointNames() ?? session.jointNames;

      if (closedChainBridge?.isReady()) {
        try {
          const cci = closedChainBridge.solveTarget(pos as Vec3, seedJoints, {
            positionOnly,
            liveDrag: options?.liveDrag,
            dragEnd: options?.dragEnd,
            weights: options?.weights?.position,
            targetSceneWorld: options?.targetSceneWorld,
            targetSceneQuaternion: options?.targetSceneQuaternion,
          });
          if (cci.success) {
            const merged = mergeIkJointsWithChain(
              session.jointNames,
              chainJointNames,
              seedJoints,
              cci.jointAngles,
            );
            return {
              q: actuatedJointsToQpos(session, merged),
              converged: true,
            };
          }
        } catch (err) {
          console.warn('closed-chain-ik bridge error, falling back to Pinocchio:', err);
        }
      }

      const result = pinocchioIk.solve(pos, qInit);
      if (!result.converged) {
        console.warn(`Pinocchio IK 未收敛 (error=${result.error.toExponential(2)})`);
      }
      const solvedJoints = qposToActuatedJoints(session, result.q);
      const merged = mergeIkJointsWithChain(
        session.jointNames,
        chainJointNames,
        seedJoints,
        solvedJoints,
      );
      return {
        q: actuatedJointsToQpos(session, merged),
        converged: result.converged,
        message: result.converged ? undefined : `IK 误差 ${result.error.toExponential(2)}`,
      };
    },
  };
}
