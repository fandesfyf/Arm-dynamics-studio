import type { Quat, Vec3 } from '../core/trajectory';

export type InterpProfile = 'linear' | 'cubic';

export type MotionTargetSource = 'joint' | 'ee';

/** 插值队列中的一帧目标（执行时统一在关节空间插值） */
export interface MotionTarget {
  id: string;
  source: MotionTargetSource;
  jointPositions: number[];
  /** 末端 URDF FK 位置（Pinocchio） */
  eePosition: Vec3;
  eeQuaternion: Quat;
  /** 选中末端 link 在 Three.js 场景中的世界坐标（与 Gizmo 同帧） */
  eeSceneWorld: [number, number, number];
}

export function createMotionTargetId(): string {
  return `mt-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 7)}`;
}

/** 三次样条至少需要 3 个路点（含起点）；否则回退线性。 */
export function resolveInterpProfile(
  profile: InterpProfile,
  waypointCountIncludingStart: number,
): InterpProfile {
  if (profile === 'cubic' && waypointCountIncludingStart >= 3) {
    return 'cubic';
  }
  return 'linear';
}
