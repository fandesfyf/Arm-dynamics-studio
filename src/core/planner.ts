import type { Quat, TrajectorySample, Vec3 } from './trajectory';

/** 关节空间期望量 */
export interface JointDesired {
  q_d: Float64Array;
  v_d: Float64Array;
  a_d: Float64Array;
}

/** 末端轨迹采样接口（T3 Trajectory.sample 可适配为此接口） */
export interface TrajectorySampler {
  getDuration(): number;
  sample(t: number): TrajectorySample;
}

/** 将 T3 Trajectory 适配为 TrajectorySampler */
export interface TrajectoryLike {
  getDuration(): number;
  sample(t: number): TrajectorySample;
}

export function asTrajectorySampler(traj: TrajectoryLike): TrajectorySampler {
  return traj;
}

/**
 * 恒定关节目标规划器：q_d 固定，v_d = 0，a_d = 0。
 * 对齐 WEB_IMPLEMENTATION_PLAN.md §5.6 runToTarget 行为。
 */
/**
 * 关节空间线性插值：qStart → qEnd，时长 duration 秒内匀速期望速度。
 */
function jointDeltaNorm(qStart: ArrayLike<number>, qEnd: ArrayLike<number>): number {
  let max = 0;
  const n = Math.min(qStart.length, qEnd.length);
  for (let i = 0; i < n; i++) {
    max = Math.max(max, Math.abs((qEnd[i] ?? 0) - (qStart[i] ?? 0)));
  }
  return max;
}

function resolvePerJointMaxVel(
  maxJointVelRadPerSec: number | number[],
  n: number,
): number[] {
  if (typeof maxJointVelRadPerSec === 'number') {
    const v = Math.max(maxJointVelRadPerSec, 1e-6);
    return Array.from({ length: n }, () => v);
  }
  const out = new Array<number>(n);
  for (let i = 0; i < n; i++) {
    out[i] = Math.max(maxJointVelRadPerSec[i] ?? 1, 1e-6);
  }
  return out;
}

/**
 * Joint-space linear interpolation with per-joint velocity cap.
 * duration = max_i |Δq_i| / maxVel_i
 */
export class JointVelocityLimitPlanner {
  private readonly qStart: Float64Array;
  private readonly qEnd: Float64Array;
  private readonly duration: number;
  private readonly zeros: Float64Array;
  private readonly maxVel: number[];

  constructor(
    qStart: ArrayLike<number>,
    qEnd: ArrayLike<number>,
    maxJointVelRadPerSec: number | number[],
    nv: number,
  ) {
    this.qStart = Float64Array.from(qStart);
    this.qEnd = Float64Array.from(qEnd);
    this.zeros = new Float64Array(nv);
    this.maxVel = resolvePerJointMaxVel(maxJointVelRadPerSec, this.qStart.length);

    let duration = 0;
    for (let i = 0; i < this.qStart.length; i++) {
      const delta = Math.abs((this.qEnd[i] ?? 0) - (this.qStart[i] ?? 0));
      if (delta > 0) {
        duration = Math.max(duration, delta / this.maxVel[i]!);
      }
    }
    this.duration = duration;
  }

  getDuration(): number {
    return this.duration;
  }

  isSettled(t: number, tol = 1e-4): boolean {
    return t >= this.duration - tol || jointDeltaNorm(this.qStart, this.qEnd) <= tol;
  }

  getDesired(t: number): JointDesired {
    const alpha =
      this.duration <= 0 ? 1 : Math.min(1, Math.max(0, t / this.duration));
    const n = this.qStart.length;
    const q_d = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      q_d[i] = (this.qStart[i] ?? 0) + alpha * ((this.qEnd[i] ?? 0) - (this.qStart[i] ?? 0));
    }
    const v_d = new Float64Array(this.zeros.length);
    if (this.duration > 0 && t < this.duration) {
      for (let i = 0; i < v_d.length; i++) {
        v_d[i] = ((this.qEnd[i] ?? 0) - (this.qStart[i] ?? 0)) / this.duration;
      }
    }
    return { q_d, v_d, a_d: this.zeros };
  }
}

export class JointInterpolationPlanner {
  private readonly qStart: Float64Array;
  private readonly qEnd: Float64Array;
  private readonly duration: number;
  private readonly zeros: Float64Array;

  constructor(qStart: ArrayLike<number>, qEnd: ArrayLike<number>, duration: number, nv: number) {
    this.qStart = Float64Array.from(qStart);
    this.qEnd = Float64Array.from(qEnd);
    this.duration = Math.max(0, duration);
    this.zeros = new Float64Array(nv);
  }

  getDuration(): number {
    return this.duration;
  }

  getDesired(t: number): JointDesired {
    const alpha =
      this.duration <= 0 ? 1 : Math.min(1, Math.max(0, t / this.duration));
    const n = this.qStart.length;
    const q_d = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      q_d[i] = (this.qStart[i] ?? 0) + alpha * ((this.qEnd[i] ?? 0) - (this.qStart[i] ?? 0));
    }
    const v_d = new Float64Array(this.zeros.length);
    if (this.duration > 0 && t < this.duration) {
      for (let i = 0; i < v_d.length; i++) {
        v_d[i] = ((this.qEnd[i] ?? 0) - (this.qStart[i] ?? 0)) / this.duration;
      }
    }
    return { q_d, v_d, a_d: this.zeros };
  }
}

export class ConstantJointPlanner {
  private readonly qTarget: Float64Array;
  private readonly zeros: Float64Array;

  constructor(qTarget: ArrayLike<number>, nv: number) {
    this.qTarget = Float64Array.from(qTarget);
    this.zeros = new Float64Array(nv);
  }

  getDesired(_time: number): JointDesired {
    return {
      q_d: this.qTarget,
      v_d: this.zeros,
      a_d: this.zeros,
    };
  }

  getTarget(): Float64Array {
    return new Float64Array(this.qTarget);
  }
}

/**
 * 轨迹时刻采样：将 TrajectorySampler 的末端位姿转为供 IK 使用的目标。
 * 关节期望 q_d/v_d/a_d 由 simulation 层通过 IK + 数值微分生成。
 */
export class TrajectoryPosePlanner {
  constructor(private readonly sampler: TrajectorySampler) {}

  getDuration(): number {
    return this.sampler.getDuration();
  }

  sampleEePose(t: number): { pos: Vec3; quat: Quat } {
    const { ee_pos, ee_quat } = this.sampler.sample(t);
    return {
      pos: [...ee_pos] as Vec3,
      quat: [...ee_quat] as Quat,
    };
  }
}
