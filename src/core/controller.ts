import type { MjData, MjModel } from '@mujoco/mujoco';
import type { MujocoModule } from '../types/robot';
import { restoreMjState, saveMjState, vecGet, vecSet, vecZero } from '../types/mujoco';

/** Conservative natural frequency — slower tracking, avoid oscillation (was 12). */
export const CONTROLLER_OMEGA = 6.0;
/** Overdamping factor vs critical (2.0); higher = less oscillation at target. */
export const CONTROLLER_KD_DAMPING = 4.5;
const OMEGA = CONTROLLER_OMEGA;
const KD_DAMPING = CONTROLLER_KD_DAMPING;
const MIN_KP = 0.5;
const MIN_INERTIA = 1e-6;
const DEFAULT_TORQUE_LIMIT = 100.0;

/** 浏览器 WASM 下 mj_fullM 会破坏后续 URDF 编译（MuJoCo XML 解析器状态损坏） */
const avoidMjFullMInBrowser =
  typeof window !== 'undefined' && import.meta.env.MODE !== 'test';

/** 关节角误差（处理 2π 环绕） */
export function angleDiff(qTarget: ArrayLike<number>, qCurrent: ArrayLike<number>): Float64Array {
  const n = qTarget.length;
  const out = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const diff = (qTarget[i] ?? 0) - (qCurrent[i] ?? 0);
    out[i] = Math.atan2(Math.sin(diff), Math.cos(diff));
  }
  return out;
}

function clipTorque(tau: Float64Array, limits: Float64Array): Float64Array {
  const out = new Float64Array(tau.length);
  for (let i = 0; i < tau.length; i++) {
    const lim = limits[i] ?? DEFAULT_TORQUE_LIMIT;
    out[i] = Math.max(-lim, Math.min(lim, tau[i] ?? 0));
  }
  return out;
}

export interface AutoGainOptions {
  omega?: number;
  kdDamping?: number;
  minKp?: number;
}

/** 由质量矩阵对角 M_ii 计算 Kp/Kd */
export function gainsFromMassDiagonal(
  massDiagonal: ArrayLike<number>,
  options: AutoGainOptions = {},
): { kp: Float64Array; kd: Float64Array } {
  const omega = options.omega ?? OMEGA;
  const kdDamping = options.kdDamping ?? KD_DAMPING;
  const minKp = options.minKp ?? MIN_KP;
  const n = massDiagonal.length;
  const kp = new Float64Array(n);
  const kd = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const mii = Math.max(massDiagonal[i] ?? 0, MIN_INERTIA);
    kp[i] = Math.max(omega * omega * mii, minKp);
    kd[i] = kdDamping * Math.sqrt(kp[i] * mii);
  }
  return { kp, kd };
}

/**
 * 计算力矩控制器 — mj_inverse 前馈 + 自适应 PD 反馈。
 * 对应旧版 controller.py / WEB_IMPLEMENTATION_PLAN.md §5.3。
 */
export class ComputedTorqueController {
  readonly kp: Float64Array;
  readonly kd: Float64Array;
  readonly torqueLimits: Float64Array;
  private readonly nq: number;

  constructor(
    private readonly mujoco: MujocoModule,
    private readonly model: MjModel,
    private readonly data: MjData,
    private readonly nv: number,
    torqueLimits?: ArrayLike<number>,
    _initialQpos?: ArrayLike<number>,
    nq?: number,
  ) {
    this.nq = nq ?? model.nq;
    // 增益在 loadRobot → recomputeAutoGains 中计算；构造时避免逆动力学/mj_fullM 拖慢加载
    this.kp = new Float64Array(this.nv).fill(MIN_KP * 20);
    this.kd = new Float64Array(this.nv).fill(MIN_KP * 2);
    this.torqueLimits = new Float64Array(this.nv);
    for (let i = 0; i < this.nv; i++) {
      const limit = torqueLimits?.[i];
      this.torqueLimits[i] =
        limit !== undefined && limit > 0 ? limit : DEFAULT_TORQUE_LIMIT;
    }
  }

  /** diag(mj_fullM(q))；浏览器 WASM 用逆动力学估计，避免 mj_fullM 导致模型无法重载 */
  massMatrixDiagonal(q: ArrayLike<number>): Float64Array {
    const saved = saveMjState(this.data, this.model);
    if (avoidMjFullMInBrowser) {
      const diag = this.massDiagonalFromInverse(q);
      restoreMjState(this.data, this.model, saved);
      this.mujoco.mj_forward(this.model, this.data);
      return diag;
    }

    vecSet(this.data.qpos, q, this.nq);
    vecZero(this.data.qvel, this.nv);
    vecZero(this.data.qacc, this.nv);
    this.mujoco.mj_forward(this.model, this.data);

    const M = new Float64Array(this.nv * this.nv);
    this.mujoco.mj_fullM(this.model, this.data, M);

    const diag = new Float64Array(this.nv);
    let hasNonZero = false;
    for (let i = 0; i < this.nv; i++) {
      diag[i] = M[i * this.nv + i] ?? 0;
      if (diag[i] > 1e-12) hasNonZero = true;
    }

    if (!hasNonZero) {
      diag.set(this.massDiagonalFromInverse(q));
    }

    restoreMjState(this.data, this.model, saved);
    this.mujoco.mj_forward(this.model, this.data);
    return diag;
  }

  /** M[i,i] ≈ τ_i | qacc=e_i − τ_i | qacc=0（v=0） */
  private massDiagonalFromInverse(q: ArrayLike<number>): Float64Array {
    const bias = this.inverseDynamics(q, new Float64Array(this.nv), new Float64Array(this.nv));
    const diag = new Float64Array(this.nv);
    for (let i = 0; i < this.nv; i++) {
      const qacc = new Float64Array(this.nv);
      qacc[i] = 1;
      const tau = this.inverseDynamics(q, new Float64Array(this.nv), qacc);
      diag[i] = Math.max((tau[i] ?? 0) - (bias[i] ?? 0), MIN_INERTIA);
    }
    return diag;
  }

  /**
   * τ = mj_inverse(q,v,a_d) + Kp·angleDiff(q_d,q) + Kd·(v_d-v)，再限幅。
   */
  computeTorque(
    qCurrent: ArrayLike<number>,
    qvelCurrent: ArrayLike<number>,
    qDesired: ArrayLike<number>,
    qvelDesired: ArrayLike<number>,
    qaccDesired: ArrayLike<number>,
  ): Float64Array {
    const tauFf = this.inverseDynamics(qCurrent, qvelCurrent, qaccDesired);
    const posError = angleDiff(qDesired, qCurrent);
    const tauFb = new Float64Array(this.nv);
    for (let i = 0; i < this.nv; i++) {
      const velError = (qvelDesired[i] ?? 0) - (qvelCurrent[i] ?? 0);
      tauFb[i] = this.kp[i] * posError[i] + this.kd[i] * velError;
    }
    const tauTotal = new Float64Array(this.nv);
    for (let i = 0; i < this.nv; i++) {
      tauTotal[i] = tauFf[i] + tauFb[i];
    }
    return clipTorque(tauTotal, this.torqueLimits);
  }

  /** mj_inverse 前馈；调用前清零 qfrc_applied 避免正反馈 */
  inverseDynamics(
    q: ArrayLike<number>,
    v: ArrayLike<number>,
    vd: ArrayLike<number>,
  ): Float64Array {
    const saved = saveMjState(this.data, this.model);
    vecZero(this.data.qfrc_applied, this.nv);
    vecSet(this.data.qpos, q, this.nq);
    vecSet(this.data.qvel, v, this.nv);
    vecSet(this.data.qacc, vd, this.nv);

    this.mujoco.mj_inverse(this.model, this.data);
    const tau = vecGet(this.data.qfrc_inverse, this.nv);

    restoreMjState(this.data, this.model, saved);
    return tau;
  }

  setGains(kp: ArrayLike<number>, kd: ArrayLike<number>): void {
    for (let i = 0; i < this.nv; i++) {
      this.kp[i] = kp.length === 1 ? (kp[0] ?? this.kp[i]) : (kp[i] ?? this.kp[i]);
      this.kd[i] = kd.length === 1 ? (kd[0] ?? this.kd[i]) : (kd[i] ?? this.kd[i]);
    }
  }

  setTorqueLimits(limits: ArrayLike<number>): void {
    for (let i = 0; i < this.nv; i++) {
      const v = limits.length === 1 ? limits[0] : limits[i];
      this.torqueLimits[i] =
        v !== undefined && v > 0 ? v : DEFAULT_TORQUE_LIMIT;
    }
  }

  getGains(): { kp: Float64Array; kd: Float64Array } {
    return { kp: new Float64Array(this.kp), kd: new Float64Array(this.kd) };
  }
}
