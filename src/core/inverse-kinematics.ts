import type { JointMapping } from '../types/robot';
import {
  solveInverseKinematics,
  type IkOptions,
  type PinocchioModelLike,
  type PinocchioModule,
} from '../pinocchio/ik';

/** T1 `joint-map.ts` 将实现的跨引擎映射接口（IK 仅依赖此最小子集） */
export interface JointMap {
  readonly mappings: readonly JointMapping[];
  mjQposToPinQ(mjQpos: Float64Array | number[]): Float64Array;
  pinQToMjQpos(
    pinQ: Float64Array | number[],
    mjQpos?: Float64Array | number[],
  ): Float64Array;
}

export interface InverseKinematicsOptions {
  maxIterations?: number;
  tolerance?: number;
  stepSize?: number;
  damping?: number;
}

export interface IkSolveResult {
  /** MuJoCo qpos 空间关节角 */
  q: Float64Array;
  converged: boolean;
  error: number;
}

const DEFAULT_MAX_ITERATIONS = 100;
const DEFAULT_TOLERANCE = 1e-4;

function arraysClose(a: ArrayLike<number>, b: ArrayLike<number>, atol = 1e-6): boolean {
  if (a.length !== b.length) {
    return false;
  }
  for (let i = 0; i < a.length; i++) {
    if (Math.abs(a[i] - b[i]) > atol) {
      return false;
    }
  }
  return true;
}

function heuristicSeeds(nv: number): Float64Array[] {
  const templates = [
    [],
    [0.5, 0, -1.0, 0, 0],
    [-0.5, 0, 1.0, 0, 0],
    [0.3, 0.3, -0.8, 0.5, 0],
  ];

  return templates.map((tpl) => {
    const seed = new Float64Array(nv);
    for (let i = 0; i < nv && i < tpl.length; i++) {
      seed[i] = tpl[i];
    }
    return seed;
  });
}

/**
 * 基于 Pinocchio DLS 的逆运动学封装。
 * 首版仅约束末端 XYZ；姿态保持由仿真侧在到达后自然保持。
 */
export class InverseKinematics {
  private endEffectorJointId: number | null = null;

  private maxIterations = DEFAULT_MAX_ITERATIONS;
  private tolerance = DEFAULT_TOLERANCE;
  private stepSize: number | undefined;
  private damping: number | undefined;

  constructor(
    private readonly pin: PinocchioModule,
    private readonly model: PinocchioModelLike,
    private readonly data: unknown,
    private readonly jointMap: JointMap,
    private readonly jointNames: string[],
  ) {}

  /** 按 Pinocchio 关节索引或 URDF 关节名设置末端 */
  setEndEffector(jointIdOrName: number | string): void {
    if (typeof jointIdOrName === 'number') {
      this.endEffectorJointId = jointIdOrName;
      return;
    }

    const idx = this.jointNames.indexOf(jointIdOrName);
    if (idx < 0) {
      throw new Error(`未知末端关节: ${jointIdOrName}`);
    }
    // Pinocchio 关节 0 为 universe，活动关节名顺序与 joint id 通常差 1
    this.endEffectorJointId = idx + 1;
  }

  setMaxIterations(maxIter: number): void {
    this.maxIterations = maxIter;
  }

  setTolerance(tol: number): void {
    this.tolerance = tol;
  }

  setStepSize(stepSize: number): void {
    this.stepSize = stepSize;
  }

  setDamping(damping: number): void {
    this.damping = damping;
  }

  getEndEffectorJointId(): number {
    return this.endEffectorJointId ?? this.model.njoints - 1;
  }

  solve(
    targetPos: Float64Array | number[],
    qInit?: Float64Array | number[] | null,
    options?: InverseKinematicsOptions,
  ): IkSolveResult {
    if (this.endEffectorJointId === null && this.jointNames.length > 0) {
      this.endEffectorJointId = this.jointNames.length;
    }

    const ikOptions: IkOptions = {
      endEffectorJointId: this.getEndEffectorJointId(),
      maxIter: options?.maxIterations ?? this.maxIterations,
      tolerance: options?.tolerance ?? this.tolerance,
      stepSize: options?.stepSize ?? this.stepSize,
      damping: options?.damping ?? this.damping,
    };

    const seeds = this.buildSeeds(qInit);
    let bestQmj = new Float64Array(0);
    let bestError = Number.POSITIVE_INFINITY;
    let bestConverged = false;

    for (const seed of seeds) {
      const result = solveInverseKinematics(
        this.pin,
        this.model,
        this.data,
        targetPos,
        seed,
        ikOptions,
      );

      if (result.error < bestError) {
        bestError = result.error;
        bestConverged = result.converged;
        bestQmj = this.jointMap.pinQToMjQpos(result.q);
      }
    }

    const converged = bestConverged || bestError < Math.max(this.tolerance * 10, 1e-2);

    return {
      q: bestQmj,
      converged,
      error: bestError,
    };
  }

  private buildSeeds(qInit?: Float64Array | number[] | null): Float64Array[] {
    const nv = this.model.nv;
    const seeds: Float64Array[] = [];

    const pushSeed = (q: Float64Array) => {
      if (!seeds.some((existing) => arraysClose(existing, q))) {
        seeds.push(q);
      }
    };

    if (qInit != null) {
      pushSeed(this.toPinQ(qInit));
    }

    for (const seed of heuristicSeeds(nv)) {
      pushSeed(seed);
    }

    if (seeds.length === 0) {
      pushSeed(new Float64Array(nv));
    }

    return seeds;
  }

  private toPinQ(q: Float64Array | number[]): Float64Array {
    const arr = q instanceof Float64Array ? q : new Float64Array(q);
    if (arr.length === this.model.nv) {
      return arr;
    }
    return this.jointMap.mjQposToPinQ(arr);
  }
}
