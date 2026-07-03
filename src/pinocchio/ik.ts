/** pinocchio-js WASM 句柄（由 loader 注入，此处仅声明 IK 所需 API） */
export interface PinocchioPlacement {
  translation: ArrayLike<number>;
}

export interface PinocchioModule {
  forwardKinematics(model: unknown, data: unknown, q: Float64Array): void;
  updateFramePlacements(model: unknown, data: unknown): void;
  getJointPlacement(data: unknown, jointId: number): PinocchioPlacement;
  computeJointJacobians(model: unknown, data: unknown, q: Float64Array): void;
  getJointJacobian(
    model: unknown,
    data: unknown,
    jointId: number,
    referenceFrame: number,
  ): Float64Array;
}

export interface PinocchioModelLike {
  njoints: number;
  nv: number;
}

export interface IkOptions {
  /** Pinocchio 关节索引；默认 model.njoints - 1 */
  endEffectorJointId?: number;
  maxIter?: number;
  tolerance?: number;
  /** DLS 每步关节更新增益，默认 0.5 */
  stepSize?: number;
  /** 阻尼系数 λ，加在 J J^T 对角线上，默认 1e-6 */
  damping?: number;
}

export interface IkResult {
  q: Float64Array;
  converged: boolean;
  error: number;
  iterations: number;
}

const LOCAL_WORLD_ALIGNED = 2;

const DEFAULT_MAX_ITER = 100;
const DEFAULT_TOLERANCE = 1e-4;
const DEFAULT_STEP_SIZE = 0.5;
const DEFAULT_DAMPING = 1e-6;

function toFloat64Array(v: Float64Array | number[]): Float64Array {
  return v instanceof Float64Array ? v : new Float64Array(v);
}

function positionError(
  current: ArrayLike<number>,
  target: ArrayLike<number>,
): { err: number[]; norm: number } {
  const dx = current[0] - target[0];
  const dy = current[1] - target[1];
  const dz = current[2] - target[2];
  return { err: [dx, dy, dz], norm: Math.hypot(dx, dy, dz) };
}

/** 3×3 线性方程组 A x = b（A 为对称正定近似） */
function solveSymmetric3(A: number[][], b: number[]): number[] {
  const det =
    A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
    A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
    A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

  if (Math.abs(det) < 1e-18) {
    return [0, 0, 0];
  }

  const invDet = 1 / det;
  const inv = [
    [
      (A[1][1] * A[2][2] - A[2][1] * A[1][2]) * invDet,
      (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invDet,
      (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet,
    ],
    [
      (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet,
      (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet,
      (A[1][0] * A[0][2] - A[0][0] * A[1][2]) * invDet,
    ],
    [
      (A[1][0] * A[2][1] - A[2][0] * A[1][1]) * invDet,
      (A[2][0] * A[0][1] - A[0][0] * A[2][1]) * invDet,
      (A[0][0] * A[1][1] - A[1][0] * A[0][1]) * invDet,
    ],
  ];

  return [
    inv[0][0] * b[0] + inv[0][1] * b[1] + inv[0][2] * b[2],
    inv[1][0] * b[0] + inv[1][1] * b[1] + inv[1][2] * b[2],
    inv[2][0] * b[0] + inv[2][1] * b[1] + inv[2][2] * b[2],
  ];
}

/**
 * 阻尼最小二乘（DLS）逆运动学：仅约束末端 XYZ 位置。
 * 算法参考 robot-analyzer-js `solvers.ts` 与 Pinocchio CLIK 示例。
 */
export function solveInverseKinematics(
  pin: PinocchioModule,
  model: PinocchioModelLike,
  data: unknown,
  targetPos: Float64Array | number[],
  initialQ: Float64Array | number[],
  options: IkOptions = {},
): IkResult {
  const maxIter = options.maxIter ?? DEFAULT_MAX_ITER;
  const tolerance = options.tolerance ?? DEFAULT_TOLERANCE;
  const stepSize = options.stepSize ?? DEFAULT_STEP_SIZE;
  const damping = options.damping ?? DEFAULT_DAMPING;
  const endEffectorJointId = options.endEffectorJointId ?? model.njoints - 1;

  const target = toFloat64Array(
    targetPos.length >= 3 ? targetPos : [targetPos[0] ?? 0, targetPos[1] ?? 0, targetPos[2] ?? 0],
  );
  const q = toFloat64Array(initialQ);

  let converged = false;
  let error = Number.POSITIVE_INFINITY;
  let iterations = 0;

  for (iterations = 0; iterations < maxIter; iterations++) {
    pin.forwardKinematics(model, data, q);
    pin.updateFramePlacements(model, data);

    const placement = pin.getJointPlacement(data, endEffectorJointId);
    const { err, norm } = positionError(placement.translation, target);
    error = norm;

    if (error < tolerance) {
      converged = true;
      break;
    }

    pin.computeJointJacobians(model, data, q);
    const jFlat = pin.getJointJacobian(model, data, endEffectorJointId, LOCAL_WORLD_ALIGNED);

    const nv = model.nv;
    const jPos: number[][] = [
      new Array(nv).fill(0),
      new Array(nv).fill(0),
      new Array(nv).fill(0),
    ];
    for (let c = 0; c < nv; c++) {
      jPos[0][c] = jFlat[c * 6 + 0];
      jPos[1][c] = jFlat[c * 6 + 1];
      jPos[2][c] = jFlat[c * 6 + 2];
    }

    const jjt: number[][] = [
      [0, 0, 0],
      [0, 0, 0],
      [0, 0, 0],
    ];
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        let sum = 0;
        for (let k = 0; k < nv; k++) {
          sum += jPos[i][k] * jPos[j][k];
        }
        if (i === j) {
          sum += damping;
        }
        jjt[i][j] = sum;
      }
    }

    const jjtInvErr = solveSymmetric3(jjt, err);
    for (let c = 0; c < nv; c++) {
      let dq = 0;
      for (let r = 0; r < 3; r++) {
        dq += jPos[r][c] * jjtInvErr[r];
      }
      q[c] -= dq * stepSize;
    }
  }

  return { q, converged, error, iterations };
}
