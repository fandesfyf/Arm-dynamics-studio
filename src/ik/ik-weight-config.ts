/** closed-chain-ik Solver 权重与迭代参数（移植自 robot_motion_editor） */

export const IK_MAX_ITERATIONS_LIMIT = 1e9;
export const IK_DRAG_LIVE_ITER_CAP = 128;

export interface PositionIkWeights {
  translationFactor: number;
  rotationFactor: number;
  maxIterations: number;
  dampingFactor: number;
  translationErrorClamp: number;
  divergeThreshold: number;
  convergedPositionTolerance: number;
}

export interface IkWeights {
  position: PositionIkWeights;
}

export const IK_WEIGHT_DEFAULTS: IkWeights = Object.freeze({
  position: Object.freeze({
    translationFactor: 1,
    rotationFactor: 0.012,
    maxIterations: 32,
    dampingFactor: 0.012,
    translationErrorClamp: 0.02,
    divergeThreshold: 0.02,
    convergedPositionTolerance: 0.004,
  }),
});

export const IK_WEIGHT_LIMITS = Object.freeze({
  translationFactor: { min: 0, max: 1, step: 0.001 },
  rotationFactor: { min: 0, max: 1, step: 0.001 },
  maxIterations: { min: 1, max: IK_MAX_ITERATIONS_LIMIT, step: 1 },
  dampingFactor: { min: 0.0001, max: 0.2, step: 0.001 },
  translationErrorClamp: { min: 0.001, max: 0.2, step: 0.001 },
  divergeThreshold: { min: 0.001, max: 0.5, step: 0.001 },
  convergedPositionTolerance: { min: 0.0005, max: 0.05, step: 0.0005 },
});

function clampNum(v: unknown, min: number, max: number, fallback: number): number {
  const n = Number(v);
  if (!Number.isFinite(n)) return fallback;
  return Math.min(max, Math.max(min, n));
}

export function sanitizeIkWeights(weights?: Partial<IkWeights> | null): IkWeights {
  const p: Partial<PositionIkWeights> = weights?.position ?? {};
  const d = IK_WEIGHT_DEFAULTS.position;
  const lim = IK_WEIGHT_LIMITS;
  return {
    position: {
      translationFactor: clampNum(p.translationFactor, lim.translationFactor.min, lim.translationFactor.max, d.translationFactor),
      rotationFactor: clampNum(p.rotationFactor, lim.rotationFactor.min, lim.rotationFactor.max, d.rotationFactor),
      maxIterations: Math.round(clampNum(p.maxIterations, lim.maxIterations.min, lim.maxIterations.max, d.maxIterations)),
      dampingFactor: clampNum(p.dampingFactor, lim.dampingFactor.min, lim.dampingFactor.max, d.dampingFactor),
      translationErrorClamp: clampNum(p.translationErrorClamp, lim.translationErrorClamp.min, lim.translationErrorClamp.max, d.translationErrorClamp),
      divergeThreshold: clampNum(p.divergeThreshold, lim.divergeThreshold.min, lim.divergeThreshold.max, d.divergeThreshold),
      convergedPositionTolerance: clampNum(
        p.convergedPositionTolerance,
        lim.convergedPositionTolerance.min,
        lim.convergedPositionTolerance.max,
        d.convergedPositionTolerance,
      ),
    },
  };
}

export function capIterationsForLiveDrag(
  weights: PositionIkWeights,
  dragging: boolean,
): PositionIkWeights {
  if (!dragging) return weights;
  return {
    ...weights,
    maxIterations: Math.min(weights.maxIterations, IK_DRAG_LIVE_ITER_CAP),
  };
}

export function getDragEndWeights(weights: PositionIkWeights): PositionIkWeights {
  return {
    ...weights,
    maxIterations: Math.min(
      IK_MAX_ITERATIONS_LIMIT,
      Math.round(weights.maxIterations * 1.35),
    ),
  };
}

export type IkGoalMode = 'position' | 'pose' | 'orientation';
