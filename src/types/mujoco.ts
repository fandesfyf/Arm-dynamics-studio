import type { MjData, MjModel } from '@mujoco/mujoco';

type MjVec = {
  size?: () => number;
  length?: number;
  get?: (i: number) => number | undefined;
  set?: (i: number, v: number) => boolean;
  [index: number]: number | undefined;
};

export function vecGet(vec: MjVec, n: number): Float64Array {
  const out = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    if (typeof vec.get === 'function') {
      out[i] = vec.get(i) ?? 0;
    } else {
      out[i] = vec[i] ?? 0;
    }
  }
  return out;
}

function hasArrayLikeLength(vec: MjVec): vec is MjVec & { length: number } {
  return typeof vec.length === 'number' && vec.length > 0;
}

export function vecSet(vec: MjVec, values: ArrayLike<number>, n?: number): void {
  const len = n ?? values.length;
  for (let i = 0; i < len; i++) {
    const v = values[i] ?? 0;
    if (hasArrayLikeLength(vec)) {
      vec[i] = v;
    } else if (typeof vec.set === 'function') {
      vec.set(i, v);
    } else {
      vec[i] = v;
    }
  }
}

export function vecZero(vec: MjVec, n: number): void {
  for (let i = 0; i < n; i++) {
    if (hasArrayLikeLength(vec)) {
      vec[i] = 0;
    } else if (typeof vec.set === 'function') {
      vec.set(i, 0);
    } else {
      vec[i] = 0;
    }
  }
}

export function saveMjState(
  data: MjData,
  model: MjModel,
): {
  qpos: Float64Array;
  qvel: Float64Array;
  qacc: Float64Array;
  qfrc_applied: Float64Array;
} {
  const nq = model.nq;
  const nv = model.nv;
  return {
    qpos: vecGet(data.qpos as MjVec, nq),
    qvel: vecGet(data.qvel as MjVec, nv),
    qacc: vecGet(data.qacc as MjVec, nv),
    qfrc_applied: vecGet(data.qfrc_applied as MjVec, nv),
  };
}

export function restoreMjState(
  data: MjData,
  model: MjModel,
  saved: {
    qpos: Float64Array;
    qvel: Float64Array;
    qacc: Float64Array;
    qfrc_applied: Float64Array;
  },
): void {
  const nv = model.nv;
  vecSet(data.qpos as MjVec, saved.qpos);
  vecSet(data.qvel as MjVec, saved.qvel, nv);
  vecSet(data.qacc as MjVec, saved.qacc, nv);
  vecSet(data.qfrc_applied as MjVec, saved.qfrc_applied, nv);
}
