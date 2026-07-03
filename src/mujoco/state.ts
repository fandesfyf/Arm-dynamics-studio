import type { MjData, MjModel } from '@mujoco/mujoco';
import type { SimulationState } from '../types/simulation';

function readVec(dataVec: { size(): number; get(i: number): number | undefined }): Float64Array {
  const n = dataVec.size();
  const out = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    out[i] = dataVec.get(i) ?? 0;
  }
  return out;
}

/** 读取 MuJoCo qpos / qvel 快照 */
export function readQpos(model: MjModel, data: MjData): Float64Array {
  void model;
  return readVec(data.qpos);
}

export function readQvel(model: MjModel, data: MjData): Float64Array {
  void model;
  return readVec(data.qvel);
}

/** 读取 qfrc_applied 作为当前施加力矩 */
export function readAppliedTorque(_model: MjModel, data: MjData): Float64Array {
  return readVec(data.qfrc_applied);
}

export function readSimulationState(
  model: MjModel,
  data: MjData,
  time = data.time,
): SimulationState {
  return {
    time,
    qpos: readQpos(model, data),
    qvel: readQvel(model, data),
    tau: readAppliedTorque(model, data),
  };
}
