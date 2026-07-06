/** 单时刻仿真状态快照（MuJoCo 真值） */
export interface SimulationState {
  time: number;
  qpos: Float64Array;
  qvel: Float64Array;
  tau: Float64Array;
}

/** 控制循环记录字段（供 DataRecorder / 曲线） */
export interface SimulationRecord {
  time: number;
  qpos: number[];
  qvel: number[];
  tau: number[];
  q_desired: number[];
  qvel_desired: number[];
  tau_commanded: number[];
  ee_pos: [number, number, number];
  ee_quat: [number, number, number, number];
}

/** 每控制周期完整状态（§5.6，含末端 FK） */
export interface SimulationStepState {
  time: number;
  qpos: Float64Array;
  qvel: Float64Array;
  tau: Float64Array;
  q_desired: Float64Array;
  qvel_desired: Float64Array;
  tau_commanded: Float64Array;
  ee_pos: number[];
  ee_quat: number[];
}

export type SimulationStepCallback = (state: SimulationStepState) => void;
export type SimulationCancelCheck = () => boolean;
export type SimulationDoneCallback = (success: boolean, message: string) => void;
export type SimulationProgressCallback = (progress: number) => void;

export interface RunToTargetOptions {
  maxTime?: number;
  tol?: number;
  stepCallback?: SimulationStepCallback;
  doneCallback?: SimulationDoneCallback;
  cancelCheck?: SimulationCancelCheck;
  pauseCheck?: () => boolean;
  yieldEvery?: number;
  onYield?: () => Promise<void>;
  /** 每 controlDt 对应 1 墙钟秒，与实时 hold 循环一致；默认 true */
  realtimePacing?: boolean;
}

export interface RunToEeTargetOptions extends RunToTargetOptions {
  eeQuat?: number[] | null;
  qInit?: Float64Array | null;
}

export interface RunTrajectoryOptions {
  totalTime?: number;
  progressCallback?: SimulationProgressCallback;
  stepCallback?: SimulationStepCallback;
  cancelCheck?: SimulationCancelCheck;
}
