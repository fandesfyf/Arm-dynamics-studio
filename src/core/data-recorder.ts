/**
 * 仿真数据记录器 — 字段对齐 WEB_IMPLEMENTATION_PLAN.md §5.6
 */
export interface SimulationRecord {
  time: number;
  qpos: number[];
  qvel: number[];
  tau: number[];
  q_desired?: number[];
  qvel_desired?: number[];
  tau_commanded?: number[];
  ee_pos: number[];
  ee_quat: number[];
}

export interface RecorderDict {
  time: number[];
  qpos: number[][];
  qvel: number[][];
  tau: number[][];
  ee_pos: number[][];
  ee_quat: number[][];
  q_desired?: number[][];
  qvel_desired?: number[][];
  tau_commanded?: number[][];
}

/** 实时曲线显示的最大采样点数（导出仍用完整 toDict） */
export const CHART_DISPLAY_MAX_POINTS = 2000;

function copyArray(v: number[] | readonly number[]): number[] {
  return Array.from(v);
}

export class DataRecorder {
  private records: SimulationRecord[] = [];
  /** 窗口裁剪起始下标，避免 shift() 反复搬移数组 */
  private startIndex = 0;
  /** 环形缓冲窗口（秒）；≤0 表示不裁剪 */
  maxDurationSec = 30;

  record(state: SimulationRecord): void {
    this.records.push({
      time: state.time,
      qpos: copyArray(state.qpos),
      qvel: copyArray(state.qvel),
      tau: copyArray(state.tau),
      ee_pos: copyArray(state.ee_pos),
      ee_quat: copyArray(state.ee_quat),
      ...(state.q_desired !== undefined ? { q_desired: copyArray(state.q_desired) } : {}),
      ...(state.qvel_desired !== undefined ? { qvel_desired: copyArray(state.qvel_desired) } : {}),
      ...(state.tau_commanded !== undefined ? { tau_commanded: copyArray(state.tau_commanded) } : {}),
    });
    this.trimToWindow();
  }

  setMaxDurationSec(sec: number): void {
    this.maxDurationSec = sec;
    this.trimToWindow();
  }

  getWindowStartTime(): number | null {
    if (this.getNumFrames() === 0) return null;
    return this.records[this.startIndex]!.time;
  }

  getLastTime(): number | null {
    if (this.records.length === 0) return null;
    return this.records[this.records.length - 1]!.time;
  }

  private frameCount(): number {
    return this.records.length - this.startIndex;
  }

  private at(frame: number): SimulationRecord {
    return this.records[this.startIndex + frame]!;
  }

  private trimToWindow(): void {
    if (this.maxDurationSec <= 0 || this.frameCount() === 0) return;
    const latest = this.records[this.records.length - 1]!.time;
    const cutoff = latest - this.maxDurationSec;
    while (this.startIndex < this.records.length && this.records[this.startIndex]!.time < cutoff) {
      this.startIndex += 1;
    }
    if (this.startIndex >= 512 && this.startIndex * 2 >= this.records.length) {
      this.records = this.records.slice(this.startIndex);
      this.startIndex = 0;
    }
  }

  getTimes(): number[] {
    const n = this.frameCount();
    const out = new Array<number>(n);
    for (let i = 0; i < n; i++) {
      out[i] = this.at(i).time;
    }
    return out;
  }

  getJointPositions(): number[][] {
    return this.mapFrames((r) => [...r.qpos]);
  }

  getJointVelocities(): number[][] {
    return this.mapFrames((r) => [...r.qvel]);
  }

  getJointTorques(): number[][] {
    return this.mapFrames((r) => [...r.tau]);
  }

  getDesiredPositions(): number[][] | null {
    if (this.frameCount() > 0 && this.at(0).q_desired !== undefined) {
      return this.mapFrames((r) => [...r.q_desired!]);
    }
    return null;
  }

  getDesiredVelocities(): number[][] | null {
    if (this.frameCount() > 0 && this.at(0).qvel_desired !== undefined) {
      return this.mapFrames((r) => [...r.qvel_desired!]);
    }
    return null;
  }

  getCommandedTorques(): number[][] | null {
    if (this.frameCount() > 0 && this.at(0).tau_commanded !== undefined) {
      return this.mapFrames((r) => [...r.tau_commanded!]);
    }
    return null;
  }

  getEndEffectorPositions(): number[][] {
    return this.mapFrames((r) => [...r.ee_pos]);
  }

  getEndEffectorQuaternions(): number[][] {
    return this.mapFrames((r) => [...r.ee_quat]);
  }

  private mapFrames<T>(fn: (r: SimulationRecord) => T): T[] {
    const n = this.frameCount();
    const out = new Array<T>(n);
    for (let i = 0; i < n; i++) {
      out[i] = fn(this.at(i));
    }
    return out;
  }

  getDuration(): number {
    const n = this.frameCount();
    if (n === 0) {
      return 0;
    }
    return this.at(n - 1).time - this.at(0).time;
  }

  getNumFrames(): number {
    return this.frameCount();
  }

  getSamplingRate(): number {
    const n = this.frameCount();
    if (n < 2) {
      return 0;
    }
    const duration = this.getDuration();
    if (duration <= 0) {
      return 0;
    }
    return (n - 1) / duration;
  }

  clear(): void {
    this.records = [];
    this.startIndex = 0;
  }

  loadFromDict(dict: RecorderDict): void {
    this.startIndex = 0;
    this.records = dict.time.map((time, i) => {
      const row: SimulationRecord = {
        time,
        qpos: copyArray(dict.qpos[i] ?? []),
        qvel: copyArray(dict.qvel[i] ?? []),
        tau: copyArray(dict.tau[i] ?? []),
        ee_pos: copyArray(dict.ee_pos[i] ?? []),
        ee_quat: copyArray(dict.ee_quat[i] ?? []),
      };
      if (dict.q_desired?.[i]) row.q_desired = copyArray(dict.q_desired[i]!);
      if (dict.qvel_desired?.[i]) row.qvel_desired = copyArray(dict.qvel_desired[i]!);
      if (dict.tau_commanded?.[i]) row.tau_commanded = copyArray(dict.tau_commanded[i]!);
      return row;
    });
    this.trimToWindow();
  }

  toDict(): RecorderDict {
    return this.buildDictFromFrames((n) => {
      const indices = new Array<number>(n);
      for (let i = 0; i < n; i++) indices[i] = i;
      return indices;
    });
  }

  /** 降采样版 dict，供实时曲线使用；导出请用 toDict() */
  toDictForDisplay(maxPoints = CHART_DISPLAY_MAX_POINTS): RecorderDict {
    const n = this.frameCount();
    if (n <= maxPoints) {
      return this.toDict();
    }
    const stride = Math.ceil(n / maxPoints);
    const indices: number[] = [];
    for (let i = 0; i < n; i += stride) {
      indices.push(i);
    }
    if (indices[indices.length - 1] !== n - 1) {
      indices.push(n - 1);
    }
    return this.buildDictFromFrames(() => indices);
  }

  private buildDictFromFrames(indicesFor: (frameCount: number) => number[]): RecorderDict {
    const n = this.frameCount();
    if (n === 0) {
      return { time: [], qpos: [], qvel: [], tau: [], ee_pos: [], ee_quat: [] };
    }

    const indices = indicesFor(n);
    const hasDesired = this.at(0).q_desired !== undefined;
    const hasVelDesired = this.at(0).qvel_desired !== undefined;
    const hasTauCmd = this.at(0).tau_commanded !== undefined;

    const time: number[] = [];
    const qpos: number[][] = [];
    const qvel: number[][] = [];
    const tau: number[][] = [];
    const ee_pos: number[][] = [];
    const ee_quat: number[][] = [];
    const q_desired: number[][] = [];
    const qvel_desired: number[][] = [];
    const tau_commanded: number[][] = [];

    for (const i of indices) {
      const r = this.at(i);
      time.push(r.time);
      qpos.push([...r.qpos]);
      qvel.push([...r.qvel]);
      tau.push([...r.tau]);
      ee_pos.push([...r.ee_pos]);
      ee_quat.push([...r.ee_quat]);
      if (hasDesired && r.q_desired) q_desired.push([...r.q_desired]);
      if (hasVelDesired && r.qvel_desired) qvel_desired.push([...r.qvel_desired]);
      if (hasTauCmd && r.tau_commanded) tau_commanded.push([...r.tau_commanded]);
    }

    const result: RecorderDict = { time, qpos, qvel, tau, ee_pos, ee_quat };
    if (hasDesired) result.q_desired = q_desired;
    if (hasVelDesired) result.qvel_desired = qvel_desired;
    if (hasTauCmd) result.tau_commanded = tau_commanded;
    return result;
  }
}
