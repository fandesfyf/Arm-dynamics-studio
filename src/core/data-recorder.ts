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

function copyArray(v: number[] | readonly number[]): number[] {
  return Array.from(v);
}

export class DataRecorder {
  private records: SimulationRecord[] = [];
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
    if (this.records.length === 0) return null;
    return this.records[0].time;
  }

  private trimToWindow(): void {
    if (this.maxDurationSec <= 0 || this.records.length === 0) return;
    const latest = this.records[this.records.length - 1]!.time;
    const cutoff = latest - this.maxDurationSec;
    while (this.records.length > 0 && this.records[0]!.time < cutoff) {
      this.records.shift();
    }
  }

  getTimes(): number[] {
    return this.records.map((r) => r.time);
  }

  getJointPositions(): number[][] {
    return this.records.map((r) => [...r.qpos]);
  }

  getJointVelocities(): number[][] {
    return this.records.map((r) => [...r.qvel]);
  }

  getJointTorques(): number[][] {
    return this.records.map((r) => [...r.tau]);
  }

  getDesiredPositions(): number[][] | null {
    if (this.records.length > 0 && this.records[0].q_desired !== undefined) {
      return this.records.map((r) => [...r.q_desired!]);
    }
    return null;
  }

  getDesiredVelocities(): number[][] | null {
    if (this.records.length > 0 && this.records[0].qvel_desired !== undefined) {
      return this.records.map((r) => [...r.qvel_desired!]);
    }
    return null;
  }

  getCommandedTorques(): number[][] | null {
    if (this.records.length > 0 && this.records[0].tau_commanded !== undefined) {
      return this.records.map((r) => [...r.tau_commanded!]);
    }
    return null;
  }

  getEndEffectorPositions(): number[][] {
    return this.records.map((r) => [...r.ee_pos]);
  }

  getEndEffectorQuaternions(): number[][] {
    return this.records.map((r) => [...r.ee_quat]);
  }

  getDuration(): number {
    if (this.records.length === 0) {
      return 0;
    }
    return this.records[this.records.length - 1].time - this.records[0].time;
  }

  getNumFrames(): number {
    return this.records.length;
  }

  getSamplingRate(): number {
    if (this.records.length < 2) {
      return 0;
    }
    const duration = this.getDuration();
    if (duration <= 0) {
      return 0;
    }
    return (this.records.length - 1) / duration;
  }

  clear(): void {
    this.records = [];
  }

  toDict(): RecorderDict {
    const result: RecorderDict = {
      time: this.getTimes(),
      qpos: this.getJointPositions(),
      qvel: this.getJointVelocities(),
      tau: this.getJointTorques(),
      ee_pos: this.getEndEffectorPositions(),
      ee_quat: this.getEndEffectorQuaternions(),
    };

    const qDesired = this.getDesiredPositions();
    if (qDesired !== null) {
      result.q_desired = qDesired;
    }

    const qvelDesired = this.getDesiredVelocities();
    if (qvelDesired !== null) {
      result.qvel_desired = qvelDesired;
    }

    const tauCmd = this.getCommandedTorques();
    if (tauCmd !== null) {
      result.tau_commanded = tauCmd;
    }

    return result;
  }
}
