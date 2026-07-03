import type { DataRecorder, RecorderDict } from '../../core/data-recorder';

/** 单类曲线（位置 / 速度 / 力矩）的输入 */
export interface ChartSeriesInput {
  times: number[];
  jointNames: string[];
  /** 实际值 [帧][关节] */
  actual: number[][];
  /** 指令值 [帧][关节]；无指令数据时为 null */
  desired: number[][] | null;
}

export type ChartMetric = 'position' | 'velocity' | 'torque' | 'ee';

const METRIC_LABELS: Record<ChartMetric, string> = {
  position: '关节位置',
  velocity: '关节速度',
  torque: '关节力矩',
  ee: '末端轨迹',
};

export function chartMetricLabel(metric: ChartMetric): string {
  return METRIC_LABELS[metric];
}

function defaultJointNames(count: number): string[] {
  return Array.from({ length: count }, (_, i) => `joint${i + 1}`);
}

function rowCount(dict: RecorderDict): number {
  return dict.qpos.length;
}

function jointCount(dict: RecorderDict): number {
  return dict.qpos[0]?.length ?? 0;
}

/** 从 DataRecorder.toDict() 适配为指定指标的 ChartSeriesInput */
export function fromRecorderDict(
  dict: RecorderDict,
  metric: ChartMetric,
  jointNames?: string[],
): ChartSeriesInput {
  const names = jointNames ?? defaultJointNames(jointCount(dict));
  const empty: ChartSeriesInput = { times: [], jointNames: names, actual: [], desired: null };

  if (rowCount(dict) === 0) {
    return empty;
  }

  switch (metric) {
    case 'position':
      return {
        times: dict.time,
        jointNames: names,
        actual: dict.qpos,
        desired: dict.q_desired ?? null,
      };
    case 'velocity':
      return {
        times: dict.time,
        jointNames: names,
        actual: dict.qvel,
        desired: dict.qvel_desired ?? null,
      };
    case 'torque':
      return {
        times: dict.time,
        jointNames: names,
        actual: dict.tau,
        desired: dict.tau_commanded ?? null,
      };
    case 'ee': {
      const eeNames = ['ee_x', 'ee_y', 'ee_z'];
      const actual = dict.ee_pos.map((p) => [p[0] ?? 0, p[1] ?? 0, p[2] ?? 0]);
      return {
        times: dict.time,
        jointNames: eeNames,
        actual,
        desired: null,
      };
    }
  }
}

/** 从 DataRecorder 实例适配 */
export function fromDataRecorder(
  recorder: DataRecorder,
  metric: ChartMetric,
  jointNames?: string[],
): ChartSeriesInput {
  return fromRecorderDict(recorder.toDict(), metric, jointNames);
}

export interface ChartSeriesBundle {
  position: ChartSeriesInput;
  velocity: ChartSeriesInput;
  torque: ChartSeriesInput;
  ee: ChartSeriesInput;
}

/** 一次适配位置 / 速度 / 力矩三组曲线 */
export function bundleFromRecorderDict(
  dict: RecorderDict,
  jointNames?: string[],
): ChartSeriesBundle {
  return {
    position: fromRecorderDict(dict, 'position', jointNames),
    velocity: fromRecorderDict(dict, 'velocity', jointNames),
    torque: fromRecorderDict(dict, 'torque', jointNames),
    ee: fromRecorderDict(dict, 'ee', jointNames),
  };
}

export function bundleFromDataRecorder(
  recorder: DataRecorder,
  jointNames?: string[],
): ChartSeriesBundle {
  return bundleFromRecorderDict(recorder.toDict(), jointNames);
}
