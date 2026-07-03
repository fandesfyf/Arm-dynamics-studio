import type { RecorderDict } from '../core/data-recorder';
import { DataRecorder } from '../core/data-recorder';

export interface CsvExportOptions {
  jointNames?: string[];
  includeHeader?: boolean;
  /** 默认 6 位小数，与旧版 %.6f 一致 */
  decimals?: number;
}

export type CsvExportMetric = 'position' | 'velocity' | 'torque' | 'ee';

export type CsvTimeRange =
  | { mode: 'all' }
  | { mode: 'window'; windowSeconds?: number };

export interface SelectiveCsvExportOptions {
  metrics: CsvExportMetric[];
  /** 关节索引；省略则导出全部关节（ee 指标不受此限） */
  jointIndices?: number[];
  timeRange?: CsvTimeRange;
  jointNames?: string[];
  includeHeader?: boolean;
  decimals?: number;
}

const DEFAULT_WINDOW_SECONDS = 30;

function defaultJointNames(n: number): string[] {
  return Array.from({ length: n }, (_, i) => `joint${i + 1}`);
}

function formatFloat(value: number, decimals: number): string {
  return value.toFixed(decimals);
}

function resolveJointNames(nJoints: number, jointNames?: string[]): string[] {
  const names = jointNames ? [...jointNames] : defaultJointNames(nJoints);
  while (names.length < nJoints) {
    names.push(`joint${names.length + 1}`);
  }
  return names.slice(0, nJoints);
}

function resolveJointIndices(nJoints: number, jointIndices?: number[]): number[] {
  if (!jointIndices || jointIndices.length === 0) {
    return Array.from({ length: nJoints }, (_, i) => i);
  }
  return [...new Set(jointIndices)]
    .filter((i) => i >= 0 && i < nJoints)
    .sort((a, b) => a - b);
}

/** 按时间范围裁剪 RecorderDict（window = 末段 N 秒） */
export function sliceDictByTimeRange(dict: RecorderDict, range: CsvTimeRange): RecorderDict {
  if (range.mode === 'all' || dict.time.length === 0) {
    return dict;
  }

  const windowSeconds = range.windowSeconds ?? DEFAULT_WINDOW_SECONDS;
  const lastTime = dict.time[dict.time.length - 1] ?? 0;
  const startTime = lastTime - windowSeconds;
  const indices: number[] = [];
  for (let i = 0; i < dict.time.length; i++) {
    if (dict.time[i] >= startTime) {
      indices.push(i);
    }
  }

  if (indices.length === 0 || indices.length === dict.time.length) {
    return dict;
  }

  const pick = <T>(arr: T[]): T[] => indices.map((i) => arr[i]);
  const pickRows = (rows: number[][]): number[][] => indices.map((i) => rows[i] ?? []);

  const sliced: RecorderDict = {
    time: pick(dict.time),
    qpos: pickRows(dict.qpos),
    qvel: pickRows(dict.qvel),
    tau: pickRows(dict.tau),
    ee_pos: pickRows(dict.ee_pos),
    ee_quat: pickRows(dict.ee_quat),
  };

  if (dict.q_desired) sliced.q_desired = pickRows(dict.q_desired);
  if (dict.qvel_desired) sliced.qvel_desired = pickRows(dict.qvel_desired);
  if (dict.tau_commanded) sliced.tau_commanded = pickRows(dict.tau_commanded);

  return sliced;
}

/**
 * 按选定指标/关节/时间范围导出 CSV。
 * 列顺序仍遵循 §5.8 分组，仅包含所选 series。
 */
export function exportDictToCsv(
  dict: RecorderDict,
  options: SelectiveCsvExportOptions,
): string {
  const {
    metrics,
    jointIndices,
    timeRange = { mode: 'all' },
    jointNames,
    includeHeader = true,
    decimals = 6,
  } = options;

  const data = sliceDictByTimeRange(dict, timeRange);

  if (data.time.length === 0) {
    return includeHeader ? 'time\n' : '';
  }

  const nJoints = data.qpos[0]?.length ?? 0;
  const allNames = resolveJointNames(nJoints, jointNames);
  const indices = resolveJointIndices(nJoints, jointIndices);
  const names = indices.map((i) => allNames[i] ?? `joint${i + 1}`);

  const wantPosition = metrics.includes('position');
  const wantVelocity = metrics.includes('velocity');
  const wantTorque = metrics.includes('torque');
  const wantEe = metrics.includes('ee');

  const hasTorque = wantTorque && data.tau.length > 0 && (data.tau[0]?.length ?? 0) > 0;

  const lines: string[] = [];

  if (includeHeader) {
    const header: string[] = ['time'];
    if (wantPosition) header.push(...names.map((n) => `${n}_pos`));
    if (wantVelocity) header.push(...names.map((n) => `${n}_vel`));
    if (hasTorque) header.push(...names.map((n) => `${n}_torque`));
    if (wantPosition && data.q_desired) {
      header.push(...names.map((n) => `${n}_pos_desired`));
    }
    if (wantVelocity && data.qvel_desired) {
      header.push(...names.map((n) => `${n}_vel_desired`));
    }
    if (wantTorque && data.tau_commanded) {
      header.push(...names.map((n) => `${n}_torque_cmd`));
    }
    if (wantEe) {
      header.push('ee_x', 'ee_y', 'ee_z');
    }
    lines.push(header.join(','));
  }

  for (let i = 0; i < data.time.length; i++) {
    const row: string[] = [formatFloat(data.time[i], decimals)];

    if (wantPosition) {
      for (const j of indices) {
        row.push(formatFloat(data.qpos[i]?.[j] ?? 0, decimals));
      }
    }
    if (wantVelocity) {
      for (const j of indices) {
        row.push(formatFloat(data.qvel[i]?.[j] ?? 0, decimals));
      }
    }
    if (hasTorque) {
      for (const j of indices) {
        row.push(formatFloat(data.tau[i]?.[j] ?? 0, decimals));
      }
    }
    if (wantPosition && data.q_desired) {
      for (const j of indices) {
        row.push(formatFloat(data.q_desired[i]?.[j] ?? 0, decimals));
      }
    }
    if (wantVelocity && data.qvel_desired) {
      for (const j of indices) {
        row.push(formatFloat(data.qvel_desired[i]?.[j] ?? 0, decimals));
      }
    }
    if (wantTorque && data.tau_commanded) {
      for (const j of indices) {
        row.push(formatFloat(data.tau_commanded[i]?.[j] ?? 0, decimals));
      }
    }
    if (wantEe) {
      const p = data.ee_pos[i] ?? [0, 0, 0];
      row.push(
        formatFloat(p[0] ?? 0, decimals),
        formatFloat(p[1] ?? 0, decimals),
        formatFloat(p[2] ?? 0, decimals),
      );
    }

    lines.push(row.join(','));
  }

  return lines.join('\n') + (lines.length > 0 ? '\n' : '');
}

/**
 * 导出 CSV 字符串。
 * 列顺序（§5.8）：time, pos, vel, torque, pos_desired, vel_desired, torque_cmd
 */
export function exportToCsv(recorder: DataRecorder, options: CsvExportOptions = {}): string {
  const { includeHeader = true, decimals = 6, jointNames } = options;
  const data = recorder.toDict();

  return exportDictToCsv(data, {
    metrics: ['position', 'velocity', 'torque'],
    jointNames,
    includeHeader,
    decimals,
    timeRange: { mode: 'all' },
  });
}

/** 浏览器下载用 Blob */
export function csvToBlob(csv: string): Blob {
  return new Blob([csv], { type: 'text/csv' });
}

/** 触发浏览器下载 */
export function downloadCsv(csv: string, filename: string): void {
  const url = URL.createObjectURL(csvToBlob(csv));
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  a.click();
  URL.revokeObjectURL(url);
}
