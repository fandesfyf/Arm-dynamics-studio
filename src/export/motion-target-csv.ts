import type { Quat, Vec3 } from '../core/trajectory';
import type { MotionTarget, MotionTargetSource } from '../types/motion-target';
import { createMotionTargetId } from '../types/motion-target';

const FIXED_COLS = [
  'index',
  'source',
  'ee_px',
  'ee_py',
  'ee_pz',
  'ee_qw',
  'ee_qx',
  'ee_qy',
  'ee_qz',
  'scene_wx',
  'scene_wy',
  'scene_wz',
] as const;

function escapeCsvCell(value: string): string {
  if (/[",\n\r]/.test(value)) {
    return `"${value.replace(/"/g, '""')}"`;
  }
  return value;
}

function parseCsvLine(line: string): string[] {
  const cells: string[] = [];
  let cur = '';
  let inQuotes = false;
  for (let i = 0; i < line.length; i++) {
    const ch = line[i]!;
    if (inQuotes) {
      if (ch === '"') {
        if (line[i + 1] === '"') {
          cur += '"';
          i += 1;
        } else {
          inQuotes = false;
        }
      } else {
        cur += ch;
      }
    } else if (ch === '"') {
      inQuotes = true;
    } else if (ch === ',') {
      cells.push(cur);
      cur = '';
    } else {
      cur += ch;
    }
  }
  cells.push(cur);
  return cells;
}

function parseSource(raw: string): MotionTargetSource {
  const s = raw.trim().toLowerCase();
  if (s === 'ee' || s === 'end_effector' || s === '末端') return 'ee';
  return 'joint';
}

/** 导出运动目标队列为 CSV（含关节角列 `q_<jointName>`） */
export function motionTargetsToCsv(
  targets: MotionTarget[],
  jointNames: string[],
): string {
  const jointCols = jointNames.map((n) => `q_${n}`);
  const header = [...FIXED_COLS, ...jointCols].join(',');
  const rows = targets.map((mt, index) => {
    const q = mt.eeQuaternion;
    const cells: string[] = [
      String(index + 1),
      mt.source,
      ...mt.eePosition.map((v) => String(v)),
      String(q[3]),
      String(q[0]),
      String(q[1]),
      String(q[2]),
      ...mt.eeSceneWorld.map((v) => String(v)),
      ...jointNames.map((_, i) => String(mt.jointPositions[i] ?? 0)),
    ];
    return cells.map(escapeCsvCell).join(',');
  });
  return [header, ...rows].join('\n');
}

export interface MotionTargetCsvParseResult {
  targets: MotionTarget[];
  warnings: string[];
}

/** 自 CSV 解析运动目标；关节列按表头 `q_<name>` 对齐，缺失列填 0 */
export function parseMotionTargetsCsv(
  csvText: string,
  jointNames: string[],
): MotionTargetCsvParseResult {
  const lines = csvText
    .split(/\r?\n/)
    .map((l) => l.trim())
    .filter((l) => l.length > 0 && !l.startsWith('#'));

  if (lines.length < 2) {
    throw new Error('CSV 至少需要表头与一行数据');
  }

  const header = parseCsvLine(lines[0]!);
  const jointColIndex = new Map<string, number>();
  for (let i = 0; i < header.length; i++) {
    const col = header[i]!.trim();
    if (col.startsWith('q_')) {
      jointColIndex.set(col.slice(2), i);
    }
  }

  const fixedIndex = new Map<string, number>();
  for (const name of FIXED_COLS) {
    const idx = header.indexOf(name);
    if (idx >= 0) fixedIndex.set(name, idx);
  }

  const warnings: string[] = [];
  const targets: MotionTarget[] = [];

  for (let row = 1; row < lines.length; row++) {
    const cells = parseCsvLine(lines[row]!);
    const get = (name: (typeof FIXED_COLS)[number], fallback = '0') =>
      cells[fixedIndex.get(name) ?? -1] ?? fallback;

    const joints = jointNames.map((name) => {
      const idx = jointColIndex.get(name);
      if (idx === undefined) {
        return 0;
      }
      const v = Number.parseFloat(cells[idx] ?? '0');
      return Number.isFinite(v) ? v : 0;
    });

    if (jointColIndex.size === 0) {
      const start = FIXED_COLS.length;
      for (let j = 0; j < jointNames.length; j++) {
        const v = Number.parseFloat(cells[start + j] ?? '0');
        joints[j] = Number.isFinite(v) ? v : 0;
      }
    }

    const eePosition: Vec3 = [
      Number.parseFloat(get('ee_px')),
      Number.parseFloat(get('ee_py')),
      Number.parseFloat(get('ee_pz')),
    ];
    const eeQuaternion: Quat = [
      Number.parseFloat(get('ee_qx')),
      Number.parseFloat(get('ee_qy')),
      Number.parseFloat(get('ee_qz')),
      Number.parseFloat(get('ee_qw')),
    ];
    const eeSceneWorld: [number, number, number] = [
      Number.parseFloat(get('scene_wx')),
      Number.parseFloat(get('scene_wy')),
      Number.parseFloat(get('scene_wz')),
    ];

    if (jointColIndex.size > 0) {
      for (const name of jointNames) {
        if (!jointColIndex.has(name)) {
          warnings.push(`第 ${row + 1} 行缺少关节列 q_${name}，已用 0`);
        }
      }
    }

    targets.push({
      id: createMotionTargetId(),
      source: parseSource(get('source', 'joint')),
      jointPositions: joints,
      eePosition,
      eeQuaternion,
      eeSceneWorld,
    });
  }

  return { targets, warnings };
}

export function downloadMotionTargetsCsv(
  targets: MotionTarget[],
  jointNames: string[],
  filename = 'motion_targets.csv',
): void {
  const blob = new Blob([motionTargetsToCsv(targets, jointNames)], {
    type: 'text/csv;charset=utf-8',
  });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  a.click();
  URL.revokeObjectURL(url);
}
