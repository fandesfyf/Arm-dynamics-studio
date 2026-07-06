import type { MotionTarget } from '../types/motion-target';
import { createMotionTargetId } from '../types/motion-target';

const INDEX_COL = 'index';

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

/** 导出运动目标队列为关节空间 CSV（`index` + `q_<jointName>`） */
export function motionTargetsToCsv(
  targets: MotionTarget[],
  jointNames: string[],
): string {
  const jointCols = jointNames.map((n) => `q_${n}`);
  const header = [INDEX_COL, ...jointCols].join(',');
  const rows = targets.map((mt, index) => {
    const cells: string[] = [
      String(index + 1),
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

function resolveJointColumns(header: string[]): {
  jointColIndex: Map<string, number>;
  legacyOffset: number;
} {
  const jointColIndex = new Map<string, number>();
  for (let i = 0; i < header.length; i++) {
    const col = header[i]!.trim();
    if (col.startsWith('q_')) {
      jointColIndex.set(col.slice(2), i);
    }
  }

  if (jointColIndex.size > 0) {
    return { jointColIndex, legacyOffset: -1 };
  }

  // 兼容旧版：index,source,ee_*,scene_*,q_* 或无 q_ 前缀的纯关节列
  const legacyFixed = [
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
  ];
  let legacyOffset = 0;
  for (const name of legacyFixed) {
    if (header[legacyOffset]?.trim() === name) {
      legacyOffset += 1;
    } else {
      break;
    }
  }
  return { jointColIndex, legacyOffset };
}

/** 自 CSV 解析运动目标；仅读取关节列 `q_<name>`，末端位姿由导入后 FK 填充 */
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

  const header = parseCsvLine(lines[0]!).map((c) => c.trim());
  const { jointColIndex, legacyOffset } = resolveJointColumns(header);

  const warnings: string[] = [];
  const targets: MotionTarget[] = [];
  const placeholderEe: [number, number, number] = [0, 0, 0];
  const placeholderQuat: [number, number, number, number] = [0, 0, 0, 1];

  for (let row = 1; row < lines.length; row++) {
    const cells = parseCsvLine(lines[row]!);

    const joints = jointNames.map((name) => {
      const idx = jointColIndex.get(name);
      if (idx === undefined) {
        return 0;
      }
      const v = Number.parseFloat(cells[idx] ?? '0');
      return Number.isFinite(v) ? v : 0;
    });

    if (jointColIndex.size === 0 && legacyOffset >= 0) {
      for (let j = 0; j < jointNames.length; j++) {
        const v = Number.parseFloat(cells[legacyOffset + j] ?? '0');
        joints[j] = Number.isFinite(v) ? v : 0;
      }
    }

    if (jointColIndex.size > 0) {
      for (const name of jointNames) {
        if (!jointColIndex.has(name)) {
          warnings.push(`第 ${row + 1} 行缺少关节列 q_${name}，已用 0`);
        }
      }
    }

    targets.push({
      id: createMotionTargetId(),
      source: 'joint',
      jointPositions: joints,
      eePosition: placeholderEe,
      eeQuaternion: placeholderQuat,
      eeSceneWorld: placeholderEe,
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
