// @ts-expect-error ESM 解析器路径
import { parseURDF, buildPinocchioModel } from 'pinocchio-js/src/urdf-parser.mjs';
import loadPinocchio from 'pinocchio-js';
import pinocchioWasmUrl from 'pinocchio-js/build/pinocchio.wasm?url';
import type { PinocchioLoadResult, PinocchioModule } from '../types/robot';

const MOVING_JOINT_TYPES = new Set([
  'revolute',
  'continuous',
  'prismatic',
  'floating',
]);

let pinModule: PinocchioModule | null = null;

const inBrowser = typeof window !== 'undefined' && import.meta.env.MODE !== 'test';

type PinocchioFactory = (opts?: {
  locateFile?: (path: string) => string;
}) => Promise<PinocchioModule>;

function isPinocchioModule(value: unknown): value is PinocchioModule {
  return (
    typeof value === 'object' &&
    value !== null &&
    'Model' in value &&
    'Data' in value
  );
}

/** 解析 Vite/CJS/ESM 互操作后的 pinocchio-js 工厂函数 */
function resolvePinocchioFactory(mod: unknown): PinocchioFactory {
  let candidate: unknown = mod;
  for (let i = 0; i < 3; i++) {
    if (typeof candidate === 'function') {
      return candidate as PinocchioFactory;
    }
    if (candidate && typeof candidate === 'object' && 'default' in candidate) {
      candidate = (candidate as { default: unknown }).default;
      continue;
    }
    break;
  }
  throw new Error(
    `pinocchio-js 模块未导出可调用的工厂函数（得到 ${typeof candidate}）`,
  );
}

async function initPinocchioModule(): Promise<PinocchioModule> {
  const factory = resolvePinocchioFactory(loadPinocchio);
  const opts = inBrowser
    ? {
        locateFile: (path: string) =>
          path.endsWith('.wasm') ? pinocchioWasmUrl : path,
      }
    : undefined;
  const loaded = await factory(opts);
  if (isPinocchioModule(loaded)) {
    return loaded;
  }
  throw new Error('pinocchio-js 初始化后未返回有效模块');
}

export async function getPinocchioModule(): Promise<PinocchioModule> {
  if (!pinModule) {
    pinModule = await initPinocchioModule();
  }
  return pinModule;
}

interface ParsedUrdfJoint {
  name: string;
  type: string;
  limits: {
    lower: number;
    upper: number;
  };
}

function extractActuatedJointNamesFromUrdf(urdfText: string): {
  jointNames: string[];
  lowerLimits: number[];
  upperLimits: number[];
  neutralConfiguration: number[];
} {
  const urdfData = parseURDF(urdfText);
  const joints = urdfData.joints as ParsedUrdfJoint[];

  const jointNames: string[] = [];
  const lowerLimits: number[] = [];
  const upperLimits: number[] = [];
  const neutralConfiguration: number[] = [];

  for (const joint of joints) {
    if (!MOVING_JOINT_TYPES.has(joint.type)) continue;
    jointNames.push(joint.name);
    const lo = joint.limits?.lower ?? -Math.PI;
    const hi = joint.limits?.upper ?? Math.PI;
    lowerLimits.push(lo);
    upperLimits.push(hi);
    neutralConfiguration.push((lo + hi) / 2);
  }

  return { jointNames, lowerLimits, upperLimits, neutralConfiguration };
}

/** 从 URDF 文本构建 Pinocchio Model */
export async function loadPinocchioFromUrdf(
  urdfText: string,
): Promise<PinocchioLoadResult> {
  const pin = await getPinocchioModule();
  const urdfData = parseURDF(urdfText);
  const model = buildPinocchioModel(pin, urdfData);

  const meta = extractActuatedJointNamesFromUrdf(urdfText);

  return {
    pin,
    model,
    jointNames: meta.jointNames,
    lowerLimits: meta.lowerLimits,
    upperLimits: meta.upperLimits,
    neutralConfiguration: meta.neutralConfiguration,
    nq: model.nq as number,
    nv: model.nv as number,
  };
}

/** T2 robot-session 使用的别名 */
export const loadPinocchioRobot = loadPinocchioFromUrdf;
