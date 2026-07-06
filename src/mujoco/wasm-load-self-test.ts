import { loadMujocoRobot, releaseActiveMujocoHandles } from './loader';
import { prepareUrdfForMujocoLoad } from '../utils/urdf-sanitize';
import { appendSpherePayloadWithRecord } from '../core/payload-editor';

export interface WasmLoadSelfTestResult {
  ok: boolean;
  label: string;
  error?: string;
  jointCount?: number;
  ms?: number;
}

async function runOne(
  label: string,
  urdfText: string,
  urdfFileName: string,
  meshes: Map<string, Uint8Array>,
  loadPhase: 'initial' | 'payload-reload' | 'manual',
): Promise<WasmLoadSelfTestResult> {
  const t0 = performance.now();
  try {
    const bundle = await loadMujocoRobot({
      urdfText,
      urdfFileName,
      meshes,
      loadPhase,
    });
    const ms = performance.now() - t0;
    const jointCount = bundle.jointNames.length;
    releaseActiveMujocoHandles();
    return { ok: true, label, jointCount, ms };
  } catch (e) {
    return {
      ok: false,
      label,
      error: e instanceof Error ? e.message : String(e),
      ms: performance.now() - t0,
    };
  }
}

/** 浏览器控制台 / Playwright 用：在真实 WASM MuJoCo 下验证 URDF 加载 */
export async function runWasmLoadSelfTest(
  urdfText?: string,
  meshes?: Map<string, Uint8Array>,
): Promise<WasmLoadSelfTestResult[]> {
  const results: WasmLoadSelfTestResult[] = [];
  const fileName = 'urdf/biped_s70_upper_body.urdf';
  const meshMap = meshes ?? new Map<string, Uint8Array>();

  let base = urdfText;
  if (!base) {
    const res = await fetch('/biped-assets/urdf/biped_s70_upper_body.urdf');
    if (!res.ok) throw new Error(`fetch biped urdf failed: ${res.status}`);
    base = await res.text();
  }

  const prepared = prepareUrdfForMujocoLoad(base);
  results.push(await runOne('initial-prepared', prepared, fileName, meshMap, 'initial'));

  const { urdfText: withPayload } = appendSpherePayloadWithRecord(prepared, {
    parentLink: 'base_link',
    mass: 0.2,
    radius: 0.03,
    mode: 'child_link',
  });
  const payloadPrepared = prepareUrdfForMujocoLoad(withPayload);
  results.push(
    await runOne('payload-reload-prepared', payloadPrepared, fileName, meshMap, 'payload-reload'),
  );

  return results;
}


declare global {
  interface Window {
    __wasmLoadSelfTest?: typeof runWasmLoadSelfTest;
  }
}

if (typeof window !== 'undefined' && import.meta.env.DEV) {
  window.__wasmLoadSelfTest = runWasmLoadSelfTest;
}
