import { resolveAssetPath } from './robot-asset-loader';
import { prepareUrdfForMujocoLoad } from './urdf-sanitize';

/** Vite public 静态 URDF（无 mesh，动力学加载不依赖 STL） */
const DEFAULT_URDF_PATH = '/robots/biped_s70_upper_body.urdf';
const DEFAULT_URDF_FILE = 'urdf/biped_s70_upper_body.urdf';
const MESH_CACHE_NAME = 'arm-sim-biped-meshes-v1';

function extractMeshRefs(urdfText: string): string[] {
  const refs = new Set<string>();
  const re = /filename="([^"]+)"/gi;
  let match: RegExpExecArray | null;
  while ((match = re.exec(urdfText)) !== null) {
    const raw = match[1];
    if (raw && /\.(stl|dae|obj|ply)$/i.test(raw)) {
      refs.add(raw);
    }
  }
  return [...refs];
}

function meshContentType(path: string): string {
  if (/\.stl$/i.test(path)) return 'model/stl';
  if (/\.dae$/i.test(path)) return 'model/vnd.collada+xml';
  return 'application/octet-stream';
}

function meshAssetUrl(relativePath: string): string {
  return `/biped-assets/${relativePath.replace(/^\/+/, '')}`;
}

function meshPathCandidates(urdfFileName: string, ref: string): string[] {
  const resolved = resolveAssetPath(urdfFileName, ref);
  const base = ref.split('/').pop() ?? ref;
  const set = new Set<string>([
    resolved,
    resolved.replace(/^urdf\//, ''),
    `meshes/${base}`,
  ]);
  if (ref.startsWith('../')) {
    set.add(ref.replace(/^\.\.\//, ''));
  }
  if (ref.startsWith('meshes/')) {
    set.add(ref);
    set.add(`urdf/${ref}`);
  }
  return [...set];
}

async function fetchMeshBytes(relativePath: string): Promise<Uint8Array> {
  const url = meshAssetUrl(relativePath);
  if (typeof caches !== 'undefined') {
    try {
      const cache = await caches.open(MESH_CACHE_NAME);
      const hit = await cache.match(url);
      if (hit) {
        return new Uint8Array(await hit.arrayBuffer());
      }
      const res = await fetch(url);
      if (!res.ok) {
        throw new Error(`HTTP ${res.status} for ${relativePath}`);
      }
      const bytes = new Uint8Array(await res.arrayBuffer());
      void cache.put(url, new Response(bytes));
      return bytes;
    } catch {
      // Cache API 不可用时回退普通 fetch
    }
  }
  const res = await fetch(url);
  if (!res.ok) {
    throw new Error(`HTTP ${res.status} for ${relativePath}`);
  }
  return new Uint8Array(await res.arrayBuffer());
}

async function fetchMeshFirstHit(candidates: string[]): Promise<{ path: string; bytes: Uint8Array }> {
  let lastErr: Error | null = null;
  for (const relativePath of candidates) {
    try {
      return { path: relativePath, bytes: await fetchMeshBytes(relativePath) };
    } catch (e) {
      lastErr = e instanceof Error ? e : new Error(String(e));
    }
  }
  throw lastErr ?? new Error(`无法加载 mesh: ${candidates.join(', ')}`);
}

function indexMeshAliases(
  meshes: Map<string, Uint8Array>,
  ref: string,
  resolvedPath: string,
  bytes: Uint8Array,
): void {
  meshes.set(resolvedPath, bytes);
  meshes.set(ref, bytes);
  meshes.set(ref.replace(/^\.\.\//, ''), bytes);
  const base = resolvedPath.split('/').pop();
  if (base) meshes.set(base, bytes);
}

let cachedUrdf: { urdfText: string; urdfFileName: string } | null = null;
let inFlightUrdf: Promise<{ urdfText: string; urdfFileName: string }> | null = null;
let cachedMeshes: Map<string, Uint8Array> | null = null;
let inFlightMeshes: Promise<Map<string, Uint8Array>> | null = null;

async function loadDefaultBipedUrdf(): Promise<{ urdfText: string; urdfFileName: string }> {
  if (cachedUrdf) return cachedUrdf;
  if (inFlightUrdf) return inFlightUrdf;

  inFlightUrdf = (async () => {
    const urdfRes = await fetch(DEFAULT_URDF_PATH);
    if (!urdfRes.ok) {
      throw new Error(`无法加载默认 URDF: HTTP ${urdfRes.status}`);
    }
    const urdfText = prepareUrdfForMujocoLoad(await urdfRes.text());
    cachedUrdf = { urdfText, urdfFileName: DEFAULT_URDF_FILE };
    return cachedUrdf;
  })();

  try {
    return await inFlightUrdf;
  } finally {
    inFlightUrdf = null;
  }
}

/**
 * 默认 biped：仅 URDF（MuJoCo/Pinocchio 可立即加载，不等待 STL）。
 * 查看器 mesh 用 {@link loadDefaultBipedMeshes} 后台加载。
 */
export async function loadDefaultBipedUpperBody(): Promise<{
  urdfText: string;
  urdfFileName: string;
  meshes: Map<string, Uint8Array>;
}> {
  const { urdfText, urdfFileName } = await loadDefaultBipedUrdf();
  return { urdfText, urdfFileName, meshes: new Map() };
}

/** 后台加载 biped STL（仅 Three.js 查看器需要） */
export async function loadDefaultBipedMeshes(): Promise<Map<string, Uint8Array>> {
  if (cachedMeshes) return cachedMeshes;
  if (inFlightMeshes) return inFlightMeshes;

  inFlightMeshes = (async () => {
    const { urdfText, urdfFileName } = await loadDefaultBipedUrdf();
    const meshes = new Map<string, Uint8Array>();
    const refs = extractMeshRefs(urdfText);
    await Promise.all(
      refs.map(async (ref) => {
        const { path, bytes } = await fetchMeshFirstHit(
          meshPathCandidates(urdfFileName, ref),
        );
        indexMeshAliases(meshes, ref, path, bytes);
      }),
    );
    cachedMeshes = meshes;
    return meshes;
  })();

  try {
    return await inFlightMeshes;
  } finally {
    inFlightMeshes = null;
  }
}

/** 应用启动：URDF 与 mesh 并行预取（mesh 不阻塞动力学就绪） */
export function prefetchDefaultBipedUpperBody(): void {
  void loadDefaultBipedUrdf().catch(() => undefined);
  void loadDefaultBipedMeshes().catch(() => undefined);
}

export { DEFAULT_URDF_PATH, meshContentType };
