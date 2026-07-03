import { resolveAssetPath } from './robot-asset-loader';

const DEFAULT_URDF_PATH = '/biped-assets/urdf/biped_s70_upper_body.urdf';
const DEFAULT_URDF_FILE = 'urdf/biped_s70_upper_body.urdf';

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

async function fetchMeshCandidates(candidates: string[]): Promise<{ path: string; bytes: Uint8Array }> {
  let lastErr: Error | null = null;
  for (const relativePath of candidates) {
    const url = `/biped-assets/${relativePath}`;
    try {
      const res = await fetch(url);
      if (!res.ok) {
        lastErr = new Error(`HTTP ${res.status} for ${relativePath}`);
        continue;
      }
      return { path: relativePath, bytes: new Uint8Array(await res.arrayBuffer()) };
    } catch (e) {
      lastErr = e instanceof Error ? e : new Error(String(e));
    }
  }
  throw lastErr ?? new Error(`无法加载 mesh: ${candidates.join(', ')}`);
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

/** 从 Vite 托管的 /biped-assets 加载默认 biped 上肢模型 */
export async function loadDefaultBipedUpperBody(): Promise<{
  urdfText: string;
  urdfFileName: string;
  meshes: Map<string, Uint8Array>;
}> {
  const urdfRes = await fetch(DEFAULT_URDF_PATH);
  if (!urdfRes.ok) {
    throw new Error(`无法加载默认 URDF: HTTP ${urdfRes.status}`);
  }
  const urdfText = await urdfRes.text();
  const meshes = new Map<string, Uint8Array>();

  const refs = extractMeshRefs(urdfText);
  await Promise.all(
    refs.map(async (ref) => {
      const { path, bytes } = await fetchMeshCandidates(meshPathCandidates(DEFAULT_URDF_FILE, ref));
      meshes.set(path, bytes);
      const resolved = resolveAssetPath(DEFAULT_URDF_FILE, ref);
      meshes.set(resolved, bytes);
      const base = path.split('/').pop();
      if (base) meshes.set(base, bytes);
      meshes.set(ref.replace(/^\.\.\//, ''), bytes);
    }),
  );

  return { urdfText, urdfFileName: DEFAULT_URDF_FILE, meshes };
}

export { DEFAULT_URDF_PATH, meshContentType };
