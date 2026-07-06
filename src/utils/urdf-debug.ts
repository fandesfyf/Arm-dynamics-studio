function normalizeAssetPath(rawPath: string): string {
  let path = rawPath.trim();
  if (path.startsWith('package://')) {
    const withoutScheme = path.slice('package://'.length);
    const slash = withoutScheme.indexOf('/');
    path = slash >= 0 ? withoutScheme.slice(slash + 1) : withoutScheme;
  }
  return path.replace(/^\.?\//, '').replace(/\\/g, '/');
}

/** 开发时在控制台打印 URDF 片段；localStorage.setItem('urdf-debug','1') 可强制开启 */
export function isUrdfDebugEnabled(): boolean {
  if (import.meta.env.DEV && typeof localStorage !== 'undefined') {
    return localStorage.getItem('urdf-debug') === '1';
  }
  return false;
}

export function logUrdfSnippet(label: string, urdfText: string, centerLine = 28, radius = 4): void {
  if (!isUrdfDebugEnabled()) return;
  const lines = urdfText.split('\n');
  const start = Math.max(0, centerLine - 1 - radius);
  const end = Math.min(lines.length, centerLine - 1 + radius);
  console.group(`[urdf-debug] ${label}`);
  for (let i = start; i < end; i++) {
    console.log(`${i + 1}: ${lines[i] ?? ''}`);
  }
  const payloadLinks = [...urdfText.matchAll(/<link name="([^"]*_payload[^"]*)"/g)].map((m) => m[1]);
  if (payloadLinks.length > 0) console.log('payload links:', payloadLinks);
  console.groupEnd();
}

export function formatUrdfLoadError(urdfText: string, detail: string, dumpPath?: string | null): string {
  const lines = urdfText.split('\n');
  const ctx: string[] = [];
  for (let i = 24; i <= 32 && i <= lines.length; i++) {
    ctx.push(`URDF 第 ${i} 行: ${lines[i - 1]?.trim() ?? ''}`);
  }
  const baseInertia = urdfText.match(/<link name="base_link"[\s\S]*?<inertia\b[^>]*\/?>/);
  const dumpHint = dumpPath ? `\n失败资源包已保存: ${dumpPath}` : '';
  return (
    `MuJoCo 加载 URDF 失败: ${detail}\n` +
    ctx.join('\n') +
    (baseInertia ? `\nbase_link inertia: ${baseInertia[0].slice(-120)}` : '\nbase_link inertia: 未找到') +
    dumpHint
  );
}

function timestampForFilename(): string {
  return new Date().toISOString().replace(/[:.]/g, '-');
}

function bytesToBase64(bytes: Uint8Array): string {
  let binary = '';
  const chunk = 0x8000;
  for (let i = 0; i < bytes.length; i += chunk) {
    binary += String.fromCharCode(...bytes.subarray(i, i + chunk));
  }
  return btoa(binary);
}
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

function resolveMeshBytes(
  ref: string,
  urdfFileName: string,
  meshes: Map<string, Uint8Array>,
): { vfsPath: string; bytes: Uint8Array } | null {
  const candidates = new Set<string>();
  const normalized = normalizeAssetPath(ref);
  const base = ref.split('/').pop() ?? ref;
  candidates.add(ref);
  candidates.add(normalized);
  candidates.add(base);
  candidates.add(`meshes/${base}`);
  candidates.add(normalized.replace(/^urdf\//, ''));
  if (ref.startsWith('../')) {
    candidates.add(ref.replace(/^\.\.\//, ''));
  }
  const urdfDir = urdfFileName.includes('/')
    ? urdfFileName.slice(0, urdfFileName.lastIndexOf('/'))
    : '';
  if (urdfDir) {
    candidates.add(`${urdfDir}/${ref}`.replace(/\/+/g, '/').replace(/^\.\//, ''));
  }

  for (const key of candidates) {
    const bytes = meshes.get(key);
    if (bytes) {
      const vfsPath = normalized.startsWith('meshes/') ? normalized : `meshes/${base}`;
      return { vfsPath: ref.startsWith('../') ? ref.replace(/^\.\.\//, '') : vfsPath, bytes };
    }
  }
  return null;
}

export interface RobotBundleDumpInput {
  urdfText: string;
  urdfFileName: string;
  meshes: Map<string, Uint8Array>;
  detail: string;
  loadPhase?: string;
  rawUrdfLength?: number;
}

/** 失败时导出完整 URDF + mesh 目录（dev server 写入 web/debug-dumps/） */
export async function dumpFailedRobotBundle(input: RobotBundleDumpInput): Promise<string | null> {
  const stamp = timestampForFilename();
  const folderName = `mujoco-failed-${stamp}`;

  const meshEntries: { path: string; base64: string }[] = [];
  const seen = new Set<string>();
  for (const ref of extractMeshRefs(input.urdfText)) {
    const resolved = resolveMeshBytes(ref, input.urdfFileName, input.meshes);
    if (!resolved || seen.has(resolved.vfsPath)) continue;
    seen.add(resolved.vfsPath);
    meshEntries.push({
      path: resolved.vfsPath,
      base64: bytesToBase64(resolved.bytes),
    });
  }

  if (import.meta.env.DEV && typeof fetch !== 'undefined') {
    try {
      const res = await fetch('/__debug/dump-robot-bundle', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          folderName,
          urdfText: input.urdfText,
          urdfFileName: input.urdfFileName,
          detail: input.detail,
          loadPhase: input.loadPhase ?? 'unknown',
          rawUrdfLength: input.rawUrdfLength ?? 0,
          hasPayload: input.urdfText.includes('_payload'),
          meshCount: meshEntries.length,
          meshes: meshEntries,
        }),
      });
      if (res.ok) {
        const data = (await res.json()) as { path?: string };
        const path = data.path ?? folderName;
        console.error('[urdf-debug] 失败资源包已写入:', path);
        return path;
      }
    } catch (e) {
      console.warn('[urdf-debug] 资源包写入失败', e);
    }
  }

  return null;
}
