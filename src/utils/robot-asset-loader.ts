import JSZip from 'jszip';

const URDF_EXT = /\.urdf$/i;
const MESH_EXT = /\.(stl|dae|obj|ply|collada)$/i;
const URDF_SKIP_DIRS = /\/(drake|deprecated|test|tests)\//i;

export interface RobotAssetExtract {
  urdfText: string;
  urdfFileName: string;
  meshes: Map<string, Uint8Array>;
}

export interface FileEntry {
  relativePath: string;
  readBytes: () => Promise<Uint8Array>;
  readText?: () => Promise<string>;
}

function normalizePath(path: string): string {
  return path.replace(/\\/g, '/').replace(/^\/+/, '');
}

function basename(path: string): string {
  const normalized = normalizePath(path);
  const idx = normalized.lastIndexOf('/');
  return idx >= 0 ? normalized.slice(idx + 1) : normalized;
}

/** 去掉 zip/文件夹共用的顶层包名（如 biped_s70/） */
export function stripPackageRoot(paths: string[]): { paths: string[]; prefix: string } {
  if (paths.length === 0) return { paths, prefix: '' };

  const split = paths.map((p) => normalizePath(p).split('/').filter(Boolean));
  const minDepth = Math.min(...split.map((p) => p.length));

  let common = 0;
  for (let i = 0; i < minDepth - 1; i++) {
    const seg = split[0]![i];
    if (split.every((p) => p[i] === seg)) common++;
    else break;
  }

  if (common === 0 && split.every((p) => p.length > 1 && p[0] === split[0]![0])) {
    common = 1;
  }

  if (common === 0) {
    return { paths: paths.map(normalizePath), prefix: '' };
  }

  const prefix = split[0]!.slice(0, common).join('/');
  return {
    prefix,
    paths: split.map((parts) => parts.slice(common).join('/')),
  };
}

export function pickUrdfPath(paths: string[]): string {
  let candidates = paths.filter((p) => URDF_EXT.test(p));
  if (candidates.length === 0) {
    throw new Error('未找到 .urdf 文件');
  }

  candidates = candidates.filter((p) => !URDF_SKIP_DIRS.test(`/${p}/`));

  const inUrdfFolder = candidates.filter((p) => /(^|\/)urdf\/[^/]+\.urdf$/i.test(p));
  const pool = inUrdfFolder.length > 0 ? inUrdfFolder : candidates;

  const scored = pool.map((p) => {
    const fileName = basename(p).replace(/\.urdf$/i, '');
    const topFolder = p.split('/')[0] ?? '';
    let score = 0;
    if (fileName === topFolder) score += 100;
    else if (topFolder && fileName.includes(topFolder)) score += 60;
    else if (fileName === 'robot') score += 20;
    if (/upper_body|upperbody|_arm\b/i.test(fileName)) score += 80;
    if (/\/urdf\//i.test(p)) score += 30;
    score -= p.split('/').length;
    return { p, score };
  });

  scored.sort((a, b) => b.score - a.score);
  return scored[0]!.p;
}

/** 将 URDF mesh 引用解析为 bundle 内相对路径（与 VFS /robot/ 根一致） */
export function resolveAssetPath(urdfFileName: string, meshRef: string): string {
  let ref = meshRef.trim();
  if (ref.startsWith('package://')) {
    const withoutScheme = ref.slice('package://'.length);
    const slash = withoutScheme.indexOf('/');
    ref = slash >= 0 ? withoutScheme.slice(slash + 1) : withoutScheme;
  }
  ref = ref.replace(/\\/g, '/');

  const baseDir = urdfFileName.includes('/')
    ? urdfFileName.slice(0, urdfFileName.lastIndexOf('/') + 1)
    : '';

  const stack = baseDir.split('/').filter(Boolean);
  for (const seg of ref.split('/')) {
    if (seg === '..') stack.pop();
    else if (seg !== '.' && seg !== '') stack.push(seg);
  }
  return stack.join('/');
}

export function findMeshBytes(
  meshes: Map<string, Uint8Array>,
  resolvedPath: string,
): Uint8Array | undefined {
  if (meshes.has(resolvedPath)) {
    return meshes.get(resolvedPath);
  }
  const fileName = basename(resolvedPath);
  for (const [path, bytes] of meshes) {
    if (path === resolvedPath || path.endsWith(`/${resolvedPath}`)) {
      return bytes;
    }
    if (basename(path).toLowerCase() === fileName.toLowerCase()) {
      return bytes;
    }
  }
  return undefined;
}

async function buildBundleFromPaths(
  rawPaths: string[],
  readFile: (path: string) => Promise<{ text?: string; bytes?: Uint8Array }>,
): Promise<RobotAssetExtract> {
  const normalized = rawPaths.map(normalizePath).filter(Boolean);
  const { paths } = stripPackageRoot(normalized);
  const pathSet = new Set(paths);

  const urdfRelPath = pickUrdfPath(paths);
  const urdfData = await readFile(urdfRelPath);
  if (!urdfData.text) {
    throw new Error(`无法读取 URDF: ${urdfRelPath}`);
  }

  const meshes = new Map<string, Uint8Array>();
  for (const path of paths) {
    if (path === urdfRelPath) continue;
    if (!MESH_EXT.test(path)) continue;
    const data = await readFile(path);
    if (data.bytes) {
      meshes.set(path, data.bytes);
    }
  }

  if (meshes.size === 0) {
    for (const path of pathSet) {
      if (path === urdfRelPath || !MESH_EXT.test(path)) continue;
      const data = await readFile(path);
      if (data.bytes) meshes.set(path, data.bytes);
    }
  }

  return {
    urdfText: urdfData.text,
    urdfFileName: urdfRelPath,
    meshes,
  };
}

async function toArrayBuffer(
  input: ArrayBuffer | Blob | File | Uint8Array,
): Promise<ArrayBuffer> {
  if (input instanceof ArrayBuffer) {
    return input;
  }
  if (input instanceof Uint8Array) {
    return input.buffer.slice(
      input.byteOffset,
      input.byteOffset + input.byteLength,
    ) as ArrayBuffer;
  }
  return input.arrayBuffer();
}

export async function extractRobotFromZip(
  input: ArrayBuffer | Blob | File | Uint8Array,
): Promise<RobotAssetExtract> {
  const buffer = await toArrayBuffer(input);
  const zip = await JSZip.loadAsync(buffer);

  const rawPaths = Object.keys(zip.files).filter((p) => !zip.files[p]!.dir);
  const zipMap = new Map(rawPaths.map((p) => [normalizePath(p), zip.files[p]!]));

  return buildBundleFromPaths(rawPaths.map(normalizePath), async (relPath) => {
    const entry =
      zipMap.get(relPath) ??
      [...zipMap.entries()].find(([p]) => p.endsWith(`/${relPath}`))?.[1];
    if (!entry) {
      throw new Error(`ZIP 内缺少文件: ${relPath}`);
    }
    if (URDF_EXT.test(relPath)) {
      return { text: await entry.async('string') };
    }
    return { bytes: await entry.async('uint8array') };
  });
}

export async function extractRobotFromFiles(
  files: File[],
): Promise<RobotAssetExtract> {
  if (files.length === 0) {
    throw new Error('未收到任何文件');
  }

  const fileMap = new Map<string, File>();
  for (const file of files) {
    const path = normalizePath(file.webkitRelativePath || file.name);
    fileMap.set(path, file);
    if (!fileMap.has(file.name)) {
      fileMap.set(file.name, file);
    }
  }

  const paths = [...new Set(fileMap.keys())];

  return buildBundleFromPaths(paths, async (relPath) => {
    const file =
      fileMap.get(relPath) ??
      [...fileMap.entries()].find(([p]) => p.endsWith(`/${relPath}`))?.[1];
    if (!file) {
      throw new Error(`文件夹内缺少文件: ${relPath}`);
    }
    if (URDF_EXT.test(relPath)) {
      return { text: await file.text() };
    }
    const buf = await file.arrayBuffer();
    return { bytes: new Uint8Array(buf) };
  });
}

export async function extractRobotFromFileList(
  file: File,
): Promise<RobotAssetExtract> {
  const lower = file.name.toLowerCase();
  if (lower.endsWith('.zip')) {
    return extractRobotFromZip(file);
  }
  if (lower.endsWith('.urdf')) {
    const urdfText = await file.text();
    return {
      urdfText,
      urdfFileName: file.name,
      meshes: new Map(),
    };
  }
  throw new Error('请上传 .urdf、.zip 或拖入含 URDF 的文件夹');
}
