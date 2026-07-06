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

/** 去掉 zip/文件夹共用的顶层包名（如 biped_s70/），仅剥离一层目录 */
export function stripPackageRoot(paths: string[]): { paths: string[]; prefix: string } {
  if (paths.length === 0) return { paths, prefix: '' };

  const split = paths.map((p) => normalizePath(p).split('/').filter(Boolean));
  const first = split[0]?.[0];
  if (
    first &&
    split.every((p) => p.length > 1 && p[0] === first) &&
    split.some((p) => p.length > 2)
  ) {
    return {
      prefix: first,
      paths: split.map((parts) => parts.slice(1).join('/')),
    };
  }

  return { paths: paths.map(normalizePath), prefix: '' };
}

function stripPathWithPrefix(path: string, prefix: string): string {
  const normalized = normalizePath(path);
  if (!prefix) return normalized;
  if (normalized === prefix) return '';
  const withSlash = `${prefix}/`;
  if (normalized.startsWith(withSlash)) {
    return normalized.slice(withSlash.length);
  }
  return normalized;
}

export interface PreparedFolderFiles {
  prefix: string;
  strippedPaths: string[];
  strippedToFile: Map<string, File>;
}

/** 将文件夹上传的文件列表规范为 strip 后的相对路径，并建立路径→File 映射 */
export function prepareFolderFiles(files: File[]): PreparedFolderFiles {
  const fileMap = new Map<string, File>();
  for (const file of files) {
    const path = normalizePath(file.webkitRelativePath || file.name);
    if (path) fileMap.set(path, file);
  }
  const rawPaths = [...fileMap.keys()];
  const { prefix, paths } = stripPackageRoot(rawPaths);
  const strippedToFile = new Map<string, File>();
  for (const [fullPath, file] of fileMap) {
    const stripped = stripPathWithPrefix(fullPath, prefix);
    if (stripped) strippedToFile.set(stripped, file);
  }
  return {
    prefix,
    strippedPaths: [...new Set(paths.filter(Boolean))],
    strippedToFile,
  };
}

function scoreUrdfPath(p: string): number {
  const fileName = basename(p).replace(/\.urdf$/i, '');
  const topFolder = p.split('/')[0] ?? '';
  let score = 0;
  if (fileName === topFolder) score += 100;
  else if (topFolder && fileName.includes(topFolder)) score += 60;
  else if (fileName === 'robot') score += 20;
  if (/upper_body|upperbody|_arm\b/i.test(fileName)) score += 80;
  if (/\/urdf\//i.test(p)) score += 30;
  score -= p.split('/').length;
  return score;
}

/** 列出 ZIP/文件夹内可选 URDF（递归子目录，按推荐度排序） */
export function listUrdfCandidates(
  paths: string[],
  options?: { includeSkippedDirs?: boolean },
): string[] {
  let candidates = paths.filter((p) => URDF_EXT.test(p));
  if (candidates.length === 0) {
    throw new Error('未找到 .urdf 文件');
  }

  if (!options?.includeSkippedDirs) {
    candidates = candidates.filter((p) => !URDF_SKIP_DIRS.test(`/${p}/`));
  }

  return candidates
    .map((p) => ({ p, score: scoreUrdfPath(p) }))
    .sort((a, b) => b.score - a.score || a.p.localeCompare(b.p))
    .map((item) => item.p);
}

export function pickUrdfPath(paths: string[]): string {
  return listUrdfCandidates(paths)[0]!;
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
  urdfRelPathOverride?: string,
): Promise<RobotAssetExtract> {
  const normalized = rawPaths.map(normalizePath).filter(Boolean);
  const { paths } = stripPackageRoot(normalized);
  const pathSet = new Set(paths);

  const urdfRelPath = urdfRelPathOverride ?? pickUrdfPath(paths);
  if (!pathSet.has(urdfRelPath)) {
    throw new Error(`URDF 不存在: ${urdfRelPath}`);
  }
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

export async function listUrdfPathsFromZip(
  input: ArrayBuffer | Blob | File | Uint8Array,
  options?: { includeSkippedDirs?: boolean },
): Promise<string[]> {
  const buffer = await toArrayBuffer(input);
  const zip = await JSZip.loadAsync(buffer);
  const rawPaths = Object.keys(zip.files).filter((p) => !zip.files[p]!.dir);
  const { paths } = stripPackageRoot(rawPaths.map(normalizePath));
  return listUrdfCandidates(paths, options);
}

export function listUrdfPathsFromFiles(
  files: File[],
  options?: { includeSkippedDirs?: boolean },
): string[] {
  const { strippedPaths } = prepareFolderFiles(files);
  return listUrdfCandidates(strippedPaths, options);
}

export async function extractRobotFromZip(
  input: ArrayBuffer | Blob | File | Uint8Array,
  urdfRelPath?: string,
): Promise<RobotAssetExtract> {
  const buffer = await toArrayBuffer(input);
  const zip = await JSZip.loadAsync(buffer);

  const rawPaths = Object.keys(zip.files).filter((p) => !zip.files[p]!.dir);
  const zipMap = new Map(rawPaths.map((p) => [normalizePath(p), zip.files[p]!]));

  return buildBundleFromPaths(
    rawPaths.map(normalizePath),
    async (relPath) => {
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
    },
    urdfRelPath,
  );
}

export async function extractRobotFromFiles(
  files: File[],
  urdfRelPath?: string,
): Promise<RobotAssetExtract> {
  if (files.length === 0) {
    throw new Error('未收到任何文件');
  }

  const { strippedPaths, strippedToFile } = prepareFolderFiles(files);

  return buildBundleFromPaths(
    strippedPaths,
    async (relPath) => {
      const file =
        strippedToFile.get(relPath) ??
        [...strippedToFile.entries()].find(([p]) => p.endsWith(`/${relPath}`))?.[1];
      if (!file) {
        throw new Error(`文件夹内缺少文件: ${relPath}`);
      }
      if (URDF_EXT.test(relPath)) {
        return { text: await file.text() };
      }
      const buf = await file.arrayBuffer();
      return { bytes: new Uint8Array(buf) };
    },
    urdfRelPath,
  );
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
