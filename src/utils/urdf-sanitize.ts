const MIN_INERTIA = 0.001;

const INERTIA_ATTRS = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'] as const;

/** Match <inertia .../> or <inertia>...</inertia> including multi-line tags. */
const INERTIA_TAG_RE = /<inertia\b[\s\S]*?(?:\/>|>\s*<\/inertia>)/gi;

function normalizeUrdfLineEndings(urdfText: string): string {
  return urdfText.replace(/\r\n/g, '\n').replace(/\r/g, '\n');
}

function stripXmlDeclaration(urdfText: string): string {
  return urdfText.replace(/^\uFEFF?<\?xml\b[^?]*\?>\s*/i, '');
}

function parseInertiaValue(raw: string | null | undefined): number {
  if (raw == null || raw.trim() === '') return 0;
  const value = Number.parseFloat(raw);
  return Number.isFinite(value) ? value : 0;
}

function parseAttrsFromTag(tag: string): Record<string, string> {
  const attrs: Record<string, string> = {};
  const re = /([\w:]+)="([^"]*)"/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(tag)) !== null) {
    const name = m[1]!.trim();
    if (!name) continue;
    const local = name.includes(':') ? (name.split(':').pop() ?? name) : name;
    if (!local) continue;
    attrs[local] = m[2]!;
  }
  return attrs;
}

export function formatInertiaTag(attrs: Record<string, string>): string {
  const values: Record<string, string> = {};
  for (const name of INERTIA_ATTRS) {
    const raw = attrs[name];
    values[name] = raw == null || raw.trim() === '' ? '0' : raw.trim();
  }
  const ixx = parseInertiaValue(values.ixx);
  const iyy = parseInertiaValue(values.iyy);
  const izz = parseInertiaValue(values.izz);
  if (ixx === 0 && iyy === 0 && izz === 0) {
    values.ixx = String(MIN_INERTIA);
    values.iyy = String(MIN_INERTIA);
    values.izz = String(MIN_INERTIA);
  }
  const body = INERTIA_ATTRS.map((a) => `${a}="${values[a]}"`).join(' ');
  // MuJoCo URDF 解析器对自闭合 <inertia .../> 在部分 WASM 环境下会误报空属性名，使用显式闭合标签。
  return `<inertia ${body}></inertia>`;
}

function fixInertiaTagMatch(tag: string): string {
  return formatInertiaTag(parseAttrsFromTag(tag));
}

function fixInertiaTagsInText(text: string): string {
  return text.replace(INERTIA_TAG_RE, fixInertiaTagMatch);
}

/** Remove empty-name / empty-value attributes that MuJoCo rejects. */
export function stripMalformedEmptyNameAttributes(xml: string): string {
  let result = xml;
  result = result.replace(/\s+="[^"]*"/g, '');
  result = result.replace(/\s+([a-zA-Z_][\w.-]*)=""/g, '');
  result = result.replace(/\s+=\s*"[^"]*"/g, '');
  return result;
}

/** Strip xmlns / prefixed tags produced by browser XMLSerializer. */
export function stripXmlNamespaces(xml: string): string {
  let result = xml.replace(/\s+xmlns(?::\w+)?="[^"]*"/gi, '');
  result = result.replace(/<(\/?)([\w-]+:)([\w-]+)/g, '<$1$3');
  result = result.replace(/([\s<])([\w-]+):([\w-]+)=/g, '$1$3=');
  return stripMalformedEmptyNameAttributes(result);
}

function normalizeInertialInner(inner: string): string {
  const inertiaMatches = [...inner.matchAll(INERTIA_TAG_RE)].map((m) => m[0]!);
  const merged: Record<string, string> = {};
  for (const tag of inertiaMatches) {
    Object.assign(merged, parseAttrsFromTag(tag));
  }
  const inertiaTag =
    inertiaMatches.length > 0 ? formatInertiaTag(merged) : formatInertiaTag({});
  const withoutInertia = inner.replace(INERTIA_TAG_RE, '').trimEnd();
  if (!withoutInertia) return inertiaTag;
  return `${withoutInertia}\n      ${inertiaTag}`;
}

/**
 * 修复 MuJoCo 无法加载的 URDF（空/全零惯量、多行 inertia、重复 inertia、xmlns 等）。
 * 纯文本处理，避免浏览器 XMLSerializer 与测试环境 DOM 行为不一致。
 */
export function sanitizeUrdfForMujoco(urdfText: string): string {
  let result = stripXmlNamespaces(urdfText);
  result = fixInertiaTagsInText(result);

  result = result.replace(/<inertial\b[^>]*>([\s\S]*?)<\/inertial>/gi, (_block, inner: string) => {
    const normalized = normalizeInertialInner(inner);
    return `<inertial>${normalized}</inertial>`;
  });

  result = stripMalformedEmptyNameAttributes(result);
  result = fixInertiaTagsInText(result);
  result = result.replace(/(<inertia\b[^>]*\/>)\s*(<\/inertial>)/gi, '$1\n    $2');
  return result;
}

function lineNumberAt(urdfText: string, index: number): number {
  return urdfText.slice(0, index).split('\n').length;
}

/** 加载前校验所有 inertia 标签具备非空属性（失败时附带行号上下文） */
export function validateUrdfInertiaForMujoco(urdfText: string): void {
  const emptyName = urdfText.search(/\s+="[^"]*"/);
  if (emptyName >= 0) {
    throw new Error(
      `URDF 含空属性名（约第 ${lineNumberAt(urdfText, emptyName)} 行），MuJoCo 无法加载`,
    );
  }
  const strayEquals = urdfText.search(/\s+=\s*"[^"]*"/);
  if (strayEquals >= 0) {
    throw new Error(
      `URDF 含无效属性片段（约第 ${lineNumberAt(urdfText, strayEquals)} 行），MuJoCo 无法加载`,
    );
  }

  for (const m of urdfText.matchAll(INERTIA_TAG_RE)) {
    const tag = m[0]!;
    const line = lineNumberAt(urdfText, m.index ?? 0);
    for (const attr of INERTIA_ATTRS) {
      const am = tag.match(new RegExp(`\\b${attr}="([^"]*)"`));
      if (!am) {
        throw new Error(`URDF inertia 缺少属性 ${attr}（约第 ${line} 行）\n${tag.slice(0, 160)}`);
      }
      if (am[1]!.trim() === '') {
        throw new Error(`URDF inertia 属性 ${attr} 为空（约第 ${line} 行）\n${tag.slice(0, 160)}`);
      }
    }
  }

  for (const bm of urdfText.matchAll(/<inertial\b[^>]*>([\s\S]*?)<\/inertial>/gi)) {
    const count = (bm[1]!.match(/<inertia\b/gi) ?? []).length;
    if (count !== 1) {
      const line = lineNumberAt(urdfText, bm.index ?? 0);
      throw new Error(`<inertial> 块含 ${count} 个 <inertia>（约第 ${line} 行）`);
    }
  }
}

/** sanitize + validate，供负载编辑与 MuJoCo 加载统一入口 */
export function finalizeUrdfForMujoco(urdfText: string): string {
  const sanitized = sanitizeUrdfForMujoco(urdfText);
  validateUrdfInertiaForMujoco(sanitized);
  return sanitized;
}
