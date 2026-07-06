const MIN_INERTIA = 0.001;

const INERTIA_ATTRS = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'] as const;

/** Match <inertia .../> or <inertia>...</inertia> including multi-line tags. */
const INERTIA_TAG_RE = /<inertia\b[\s\S]*?(?:\/>|>\s*<\/inertia>)/gi;

const TORSO_ZERO_INERTIA_RE =
  /(<link name="torso">[\s\S]*?<inertia\b)(\s+ixx="0"\s+ixy="0"\s+ixz="0"\s+iyy="0"\s+iyz="0"\s+izz="0"\s*)\/>/i;

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

/** 新建/修复用：紧凑自闭合 `<inertia .../>`（`/` 前不能有空格） */
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
  return `<inertia ${body}/>`;
}

function isValidInertiaTag(tag: string): boolean {
  if (/\s+="/.test(tag)) return false;
  if (/"\s+[>\/]/.test(tag)) return false;
  if (/<inertia\b[^>]*?\s+\/>/.test(tag)) return false;
  const attrs = parseAttrsFromTag(tag);
  for (const name of INERTIA_ATTRS) {
    const raw = attrs[name];
    if (raw == null || raw.trim() === '') return false;
  }
  const ixx = parseInertiaValue(attrs.ixx);
  const iyy = parseInertiaValue(attrs.iyy);
  const izz = parseInertiaValue(attrs.izz);
  return !(ixx === 0 && iyy === 0 && izz === 0);
}

function hasXmlnsOrDomArtifacts(xml: string): boolean {
  return /xmlns/i.test(xml) || /\s+="[^"]*"/.test(xml) || /:[\w-]+=/.test(xml);
}

function hasBrokenInertialBlocks(xml: string): boolean {
  for (const m of xml.matchAll(/<inertial\b[\s\S]*?<\/inertial>/gi)) {
    const block = m[0]!;
    const tags = [...block.matchAll(INERTIA_TAG_RE)].map((x) => x[0]!);
    if (tags.length === 0) return true;
    if (tags.length > 1) return true;
    if (!isValidInertiaTag(tags[0]!)) return true;
  }
  return false;
}

function fixInertialBlock(block: string): string {
  const tags = [...block.matchAll(INERTIA_TAG_RE)].map((m) => m[0]!);
  let inertiaTag: string;
  if (tags.length === 0) {
    inertiaTag = formatInertiaTag({});
  } else {
    const preferred = tags.find((t) => isValidInertiaTag(t)) ?? tags[0]!;
    inertiaTag = isValidInertiaTag(preferred) ? preferred : formatInertiaTag(parseAttrsFromTag(preferred));
  }
  const without = block.replace(INERTIA_TAG_RE, '');
  return without.replace(/<\/inertial>/i, `  ${inertiaTag}\n  </inertial>`);
}

function fixBrokenInertialBlocksOnly(urdfText: string): string {
  return urdfText.replace(/<inertial\b[\s\S]*?<\/inertial>/gi, (block) => fixInertialBlock(block));
}

/** MuJoCo WASM 要求自闭合标签为 `/>`，` />` 会解析出空属性名 */
export function tightenSelfClosingTagsForMujoco(urdfText: string): string {
  return urdfText
    .replace(/ \/>/g, '/>')
    .replace(/ ><\/(\w+)>/g, '/></$1>');
}

/** @deprecated 使用 {@link tightenSelfClosingTagsForMujoco} */
export function tightenInertiaTagsForMujoco(urdfText: string): string {
  return tightenSelfClosingTagsForMujoco(urdfText);
}

/** 仅修复 biped torso 全零惯量（O(1) 字符串替换，不扫描全文） */
function fixTorsoZeroInertia(urdfText: string): string {
  return urdfText.replace(
    TORSO_ZERO_INERTIA_RE,
    '$1 ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>',
  );
}

/** Strip xmlns / prefixed tags produced by browser XMLSerializer. */
export function stripXmlNamespaces(xml: string): string {
  let result = xml.replace(/\s+xmlns(?::\w+)?="[^"]*"/gi, '');
  result = result.replace(/<(\/?)([\w-]+:)([\w-]+)/g, '<$1$3');
  result = result.replace(/([\s<])([\w-]+):([\w-]+)=/g, '$1$3=');
  result = result.replace(/\s+="[^"]*"/g, '');
  result = result.replace(/\s+([a-zA-Z_][\w.-]*)=""/g, '');
  return result;
}

/**
 * MuJoCo 加载前 URDF 预处理：默认仅 strip xmlns + 修 torso + 收紧 inertia 空格。
 * 只有 DOM round-trip / 空惯量等异常才做全文 inertia 修复。
 */
export function sanitizeUrdfForMujoco(urdfText: string): string {
  let result = stripXmlDeclaration(urdfText);
  result = stripXmlNamespaces(result);
  result = fixTorsoZeroInertia(result);
  result = tightenSelfClosingTagsForMujoco(result);
  if (hasXmlnsOrDomArtifacts(urdfText) || hasBrokenInertialBlocks(result)) {
    result = fixBrokenInertialBlocksOnly(result);
    result = tightenSelfClosingTagsForMujoco(result);
  }
  return result;
}

function stripXmlDeclaration(urdfText: string): string {
  return urdfText.replace(/^\uFEFF?<\?xml\b[^?]*\?>\s*/i, '');
}

/** 加载管线统一入口：去 XML 声明 + MuJoCo WASM 安全化 */
export function prepareUrdfForMujocoLoad(urdfText: string): string {
  return sanitizeUrdfForMujoco(urdfText);
}

/** 加载前校验（仅检查 <inertia> 标签，不误伤 <mass/> 等合法空格） */
export function validateUrdfInertiaForMujoco(urdfText: string): void {
  for (const m of urdfText.matchAll(INERTIA_TAG_RE)) {
    const tag = m[0]!;
    const line = urdfText.slice(0, m.index ?? 0).split('\n').length;
    if (/<inertia\b[^>]*?\s+\/>/.test(tag) || /"\s+[>\/]/.test(tag)) {
      throw new Error(`URDF <inertia> 在 / 前含空格（约第 ${line} 行）`);
    }
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
}

export function finalizeUrdfForMujoco(urdfText: string): string {
  const sanitized = sanitizeUrdfForMujoco(urdfText);
  validateUrdfInertiaForMujoco(sanitized);
  return sanitized;
}
