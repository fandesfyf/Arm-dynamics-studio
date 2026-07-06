import type { InertiaTensor } from './mass-editor';
import { MassEditor } from './mass-editor';
import { parseLinkNames } from '../utils/urdf-base-fixture';
import { formatInertiaTag, sanitizeUrdfForMujoco } from '../utils/urdf-sanitize';
import { isUrdfDebugEnabled, logUrdfSnippet } from '../utils/urdf-debug';

/** 6D 外力/力矩 [fx, fy, fz, tx, ty, tz]，link 坐标系 */
export type Wrench6 = [number, number, number, number, number, number];

export type SpherePayloadMode = 'child_link' | 'modify_inertial';

export interface SpherePayloadOptions {
  parentLink: string;
  mass: number;
  radius: number;
  mode?: SpherePayloadMode;
  suffix?: string;
}

export interface AttachUrdfSnippetOptions {
  parentLink: string;
  snippetXml: string;
  jointName?: string;
  prefix?: string;
}

export interface ParsedUrdfSnippet {
  links: string[];
  joints: string[];
}

export type PayloadKind = 'child_link' | 'modify_inertial';

export interface PayloadRecord {
  id: string;
  kind: PayloadKind;
  parentLink: string;
  payloadLink?: string;
  jointName?: string;
  mass: number;
  radius: number;
  /** modify_inertial 模式下保存原始惯量以便还原 */
  originalInertial?: {
    mass: number;
    inertia: InertiaTensor;
  };
}

/** 球体负载 link 命名约定：{parent}_payload[_N]_sphere[_M] */
export const SPHERE_PAYLOAD_LINK_PATTERN = /_payload(?:_\d+)?_sphere(?:_\d+)?$/;

/** 3D 视图中球体负载显示色（深褐色） */
export const SPHERE_PAYLOAD_VISUAL_COLOR = '#5c4033';

export interface SpherePayloadDisplayItem {
  id: string;
  kind: PayloadKind;
  parentLink: string;
  payloadLink?: string;
  mass: number;
  radius: number;
}

const WRENCH_KEYS = ['fx', 'fy', 'fz', 'tx', 'ty', 'tz'] as const;
const MIN_INERTIA_VALUE = 0.001;

/** 实心球体惯量 I = (2/5) m r² */
export function solidSphereInertia(mass: number, radius: number): InertiaTensor {
  if (mass <= 0) throw new Error(`质量必须为正数，当前值: ${mass}`);
  if (radius <= 0) throw new Error(`半径必须为正数，当前值: ${radius}`);
  const i = (2 / 5) * mass * radius * radius;
  return { ixx: i, ixy: 0, ixz: 0, iyy: i, iyz: 0, izz: i };
}

/** 在已有 link 名列表中生成唯一后缀名 */
export function makeUniqueLinkName(existing: string[], prefix: string): string {
  let index = 0;
  let candidate = `${prefix}_${index}`;
  const taken = new Set(existing);
  while (taken.has(candidate)) {
    index += 1;
    candidate = `${prefix}_${index}`;
  }
  return candidate;
}

export function parseWrenchValues(
  fields: Partial<Record<(typeof WRENCH_KEYS)[number], number>>,
): Wrench6 {
  return WRENCH_KEYS.map((key) => {
    const value = fields[key];
    return Number.isFinite(value) ? (value as number) : 0;
  }) as Wrench6;
}

export function wrenchIsZero(wrench: Wrench6, eps = 1e-12): boolean {
  return wrench.every((v) => Math.abs(v) < eps);
}

/** 从 URDF 片段解析 link / joint 名（不含 robot 根也可） */
export function parseUrdfSnippet(snippetXml: string): ParsedUrdfSnippet {
  const wrapped = snippetXml.trim().startsWith('<robot')
    ? snippetXml
    : `<robot name="snippet">${snippetXml}</robot>`;
  const doc = new DOMParser().parseFromString(wrapped, 'application/xml');
  const parseError = doc.querySelector('parsererror');
  if (parseError) {
    throw new Error(`URDF 片段解析失败: ${parseError.textContent ?? 'unknown'}`);
  }
  const robot = doc.querySelector('robot');
  if (!robot) throw new Error('URDF 片段缺少 <robot> 或 link/joint 元素');

  const links: string[] = [];
  for (const link of robot.querySelectorAll(':scope > link')) {
    const name = link.getAttribute('name');
    if (name) links.push(name);
  }

  const joints: string[] = [];
  for (const joint of robot.querySelectorAll(':scope > joint')) {
    const name = joint.getAttribute('name');
    if (name) joints.push(name);
  }

  if (links.length === 0) {
    throw new Error('URDF 片段中未找到 <link>');
  }

  return { links, joints };
}

/** 推断片段根 link（非任何 joint 的 child） */
export function extractSnippetRootLink(snippetXml: string): string {
  const { links } = parseUrdfSnippet(snippetXml);
  const wrapped = snippetXml.trim().startsWith('<robot')
    ? snippetXml
    : `<robot name="snippet">${snippetXml}</robot>`;
  const doc = new DOMParser().parseFromString(wrapped, 'application/xml');
  const robot = doc.querySelector('robot')!;

  const childLinks = new Set<string>();
  for (const joint of robot.querySelectorAll(':scope > joint')) {
    const child = joint.querySelector(':scope > child')?.getAttribute('link');
    if (child) childLinks.add(child);
  }

  const roots = links.filter((name) => !childLinks.has(name));
  if (roots.length === 1) return roots[0]!;
  if (roots.length > 1) return roots[0]!;
  return links[0]!;
}

function insertBeforeRobotClose(urdfText: string, injection: string): string {
  const closeIdx = urdfText.lastIndexOf('</robot>');
  if (closeIdx < 0) throw new Error('URDF 缺少 </robot>');
  return urdfText.slice(0, closeIdx) + injection + urdfText.slice(closeIdx);
}

function formatInertia(inertia: InertiaTensor): string {
  const fmt = (value: number) => {
    const clamped = Math.abs(value) < MIN_INERTIA_VALUE && value !== 0 ? MIN_INERTIA_VALUE : value;
    return clamped.toFixed(9).replace(/\.?0+$/, '') || '0';
  };
  return formatInertiaTag({
    ixx: fmt(inertia.ixx),
    ixy: fmt(inertia.ixy),
    ixz: fmt(inertia.ixz),
    iyy: fmt(inertia.iyy),
    iyz: fmt(inertia.iyz),
    izz: fmt(inertia.izz),
  });
}

function buildSphereChildLinkXml(
  payloadLink: string,
  jointName: string,
  parentLink: string,
  mass: number,
  radius: number,
  inertia: InertiaTensor,
): string {
  return `
  <link name="${payloadLink}">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="${radius}"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><sphere radius="${radius}"/></geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="${mass}"/>
      ${formatInertia(inertia)}
    </inertial>
  </link>
  <joint name="${jointName}" type="fixed">
    <parent link="${parentLink}"/>
    <child link="${payloadLink}"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
`;
}

function assertLinkExists(urdfText: string, linkName: string): void {
  const links = parseLinkNames(urdfText);
  if (!links.includes(linkName)) {
    throw new Error(`未找到 link: ${linkName}`);
  }
}

/**
 * 在指定 link 上添加球体负载。
 * - child_link：追加 payload link + fixed joint + sphere geom
 * - modify_inertial：将负载质量/惯量叠加到已有 link 的 inertial
 */
export function appendSpherePayloadUrdf(
  urdfText: string,
  options: SpherePayloadOptions,
): string {
  return appendSpherePayloadWithRecord(urdfText, options).urdfText;
}

export interface AppendSpherePayloadResult {
  urdfText: string;
  record: PayloadRecord;
}

export function appendSpherePayloadWithRecord(
  urdfText: string,
  options: SpherePayloadOptions,
): AppendSpherePayloadResult {
  const { parentLink, mass, radius, mode = 'child_link' } = options;
  if (mass <= 0) throw new Error(`质量必须为正数，当前值: ${mass}`);
  if (radius <= 0) throw new Error(`半径必须为正数，当前值: ${radius}`);
  assertLinkExists(urdfText, parentLink);

  const inertia = solidSphereInertia(mass, radius);

  if (mode === 'modify_inertial') {
    const editor = new MassEditor(urdfText);
    const existing = editor.getLinkInertials().find((item) => item.linkName === parentLink);
    const baseMass = existing?.mass ?? 0;
    const baseInertia = existing?.inertia ?? { ixx: 0, iyy: 0, izz: 0, ixy: 0, ixz: 0, iyz: 0 };
    editor.setLinkMass(parentLink, baseMass + mass);
    editor.setLinkInertia(
      parentLink,
      baseInertia.ixx + inertia.ixx,
      baseInertia.iyy + inertia.iyy,
      baseInertia.izz + inertia.izz,
    );
    const record: PayloadRecord = {
      id: `${parentLink}_modify_${Date.now()}`,
      kind: 'modify_inertial',
      parentLink,
      mass,
      radius,
      originalInertial: { mass: baseMass, inertia: { ...baseInertia } },
    };
    return { urdfText: sanitizeUrdfForMujoco(editor.serialize()), record };
  }

  const existingLinks = parseLinkNames(urdfText);
  const suffix = options.suffix ?? makeUniqueLinkName(existingLinks, `${parentLink}_payload`);
  const payloadLink = makeUniqueLinkName(existingLinks, `${suffix}_sphere`);
  const jointName = makeUniqueLinkName(
    [...existingLinks, payloadLink],
    `${payloadLink}_fixed`,
  );

  const injection = buildSphereChildLinkXml(
    payloadLink,
    jointName,
    parentLink,
    mass,
    radius,
    inertia,
  );
  const record: PayloadRecord = {
    id: payloadLink,
    kind: 'child_link',
    parentLink,
    payloadLink,
    jointName,
    mass,
    radius,
  };
  const urdfTextOut = insertBeforeRobotClose(urdfText, injection);
  if (isUrdfDebugEnabled()) {
    logUrdfSnippet('appendSpherePayload', urdfTextOut);
  }
  return {
    urdfText: urdfTextOut,
    record,
  };
}

/** 从 URDF 文本推断已添加的球体负载 link（按文档顺序） */
export function listSpherePayloadLinks(urdfText: string): string[] {
  return parseLinkNames(urdfText).filter((name) => SPHERE_PAYLOAD_LINK_PATTERN.test(name));
}

function getSpherePayloadParentLink(doc: Document, payloadLink: string): string | null {
  const robot = doc.querySelector('robot');
  if (!robot) return null;
  for (const joint of robot.querySelectorAll(':scope > joint')) {
    const child = joint.querySelector(':scope > child')?.getAttribute('link');
    if (child !== payloadLink) continue;
    return joint.querySelector(':scope > parent')?.getAttribute('link') ?? null;
  }
  return null;
}

function parseSphereLinkMassRadius(
  linkEl: Element,
): { mass: number; radius: number } | null {
  const mass = Number(linkEl.querySelector(':scope > inertial > mass')?.getAttribute('value'));
  const radius = Number(
    linkEl.querySelector(':scope > visual geometry sphere')?.getAttribute('radius') ??
      linkEl.querySelector(':scope > collision geometry sphere')?.getAttribute('radius'),
  );
  if (!Number.isFinite(mass) || !Number.isFinite(radius) || mass <= 0 || radius <= 0) {
    return null;
  }
  return { mass, radius };
}

/** 汇总当前 URDF 中的球体负载（优先 payloadRecords，并补齐 URDF 中未记录的子 link） */
export function listSpherePayloadDisplayItems(
  urdfText: string,
  records: PayloadRecord[],
): SpherePayloadDisplayItem[] {
  const items: SpherePayloadDisplayItem[] = records.map((record) => ({
    id: record.id,
    kind: record.kind,
    parentLink: record.parentLink,
    payloadLink: record.payloadLink,
    mass: record.mass,
    radius: record.radius,
  }));

  const doc = parseUrdfDocument(urdfText);
  const robot = doc.querySelector('robot');
  if (!robot) return items;

  const knownPayloadLinks = new Set(
    items.filter((item) => item.payloadLink).map((item) => item.payloadLink!),
  );

  for (const linkName of listSpherePayloadLinks(urdfText)) {
    if (knownPayloadLinks.has(linkName)) continue;
    const linkEl = Array.from(robot.querySelectorAll(':scope > link')).find(
      (el) => el.getAttribute('name') === linkName,
    );
    if (!linkEl) continue;
    const parsed = parseSphereLinkMassRadius(linkEl);
    const parentLink = getSpherePayloadParentLink(doc, linkName);
    if (!parsed || !parentLink) continue;
    items.push({
      id: linkName,
      kind: 'child_link',
      parentLink,
      payloadLink: linkName,
      mass: parsed.mass,
      radius: parsed.radius,
    });
  }

  return items;
}

function parseUrdfDocument(urdfText: string): Document {
  const doc = new DOMParser().parseFromString(urdfText, 'application/xml');
  const parseError = doc.querySelector('parsererror');
  if (parseError) {
    throw new Error(`URDF 解析失败: ${parseError.textContent ?? 'unknown'}`);
  }
  return doc;
}

function serializeUrdfDocument(doc: Document): string {
  return new XMLSerializer().serializeToString(doc);
}

function findDirectChildJoint(doc: Document, childLink: string): Element | null {
  const robot = doc.querySelector('robot');
  if (!robot) return null;
  for (const joint of robot.querySelectorAll(':scope > joint')) {
    const child = joint.querySelector(':scope > child')?.getAttribute('link');
    if (child === childLink) return joint;
  }
  return null;
}

/**
 * 移除匹配 *_payload*_sphere* 的 link 及其直连 joint。
 * 不处理 modify_inertial（需配合 payloadRecords 还原）。
 */
export function removeSpherePayloads(urdfText: string, linkNames?: string[]): string {
  const targets = linkNames ?? listSpherePayloadLinks(urdfText);
  if (targets.length === 0) return urdfText;

  const doc = parseUrdfDocument(urdfText);
  const robot = doc.querySelector('robot');
  if (!robot) return urdfText;

  const removeSet = new Set(targets);
  for (const linkName of targets) {
    findDirectChildJoint(doc, linkName)?.remove();
    const linkEl = Array.from(robot.querySelectorAll(':scope > link')).find(
      (el) => el.getAttribute('name') === linkName,
    );
    linkEl?.remove();
  }

  for (const joint of [...robot.querySelectorAll(':scope > joint')]) {
    const child = joint.querySelector(':scope > child')?.getAttribute('link');
    if (child && removeSet.has(child)) joint.remove();
  }

  return sanitizeUrdfForMujoco(serializeUrdfDocument(doc));
}

/** 移除指定 parent link 上最后一个球体负载 */
export function removeLastSpherePayloadOnLink(urdfText: string, parentLink: string): string {
  const doc = parseUrdfDocument(urdfText);
  const robot = doc.querySelector('robot');
  if (!robot) throw new Error('URDF 缺少 <robot>');

  let targetLink: string | null = null;
  let targetJoint: Element | null = null;

  for (const joint of robot.querySelectorAll(':scope > joint')) {
    const parent = joint.querySelector(':scope > parent')?.getAttribute('link');
    const child = joint.querySelector(':scope > child')?.getAttribute('link');
    if (parent !== parentLink || !child || !SPHERE_PAYLOAD_LINK_PATTERN.test(child)) continue;
    targetLink = child;
    targetJoint = joint;
  }

  if (!targetLink || !targetJoint) {
    throw new Error(`在 ${parentLink} 上未找到球体负载`);
  }

  targetJoint.remove();
  Array.from(robot.querySelectorAll(':scope > link'))
    .find((el) => el.getAttribute('name') === targetLink)
    ?.remove();

  return sanitizeUrdfForMujoco(serializeUrdfDocument(doc));
}

/** 使用 session 记录还原 modify_inertial 叠加 */
export function revertModifyInertialPayload(
  urdfText: string,
  record: PayloadRecord,
): string {
  if (record.kind !== 'modify_inertial' || !record.originalInertial) {
    throw new Error('无效的 modify_inertial 记录');
  }
  const editor = new MassEditor(urdfText);
  const { mass, inertia } = record.originalInertial;
  editor.setLinkMass(record.parentLink, Math.max(mass, 1e-9));
  editor.setLinkInertia(
    record.parentLink,
    Math.max(inertia.ixx, MIN_INERTIA_VALUE),
    Math.max(inertia.iyy, MIN_INERTIA_VALUE),
    Math.max(inertia.izz, MIN_INERTIA_VALUE),
  );
  return sanitizeUrdfForMujoco(editor.serialize());
}

function prefixSnippetNames(snippetXml: string, prefix: string): string {
  const wrapped = snippetXml.trim().startsWith('<robot')
    ? snippetXml
    : `<robot name="snippet">${snippetXml}</robot>`;
  const doc = new DOMParser().parseFromString(wrapped, 'application/xml');
  const robot = doc.querySelector('robot');
  if (!robot) throw new Error('URDF 片段缺少 <robot> 或 link/joint 元素');

  for (const link of robot.querySelectorAll(':scope > link')) {
    const name = link.getAttribute('name');
    if (name) link.setAttribute('name', `${prefix}${name}`);
  }
  for (const joint of robot.querySelectorAll(':scope > joint')) {
    const name = joint.getAttribute('name');
    if (name) joint.setAttribute('name', `${prefix}${name}`);
    const parent = joint.querySelector(':scope > parent');
    const child = joint.querySelector(':scope > child');
    const parentName = parent?.getAttribute('link');
    const childName = child?.getAttribute('link');
    if (parentName) parent!.setAttribute('link', `${prefix}${parentName}`);
    if (childName) child!.setAttribute('link', `${prefix}${childName}`);
  }

  const parts: string[] = [];
  for (const node of robot.childNodes) {
    if (node.nodeType === Node.ELEMENT_NODE) {
      parts.push(new XMLSerializer().serializeToString(node));
    }
  }
  return parts.join('\n');
}

/**
 * 将 URDF 片段 fixed 拼接到 parent link（MVP：单根 link 或 link 树）。
 * 自动为片段内名称加 prefix，并注入 parent→root 的 fixed joint。
 */
export function attachUrdfSnippet(
  urdfText: string,
  options: AttachUrdfSnippetOptions,
): string {
  const { parentLink, snippetXml } = options;
  assertLinkExists(urdfText, parentLink);

  const rootLinkRaw = extractSnippetRootLink(snippetXml);
  const prefix = options.prefix ?? `attached_${parentLink}_`;
  const prefixedSnippet = prefixSnippetNames(snippetXml, prefix);
  const rootLink = `${prefix}${rootLinkRaw}`;

  const existingLinks = parseLinkNames(urdfText);
  const jointName =
    options.jointName ??
    makeUniqueLinkName(existingLinks, `${rootLink}_to_${parentLink}`);

  const attachJoint = `
  <joint name="${jointName}" type="fixed">
    <parent link="${parentLink}"/>
    <child link="${rootLink}"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
`;

  return sanitizeUrdfForMujoco(
    insertBeforeRobotClose(urdfText, `\n${prefixedSnippet}\n${attachJoint}`),
  );
}
