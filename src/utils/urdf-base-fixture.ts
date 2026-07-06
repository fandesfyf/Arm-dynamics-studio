import { sanitizeUrdfForMujoco } from './urdf-sanitize';

/** 固定基座候选 link（按优先级） */
export const BASE_LINK_CANDIDATES = [
  'base_link',
  'baselink',
  'base',
  'torso',
  'torso_link',
  'waist_yaw_link',
  'waist_yaw',
  'pelvis',
  'trunk',
  'chest',
] as const;

export interface UrdfJointRef {
  name: string;
  type: string;
  parent: string;
  child: string;
}

export interface UrdfBaseFixtureResult {
  urdfText: string;
  baseLink: string;
  endEffectorLink: string | null;
  changed: boolean;
}

export function parseJointBlocks(urdfText: string): UrdfJointRef[] {
  const blocks = urdfText.match(/<joint\b[\s\S]*?<\/joint>/g) ?? [];
  const joints: UrdfJointRef[] = [];
  for (const block of blocks) {
    const name = block.match(/name="([^"]+)"/)?.[1];
    const type = block.match(/type="([^"]+)"/)?.[1] ?? 'fixed';
    const parent = block.match(/<parent\s+link="([^"]+)"/)?.[1];
    const child = block.match(/<child\s+link="([^"]+)"/)?.[1];
    if (!name || !parent || !child) continue;
    joints.push({ name, type, parent, child });
  }
  return joints;
}

export function parseLinkNames(urdfText: string): string[] {
  const names: string[] = [];
  const re = /<link\s+name="([^"]+)"/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(urdfText)) !== null) {
    names.push(m[1]);
  }
  return names;
}

/** 从 URDF 推断固定基座 link */
export function detectBaseLink(urdfText: string): string {
  const links = new Set(parseLinkNames(urdfText));
  const joints = parseJointBlocks(urdfText);

  const childCount = new Map<string, number>();
  for (const link of links) childCount.set(link, 0);
  for (const joint of joints) {
    childCount.set(joint.child, (childCount.get(joint.child) ?? 0) + 1);
  }

  const roots = [...links].filter((link) => (childCount.get(link) ?? 0) === 0);
  const lowerRoots = roots.map((r) => r.toLowerCase());

  for (const candidate of BASE_LINK_CANDIDATES) {
    const idx = lowerRoots.indexOf(candidate);
    if (idx >= 0) return roots[idx]!;
    const exact = roots.find((r) => r.toLowerCase() === candidate);
    if (exact) return exact;
    const fuzzy = roots.find((r) => r.toLowerCase().includes(candidate));
    if (fuzzy) return fuzzy;
  }

  const nonWorld = roots.filter((r) => r !== 'world');
  if (nonWorld.length > 0) return nonWorld[0]!;

  return 'base_link';
}

/** 运动链叶节点（不作为任何 joint 的 parent） */
export function collectLeafLinks(urdfText: string): string[] {
  const links = parseLinkNames(urdfText).filter((name) => name !== 'world');
  const joints = parseJointBlocks(urdfText);
  const parents = new Set(joints.map((j) => j.parent));
  return links.filter((name) => !parents.has(name));
}

function scoreEndEffectorLink(name: string, isLeaf: boolean): number {
  let score = 0;
  if (isLeaf) score += 50;
  if (/end_effector$/i.test(name)) score += 100;
  if (name === 'ee_link' || /^ee_/i.test(name)) score += 80;
  if (/hand|gripper|tool|tip|wrist|palm|finger/i.test(name)) score += 60;
  if (/link$/i.test(name) && !/base/i.test(name)) score += 10;
  if (/base|torso|pelvis|trunk|waist|root|world/i.test(name)) score -= 40;
  return score;
}

/** 末端 link 下拉选项：列出 URDF 中全部 link（叶节点与常见末端命名优先排序） */
export function listEndEffectorLinkCandidates(urdfText: string): string[] {
  const links = parseLinkNames(urdfText).filter((name) => name !== 'world');
  if (links.length === 0) return [];

  const leafSet = new Set(collectLeafLinks(urdfText));

  return links
    .map((name) => ({
      name,
      score: scoreEndEffectorLink(name, leafSet.has(name)),
    }))
    .sort((a, b) => b.score - a.score || a.name.localeCompare(b.name))
    .map((item) => item.name);
}

/** 推断默认末端 link（在 listEndEffectorLinkCandidates 排序后的首项） */
export function detectEndEffectorLink(urdfText: string): string | null {
  return listEndEffectorLinkCandidates(urdfText)[0] ?? null;
}

/**
 * 将末端 link 名解析为活动关节名（沿 fixed 关节向上追溯）。
 * 若传入的已是关节名则直接返回。
 */
export function resolveEndEffectorJointName(
  urdfText: string,
  jointNames: string[],
  endEffectorLinkOrJoint?: string,
): string | null {
  if (!endEffectorLinkOrJoint) return null;
  if (jointNames.includes(endEffectorLinkOrJoint)) {
    return endEffectorLinkOrJoint;
  }

  const blocks = urdfText.match(/<joint\b[\s\S]*?<\/joint>/g) ?? [];
  for (const block of blocks) {
    const child = block.match(/<child\s+link="([^"]+)"/)?.[1];
    if (child !== endEffectorLinkOrJoint) continue;

    const jointName = block.match(/name="([^"]+)"/)?.[1];
    if (jointName && jointNames.includes(jointName)) {
      return jointName;
    }

    const jointType = block.match(/type="([^"]+)"/)?.[1] ?? 'fixed';
    const parentLink = block.match(/<parent\s+link="([^"]+)"/)?.[1];
    if (parentLink && jointType === 'fixed') {
      const upstream = resolveEndEffectorJointName(urdfText, jointNames, parentLink);
      if (upstream) return upstream;
    }
  }

  return null;
}

function hasWorldFixedToBase(urdfText: string, baseLink: string): boolean {
  const joints = parseJointBlocks(urdfText);
  return joints.some(
    (j) =>
      j.type === 'fixed' &&
      j.parent === 'world' &&
      j.child === baseLink,
  );
}

/**
 * 若 URDF 尚无 world 固定基座，则注入 world + fixed joint。
 * 若根 link 不是目标基座，会在原树上追加 world→base 固定关节（MuJoCo 支持多根时取 world 为根）。
 */
export function ensureFixedBase(
  urdfText: string,
  preferredBaseLink?: string,
): UrdfBaseFixtureResult {
  const baseLink = preferredBaseLink || detectBaseLink(urdfText);
  const endEffectorLink = detectEndEffectorLink(urdfText);

  if (hasWorldFixedToBase(urdfText, baseLink)) {
    return { urdfText, baseLink, endEffectorLink, changed: false };
  }

  if (urdfText.includes('<link name="world"')) {
    const joints = parseJointBlocks(urdfText);
    const worldChild = joints.find((j) => j.parent === 'world')?.child;
    if (worldChild) {
      return { urdfText, baseLink: worldChild, endEffectorLink, changed: false };
    }
  }

  const injection = `
  <link name="world"/>
  <joint name="world_to_${baseLink}" type="fixed">
    <parent link="world"/>
    <child link="${baseLink}"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
`;

  const robotOpen = urdfText.match(/<robot\b[^>]*>/);
  if (!robotOpen) {
    return { urdfText, baseLink, endEffectorLink, changed: false };
  }

  const insertAt = robotOpen.index! + robotOpen[0].length;
  const patched =
    urdfText.slice(0, insertAt) + injection + urdfText.slice(insertAt);

  return {
    urdfText: sanitizeUrdfForMujoco(patched),
    baseLink,
    endEffectorLink,
    changed: true,
  };
}
