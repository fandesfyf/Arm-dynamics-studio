#!/usr/bin/env node
/**
 * 从 biped_s70 全量 URDF 提取上肢（含躯干链、双臂），输出到源资产目录与 web/public。
 */
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

const SOURCE_URDF =
  '/home/fandes/workspace/mimic_X1/source/whole_body_tracking/whole_body_tracking/assets/biped_s70/urdf/biped_s70.urdf';

const OUTPUT_PATHS = [
  '/home/fandes/workspace/mimic_X1/source/whole_body_tracking/whole_body_tracking/assets/biped_s70/urdf/biped_s70_upper_body.urdf',
  path.resolve(__dirname, '../public/robots/biped_s70_upper_body.urdf'),
];

const EXCLUDE_LINK_RE =
  /^(leg_[lr]\d|dummy_link|l_knee_|l_[lr]_bar|r_knee_|r_[lr]_bar|[lr]{1,2}_foot_|r_foot_|head_|zhead_)/;

const EXCLUDE_JOINT_RE =
  /^(leg_[lr]|l_knee_|l_[lr]_bar|r_knee_|r_[lr]_bar|[lr]{1,2}_foot_|r_foot_|zhead_|head_)/;

function extractBlocks(xml, tag) {
  const re = new RegExp(`<${tag}\\b[\\s\\S]*?<\\/${tag}>`, 'g');
  return xml.match(re) ?? [];
}

function attr(block, name) {
  const m = block.match(new RegExp(`${name}="([^"]*)"`));
  return m?.[1] ?? null;
}

function shouldKeepLink(name) {
  if (!name) return false;
  if (name === 'base_link') return true;
  if (EXCLUDE_LINK_RE.test(name)) return false;
  if (
    name.startsWith('waist_') ||
    name === 'torso' ||
    name.endsWith('_arm_base') ||
    name.startsWith('zarm_') ||
    name.startsWith('L_foream_') ||
    name.startsWith('R_foream_')
  ) {
    return true;
  }
  return false;
}

function shouldKeepJoint(name, parent, child) {
  if (!name || !parent || !child) return false;
  if (EXCLUDE_JOINT_RE.test(name)) return false;
  return shouldKeepLink(parent) && shouldKeepLink(child);
}

function extractUpperBodyUrdf(sourceXml) {
  const robotNameMatch = sourceXml.match(/<robot\s+name="([^"]*)"/);
  const robotName = robotNameMatch?.[1] ?? 'biped_s70';

  const keptLinks = new Set(['base_link']);
  for (const block of extractBlocks(sourceXml, 'link')) {
    const name = attr(block, 'name');
    if (shouldKeepLink(name)) keptLinks.add(name);
  }

  const keptJoints = [];
  for (const block of extractBlocks(sourceXml, 'joint')) {
    const name = attr(block, 'name');
    const parentLink = block.match(/<parent\s+link="([^"]+)"/)?.[1];
    const childLink = block.match(/<child\s+link="([^"]+)"/)?.[1];
    if (!shouldKeepJoint(name, parentLink, childLink)) continue;
    keptLinks.add(parentLink);
    keptLinks.add(childLink);
    keptJoints.push(block);
  }

  const linkBlocks = extractBlocks(sourceXml, 'link').filter((block) => {
    const name = attr(block, 'name');
    return name && keptLinks.has(name);
  });

  const header = `<?xml version="1.0"?>
<robot name="${robotName}_upper_body">
  <!-- 由 scripts/extract-upper-body-urdf.mjs 从 biped_s70.urdf 自动生成：仅保留 base_link、躯干与双臂 -->
  <link name="world"/>
  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
`;

  const footer = '\n</robot>\n';
  return header + linkBlocks.join('\n\n') + '\n\n' + keptJoints.join('\n\n') + footer;
}

function main() {
  const sourceXml = fs.readFileSync(SOURCE_URDF, 'utf8');
  const out = extractUpperBodyUrdf(sourceXml);
  for (const outPath of OUTPUT_PATHS) {
    fs.mkdirSync(path.dirname(outPath), { recursive: true });
    fs.writeFileSync(outPath, out, 'utf8');
    console.log('wrote', outPath);
  }
}

main();
