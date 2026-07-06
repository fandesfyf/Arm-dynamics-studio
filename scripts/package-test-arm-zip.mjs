#!/usr/bin/env node
/**
 * 从 kuavo biped_s56 的 biped_v3_arm.urdf + meshes 生成 test_arm 部署包。
 *
 * 输出：
 *   public/robots/test_arm/urdf/test_arm.urdf
 *   public/robots/test_arm/meshes/*.STL
 *   public/robots/test_arm.zip
 *   public/robots/test_arm.urdf（扁平回退，mesh 路径为 test_arm/meshes/）
 */
import { copyFileSync, mkdirSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { basename, dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import JSZip from 'jszip';

const __dirname = dirname(fileURLToPath(import.meta.url));
const ROOT = join(__dirname, '..');
const SOURCE_ROOT =
  process.env.BIPED_S56_ROOT ??
  '/home/fandes/workspace/kuavo-ros-controldev/src/kuavo_assets/models/biped_s56';
const SOURCE_URDF = join(SOURCE_ROOT, 'urdf/drake/biped_v3_arm.urdf');
const SOURCE_MESH_DIR = join(SOURCE_ROOT, 'meshes');
const OUT_DIR = join(ROOT, 'public/robots/test_arm');
const ZIP_PATH = join(ROOT, 'public/robots/test_arm.zip');
const FLAT_URDF_PATH = join(ROOT, 'public/robots/test_arm.urdf');

function stripXmlDeclaration(urdf) {
  return urdf.replace(/^\uFEFF?<\?xml\b[^?]*\?>\s*/i, '');
}

function transformUrdfForZip(raw) {
  let urdf = stripXmlDeclaration(raw);
  urdf = urdf.replace(/<robot\s+name="[^"]*"/, '<robot name="test_arm"');
  urdf = urdf.replace(/<mujoco>[\s\S]*?<\/mujoco>\s*/gi, '');
  urdf = urdf.replace(/filename="\.\.\/\.\.\/meshes\//g, 'filename="../meshes/');
  return urdf;
}

function transformUrdfForFlat(zipUrdf) {
  return zipUrdf.replace(/filename="\.\.\/meshes\//g, 'filename="test_arm/meshes/');
}

function extractMeshRefs(urdf) {
  const refs = new Set();
  const re = /filename="([^"]+\.(?:stl|STL|dae|obj|ply))"/gi;
  let m;
  while ((m = re.exec(urdf)) !== null) {
    refs.add(basename(m[1]));
  }
  return [...refs].sort();
}

async function main() {
  const raw = readFileSync(SOURCE_URDF, 'utf-8');
  const zipUrdf = transformUrdfForZip(raw);
  const flatUrdf = transformUrdfForFlat(zipUrdf);
  const meshNames = extractMeshRefs(zipUrdf);

  if (meshNames.length === 0) {
    throw new Error('URDF 中未找到 mesh 引用');
  }

  rmSync(OUT_DIR, { recursive: true, force: true });
  const meshDir = join(OUT_DIR, 'meshes');
  const urdfDir = join(OUT_DIR, 'urdf');
  mkdirSync(meshDir, { recursive: true });
  mkdirSync(urdfDir, { recursive: true });

  for (const name of meshNames) {
    const src = join(SOURCE_MESH_DIR, name);
    copyFileSync(src, join(meshDir, name));
  }

  writeFileSync(join(urdfDir, 'test_arm.urdf'), zipUrdf);
  writeFileSync(FLAT_URDF_PATH, flatUrdf);

  const zip = new JSZip();
  const prefix = 'test_arm';
  zip.file(`${prefix}/urdf/test_arm.urdf`, zipUrdf);
  for (const name of meshNames) {
    zip.file(`${prefix}/meshes/${name}`, readFileSync(join(meshDir, name)));
  }
  const buf = await zip.generateAsync({ type: 'nodebuffer', compression: 'DEFLATE' });
  writeFileSync(ZIP_PATH, buf);

  console.log(`Source: ${SOURCE_URDF}`);
  console.log(`Meshes: ${meshNames.join(', ')}`);
  console.log(`Wrote ${OUT_DIR}`);
  console.log(`Wrote ${ZIP_PATH} (${(buf.length / 1024 / 1024).toFixed(2)} MiB)`);
  console.log(`Wrote ${FLAT_URDF_PATH}`);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
