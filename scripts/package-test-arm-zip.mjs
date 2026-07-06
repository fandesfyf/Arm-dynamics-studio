#!/usr/bin/env node
/**
 * 生成 test_arm 测试包：urdf/ + meshes/ 目录结构，并打包为 public/robots/test_arm.zip
 */
import { mkdirSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import JSZip from 'jszip';

const __dirname = dirname(fileURLToPath(import.meta.url));
const ROOT = join(__dirname, '..');
const OUT_DIR = join(ROOT, 'public/robots/test_arm');
const ZIP_PATH = join(ROOT, 'public/robots/test_arm.zip');

function facet(normal, a, b, c) {
  const [nx, ny, nz] = normal;
  const fmt = (v) => v.map((x) => x.toFixed(6)).join(' ');
  return `  facet normal ${nx} ${ny} ${nz}
    outer loop
      vertex ${fmt(a)}
      vertex ${fmt(b)}
      vertex ${fmt(c)}
    endloop
  endfacet`;
}

function stlSolid(name, facets) {
  return `solid ${name}\n${facets.join('\n')}\nendsolid ${name}\n`;
}

/** 轴对齐盒子，中心在 (cx,cy,cz) */
function boxStl(name, sx, sy, sz, cx = 0, cy = 0, cz = 0) {
  const hx = sx / 2;
  const hy = sy / 2;
  const hz = sz / 2;
  const v = (x, y, z) => [cx + x, cy + y, cz + z];
  const p = {
    nnn: v(-hx, -hy, -hz),
    pnn: v(hx, -hy, -hz),
    ppn: v(hx, hy, -hz),
    npn: v(-hx, hy, -hz),
    nnp: v(-hx, -hy, hz),
    pnp: v(hx, -hy, hz),
    ppp: v(hx, hy, hz),
    npp: v(-hx, hy, hz),
  };
  const facets = [
    facet([0, 0, -1], p.nnn, p.pnn, p.ppn),
    facet([0, 0, -1], p.nnn, p.ppn, p.npn),
    facet([0, 0, 1], p.nnp, p.ppp, p.pnp),
    facet([0, 0, 1], p.nnp, p.npp, p.ppp),
    facet([0, -1, 0], p.nnn, p.pnp, p.pnn),
    facet([0, -1, 0], p.nnn, p.nnp, p.pnp),
    facet([0, 1, 0], p.npn, p.ppn, p.ppp),
    facet([0, 1, 0], p.npn, p.ppp, p.npp),
    facet([-1, 0, 0], p.nnn, p.npn, p.npp),
    facet([-1, 0, 0], p.nnn, p.npp, p.nnp),
    facet([1, 0, 0], p.pnn, p.pnp, p.ppn),
    facet([1, 0, 0], p.pnn, p.ppp, p.pnp),
  ];
  return stlSolid(name, facets);
}

/** 低分段圆柱（沿 Z），中心在 (cx,cy,cz) */
function cylinderStl(name, radius, height, segments = 8, cx = 0, cy = 0, cz = 0) {
  const hz = height / 2;
  const facets = [];
  const bottom = [];
  const top = [];
  for (let i = 0; i < segments; i++) {
    const a0 = (i / segments) * Math.PI * 2;
    const a1 = ((i + 1) / segments) * Math.PI * 2;
    const x0 = cx + radius * Math.cos(a0);
    const y0 = cy + radius * Math.sin(a0);
    const x1 = cx + radius * Math.cos(a1);
    const y1 = cy + radius * Math.sin(a1);
    bottom.push([x0, y0, cz - hz]);
    top.push([x0, y0, cz + hz]);
    const bc = [cx, cy, cz - hz];
    const tc = [cx, cy, cz + hz];
    facets.push(facet([0, 0, -1], bc, [x1, y1, cz - hz], [x0, y0, cz - hz]));
    facets.push(facet([0, 0, 1], tc, [x0, y0, cz + hz], [x1, y1, cz + hz]));
    facets.push(facet([Math.cos((a0 + a1) / 2), Math.sin((a0 + a1) / 2), 0], [x0, y0, cz - hz], [x0, y0, cz + hz], [x1, y1, cz + hz]));
    facets.push(facet([Math.cos((a0 + a1) / 2), Math.sin((a0 + a1) / 2), 0], [x0, y0, cz - hz], [x1, y1, cz + hz], [x1, y1, cz - hz]));
  }
  return stlSolid(name, facets);
}

function sphereStl(name, radius, segments = 6) {
  return boxStl(name, radius * 2, radius * 2, radius * 2);
}

const MESHES = [
  { file: 'base_link.STL', content: cylinderStl('base_link', 0.05, 0.1) },
  { file: 'link1.STL', content: cylinderStl('link1', 0.04, 0.2, 8, 0, 0, 0.1) },
  { file: 'link2.STL', content: cylinderStl('link2', 0.035, 0.18, 8, 0, 0, 0.09) },
  { file: 'link3.STL', content: cylinderStl('link3', 0.03, 0.15, 8, 0, 0, 0.075) },
  { file: 'link4.STL', content: cylinderStl('link4', 0.025, 0.1, 8, 0, 0, 0.05) },
  { file: 'link5.STL', content: boxStl('link5', 0.08, 0.08, 0.03, 0, 0, 0.015) },
  { file: 'ee_link.STL', content: sphereStl('ee_link', 0.015) },
];

const URDF = `<?xml version="1.0" encoding="utf-8"?>
<robot name="test_arm">
  <link name="world"/>
  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    <visual>
      <geometry><mesh filename="../meshes/base_link.STL"/></geometry>
      <material name="gray"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="0.05" length="0.1"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>
  <link name="link1">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/link1.STL"/></geometry>
      <material name="blue"><color rgba="0.2 0.2 0.8 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry><cylinder radius="0.04" length="0.2"/></geometry>
    </collision>
    <inertial>
      <mass value="0.5"/><origin xyz="0 0 0.1"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>
  <link name="link2">
    <visual>
      <origin xyz="0 0 0.09" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/link2.STL"/></geometry>
      <material name="green"><color rgba="0.2 0.8 0.2 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0.09" rpy="0 0 0"/>
      <geometry><cylinder radius="0.035" length="0.18"/></geometry>
    </collision>
    <inertial>
      <mass value="0.4"/><origin xyz="0 0 0.09"/>
      <inertia ixx="0.0015" ixy="0" ixz="0" iyy="0.0015" iyz="0" izz="0.0004"/>
    </inertial>
  </link>

  <joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0.18" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="80" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>
  <link name="link3">
    <visual>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/link3.STL"/></geometry>
      <material name="red"><color rgba="0.8 0.2 0.2 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
      <geometry><cylinder radius="0.03" length="0.15"/></geometry>
    </collision>
    <inertial>
      <mass value="0.3"/><origin xyz="0 0 0.075"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0003"/>
    </inertial>
  </link>

  <joint name="joint4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="50" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>
  <link name="link4">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/link4.STL"/></geometry>
      <material name="yellow"><color rgba="0.8 0.8 0.2 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry><cylinder radius="0.025" length="0.1"/></geometry>
    </collision>
    <inertial>
      <mass value="0.2"/><origin xyz="0 0 0.05"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0002"/>
    </inertial>
  </link>

  <joint name="joint5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="30" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>
  <link name="link5">
    <visual>
      <origin xyz="0 0 0.015" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/link5.STL"/></geometry>
      <material name="purple"><color rgba="0.6 0.2 0.8 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 0.015" rpy="0 0 0"/>
      <geometry><box size="0.08 0.08 0.03"/></geometry>
    </collision>
    <inertial>
      <mass value="0.15"/><origin xyz="0 0 0.015"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="ee_joint" type="fixed">
    <parent link="link5"/><child link="ee_link"/>
    <origin xyz="0 0 0.03" rpy="0 0 0"/>
  </joint>
  <link name="ee_link">
    <visual>
      <geometry><mesh filename="../meshes/ee_link.STL"/></geometry>
      <material name="orange"><color rgba="1.0 0.5 0.0 1"/></material>
    </visual>
    <collision><geometry><sphere radius="0.015"/></geometry></collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
  </link>

  <transmission name="trans1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint1"><hardwareInterface>EffortJointInterface</hardwareInterface></joint>
    <actuator name="motor1"><mechanicalReduction>1</mechanicalReduction></actuator>
  </transmission>
  <transmission name="trans2">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint2"><hardwareInterface>EffortJointInterface</hardwareInterface></joint>
    <actuator name="motor2"><mechanicalReduction>1</mechanicalReduction></actuator>
  </transmission>
  <transmission name="trans3">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint3"><hardwareInterface>EffortJointInterface</hardwareInterface></joint>
    <actuator name="motor3"><mechanicalReduction>1</mechanicalReduction></actuator>
  </transmission>
  <transmission name="trans4">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint4"><hardwareInterface>EffortJointInterface</hardwareInterface></joint>
    <actuator name="motor4"><mechanicalReduction>1</mechanicalReduction></actuator>
  </transmission>
  <transmission name="trans5">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint5"><hardwareInterface>EffortJointInterface</hardwareInterface></joint>
    <actuator name="motor5"><mechanicalReduction>1</mechanicalReduction></actuator>
  </transmission>
</robot>
`;

async function main() {
  const meshDir = join(OUT_DIR, 'meshes');
  const urdfDir = join(OUT_DIR, 'urdf');
  mkdirSync(meshDir, { recursive: true });
  mkdirSync(urdfDir, { recursive: true });

  for (const { file, content } of MESHES) {
    writeFileSync(join(meshDir, file), content);
  }
  writeFileSync(join(urdfDir, 'test_arm.urdf'), URDF);

  const zip = new JSZip();
  const prefix = 'test_arm';
  zip.file(`${prefix}/urdf/test_arm.urdf`, URDF);
  for (const { file, content } of MESHES) {
    zip.file(`${prefix}/meshes/${file}`, content);
  }
  const buf = await zip.generateAsync({ type: 'nodebuffer', compression: 'DEFLATE' });
  writeFileSync(ZIP_PATH, buf);

  console.log(`Wrote ${OUT_DIR}`);
  console.log(`Wrote ${ZIP_PATH} (${buf.length} bytes)`);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
