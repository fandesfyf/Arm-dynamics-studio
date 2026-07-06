import { describe, expect, it } from 'vitest';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import {
  detectBaseLink,
  detectEndEffectorLink,
  ensureFixedBase,
  listEndEffectorLinkCandidates,
  resolveEndEffectorJointName,
} from './urdf-base-fixture';

const testArm = readFileSync(
  resolve(__dirname, '../fixtures/simple_test_arm.urdf'),
  'utf8',
);

describe('urdf-base-fixture', () => {
  it('detects base_link from test_arm', () => {
    expect(detectBaseLink(testArm)).toBe('base_link');
  });

  it('does not duplicate world joint when already fixed', () => {
    const result = ensureFixedBase(testArm);
    expect(result.changed).toBe(false);
    expect(result.baseLink).toBe('base_link');
  });

  it('injects world fixed joint for root-only urdf', () => {
    const minimal = `<?xml version="1.0"?>
<robot name="mini">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>`;
    const result = ensureFixedBase(minimal);
    expect(result.changed).toBe(true);
    expect(result.urdfText).toContain('world_to_base_link');
    expect(result.baseLink).toBe('base_link');
  });

  it('detects torso as base when present without base_link', () => {
    const urdf = `<?xml version="1.0"?>
<robot name="x">
  <link name="torso"/><link name="arm"/>
  <joint name="j" type="revolute">
    <parent link="torso"/><child link="arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>`;
    expect(detectBaseLink(urdf)).toBe('torso');
  });

  it('detects zarm end effector link', () => {
    const upper = readFileSync(
      resolve(__dirname, '../../public/robots/biped_s70_upper_body.urdf'),
      'utf8',
    );
    expect(detectEndEffectorLink(upper)).toBe('zarm_l7_end_effector');
  });

  it('lists end effector candidates with hands first', () => {
    const upper = readFileSync(
      resolve(__dirname, '../../public/robots/biped_s70_upper_body.urdf'),
      'utf8',
    );
    const candidates = listEndEffectorLinkCandidates(upper);
    expect(candidates[0]).toBe('zarm_l7_end_effector');
    expect(candidates).toContain('zarm_r7_end_effector');
    expect(candidates.indexOf('zarm_l7_end_effector')).toBeLessThan(
      candidates.indexOf('base_link'),
    );
  });

  it('lists all links for generic robots with custom names', () => {
    const urdf = `<?xml version="1.0"?>
<robot name="custom_arm">
  <link name="base_link"/>
  <link name="shoulder"/>
  <link name="my_custom_tip"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="shoulder"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="shoulder"/><child link="my_custom_tip"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>`;
    const candidates = listEndEffectorLinkCandidates(urdf);
    expect(candidates).toEqual(['my_custom_tip', 'shoulder', 'base_link']);
    expect(detectEndEffectorLink(urdf)).toBe('my_custom_tip');
  });

  it('resolves end effector link to parent revolute joint', () => {
    const upper = readFileSync(
      resolve(__dirname, '../../public/robots/biped_s70_upper_body.urdf'),
      'utf8',
    );
    const jointNames: string[] = [];
    for (const block of upper.match(/<joint\b[\s\S]*?<\/joint>/g) ?? []) {
      if (!/type="revolute"/.test(block)) continue;
      const name = block.match(/name="([^"]+)"/)?.[1];
      if (name) jointNames.push(name);
    }
    expect(resolveEndEffectorJointName(upper, jointNames, 'zarm_l7_end_effector')).toBe(
      'zarm_l7_joint',
    );
  });
});
