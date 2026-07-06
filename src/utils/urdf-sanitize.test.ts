/**
 * @vitest-environment happy-dom
 */
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import { appendSpherePayloadUrdf } from '../core/payload-editor';
import { sanitizeUrdfForMujoco, validateUrdfInertiaForMujoco, finalizeUrdfForMujoco } from './urdf-sanitize';

const __dirname = dirname(fileURLToPath(import.meta.url));
const BIPED_URDF = readFileSync(
  join(__dirname, '../../public/robots/biped_s70_upper_body.urdf'),
  'utf-8',
);

describe('sanitizeUrdfForMujoco', () => {
  it('bumps zero diagonal inertia on biped torso', () => {
    const sanitized = sanitizeUrdfForMujoco(BIPED_URDF);
    const torsoMatch = sanitized.match(
      /<link name="torso"[\s\S]*?<inertia\s([^>]*)(?:\/>|>)/,
    );
    expect(torsoMatch).not.toBeNull();
    const attrs = torsoMatch![1]!;
    expect(attrs).toMatch(/\bixx="0\.001"/);
    expect(attrs).toMatch(/\biyy="0\.001"/);
    expect(attrs).toMatch(/\bizz="0\.001"/);
  });

  it('fills empty inertia element attributes', () => {
    const urdf = `<robot name="t">
  <link name="arm">
    <inertial>
      <mass value="1"/>
      <inertia/>
    </inertial>
  </link>
</robot>`;
    const sanitized = sanitizeUrdfForMujoco(urdf);
    expect(sanitized).toMatch(/\bixx="0\.001"/);
    expect(sanitized).not.toMatch(/<inertia\s*\/>/);
  });

  it('creates inertia when inertial has mass only', () => {
    const urdf = `<robot name="t">
  <link name="arm">
    <inertial><mass value="1"/></inertial>
  </link>
</robot>`;
    const sanitized = sanitizeUrdfForMujoco(urdf);
    expect(sanitized).toMatch(/<inertia[^>]*ixx="0\.001"/);
  });

  it('replaces empty-string inertia attributes', () => {
    const urdf = `<robot name="t">
  <link name="arm">
    <inertial>
      <mass value="1"/>
      <inertia ixx="" ixy="" ixz="" iyy="" iyz="" izz=""/>
    </inertial>
  </link>
</robot>`;
    const sanitized = sanitizeUrdfForMujoco(urdf);
    expect(sanitized).toMatch(/\bixx="0\.001"/);
    expect(sanitized).not.toMatch(/\bixx=""/);
    expect(sanitized).not.toMatch(/\biyy=""/);
    expect(sanitized).not.toMatch(/\bizz=""/);
  });

  it('strips xmlns from XMLSerializer output', () => {
    const serialized = `<robot xmlns="http://www.ros.org" name="t">
  <link xmlns:ns="http://example.com" name="arm">
    <inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
</robot>`;
    const sanitized = sanitizeUrdfForMujoco(serialized);
    expect(sanitized).not.toMatch(/xmlns/);
    expect(sanitized).toMatch(/<link name="arm">/);
  });

  it('strips prefixed inertia attributes from DOM round-trip', () => {
    const serialized = `<robot name="t">
  <link name="arm">
    <inertial>
      <mass value="1"/>
      <inertia ns:ixx="0.01" ns:ixy="0" ns:ixz="0" ns:iyy="0.01" ns:iyz="0" ns:izz="0.01"/>
    </inertial>
  </link>
</robot>`;
    const sanitized = sanitizeUrdfForMujoco(serialized);
    expect(sanitized).not.toMatch(/ns:/);
    expect(() => validateUrdfInertiaForMujoco(sanitized)).not.toThrow();
    expect(sanitized).toMatch(/\bixx="0\.01"/);
  });

  it('fixes multi-line inertia tags', () => {
    const urdf = `<robot name="t">
  <link name="arm">
    <inertial>
      <mass value="1"/>
      <inertia ixx=""
        ixy="" ixz=""
        iyy="" iyz="" izz=""/>
    </inertial>
  </link>
</robot>`;
    const sanitized = sanitizeUrdfForMujoco(urdf);
    expect(sanitized).toMatch(/\bixx="0\.001"/);
    expect(() => validateUrdfInertiaForMujoco(sanitized)).not.toThrow();
  });

  it('deduplicates empty duplicate inertia in inertial block', () => {
    const urdf = `<robot name="t">
  <link name="arm">
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      <inertia/>
    </inertial>
  </link>
</robot>`;
    const sanitized = sanitizeUrdfForMujoco(urdf);
    expect((sanitized.match(/<inertia\b/gi) ?? []).length).toBe(1);
    expect(() => validateUrdfInertiaForMujoco(sanitized)).not.toThrow();
  });

  it('strips empty-name attributes that MuJoCo rejects', () => {
    const urdf = `<robot name="t">
  <link name="arm">
    <inertial>
      <mass value="1"/>
      <inertia ="broken" ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
</robot>`;
    const sanitized = sanitizeUrdfForMujoco(urdf);
    expect(sanitized).not.toMatch(/\s+="[^"]*"/);
    expect(() => validateUrdfInertiaForMujoco(sanitized)).not.toThrow();
    expect(sanitized).toMatch(/\bixx="0\.01"/);
  });

  it('tightens all self-closing tags for WASM', () => {
    const loose = `<robot><link name="a"><inertial><origin xyz="0 0 0" rpy="0 0 0" /><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" /></inertial></link></robot>`;
    const fixed = sanitizeUrdfForMujoco(loose);
    expect(fixed).not.toMatch(/ \/>/);
    expect(fixed).toMatch(/rpy="0 0 0"\/>/);
    expect(fixed).toMatch(/izz="0\.01"\/>/);
  });

  it('tightens space before slash on inertia tags', () => {
    const loose = `<robot><link name="a"><inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" /></inertial></link></robot>`;
    const fixed = sanitizeUrdfForMujoco(loose);
    expect(fixed).not.toMatch(/<inertia\b[^>]*?\s+\/>/);
    expect(fixed).toMatch(/izz="0\.01"\/>/);
  });

  it('does not corrupt valid self-closing inertia except tightening slash', () => {
    const urdf = readFileSync(
      join(__dirname, '../fixtures/biped_user_download_v2.urdf'),
      'utf-8',
    );
    const before = urdf.match(
      /<link name="base_link"[\s\S]*?<inertia\b[\s\S]*?\/>/,
    )?.[0];
    expect(before).toBeTruthy();
    const sanitized = sanitizeUrdfForMujoco(urdf);
    expect(sanitized).not.toMatch(/<inertia\b[^>]*?\s+\/>/);
    const after = sanitized.match(
      /<link name="base_link"[\s\S]*?<inertia\b[\s\S]*?(?:\/>|><\/inertia>)/,
    )?.[0];
    expect(after).toBeTruthy();
    expect(after).toMatch(/\bixx="/);
  });

  it('rejects WASM-breaking space before closing bracket', () => {
    const bad = `<robot><link name="a"><inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" /></inertial></link></robot>`;
    expect(() => validateUrdfInertiaForMujoco(bad)).toThrow(/inertia.*空格/);
    const fixed = sanitizeUrdfForMujoco(bad);
    expect(() => validateUrdfInertiaForMujoco(fixed)).not.toThrow();
  });
});
