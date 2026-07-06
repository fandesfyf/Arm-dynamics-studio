/**
 * @vitest-environment happy-dom
 */
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import { MassEditor } from './mass-editor';

const __dirname = dirname(fileURLToPath(import.meta.url));
const TEST_ARM_URDF = readFileSync(
  join(__dirname, '../fixtures/simple_test_arm.urdf'),
  'utf-8',
);

describe('MassEditor', () => {
  it('getLinkInertials reads mass and inertia from URDF DOM', () => {
    const editor = new MassEditor(TEST_ARM_URDF);
    const inertials = editor.getLinkInertials();

    const base = inertials.find((item) => item.linkName === 'base_link');
    expect(base).toBeDefined();
    expect(base!.mass).toBeCloseTo(1.0, 6);
    expect(base!.inertia.ixx).toBeCloseTo(0.001, 6);
    expect(base!.inertia.iyy).toBeCloseTo(0.001, 6);
    expect(base!.inertia.izz).toBeCloseTo(0.001, 6);

    const link1 = inertials.find((item) => item.linkName === 'link1');
    expect(link1).toBeDefined();
    expect(link1!.mass).toBeCloseTo(0.5, 6);
    expect(link1!.com.z).toBeCloseTo(0.1, 6);
  });

  it('setLinkMass updates serialized XML', () => {
    const editor = new MassEditor(TEST_ARM_URDF);
    editor.setLinkMass('base_link', 2.5);
    const xml = editor.serialize();

    const roundTrip = new MassEditor(xml);
    const base = roundTrip.getLinkInertials().find((item) => item.linkName === 'base_link');
    expect(base!.mass).toBeCloseTo(2.5, 6);
    expect(xml).toContain('value="2.5"');
  });

  it('setLinkInertia and setJointLimits round-trip through serialize', () => {
    const editor = new MassEditor(TEST_ARM_URDF);
    editor.setLinkInertia('link2', 0.01, 0.02, 0.03);
    editor.setJointLimits('joint1', {
      lower: -1.5,
      upper: 1.5,
      effort: 120,
      velocity: 2.0,
    });

    const xml = editor.serialize();
    const roundTrip = new MassEditor(xml);

    const link2 = roundTrip.getLinkInertials().find((item) => item.linkName === 'link2');
    expect(link2!.inertia.ixx).toBeCloseTo(0.01, 6);
    expect(link2!.inertia.iyy).toBeCloseTo(0.02, 6);
    expect(link2!.inertia.izz).toBeCloseTo(0.03, 6);

    const joint1 = roundTrip.getJointLimits().find((item) => item.jointName === 'joint1');
    expect(joint1!.lower).toBeCloseTo(-1.5, 6);
    expect(joint1!.upper).toBeCloseTo(1.5, 6);
    expect(joint1!.effort).toBeCloseTo(120, 6);
    expect(joint1!.velocity).toBeCloseTo(2.0, 6);
  });

  it('rejects non-positive mass', () => {
    const editor = new MassEditor(TEST_ARM_URDF);
    expect(() => editor.setLinkMass('base_link', 0)).toThrow(/质量必须为正数/);
  });
});
