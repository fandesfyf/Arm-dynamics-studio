/**
 * @vitest-environment happy-dom
 */
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import {
  appendSpherePayloadUrdf,
  appendSpherePayloadWithRecord,
  attachUrdfSnippet,
  extractSnippetRootLink,
  listSpherePayloadLinks,
  makeUniqueLinkName,
  parseUrdfSnippet,
  parseWrenchValues,
  removeLastSpherePayloadOnLink,
  removeSpherePayloads,
  revertModifyInertialPayload,
  solidSphereInertia,
  wrenchIsZero,
} from './payload-editor';
import { parseLinkNames } from '../utils/urdf-base-fixture';

const __dirname = dirname(fileURLToPath(import.meta.url));
const TEST_ARM_URDF = readFileSync(
  join(__dirname, '../../public/robots/test_arm.urdf'),
  'utf-8',
);

describe('solidSphereInertia', () => {
  it('computes I = 2/5 m r^2 on diagonal', () => {
    const inertia = solidSphereInertia(1.0, 0.1);
    const expected = (2 / 5) * 1.0 * 0.01;
    expect(inertia.ixx).toBeCloseTo(expected, 8);
    expect(inertia.iyy).toBeCloseTo(expected, 8);
    expect(inertia.izz).toBeCloseTo(expected, 8);
    expect(inertia.ixy).toBe(0);
  });

  it('rejects non-positive mass or radius', () => {
    expect(() => solidSphereInertia(0, 0.1)).toThrow(/质量必须为正数/);
    expect(() => solidSphereInertia(1, 0)).toThrow(/半径必须为正数/);
  });
});

describe('makeUniqueLinkName', () => {
  it('appends numeric suffix when name taken', () => {
    expect(makeUniqueLinkName(['foo', 'foo_0'], 'foo')).toBe('foo_1');
  });
});

describe('parseWrenchValues', () => {
  it('fills missing components with zero', () => {
    expect(parseWrenchValues({ fx: 1, tz: 2 })).toEqual([1, 0, 0, 0, 0, 2]);
  });

  it('detects zero wrench', () => {
    expect(wrenchIsZero([0, 0, 0, 0, 0, 0])).toBe(true);
    expect(wrenchIsZero([0, 0, 0, 0, 0, 1])).toBe(false);
  });
});

describe('parseUrdfSnippet', () => {
  it('parses link and joint names from fragment', () => {
    const snippet = `
      <link name="tool_base"/>
      <link name="tool"/>
      <joint name="tool_joint" type="fixed">
        <parent link="tool_base"/>
        <child link="tool"/>
      </joint>
    `;
    const parsed = parseUrdfSnippet(snippet);
    expect(parsed.links).toEqual(['tool_base', 'tool']);
    expect(parsed.joints).toEqual(['tool_joint']);
    expect(extractSnippetRootLink(snippet)).toBe('tool_base');
  });
});

describe('appendSpherePayloadUrdf', () => {
  it('adds child link with sphere geom and fixed joint', () => {
    const xml = appendSpherePayloadUrdf(TEST_ARM_URDF, {
      parentLink: 'link2',
      mass: 0.2,
      radius: 0.03,
      mode: 'child_link',
      suffix: 'test_payload',
    });

    expect(xml).toContain('test_payload_sphere');
    expect(xml).toContain('<sphere radius="0.03"/>');
    expect(xml).toContain('<parent link="link2"/>');
    expect(xml).toMatch(/<inertia[^>]*ixx="[^"]+"/);
    expect(parseLinkNames(xml).some((name) => name.includes('test_payload_sphere'))).toBe(true);
  });

  it('modifies inertial on existing link', () => {
    const xml = appendSpherePayloadUrdf(TEST_ARM_URDF, {
      parentLink: 'base_link',
      mass: 0.5,
      radius: 0.05,
      mode: 'modify_inertial',
    });

    const massMatch = xml.match(
      /<link name="base_link"[\s\S]*?<mass value="([^"]+)"/,
    );
    expect(massMatch).not.toBeNull();
    expect(Number.parseFloat(massMatch![1]!)).toBeCloseTo(1.5, 6);
  });

  it('modify_inertial output contains complete inertia attributes', () => {
    const urdfMissingInertia = `<robot name="test">
  <link name="arm">
    <inertial><mass value="1"/></inertial>
  </link>
</robot>`;

    const xml = appendSpherePayloadUrdf(urdfMissingInertia, {
      parentLink: 'arm',
      mass: 0.5,
      radius: 0.05,
      mode: 'modify_inertial',
    });

    const inertiaMatch = xml.match(
      /<link name="arm"[\s\S]*?<inertia\s([^>]*)(?:\/>|>)/,
    );
    expect(inertiaMatch).not.toBeNull();
    const attrs = inertiaMatch![1]!;
    expect(attrs).toMatch(/\bixx="/);
    expect(attrs).toMatch(/\bixy="0"/);
    expect(attrs).toMatch(/\bixz="0"/);
    expect(attrs).toMatch(/\biyy="/);
    expect(attrs).toMatch(/\biyz="0"/);
    expect(attrs).toMatch(/\bizz="/);

    const payloadInertia = solidSphereInertia(0.5, 0.05);
    const ixxMatch = attrs.match(/\bixx="([^"]+)"/);
    expect(ixxMatch).not.toBeNull();
    expect(Number.parseFloat(ixxMatch![1]!)).toBeCloseTo(payloadInertia.ixx, 8);
  });

  it('child_link payload has no inline visual material (MuJoCo-safe)', () => {
    const xml = appendSpherePayloadUrdf(TEST_ARM_URDF, {
      parentLink: 'link2',
      mass: 0.2,
      radius: 0.03,
      mode: 'child_link',
      suffix: 'mat_test_payload',
    });
    const payloadBlock = xml.match(/<link name="mat_test_payload_sphere_0">[\s\S]*?<\/link>/)?.[0] ?? '';
    expect(payloadBlock).not.toMatch(/<material\b/i);
    expect(payloadBlock).toMatch(/<sphere radius="0\.03"\/>/);
  });

  it('rejects unknown parent link', () => {
    expect(() =>
      appendSpherePayloadUrdf(TEST_ARM_URDF, {
        parentLink: 'missing_link',
        mass: 0.1,
        radius: 0.02,
      }),
    ).toThrow(/未找到 link/);
  });
});

describe('removeSpherePayloads', () => {
  it('removes child_link payload links and joints', () => {
    const { urdfText, record } = appendSpherePayloadWithRecord(TEST_ARM_URDF, {
      parentLink: 'link2',
      mass: 0.2,
      radius: 0.03,
      mode: 'child_link',
      suffix: 'rm_payload',
    });
    expect(listSpherePayloadLinks(urdfText)).toContain(record.payloadLink);

    const cleaned = removeSpherePayloads(urdfText);
    expect(listSpherePayloadLinks(cleaned)).toHaveLength(0);
    expect(cleaned).not.toContain(record.payloadLink!);
    expect(cleaned).not.toContain(record.jointName!);
  });

  it('removeLastSpherePayloadOnLink removes only the latest on parent', () => {
    let xml = appendSpherePayloadUrdf(TEST_ARM_URDF, {
      parentLink: 'link2',
      mass: 0.1,
      radius: 0.02,
      suffix: 'a_payload',
    });
    xml = appendSpherePayloadUrdf(xml, {
      parentLink: 'link2',
      mass: 0.2,
      radius: 0.03,
      suffix: 'b_payload',
    });
    expect(listSpherePayloadLinks(xml)).toHaveLength(2);

    const once = removeLastSpherePayloadOnLink(xml, 'link2');
    expect(listSpherePayloadLinks(once)).toHaveLength(1);
    expect(once).toContain('a_payload');
    expect(once).not.toContain('b_payload');
  });

  it('reverts modify_inertial using stored record', () => {
    const { urdfText, record } = appendSpherePayloadWithRecord(TEST_ARM_URDF, {
      parentLink: 'base_link',
      mass: 0.5,
      radius: 0.05,
      mode: 'modify_inertial',
    });
    const reverted = revertModifyInertialPayload(urdfText, record);
    const massMatch = reverted.match(
      /<link name="base_link"[\s\S]*?<mass value="([^"]+)"/,
    );
    expect(Number.parseFloat(massMatch![1]!)).toBeCloseTo(1.0, 6);
  });
});

describe('attachUrdfSnippet', () => {
  it('prefixes snippet names and adds fixed joint to parent', () => {
    const snippet = `
      <link name="gripper">
        <inertial><mass value="0.1"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
      </link>
    `;

    const xml = attachUrdfSnippet(TEST_ARM_URDF, {
      parentLink: 'link3',
      snippetXml: snippet,
      prefix: 'tool_',
    });

    expect(xml).toContain('<link name="tool_gripper">');
    expect(xml).toContain('<parent link="link3"/>');
    expect(xml).toContain('<child link="tool_gripper"/>');
  });
});
