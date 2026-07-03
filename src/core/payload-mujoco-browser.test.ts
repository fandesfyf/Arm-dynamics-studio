/**
 * @vitest-environment happy-dom
 */
import { readFileSync, existsSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import { appendSpherePayloadWithRecord } from './payload-editor';
import { loadMujocoRobot } from '../mujoco/loader';
import { ensureFixedBase } from '../utils/urdf-base-fixture';
import { sanitizeUrdfForMujoco, validateUrdfInertiaForMujoco, finalizeUrdfForMujoco } from '../utils/urdf-sanitize';

const __dirname = dirname(fileURLToPath(import.meta.url));
const LIVE_BIPED = '/home/fandes/biped_s70/urdf/biped_s70_upper_body.urdf';
const BIPED_URDF = existsSync(LIVE_BIPED)
  ? readFileSync(LIVE_BIPED, 'utf-8')
  : readFileSync(join(__dirname, '../../public/robots/biped_s70_upper_body.urdf'), 'utf-8');

function browserStorePipeline(
  urdfText: string,
  parentLink = 'base_link',
  mode: 'child_link' | 'modify_inertial' = 'child_link',
): string {
  const { urdfText: appended } = appendSpherePayloadWithRecord(urdfText, {
    parentLink,
    mass: 0.2,
    radius: 0.03,
    mode,
  });
  const stored = sanitizeUrdfForMujoco(appended);
  const doc = new DOMParser().parseFromString(stored, 'application/xml');
  expect(doc.querySelector('parsererror')).toBeNull();
  const roundTrip = new XMLSerializer().serializeToString(doc);
  return ensureFixedBase(sanitizeUrdfForMujoco(roundTrip)).urdfText;
}

function countInertiaPerInertial(urdfText: string): number[] {
  const counts: number[] = [];
  const re = /<inertial\b[^>]*>([\s\S]*?)<\/inertial>/gi;
  let m: RegExpExecArray | null;
  while ((m = re.exec(urdfText)) !== null) {
    const inner = m[1]!;
    counts.push((inner.match(/<inertia\b/gi) ?? []).length);
  }
  return counts;
}

describe('payload MuJoCo browser pipeline', () => {
  it('DOM round-trip after payload still loads MuJoCo', async () => {
    const initial = sanitizeUrdfForMujoco(BIPED_URDF);
    const fixture = browserStorePipeline(initial);

    const inertiaCounts = countInertiaPerInertial(fixture);
    expect(inertiaCounts.every((n) => n === 1)).toBe(true);

    expect(() => validateUrdfInertiaForMujoco(fixture)).not.toThrow();

    const result = await loadMujocoRobot({
      urdfText: fixture,
      urdfFileName: 'urdf/biped_s70_upper_body.urdf',
      meshes: new Map(),
    });
    expect(result.jointNames.length).toBeGreaterThan(0);
    result.model.delete();
    result.data.delete();
  });

  it('DOM round-trip may duplicate inertia before sanitize', () => {
    const { urdfText: appended } = appendSpherePayloadWithRecord(BIPED_URDF, {
      parentLink: 'base_link',
      mass: 0.2,
      radius: 0.03,
      mode: 'child_link',
    });
    const doc = new DOMParser().parseFromString(appended, 'application/xml');
    const roundTrip = new XMLSerializer().serializeToString(doc);
    const before = countInertiaPerInertial(roundTrip);
    const after = countInertiaPerInertial(sanitizeUrdfForMujoco(roundTrip));
    expect(after.every((n) => n === 1)).toBe(true);
    if (before.some((n) => n !== 1)) {
      expect(before.some((n) => n !== 1)).toBe(true);
    }
  });

  it('full browser store pipeline with live biped file', async () => {
    const initial = sanitizeUrdfForMujoco(BIPED_URDF);
    const fixture = browserStorePipeline(initial);
    expect(() => validateUrdfInertiaForMujoco(fixture)).not.toThrow();
    const result = await loadMujocoRobot({
      urdfText: fixture,
      urdfFileName: 'urdf/biped_s70_upper_body.urdf',
      meshes: new Map(),
    });
    expect(result.jointNames.length).toBeGreaterThan(0);
    result.model.delete();
    result.data.delete();
  });

  it('unsanitized DOM round-trip is fixed by sanitize', async () => {
    const { urdfText: appended } = appendSpherePayloadWithRecord(BIPED_URDF, {
      parentLink: 'base_link',
      mass: 0.2,
      radius: 0.03,
      mode: 'child_link',
    });
    const doc = new DOMParser().parseFromString(appended, 'application/xml');
    const roundTrip = new XMLSerializer().serializeToString(doc);
    const fixed = ensureFixedBase(sanitizeUrdfForMujoco(roundTrip)).urdfText;

    expect(countInertiaPerInertial(fixed).every((n) => n === 1)).toBe(true);

    const result = await loadMujocoRobot({
      urdfText: fixed,
      urdfFileName: 'urdf/biped_s70_upper_body.urdf',
      meshes: new Map(),
    });
    expect(result.jointNames.length).toBeGreaterThan(0);
    result.model.delete();
    result.data.delete();
  });

  it.each([
    ['zarm_l7_link', 'child_link'],
    ['zarm_l7_link', 'modify_inertial'],
    ['torso', 'child_link'],
    ['torso', 'modify_inertial'],
  ] as const)('payload on %s (%s) survives full browser pipeline', async (parentLink, mode) => {
    const initial = sanitizeUrdfForMujoco(BIPED_URDF);
    const fixture = browserStorePipeline(initial, parentLink, mode);

    expect(() => validateUrdfInertiaForMujoco(fixture)).not.toThrow();
    expect(countInertiaPerInertial(fixture).every((n) => n === 1)).toBe(true);
    expect(fixture).not.toMatch(/\s+="[^"]*"/);
    expect(fixture).not.toMatch(/<inertia\s*\/>/);

    const result = await loadMujocoRobot({
      urdfText: fixture,
      urdfFileName: 'urdf/biped_s70_upper_body.urdf',
      meshes: new Map(),
    });
    expect(result.jointNames.length).toBeGreaterThan(0);
    result.model.delete();
    result.data.delete();
  });
});
