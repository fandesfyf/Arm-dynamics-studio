/**
 * @vitest-environment happy-dom
 */
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import { appendSpherePayloadUrdf } from './payload-editor';
import { loadMujocoRobot } from '../mujoco/loader';
import { ensureFixedBase } from '../utils/urdf-base-fixture';
import { sanitizeUrdfForMujoco } from '../utils/urdf-sanitize';

const __dirname = dirname(fileURLToPath(import.meta.url));
const BIPED_URDF = readFileSync(
  join(__dirname, '../../public/robots/biped_s70_upper_body.urdf'),
  'utf-8',
);

describe('appendSpherePayloadUrdf + MuJoCo load', () => {
  async function expectMujocoLoad(urdfText: string): Promise<void> {
    const fixture = ensureFixedBase(sanitizeUrdfForMujoco(urdfText));

    const emptyInertia = fixture.urdfText.match(/<inertia(?:\s[^>]*)?\/>/g)?.filter(
      (tag) => !/\bixx=/.test(tag),
    );
    expect(emptyInertia ?? []).toEqual([]);

    const result = await loadMujocoRobot({
      urdfText: fixture.urdfText,
      urdfFileName: 'biped_s70_upper_body.urdf',
      meshes: new Map(),
    });
    expect(result.jointNames.length).toBeGreaterThan(0);
    result.model.delete();
    result.data.delete();
  }

  it('loads after full browser PayloadPanel → loadRobot pipeline on base_link', async () => {
    const withPayload = appendSpherePayloadUrdf(BIPED_URDF, {
      parentLink: 'base_link',
      mass: 0.2,
      radius: 0.03,
      mode: 'child_link',
    });
    const stored = sanitizeUrdfForMujoco(withPayload);
    const fixture = ensureFixedBase(sanitizeUrdfForMujoco(stored));
    await expectMujocoLoad(fixture.urdfText);
  });

  it('loads biped after child_link sphere payload on base_link', async () => {
    const withPayload = appendSpherePayloadUrdf(BIPED_URDF, {
      parentLink: 'base_link',
      mass: 0.2,
      radius: 0.03,
      mode: 'child_link',
    });
    await expectMujocoLoad(withPayload);
  });

  it('loads biped after child_link sphere payload', async () => {
    const withPayload = appendSpherePayloadUrdf(BIPED_URDF, {
      parentLink: 'zarm_l7_link',
      mass: 0.2,
      radius: 0.03,
      mode: 'child_link',
    });
    await expectMujocoLoad(withPayload);
  });

  it('loads biped after modify_inertial on zero-inertia torso', async () => {
    const withPayload = appendSpherePayloadUrdf(BIPED_URDF, {
      parentLink: 'torso',
      mass: 0.2,
      radius: 0.03,
      mode: 'modify_inertial',
    });
    await expectMujocoLoad(withPayload);
  });

  it('loads after DOM round-trip of child_link payload', async () => {
    const withPayload = appendSpherePayloadUrdf(BIPED_URDF, {
      parentLink: 'zarm_l7_link',
      mass: 0.2,
      radius: 0.03,
      mode: 'child_link',
    });
    const doc = new DOMParser().parseFromString(withPayload, 'application/xml');
    const roundTrip = new XMLSerializer().serializeToString(doc);
    await expectMujocoLoad(roundTrip);
  });

  it('loads biped after remove-all payloads reverts URDF', async () => {
    const withPayload = appendSpherePayloadUrdf(BIPED_URDF, {
      parentLink: 'base_link',
      mass: 0.2,
      radius: 0.03,
      mode: 'child_link',
    });
    const { removeSpherePayloads } = await import('./payload-editor');
    const cleaned = removeSpherePayloads(withPayload);
    await expectMujocoLoad(cleaned);
  });
});
