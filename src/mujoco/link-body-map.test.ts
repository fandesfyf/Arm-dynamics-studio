/**
 * @vitest-environment happy-dom
 */
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import { loadMujocoRobot } from './loader';
import {
  buildUrdfLinkBodyBindings,
  collectMjBodyNames,
  resolveLinkToMjBodyId,
} from './link-body-map';
import { applyExternalWrenches } from './external-wrench';
import { vecGet } from '../types/mujoco';
import { detectBaseLink } from '../utils/urdf-base-fixture';
import { ensureFixedBase } from '../utils/urdf-base-fixture';
import { sanitizeUrdfForMujoco } from '../utils/urdf-sanitize';

const __dirname = dirname(fileURLToPath(import.meta.url));
const BIPED_URDF = readFileSync(
  join(__dirname, '../../public/robots/biped_s70_upper_body.urdf'),
  'utf-8',
);

describe('link-body-map', () => {
  it('maps base_link to world and fixed end_effector to parent arm body', async () => {
    const fixture = ensureFixedBase(sanitizeUrdfForMujoco(BIPED_URDF));
    const result = await loadMujocoRobot({
      urdfText: fixture.urdfText,
      urdfFileName: 'biped_s70_upper_body.urdf',
      meshes: new Map(),
    });

    const baseLink = detectBaseLink(fixture.urdfText);
    const bindings = buildUrdfLinkBodyBindings(
      result.mujoco,
      result.model,
      fixture.urdfText,
      baseLink,
    );

    expect(bindings.get('base_link')?.bodyId).toBeGreaterThanOrEqual(0);
    expect(bindings.get('zarm_l7_link')?.bodyId).toBeGreaterThanOrEqual(0);

    const ee = bindings.get('zarm_l7_end_effector');
    const arm = bindings.get('zarm_l7_link');
    expect(ee).toBeDefined();
    expect(arm).toBeDefined();
    expect(ee!.bodyId).toBe(arm!.bodyId);
    expect(ee!.fixedChain).toHaveLength(1);
    expect(ee!.fixedChain[0]!.xyz[2]).toBeCloseTo(-0.17);

    const baseBodyId = resolveLinkToMjBodyId(result.mujoco, result.model, 'base_link', {
      baseLink,
    });
    expect(result.model.body(baseBodyId).name).toBe('world');

    const bodies = collectMjBodyNames(result.mujoco, result.model);
    expect(bodies).not.toContain('base_link');
    expect(bodies).not.toContain('zarm_l7_end_effector');

    result.model.delete();
    result.data.delete();
  });
});

describe('external-wrench', () => {
  it('applies wrench on end_effector via mj_applyFT without error', async () => {
    const fixture = ensureFixedBase(sanitizeUrdfForMujoco(BIPED_URDF));
    const result = await loadMujocoRobot({
      urdfText: fixture.urdfText,
      urdfFileName: 'biped_s70_upper_body.urdf',
      meshes: new Map(),
    });

    const bindings = buildUrdfLinkBodyBindings(
      result.mujoco,
      result.model,
      fixture.urdfText,
      detectBaseLink(fixture.urdfText),
    );

    const wrenches = new Map<string, [number, number, number, number, number, number]>([
      ['zarm_l7_end_effector', [0, 0, 20, 0, 0, 0]],
      ['zarm_l7_link', [0, 0, 10, 0, 0, 0]],
    ]);

    expect(() =>
      applyExternalWrenches(
        result.mujoco,
        result.model,
        result.data,
        wrenches,
        result.model.nv,
        { linkBodyBindings: bindings, zeroQfrcBeforeApply: result.model.nu > 0 },
      ),
    ).not.toThrow();

    const qfrc = vecGet(result.data.qfrc_applied, result.model.nv);
    const sum = qfrc.reduce((a, b) => a + Math.abs(b), 0);
    expect(sum).toBeGreaterThan(0);

    result.model.delete();
    result.data.delete();
  });
});
