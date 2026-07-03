/**
 * @vitest-environment happy-dom
 */
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import { appendSpherePayloadWithRecord } from './payload-editor';
import { RobotSession } from './robot-session';
import { finalizeUrdfForMujoco, sanitizeUrdfForMujoco } from '../utils/urdf-sanitize';
import { ensureFixedBase } from '../utils/urdf-base-fixture';
import { loadMujocoRobot } from '../mujoco/loader';

const __dirname = dirname(fileURLToPath(import.meta.url));
const USER_V2 = readFileSync(
  join(__dirname, '../fixtures/biped_user_download_v2.urdf'),
  'utf-8',
);

describe('user download v2 URDF repro', () => {
  it('raw user v2 loads in MuJoCo without sanitize', async () => {
    const fixture = ensureFixedBase(USER_V2);
    const result = await loadMujocoRobot({
      urdfText: fixture.urdfText,
      urdfFileName: 'urdf/biped_s70_upper_body.urdf',
      meshes: new Map(),
    });
    expect(result.jointNames.length).toBeGreaterThan(0);
    result.model.delete();
    result.data.delete();
  });

  it('user v2 + child_link payload loads via RobotSession', async () => {
    const { urdfText } = appendSpherePayloadWithRecord(USER_V2, {
      parentLink: 'zarm_l7_link',
      mass: 0.2,
      radius: 0.03,
      mode: 'child_link',
    });
    expect(urdfText).toContain('_payload');
    const fixture = ensureFixedBase(finalizeUrdfForMujoco(urdfText));
    const session = await RobotSession.create({
      urdfXml: fixture.urdfText,
      urdfFileName: 'urdf/biped_s70_upper_body.urdf',
      meshes: new Map(),
    });
    expect(session.jointNames.length).toBeGreaterThan(0);
    session.dispose();
  });

  it('sanitize does not corrupt line 29 inertia attrs', () => {
    const sanitized = sanitizeUrdfForMujoco(USER_V2);
    const line = sanitized.split('\n')[28]!;
    expect(line).toMatch(/\bixx="/);
    expect(line).toMatch(/\bizz="/);
    expect(line).not.toMatch(/\s+"/);
  });
});
