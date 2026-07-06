/**
 * @vitest-environment happy-dom
 */
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { describe, expect, it } from 'vitest';
import { loadMujocoRobot } from '../mujoco/loader';

const DUMP = readFileSync(
  join(process.cwd(), 'debug-dumps/mujoco-failed-2026-07-04T05-16-29-383Z/urdf/biped_s70_upper_body.urdf'),
  'utf-8',
);

describe('debug dump repro', () => {
  it('loads failed dump as-is (node mujoco)', async () => {
    const result = await loadMujocoRobot({
      urdfText: DUMP,
      urdfFileName: 'urdf/biped_s70_upper_body.urdf',
      meshes: new Map(),
    });
    expect(result.jointNames.length).toBeGreaterThan(0);
    result.model.delete();
    result.data.delete();
  });

  it('loads failed dump after prepareUrdfForMujocoLoad', async () => {
    const { prepareUrdfForMujocoLoad } = await import('../utils/urdf-sanitize');
    const fixed = prepareUrdfForMujocoLoad(DUMP);
    expect(fixed).not.toMatch(/ \/>/);
    expect(fixed.split('\n')[27]).toMatch(/izz="0\.006817"\/>/);
    const result = await loadMujocoRobot({
      urdfText: fixed,
      urdfFileName: 'urdf/biped_s70_upper_body.urdf',
      meshes: new Map(),
    });
    expect(result.jointNames.length).toBeGreaterThan(0);
    result.model.delete();
    result.data.delete();
  });
});
