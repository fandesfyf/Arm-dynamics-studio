/**
 * @vitest-environment happy-dom
 */
import { readFileSync, existsSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import {
  extractRobotFromZip,
  pickUrdfPath,
  resolveAssetPath,
  stripPackageRoot,
} from './robot-asset-loader';
import { RobotSession } from '../core/robot-session';

const BIPED_ZIP =
  '/home/fandes/workspace/mimic_X1/source/whole_body_tracking/whole_body_tracking/assets/biped_s70.zip';

describe('robot-asset-loader', () => {
  it('stripPackageRoot removes single top-level folder', () => {
    const { paths, prefix } = stripPackageRoot([
      'biped_s70/urdf/biped_s70.urdf',
      'biped_s70/meshes/base_link.STL',
    ]);
    expect(prefix).toBe('biped_s70');
    expect(paths).toEqual(['urdf/biped_s70.urdf', 'meshes/base_link.STL']);
  });

  it('pickUrdfPath prefers upper_body urdf when present', () => {
    const picked = pickUrdfPath([
      'biped_s70/urdf/biped_s70.urdf',
      'biped_s70/urdf/biped_s70_upper_body.urdf',
    ]);
    expect(picked).toBe('biped_s70/urdf/biped_s70_upper_body.urdf');
  });

  it('pickUrdfPath prefers main urdf over drake subfolder', () => {
    const picked = pickUrdfPath([
      'biped_s70/urdf/drake/biped_v3_full.urdf',
      'biped_s70/urdf/biped_s70.urdf',
    ]);
    expect(picked).toBe('biped_s70/urdf/biped_s70.urdf');
  });

  it('resolveAssetPath resolves ../meshes from urdf folder', () => {
    expect(resolveAssetPath('urdf/biped_s70.urdf', '../meshes/base_link.STL')).toBe(
      'meshes/base_link.STL',
    );
  });

  it.skipIf(!existsSync(BIPED_ZIP))(
    'extractRobotFromZip loads biped_s70.zip with meshes',
    async () => {
      const buffer = readFileSync(BIPED_ZIP);
      const bundle = await extractRobotFromZip(buffer);
      expect(bundle.urdfFileName).toBe('urdf/biped_s70.urdf');
      expect(bundle.meshes.size).toBeGreaterThan(10);
      expect(bundle.urdfText).toContain('<robot name="biped_s70"');
    },
  );

  it.skipIf(!existsSync(BIPED_ZIP))(
    'RobotSession loads biped_s70 from zip bundle',
    async () => {
      const buffer = readFileSync(BIPED_ZIP);
      const bundle = await extractRobotFromZip(buffer);
      const session = await RobotSession.create({
        urdfXml: bundle.urdfText,
        urdfFileName: bundle.urdfFileName,
        meshes: bundle.meshes,
      });
      expect(session.jointNames.length).toBeGreaterThan(5);
      session.dispose();
    },
    120_000,
  );
});
