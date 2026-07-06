/**
 * @vitest-environment happy-dom
 */
import { readFileSync, existsSync } from 'node:fs';
import { join } from 'node:path';
import { describe, expect, it } from 'vitest';
import {
  extractRobotFromFiles,
  extractRobotFromZip,
  listUrdfCandidates,
  listUrdfPathsFromFiles,
  listUrdfPathsFromZip,
  pickUrdfPath,
  prepareFolderFiles,
  resolveAssetPath,
  stripPackageRoot,
} from './robot-asset-loader';
import { RobotSession } from '../core/robot-session';

const BIPED_ZIP =
  '/home/fandes/workspace/mimic_X1/source/whole_body_tracking/whole_body_tracking/assets/biped_s70.zip';

const TEST_ARM_ZIP = join(__dirname, '../../public/robots/test_arm.zip');

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

  it('listUrdfCandidates returns all urdfs sorted by preference', () => {
    const listed = listUrdfCandidates([
      'biped_s70/urdf/drake/biped_v3_full.urdf',
      'biped_s70/urdf/biped_s70.urdf',
      'biped_s70/urdf/biped_s70_upper_body.urdf',
    ]);
    expect(listed).toEqual([
      'biped_s70/urdf/biped_s70_upper_body.urdf',
      'biped_s70/urdf/biped_s70.urdf',
    ]);
  });

  it('listUrdfCandidates can include nested urdfs under skipped dirs for picker', () => {
    const listed = listUrdfCandidates(
      [
        'urdf/biped_s56.urdf',
        'urdf/biped_s56_gazebo.urdf',
        'urdf/drake/biped_v3.urdf',
      ],
      { includeSkippedDirs: true },
    );
    expect(listed).toEqual([
      'urdf/biped_s56_gazebo.urdf',
      'urdf/biped_s56.urdf',
      'urdf/drake/biped_v3.urdf',
    ]);
  });

  it('listUrdfPathsFromFiles lists stripped recursive urdf paths', () => {
    const urdf = new File(['<robot name="biped_s56"/>'], 'biped_s56.urdf', { type: 'text/xml' });
    const gazebo = new File(['<robot name="gazebo"/>'], 'biped_s56_gazebo.urdf', { type: 'text/xml' });
    Object.defineProperty(urdf, 'webkitRelativePath', {
      value: 'biped_s56/urdf/biped_s56.urdf',
    });
    Object.defineProperty(gazebo, 'webkitRelativePath', {
      value: 'biped_s56/urdf/biped_s56_gazebo.urdf',
    });
    const listed = listUrdfPathsFromFiles([urdf, gazebo], { includeSkippedDirs: true });
    expect(listed).toEqual(['urdf/biped_s56_gazebo.urdf', 'urdf/biped_s56.urdf']);
  });

  it('extractRobotFromFiles loads user-selected stripped urdf path', async () => {
    const urdf = new File(['<robot name="biped_s56"/>'], 'biped_s56.urdf', { type: 'text/xml' });
    const mesh = new File([new Uint8Array([1, 2, 3])], 'base.stl', {
      type: 'application/octet-stream',
    });
    Object.defineProperty(urdf, 'webkitRelativePath', {
      value: 'biped_s56/urdf/biped_s56.urdf',
    });
    Object.defineProperty(mesh, 'webkitRelativePath', {
      value: 'biped_s56/meshes/base.stl',
    });
    const bundle = await extractRobotFromFiles([urdf, mesh], 'urdf/biped_s56.urdf');
    expect(bundle.urdfFileName).toBe('urdf/biped_s56.urdf');
    expect(bundle.urdfText).toContain('biped_s56');
    expect(bundle.meshes.get('meshes/base.stl')).toBeDefined();
  });

  it('prepareFolderFiles ignores basename alias pollution', () => {
    const urdf = new File(['x'], 'biped_s56.urdf', { type: 'text/xml' });
    Object.defineProperty(urdf, 'webkitRelativePath', {
      value: 'biped_s56/urdf/biped_s56.urdf',
    });
    const prepared = prepareFolderFiles([urdf]);
    expect(prepared.strippedPaths).toEqual(['urdf/biped_s56.urdf']);
    expect(prepared.strippedToFile.get('urdf/biped_s56.urdf')).toBe(urdf);
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
    'listUrdfPathsFromZip returns urdf paths from biped zip',
    async () => {
      const buffer = readFileSync(BIPED_ZIP);
      const paths = await listUrdfPathsFromZip(buffer);
      expect(paths.length).toBeGreaterThan(0);
      expect(paths[0]).toMatch(/\.urdf$/i);
    },
  );

  it.skipIf(!existsSync(BIPED_ZIP))(
    'extractRobotFromZip accepts explicit urdf path',
    async () => {
      const buffer = readFileSync(BIPED_ZIP);
      const paths = await listUrdfPathsFromZip(buffer);
      const target = paths[0]!;
      const bundle = await extractRobotFromZip(buffer, target);
      expect(bundle.urdfFileName).toBe(target);
      expect(bundle.urdfText).toContain('<robot');
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

  it('extractRobotFromZip loads bundled test_arm.zip with simplified meshes', async () => {
    const buffer = readFileSync(TEST_ARM_ZIP);
    const bundle = await extractRobotFromZip(buffer);
    expect(bundle.urdfFileName).toBe('urdf/test_arm.urdf');
    expect(bundle.meshes.size).toBe(7);
    expect(bundle.urdfText).toContain('<robot name="test_arm"');
    const session = await RobotSession.create({
      urdfXml: bundle.urdfText,
      urdfFileName: bundle.urdfFileName,
      meshes: bundle.meshes,
    });
    expect(session.jointNames).toEqual([
      'joint1',
      'joint2',
      'joint3',
      'joint4',
      'joint5',
    ]);
    session.dispose();
  });
});
