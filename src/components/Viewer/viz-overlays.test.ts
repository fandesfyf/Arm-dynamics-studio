import { describe, expect, it } from 'vitest';
import { Mesh, BoxGeometry, MeshBasicMaterial, Object3D } from 'three';
import { hideAllRobotMeshes } from './viz-overlays';

describe('viz-overlays skeleton mode', () => {
  it('hideAllRobotMeshes hides all meshes under robot root', () => {
    const robot = new Object3D();
    const mesh = new Mesh(new BoxGeometry(), new MeshBasicMaterial());
    robot.add(mesh);
    hideAllRobotMeshes(robot as import('urdf-loader').URDFRobot);
    expect(mesh.visible).toBe(false);
  });
});
