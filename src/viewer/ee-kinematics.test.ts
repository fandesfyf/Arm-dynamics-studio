/**
 * @vitest-environment happy-dom
 */
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import type { URDFRobot } from 'urdf-loader';
import {
  applyAllJointAngles,
  urdfTargetToWorld,
  worldToUrdfTarget,
  readEeWorldFromRobot,
  Z_UP_TO_Y_UP,
} from './ee-kinematics';
import { collectFullVizChainFrames } from '../ik/ik-chain-utils';
import { attachReferenceTfFrames, disposeOverlayMarkers } from '../components/Viewer/viz-overlays';

const __dirname = dirname(fileURLToPath(import.meta.url));
const BIPED_URDF = readFileSync(
  join(__dirname, '../../public/robots/biped_s70_upper_body.urdf'),
  'utf-8',
);

function parseBipedRobot(): URDFRobot {
  const loader = new URDFLoader();
  loader.parseVisual = false;
  loader.parseCollision = false;
  const robot = loader.parse(BIPED_URDF) as URDFRobot;
  robot.rotation.x = Z_UP_TO_Y_UP;
  return robot;
}

describe('ee-kinematics', () => {
  it('round-trips urdfTargetToWorld and worldToUrdfTarget', () => {
    const fk: Vec3 = [0.4, 0.2, 0.9];
    expect(worldToUrdfTarget(urdfTargetToWorld(fk))).toEqual(fk);
  });

  it('readEeWorldFromRobot matches link world matrix', () => {
    const robot = parseBipedRobot();
    applyAllJointAngles(robot, ['waist_yaw_joint'], [0.3]);
    const fromApi = readEeWorldFromRobot(robot, 'zarm_l7_end_effector');
    const link = robot.links.zarm_l7_end_effector!;
    const manual = new THREE.Vector3();
    link.getWorldPosition(manual);
    expect(fromApi!.distanceTo(manual)).toBeLessThan(1e-6);
  });

  it('TF chain endpoints stay within robot bounding volume', () => {
    const robot = parseBipedRobot();
    const jointNames = Object.keys(robot.joints).filter((n) => n.includes('zarm_l'));
    const angles = jointNames.map((_, i) => 0.1 * (i + 1));
    applyAllJointAngles(robot, jointNames, angles);

    const markers = attachReferenceTfFrames(robot, 'zarm_l7_end_effector', {
      frameSize: 0.1,
      showChainLines: true,
    });

    const box = new THREE.Box3().setFromObject(robot);
    box.expandByScalar(0.15);

    const { links } = collectFullVizChainFrames(robot, 'zarm_l7_end_effector');
    const pos = new THREE.Vector3();
    for (const link of links) {
      link.getWorldPosition(pos);
      expect(box.containsPoint(pos)).toBe(true);
    }

    disposeOverlayMarkers(markers);
  });
});

type Vec3 = [number, number, number];
