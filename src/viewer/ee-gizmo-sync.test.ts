/**
 * @vitest-environment happy-dom
 */
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import URDFLoader from 'urdf-loader';
import type { URDFRobot } from 'urdf-loader';
import { applyAllJointAngles, Z_UP_TO_Y_UP } from './ee-kinematics';
import { computeGizmoWorldPosition } from './ee-gizmo-sync';

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

describe('ee-gizmo-sync', () => {
  it('uses visual EE world pose by default (FK from main robot)', () => {
    const robot = parseBipedRobot();
    applyAllJointAngles(robot, ['waist_yaw_joint', 'zarm_l1_joint'], [0.4, 0.5]);

    const gizmo = computeGizmoWorldPosition(robot, 'zarm_l7_end_effector', [0, 0, 0]);
    const torso = computeGizmoWorldPosition(robot, 'base_link', [0, 0, 0]);

    expect(gizmo).not.toBeNull();
    expect(torso).not.toBeNull();
    const dx = gizmo!.x - torso!.x;
    const dy = gizmo!.y - torso!.y;
    const dz = gizmo!.z - torso!.z;
    expect(Math.hypot(dx, dy, dz)).toBeGreaterThan(0.15);
  });

  it('preferTarget uses eeTarget world position when panel-edited', () => {
    const robot = parseBipedRobot();
    applyAllJointAngles(robot, ['waist_yaw_joint', 'zarm_l1_joint'], [0.4, 0.5]);

    const fk = computeGizmoWorldPosition(robot, 'zarm_l7_end_effector', [0.3, 0.1, 0.5]);
    const fromTarget = computeGizmoWorldPosition(
      robot,
      'zarm_l7_end_effector',
      [0.3, 0.1, 0.5],
      { preferTarget: true },
    );

    expect(fk).not.toBeNull();
    expect(fromTarget).not.toBeNull();
    expect(fromTarget!.x).not.toBeCloseTo(fk!.x, 3);
    expect(fromTarget!.y).toBeCloseTo(0.5, 4);
    expect(fromTarget!.z).toBeCloseTo(-0.1, 4);
  });

  it('defaults to FK even when eeTarget differs from visual pose', () => {
    const robot = parseBipedRobot();
    applyAllJointAngles(robot, ['waist_yaw_joint', 'zarm_l1_joint'], [0.4, 0.5]);

    const staleTarget = [0, 0, 0] as [number, number, number];
    const fromFk = computeGizmoWorldPosition(robot, 'zarm_l7_end_effector', staleTarget);
    const fromTarget = computeGizmoWorldPosition(robot, 'zarm_l7_end_effector', staleTarget, {
      preferTarget: true,
    });

    expect(fromFk).not.toBeNull();
    expect(fromTarget).not.toBeNull();
    expect(fromFk!.x).not.toBeCloseTo(fromTarget!.x, 3);
  });

  it('returns null when robot ready but EE link missing (no stale fallback)', () => {
    const robot = parseBipedRobot();
    expect(computeGizmoWorldPosition(robot, 'nonexistent_link', [0.3, 0, 0.4])).toBeNull();
  });
});
