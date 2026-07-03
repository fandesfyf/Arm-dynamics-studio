/**
 * @vitest-environment happy-dom
 */
import { readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { describe, expect, it } from 'vitest';
import URDFLoader from 'urdf-loader';
import type { URDFRobot } from 'urdf-loader';
import {
  collectAllReferenceTfFrames,
  collectFullVizChainFrames,
  detectArmSideFromName,
  inferChainJointNames,
} from './ik-chain-utils';

const __dirname = dirname(fileURLToPath(import.meta.url));
const BIPED_URDF = readFileSync(
  join(__dirname, '../../public/robots/biped_s70_upper_body.urdf'),
  'utf-8',
);

function parseBipedRobot(): URDFRobot {
  const loader = new URDFLoader();
  loader.parseVisual = false;
  loader.parseCollision = false;
  return loader.parse(BIPED_URDF) as URDFRobot;
}

describe('ik-chain-utils', () => {
  it('detects zarm_l / zarm_r arm sides', () => {
    expect(detectArmSideFromName('zarm_l7_end_effector')).toBe('left');
    expect(detectArmSideFromName('zarm_l3_link')).toBe('left');
    expect(detectArmSideFromName('l_arm_base')).toBe('left');
    expect(detectArmSideFromName('zarm_r4_link')).toBe('right');
    expect(detectArmSideFromName('r_arm_base')).toBe('right');
    expect(detectArmSideFromName('base_link')).toBeUndefined();
  });

  it('collectFullVizChainFrames walks to base_link for left arm EE', () => {
    const robot = parseBipedRobot();
    const { links, joints } = collectFullVizChainFrames(robot, 'zarm_l7_end_effector');

    const linkNames = links.map((l) => (l as { urdfName?: string; name: string }).urdfName || l.name);
    expect(linkNames[0]).toBe('base_link');
    expect(linkNames).toContain('waist_roll');
    expect(linkNames).toContain('waist_yaw');
    expect(linkNames).toContain('l_arm_base');
    expect(linkNames).toContain('zarm_l1_link');
    expect(linkNames.at(-1)).toBe('zarm_l7_end_effector');
    expect(linkNames.some((n) => n.includes('zarm_r'))).toBe(false);
    expect(linkNames.some((n) => n.includes('r_arm'))).toBe(false);
    expect(joints.length).toBe(linkNames.length - 1);
    expect(linkNames.length).toBeGreaterThanOrEqual(10);
  });

  it('collectAllReferenceTfFrames includes both arms and torso', () => {
    const robot = parseBipedRobot();
    const { links, joints } = collectAllReferenceTfFrames(robot);

    const linkNames = links.map((l) => (l as { urdfName?: string; name: string }).urdfName || l.name);
    expect(linkNames).toContain('base_link');
    expect(linkNames).toContain('waist_yaw');
    expect(linkNames).toContain('zarm_l7_end_effector');
    expect(linkNames).toContain('zarm_r7_end_effector');
    expect(linkNames.some((n) => n.includes('zarm_r'))).toBe(true);
    expect(joints.length).toBeGreaterThan(10);
    expect(links.length).toBeGreaterThan(joints.length);
  });

  it('inferChainJointNames still stops before waist (IK chain)', () => {
    const robot = parseBipedRobot();
    const ikJoints = inferChainJointNames(robot, 'zarm_l7_end_effector');
    expect(ikJoints.some((n) => n.toLowerCase().includes('waist'))).toBe(false);
    expect(ikJoints.some((n) => n.includes('zarm_r'))).toBe(false);
    expect(ikJoints.length).toBeGreaterThan(0);
  });

  it('inferChainJointNames respects rootLink base_link', () => {
    const robot = parseBipedRobot();
    const ikJoints = inferChainJointNames(robot, 'zarm_l7_end_effector', {
      rootLink: 'base_link',
    });
    expect(ikJoints.some((n) => n.includes('zarm_r'))).toBe(false);
    expect(ikJoints.length).toBeGreaterThan(0);
  });
});
