/**
 * @vitest-environment happy-dom
 */
import { describe, expect, it } from 'vitest';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { RobotSession } from '../core/robot-session';
import { ensureFixedBase } from './urdf-base-fixture';
import { sanitizeUrdfForMujoco } from './urdf-sanitize';
import { actuatedJointsToQpos, qposToActuatedJoints } from './joint-qpos';
import { vecGet } from '../types/mujoco';

describe('joint-qpos', () => {
  it('round-trips actuated joints for upper body', async () => {
    const urdf = readFileSync(
      resolve(__dirname, '../../public/robots/biped_s70_upper_body.urdf'),
      'utf8',
    );
    const fixture = ensureFixedBase(sanitizeUrdfForMujoco(urdf));
    const session = await RobotSession.create({
      urdfXml: fixture.urdfText,
      urdfFileName: 'biped_s70_upper_body.urdf',
    });

    const joints = qposToActuatedJoints(session);
    expect(joints.length).toBe(session.jointNames.length);

    const modified = joints.map((v, i) => (i === 0 ? v + 0.1 : v));
    const qFull = actuatedJointsToQpos(session, modified);
    expect(qFull.length).toBe(session.nq);

    const roundTrip = qposToActuatedJoints(session, qFull);
    expect(roundTrip[0]).toBeCloseTo(modified[0]!, 5);

    const qRead = vecGet(session.data.qpos, session.nq);
    expect(qRead.length).toBe(session.nq);

    session.dispose();
  });
});
