import { describe, expect, it } from 'vitest';
import {
  applyFixedOriginsToPlacement,
  collectFixedOriginsToLink,
  fkToScene,
  sceneToFk,
} from './ee-frame-utils';

const BIPED_EE_JOINT = `<joint name="zarm_l7_end_effector_joint" type="fixed">
    <parent link="zarm_l7_link" />
    <child link="zarm_l7_end_effector" />
    <origin xyz="0 0 -0.17" rpy="0 0 0" />
  </joint>`;

describe('ee-frame-utils', () => {
  it('round-trips fkToScene and sceneToFk', () => {
    const fk: [number, number, number] = [0.4, 0.2, 0.9];
    expect(sceneToFk(fkToScene(fk))).toEqual(fk);
  });

  it('collects fixed origin chain to end-effector link', () => {
    const origins = collectFixedOriginsToLink(
      BIPED_EE_JOINT,
      'zarm_l7_end_effector',
      'zarm_l7_joint',
    );
    expect(origins).toHaveLength(1);
    expect(origins[0]!.xyz).toEqual([0, 0, -0.17]);
  });

  it('applies fixed offset along parent Z axis', () => {
    const { pos } = applyFixedOriginsToPlacement(
      [0, 0, 0],
      [1, 0, 0, 0, 1, 0, 0, 0, 1],
      collectFixedOriginsToLink(
        BIPED_EE_JOINT,
        'zarm_l7_end_effector',
        'zarm_l7_joint',
      ),
    );
    expect(pos[2]).toBeCloseTo(-0.17, 5);
  });
});
