import { useMemo, useRef } from 'react';
import { Text } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import {
  ArrowHelper,
  Group,
  Matrix3,
  Vector3,
} from 'three';
import { useSessionStore } from '../../stores/session-store';
import type { Wrench6 } from '../../core/payload-editor';
import { getUrdfLinkObject } from '../../ik/ik-chain-utils';
import { getMainUrdfRobot } from '../../utils/viewer-robot-registry';

const FORCE_COLOR = 0xff5533;
const TORQUE_COLOR = 0x33aaff;

const _force = new Vector3();
const _torque = new Vector3();
const _worldDir = new Vector3();
const _worldPos = new Vector3();
const _rot = new Matrix3();

function wrenchForceTorqueMagnitude(wrench: Wrench6): { force: number; torque: number } {
  const [fx, fy, fz, tx, ty, tz] = wrench;
  return {
    force: Math.hypot(fx, fy, fz),
    torque: Math.hypot(tx, ty, tz),
  };
}

function displayArrowLength(magnitude: number, scale = 0.02): number {
  return Math.min(0.55, Math.max(0.06, magnitude * scale));
}

interface LinkWrenchArrowsProps {
  linkName: string;
  wrench: Wrench6;
}

function LinkWrenchArrows({ linkName, wrench }: LinkWrenchArrowsProps) {
  const labelGroupRef = useRef<Group>(null);
  const forceArrow = useMemo(
    () => new ArrowHelper(new Vector3(0, 1, 0), new Vector3(), 0.1, FORCE_COLOR, 0.03, 0.018),
    [],
  );
  const torqueArrow = useMemo(
    () => new ArrowHelper(new Vector3(0, 0, 1), new Vector3(), 0.08, TORQUE_COLOR, 0.022, 0.012),
    [],
  );
  const labelPos = useRef(new Vector3());

  const { force: forceMag, torque: torqueMag } = wrenchForceTorqueMagnitude(wrench);

  useFrame(() => {
    const robot = getMainUrdfRobot();
    if (!robot) {
      forceArrow.visible = false;
      torqueArrow.visible = false;
      return;
    }

    const link = getUrdfLinkObject(robot, linkName);
    if (!link) {
      forceArrow.visible = false;
      torqueArrow.visible = false;
      return;
    }

    link.updateWorldMatrix(true, false);
    link.getWorldPosition(_worldPos);
    _rot.setFromMatrix4(link.matrixWorld);

    labelPos.current.copy(_worldPos);

    const [fx, fy, fz, tx, ty, tz] = wrench;

    if (forceMag > 1e-6) {
      _force.set(fx, fy, fz);
      _worldDir.copy(_force).applyMatrix3(_rot).normalize();
      forceArrow.position.copy(_worldPos);
      forceArrow.setDirection(_worldDir);
      forceArrow.setLength(
        displayArrowLength(forceMag),
        displayArrowLength(forceMag) * 0.22,
        displayArrowLength(forceMag) * 0.14,
      );
      forceArrow.visible = true;
      labelPos.current.copy(_worldPos);
      labelPos.current.y += displayArrowLength(forceMag) * 0.15;
    } else {
      forceArrow.visible = false;
      labelPos.current.copy(_worldPos);
    }

    if (torqueMag > 1e-6) {
      _torque.set(tx, ty, tz);
      _worldDir.copy(_torque).applyMatrix3(_rot).normalize();
      torqueArrow.position.copy(_worldPos);
      torqueArrow.setDirection(_worldDir);
      torqueArrow.setLength(
        displayArrowLength(torqueMag, 0.015),
        displayArrowLength(torqueMag, 0.015) * 0.25,
        displayArrowLength(torqueMag, 0.015) * 0.12,
      );
      torqueArrow.visible = true;
    } else {
      torqueArrow.visible = false;
    }

    labelGroupRef.current?.position.copy(labelPos.current);
  });

  return (
    <group>
      <primitive object={forceArrow} />
      <primitive object={torqueArrow} />
      {(forceMag > 1e-6 || torqueMag > 1e-6) && (
        <group ref={labelGroupRef}>
          <Text
            position={[0, 0, 0]}
            fontSize={0.04}
            color="#ffccbb"
            anchorX="center"
            anchorY="bottom"
            outlineWidth={0.003}
            outlineColor="#000000"
          >
            {`${linkName}\nF=${forceMag.toFixed(1)}N${torqueMag > 1e-6 ? ` τ=${torqueMag.toFixed(1)}N·m` : ''}`}
          </Text>
        </group>
      )}
    </group>
  );
}

/** 在 URDF link 上显示外力/力矩箭头（link 坐标系） */
export function ExternalWrenchMarkers() {
  const externalWrenches = useSessionStore((s) => s.externalWrenches);
  const jointPositions = useSessionStore((s) => s.jointPositions);
  const jointNames = useSessionStore((s) => s.robotInfo?.jointNames ?? []);

  const entries = useMemo(() => {
    void jointPositions;
    void jointNames;
    return [...externalWrenches.entries()].filter(([, w]) => !w.every((v) => Math.abs(v) < 1e-9));
  }, [externalWrenches, jointPositions, jointNames]);

  if (entries.length === 0) return null;

  return (
    <group>
      {entries.map(([linkName, wrench]) => (
        <LinkWrenchArrows key={linkName} linkName={linkName} wrench={wrench} />
      ))}
    </group>
  );
}
