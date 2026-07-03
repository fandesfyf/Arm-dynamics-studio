import { useMemo } from 'react';
import { Line, Text } from '@react-three/drei';
import { useSessionStore } from '../../stores/session-store';

const MARKER_COLOR = '#ffb020';
const LINE_COLOR = '#ffb020';

export function MotionTargetMarkers() {
  const motionTargets = useSessionStore((s) => s.motionTargets);
  const controlMode = useSessionStore((s) => s.controlMode);

  const points = useMemo(() => {
    if (motionTargets.length === 0) return [];
    return motionTargets.map((mt) => mt.eeSceneWorld);
  }, [motionTargets]);

  if (controlMode !== 'interpolate' || points.length === 0) {
    return null;
  }

  return (
    <group>
      {points.length >= 2 && (
        <Line
          points={points}
          color={LINE_COLOR}
          lineWidth={2}
          dashed
          dashSize={0.04}
          gapSize={0.03}
        />
      )}
      {points.map((p, i) => (
        <group key={motionTargets[i]!.id} position={p}>
          <mesh>
            <sphereGeometry args={[0.025, 16, 16]} />
            <meshStandardMaterial color={MARKER_COLOR} emissive={MARKER_COLOR} emissiveIntensity={0.35} />
          </mesh>
          <Text
            position={[0, 0.05, 0]}
            fontSize={0.045}
            color="#ffe8b0"
            anchorX="center"
            anchorY="bottom"
            outlineWidth={0.004}
            outlineColor="#000000"
          >
            {String(i + 1)}
          </Text>
        </group>
      ))}
    </group>
  );
}
