import { Suspense, useMemo } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid, Environment } from '@react-three/drei';
import { useSessionStore } from '../../stores/session-store';
import { useVizStore } from '../../stores/viz-store';
import { EndEffectorControlsR3f } from '../../ik/end-effector-controls-r3f';
import { MotionTargetMarkers } from './MotionTargetMarkers';
import { ExternalWrenchMarkers } from './ExternalWrenchMarkers';
import { ReferencePoseOverlay } from './ReferencePoseOverlay';
import { UrdfModel } from './UrdfModel';

function SceneContent() {
  const urdfText = useSessionStore((s) => s.urdfText);
  const urdfFileName = useSessionStore((s) => s.urdfFileName);
  const meshAssets = useSessionStore((s) => s.meshAssets);
  const robotInfo = useSessionStore((s) => s.robotInfo);
  const jointNames = useSessionStore((s) => s.robotInfo?.jointNames ?? []);
  const jointPositions = useSessionStore((s) => s.jointPositions);
  const jointTargets = useSessionStore((s) => s.jointTargets);
  const referenceJointPositions = useSessionStore((s) => s.referenceJointPositions);
  const endEffectorLink = useSessionStore((s) => s.endEffectorLink);
  const showCollision = useVizStore((s) => s.showCollision);
  const showInertia = useVizStore((s) => s.showInertia);
  const modelOpacity = useVizStore((s) => s.modelOpacity);
  const showJointAxes = useVizStore((s) => s.showJointAxes);
  const referencePoseStyle = useVizStore((s) => s.referencePoseStyle);
  const referenceTfFrameSize = useVizStore((s) => s.referenceTfFrameSize);
  const referenceTfShowChainLines = useVizStore((s) => s.referenceTfShowChainLines);
  const controlLayer = useSessionStore((s) => s.controlLayer);
  const ikEnabled = useSessionStore((s) => s.ikEnabled);

  const referenceJoints = useMemo(() => {
    if (controlLayer === 'joint') {
      return jointTargets.length > 0 ? jointTargets : null;
    }
    return referenceJointPositions.length > 0 ? referenceJointPositions : null;
  }, [controlLayer, jointTargets, referenceJointPositions]);

  const showEeGizmo = controlLayer === 'ee' && ikEnabled;

  if (!urdfText || !urdfFileName) {
    return null;
  }

  return (
    <>
      <UrdfModel
        urdfText={urdfText}
        urdfFileName={urdfFileName}
        meshAssets={meshAssets}
        jointNames={jointNames}
        jointPositions={jointPositions}
        showCollision={showCollision}
        showInertia={showInertia}
        modelOpacity={modelOpacity}
        showJointAxes={showJointAxes}
      />
      {referenceJoints && (
        <ReferencePoseOverlay
          style={referencePoseStyle}
          urdfText={urdfText}
          urdfFileName={urdfFileName}
          meshAssets={meshAssets}
          jointNames={jointNames}
          referenceJointPositions={referenceJoints}
          endEffectorLink={endEffectorLink}
          modelOpacity={modelOpacity}
          referenceTfFrameSize={referenceTfFrameSize}
          referenceTfShowChainLines={referenceTfShowChainLines}
        />
      )}
      {robotInfo && showEeGizmo && <EndEffectorControlsR3f />}
      <MotionTargetMarkers />
      <ExternalWrenchMarkers />
    </>
  );
}

interface RobotViewerProps {
  onLoadTestArm?: () => void;
}

export function RobotViewer(_props: RobotViewerProps) {
  const urdfText = useSessionStore((s) => s.urdfText);
  const loading = useSessionStore((s) => s.loading);

  return (
    <div className="viewer-panel">
      <Canvas shadows camera={{ position: [1.2, 1.0, 1.2], fov: 45 }}>
        <color attach="background" args={['#0d0d12']} />
        <ambientLight intensity={0.45} />
        <directionalLight position={[4, 6, 3]} intensity={1.1} castShadow />
        <Grid
          infiniteGrid
          fadeDistance={12}
          fadeStrength={1}
          cellSize={0.1}
          sectionSize={0.5}
          position={[0, -0.01, 0]}
          cellColor="#2a2a3a"
          sectionColor="#3a3a50"
        />
        <Suspense fallback={null}>
          {urdfText && <SceneContent />}
        </Suspense>
        <OrbitControls makeDefault target={[0, 0.35, 0]} />
        <Environment preset="city" />
      </Canvas>

      {!urdfText && !loading && (
        <div className="viewer-empty">
          <span className="viewer-empty-icon" aria-hidden>🦾</span>
          <p className="viewer-empty-title">尚未加载机器人模型</p>
          <p className="viewer-empty-hint">
            正在加载默认模型 <code>biped_s70_upper_body</code>…
          </p>
        </div>
      )}

      {loading && (
        <div className="viewer-loading">
          <div style={{ textAlign: 'center' }}>
            <div className="viewer-loading-spinner" />
            <p className="viewer-loading-text">正在加载模型…</p>
          </div>
        </div>
      )}
    </div>
  );
}
