import { useEffect, useMemo, useRef } from 'react';
import URDFLoader from 'urdf-loader';
import type { URDFRobot } from 'urdf-loader';
import type { Object3D } from 'three';
import type { ReferencePoseStyle } from '../../stores/viz-store';
import { registerReferenceUrdfRobot } from '../../utils/viewer-robot-registry';
import { applyAllJointAngles, Z_UP_TO_Y_UP } from '../../viewer/ee-kinematics';
import { UrdfModel } from './UrdfModel';
import {
  overlayIsActive,
  overlayShowsGhostMesh,
  overlayShowsTfMarkers,
} from './reference-overlay-mode';
import {
  attachReferenceTfFrames,
  disposeOverlayMarkers,
  hideAllRobotMeshes,
} from './viz-overlays';

export interface ReferencePoseOverlayProps {
  style: ReferencePoseStyle;
  urdfText: string;
  urdfFileName: string;
  meshAssets: Map<string, Uint8Array>;
  jointNames: string[];
  referenceJointPositions: number[];
  endEffectorLink: string;
  modelOpacity: number;
  referenceTfFrameSize: number;
  referenceTfShowChainLines: boolean;
}

function parseSkeletonUrdfRobot(urdfText: string): URDFRobot {
  const loader = new URDFLoader();
  loader.parseVisual = false;
  loader.parseCollision = false;
  const robot = loader.parse(urdfText) as URDFRobot;
  robot.rotation.x = Z_UP_TO_Y_UP;
  hideAllRobotMeshes(robot);
  return robot;
}

interface ReferenceTfSkeletonProps {
  urdfText: string;
  jointNames: string[];
  referenceJointPositions: number[];
  endEffectorLink: string;
  referenceTfFrameSize: number;
  referenceTfShowChainLines: boolean;
}

function ReferenceTfSkeleton({
  urdfText,
  jointNames,
  referenceJointPositions,
  endEffectorLink,
  referenceTfFrameSize,
  referenceTfShowChainLines,
}: ReferenceTfSkeletonProps) {
  const tfMarkersRef = useRef<Object3D[]>([]);
  const robot = useMemo(() => parseSkeletonUrdfRobot(urdfText), [urdfText]);

  useEffect(() => {
    registerReferenceUrdfRobot(robot);
    return () => {
      disposeOverlayMarkers(tfMarkersRef.current);
      tfMarkersRef.current = [];
      registerReferenceUrdfRobot(null);
    };
  }, [robot]);

  useEffect(() => {
    applyAllJointAngles(robot, jointNames, referenceJointPositions);
    disposeOverlayMarkers(tfMarkersRef.current);
    tfMarkersRef.current = attachReferenceTfFrames(robot, endEffectorLink, {
      frameSize: referenceTfFrameSize,
      showChainLines: referenceTfShowChainLines,
    });
  }, [
    robot,
    endEffectorLink,
    referenceTfFrameSize,
    referenceTfShowChainLines,
    jointNames,
    referenceJointPositions,
  ]);

  return <primitive object={robot} />;
}

export function ReferencePoseOverlay({
  style,
  urdfText,
  urdfFileName,
  meshAssets,
  jointNames,
  referenceJointPositions,
  endEffectorLink,
  modelOpacity,
  referenceTfFrameSize,
  referenceTfShowChainLines,
}: ReferencePoseOverlayProps) {
  if (!overlayIsActive(style, referenceJointPositions.length)) {
    return null;
  }

  if (overlayShowsTfMarkers(style)) {
    return (
      <ReferenceTfSkeleton
        urdfText={urdfText}
        jointNames={jointNames}
        referenceJointPositions={referenceJointPositions}
        endEffectorLink={endEffectorLink}
        referenceTfFrameSize={referenceTfFrameSize}
        referenceTfShowChainLines={referenceTfShowChainLines}
      />
    );
  }

  if (overlayShowsGhostMesh(style)) {
    return (
      <UrdfModel
        urdfText={urdfText}
        urdfFileName={urdfFileName}
        meshAssets={meshAssets}
        jointNames={jointNames}
        jointPositions={referenceJointPositions}
        showCollision={false}
        showInertia={false}
        modelOpacity={modelOpacity}
        showJointAxes={false}
        ghost
        onRobotReady={(instance) => {
          registerReferenceUrdfRobot(instance);
        }}
      />
    );
  }

  return null;
}
