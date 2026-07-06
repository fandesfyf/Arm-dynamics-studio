import { useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import type { URDFRobot } from 'urdf-loader';
import {
  SPHERE_PAYLOAD_LINK_PATTERN,
  SPHERE_PAYLOAD_VISUAL_COLOR,
} from '../../core/payload-editor';
import { computeGhostOpacity } from '../../stores/viz-store';
import { createMeshUrlModifier } from '../../utils/urdf-mesh-resolver';
import { registerMainUrdfRobot } from '../../utils/viewer-robot-registry';
import {
  applyCollisionVisibility,
  applyGhostVisualStyle,
  applyVisualOpacity,
  attachInertiaMarkers,
  attachJointAxisMarkers,
  disposeOverlayMarkers,
  ensureCollisionMaterialsPrepared,
  ensureVisualMaterialsPrepared,
} from './viz-overlays';

const Z_UP_TO_Y_UP = -Math.PI / 2;

export interface UrdfModelProps {
  urdfText: string;
  urdfFileName: string;
  meshAssets: Map<string, Uint8Array>;
  jointNames: string[];
  jointPositions: number[];
  showCollision: boolean;
  showInertia: boolean;
  modelOpacity: number;
  showJointAxes: boolean;
  ghost?: boolean;
  onRobotReady?: (robot: URDFRobot | null) => void;
}

export function UrdfModel({
  urdfText,
  urdfFileName,
  meshAssets,
  jointNames,
  jointPositions,
  showCollision,
  showInertia,
  modelOpacity,
  showJointAxes,
  ghost = false,
  onRobotReady,
}: UrdfModelProps) {
  const blobUrlsRef = useRef<string[]>([]);
  const robotRef = useRef<URDFRobot | null>(null);
  const styleRef = useRef({ ghost, opacity: 0.7 });
  const jointAxisMarkersRef = useRef<ReturnType<typeof attachJointAxisMarkers>>([]);
  const inertiaMarkersRef = useRef<ReturnType<typeof attachInertiaMarkers>>([]);

  const visualOpacity = ghost ? computeGhostOpacity(modelOpacity) : modelOpacity;
  styleRef.current = { ghost, opacity: visualOpacity };

  const robot = useMemo(() => {
    blobUrlsRef.current.forEach((u) => URL.revokeObjectURL(u));
    blobUrlsRef.current = [];

    const manager = new THREE.LoadingManager();
    manager.onLoad = () => {
      const parsed = robotRef.current;
      if (!parsed) return;
      const { ghost: isGhost, opacity } = styleRef.current;
      if (isGhost) {
        applyGhostVisualStyle(parsed, opacity);
      } else {
        applyVisualOpacity(parsed, opacity);
      }
    };

    const loader = new URDFLoader(manager);
    loader.parseCollision = true;
    const urlModifier = createMeshUrlModifier(urdfFileName, meshAssets);
    const trackingModifier = (url: string) => {
      const resolved = urlModifier(url);
      if (resolved.startsWith('blob:')) {
        blobUrlsRef.current.push(resolved);
      }
      return resolved;
    };
    loader.manager.setURLModifier(trackingModifier);

    const parsed = loader.parse(urdfText) as URDFRobot;
    parsed.rotation.x = Z_UP_TO_Y_UP;
    parsed.traverse((child) => {
      child.castShadow = !ghost;
      child.receiveShadow = !ghost;
      const linkName = (child as { name?: string }).name;
      if (linkName && SPHERE_PAYLOAD_LINK_PATTERN.test(linkName)) {
        child.traverse((mesh) => {
          const m = mesh as THREE.Mesh;
          if (m.isMesh) {
            m.material = new THREE.MeshStandardMaterial({
              color: SPHERE_PAYLOAD_VISUAL_COLOR,
              transparent: true,
              opacity: 0.9,
            });
          }
        });
      }
    });
    ensureVisualMaterialsPrepared(parsed);
    ensureCollisionMaterialsPrepared(parsed);
    robotRef.current = parsed;
    return parsed;
  }, [ghost, urdfText, urdfFileName, meshAssets]);

  const overlayCollision = ghost ? false : showCollision;
  const overlayInertia = ghost ? false : showInertia;
  const overlayJointAxes = ghost ? false : showJointAxes;

  useEffect(() => {
    if (ghost) {
      onRobotReady?.(robot);
      return () => onRobotReady?.(null);
    }
    registerMainUrdfRobot(robot);
    onRobotReady?.(robot);
    return () => {
      registerMainUrdfRobot(null);
      onRobotReady?.(null);
    };
  }, [ghost, onRobotReady, robot]);

  useEffect(() => {
    return () => {
      blobUrlsRef.current.forEach((u) => URL.revokeObjectURL(u));
      blobUrlsRef.current = [];
    };
  }, []);

  useEffect(() => {
    for (let i = 0; i < jointNames.length; i++) {
      robot.setJointValue(jointNames[i], jointPositions[i] ?? 0);
    }
  }, [robot, jointNames, jointPositions]);

  useEffect(() => {
    if (ghost) {
      applyGhostVisualStyle(robot, visualOpacity);
    } else {
      applyVisualOpacity(robot, visualOpacity);
    }
  }, [ghost, robot, visualOpacity]);

  useEffect(() => {
    applyCollisionVisibility(robot, overlayCollision);
  }, [robot, overlayCollision]);

  useEffect(() => {
    disposeOverlayMarkers(jointAxisMarkersRef.current);
    jointAxisMarkersRef.current = [];
    if (overlayJointAxes) {
      jointAxisMarkersRef.current = attachJointAxisMarkers(robot);
    }
    return () => {
      disposeOverlayMarkers(jointAxisMarkersRef.current);
      jointAxisMarkersRef.current = [];
    };
  }, [robot, overlayJointAxes]);

  useEffect(() => {
    disposeOverlayMarkers(inertiaMarkersRef.current);
    inertiaMarkersRef.current = [];
    if (overlayInertia) {
      inertiaMarkersRef.current = attachInertiaMarkers(robot);
    }
    return () => {
      disposeOverlayMarkers(inertiaMarkersRef.current);
      inertiaMarkersRef.current = [];
    };
  }, [robot, overlayInertia]);

  return <primitive object={robot} />;
}
