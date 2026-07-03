import { useCallback, useEffect, useLayoutEffect, useRef } from 'react';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import type { OrbitControls as OrbitControlsImpl } from 'three-stdlib';
import type { URDFRobot } from 'urdf-loader';
import type { Vec3 } from '../core/trajectory';
import { useEeIkOptional } from '../contexts/ee-ik-context';
import { useSessionStore } from '../stores/session-store';
import {
  getMainUrdfRobot,
  onMainUrdfRobotChange,
} from '../utils/viewer-robot-registry';
import { worldToUrdfTarget } from '../viewer/ee-kinematics';
import { computeGizmoWorldPose } from '../viewer/ee-gizmo-sync';

const LIVE_IK_THROTTLE_MS = 40;
const GIZMO_SIZE = 0.72;

/**
 * Imperative end-effector gizmo (proxy + native TransformControls).
 * Translate + rotate controls share one proxy (position + quaternion).
 */
export function EndEffectorControlsR3f() {
  const { camera, gl, scene, controls: orbitControls } = useThree();

  const eeTarget = useSessionStore((s) => s.eeTarget);
  const eeTargetQuat = useSessionStore((s) => s.eeTargetQuat);
  const endEffectorLink = useSessionStore((s) => s.endEffectorLink);
  const jointPositions = useSessionStore((s) => s.jointPositions);
  const ikEnabled = useSessionStore((s) => s.ikEnabled);
  const controlLayer = useSessionStore((s) => s.controlLayer);
  const eeTargetDirty = useSessionStore((s) => s.eeTargetDirty);
  const eeGizmoSyncVersion = useSessionStore((s) => s.eeGizmoSyncVersion);
  const setEeTarget = useSessionStore((s) => s.setEeTarget);
  const setEeTargetQuat = useSessionStore((s) => s.setEeTargetQuat);
  const setEeTargetDirty = useSessionStore((s) => s.setEeTargetDirty);
  const setIkDragActive = useSessionStore((s) => s.setIkDragActive);

  const eeIk = useEeIkOptional();

  const proxyRef = useRef<THREE.Object3D | null>(null);
  const controlsTranslateRef = useRef<TransformControls | null>(null);
  const controlsRotateRef = useRef<TransformControls | null>(null);
  const axesHelperRef = useRef<THREE.AxesHelper | null>(null);
  const draggingRef = useRef(false);
  const simWasRunningRef = useRef(false);
  const lastSolveAtRef = useRef(0);
  const mainRobotRef = useRef<URDFRobot | null>(null);
  const prevEndEffectorLinkRef = useRef(endEffectorLink);

  const enabled = ikEnabled && controlLayer === 'ee';

  const apiRef = useRef({
    eeIk,
    ikEnabled,
    setEeTarget,
    setEeTargetQuat,
    setEeTargetDirty,
    setIkDragActive,
  });
  apiRef.current = {
    eeIk,
    ikEnabled,
    setEeTarget,
    setEeTargetQuat,
    setEeTargetDirty,
    setIkDragActive,
  };

  const setOrbitEnabled = useCallback(
    (on: boolean) => {
      const oc = orbitControls as OrbitControlsImpl | null;
      if (oc) oc.enabled = on;
    },
    [orbitControls],
  );

  const readTargetFromProxy = useCallback((): {
    fk: Vec3;
    sceneWorld: [number, number, number];
    sceneQuaternion: [number, number, number, number];
  } | null => {
    const proxy = proxyRef.current;
    if (!proxy) return null;
    const { x, y, z } = proxy.position;
    const { x: qx, y: qy, z: qz, w: qw } = proxy.quaternion;
    return {
      sceneWorld: [x, y, z],
      sceneQuaternion: [qx, qy, qz, qw],
      fk: worldToUrdfTarget([x, y, z]),
    };
  }, []);

  const runLiveIk = useCallback(
    (
      target: Vec3,
      sceneWorld: [number, number, number],
      sceneQuaternion: [number, number, number, number],
      opts: { liveDrag?: boolean; dragEnd?: boolean },
    ) => {
      const { eeIk: ik, ikEnabled: ikOn } = apiRef.current;
      if (!ik || !ikOn) return;
      void ik.solveEeIkLive(target, {
        ...opts,
        targetSceneWorld: sceneWorld,
        targetSceneQuaternion: sceneQuaternion,
      });
    },
    [],
  );

  const applySampleToStore = useCallback((sample: { fk: Vec3 }) => {
    apiRef.current.setEeTarget(sample.fk);
    apiRef.current.setEeTargetDirty(true);
  }, []);

  const onDragMove = useCallback(() => {
    const tc = controlsTranslateRef.current;
    const rc = controlsRotateRef.current;
    const dragging = tc?.dragging || rc?.dragging;
    if (!dragging || !apiRef.current.ikEnabled) return;

    const now = performance.now();
    if (now - lastSolveAtRef.current < LIVE_IK_THROTTLE_MS) return;
    lastSolveAtRef.current = now;

    const sample = readTargetFromProxy();
    if (!sample) return;

    applySampleToStore(sample);
    runLiveIk(sample.fk, sample.sceneWorld, sample.sceneQuaternion, { liveDrag: true });
  }, [applySampleToStore, readTargetFromProxy, runLiveIk]);

  const onDragEnd = useCallback(async () => {
    if (!draggingRef.current) return;
    draggingRef.current = false;
    setOrbitEnabled(true);

    const sample = readTargetFromProxy();
    if (!sample) {
      apiRef.current.setIkDragActive(false);
      return;
    }

    applySampleToStore(sample);

    const proxy = proxyRef.current;
    if (proxy) {
      proxy.position.set(sample.sceneWorld[0], sample.sceneWorld[1], sample.sceneWorld[2]);
      proxy.quaternion.set(
        sample.sceneQuaternion[0],
        sample.sceneQuaternion[1],
        sample.sceneQuaternion[2],
        sample.sceneQuaternion[3],
      );
      proxy.updateMatrixWorld(true);
    }

    const { eeIk: ik, ikEnabled: ikOn } = apiRef.current;
    if (ik && ikOn) {
      const result = await ik.solveEeIkLive(sample.fk, {
        dragEnd: true,
        targetSceneWorld: sample.sceneWorld,
        targetSceneQuaternion: sample.sceneQuaternion,
      });
      apiRef.current.setIkDragActive(false);
      if (!result.converged) return;
      const mode = useSessionStore.getState().controlMode;
      if (simWasRunningRef.current && mode === 'realtime') {
        void ik.onEeDragCommit?.({ simWasRunning: true });
      }
    } else {
      apiRef.current.setIkDragActive(false);
    }
  }, [applySampleToStore, readTargetFromProxy, setOrbitEnabled]);

  const onDragMoveRef = useRef(onDragMove);
  const onDragEndRef = useRef(onDragEnd);
  onDragMoveRef.current = onDragMove;
  onDragEndRef.current = onDragEnd;

  const syncProxyFromFk = useCallback(
    (opts?: { preferTarget?: boolean }) => {
      if (draggingRef.current) return;
      const proxy = proxyRef.current;
      if (!proxy) return;

      const state = useSessionStore.getState();
      if (state.interpolationActive && (opts?.preferTarget ?? eeTargetDirty)) {
        return;
      }

      const preferTarget = opts?.preferTarget ?? eeTargetDirty;
      const pose = computeGizmoWorldPose(
        mainRobotRef.current ?? getMainUrdfRobot(),
        endEffectorLink,
        eeTarget,
        eeTargetQuat,
        { preferTarget },
      );
      if (!pose) return;

      proxy.position.copy(pose.position);
      proxy.quaternion.copy(pose.quaternion);
      proxy.updateMatrixWorld(true);
    },
    [eeTarget, eeTargetDirty, eeTargetQuat, endEffectorLink],
  );

  const attachTransformControl = useCallback(
    (tc: TransformControls, mode: 'translate' | 'rotate') => {
      tc.setSpace('world');
      tc.setMode(mode);
      tc.size = GIZMO_SIZE;

      const onDraggingChanged = (e: THREE.Event & { value: boolean }) => {
        if (mode === 'translate' && controlsRotateRef.current) {
          controlsRotateRef.current.enabled = !e.value;
        }
        if (mode === 'rotate' && controlsTranslateRef.current) {
          controlsTranslateRef.current.enabled = !e.value;
        }

        setOrbitEnabled(!e.value);
        if (e.value) {
          draggingRef.current = true;
          simWasRunningRef.current = useSessionStore.getState().simStatus === 'running';
          apiRef.current.setIkDragActive(true);
          lastSolveAtRef.current = 0;
        } else if (draggingRef.current) {
          onDragEndRef.current();
        }
      };

      const onChange = () => {
        if (tc.dragging) onDragMoveRef.current();
      };

      tc.addEventListener('dragging-changed', onDraggingChanged as never);
      tc.addEventListener('change', onChange);
      return () => {
        tc.removeEventListener('dragging-changed', onDraggingChanged as never);
        tc.removeEventListener('change', onChange);
      };
    },
    [setOrbitEnabled],
  );

  useEffect(() => {
    const proxy = new THREE.Object3D();
    const axes = new THREE.AxesHelper(0.08);
    proxy.add(axes);
    proxyRef.current = proxy;
    axesHelperRef.current = axes;

    const tcTranslate = new TransformControls(camera, gl.domElement);
    const tcRotate = new TransformControls(camera, gl.domElement);
    controlsTranslateRef.current = tcTranslate;
    controlsRotateRef.current = tcRotate;

    const cleanupTranslate = attachTransformControl(tcTranslate, 'translate');
    const cleanupRotate = attachTransformControl(tcRotate, 'rotate');

    const helperT = tcTranslate.getHelper();
    const helperR = tcRotate.getHelper();
    scene.add(proxy);
    scene.add(helperT);
    scene.add(helperR);

    return () => {
      cleanupTranslate();
      cleanupRotate();
      tcTranslate.detach();
      tcRotate.detach();
      tcTranslate.dispose();
      tcRotate.dispose();
      scene.remove(helperT);
      scene.remove(helperR);
      proxy.remove(axes);
      axes.dispose();
      scene.remove(proxy);
      controlsTranslateRef.current = null;
      controlsRotateRef.current = null;
      proxyRef.current = null;
      axesHelperRef.current = null;
    };
  }, [attachTransformControl, camera, gl, scene]);

  useEffect(() => {
    const tcT = controlsTranslateRef.current;
    const tcR = controlsRotateRef.current;
    const proxy = proxyRef.current;
    if (!tcT || !tcR || !proxy) return;

    tcT.camera = camera;
    tcR.camera = camera;

    if (enabled) {
      tcT.attach(proxy);
      tcR.attach(proxy);
      tcT.enabled = true;
      tcR.enabled = true;
    } else {
      tcT.detach();
      tcR.detach();
      tcT.enabled = false;
      tcR.enabled = false;
    }
  }, [camera, enabled]);

  useEffect(() => {
    return onMainUrdfRobotChange((robot) => {
      mainRobotRef.current = robot;
      syncProxyFromFk();
    });
  }, [syncProxyFromFk]);

  useLayoutEffect(() => {
    const linkChanged = prevEndEffectorLinkRef.current !== endEffectorLink;
    prevEndEffectorLinkRef.current = endEffectorLink;
    syncProxyFromFk({ preferTarget: linkChanged ? false : eeTargetDirty });
  }, [endEffectorLink, eeGizmoSyncVersion, eeTargetDirty, syncProxyFromFk]);

  useEffect(() => {
    syncProxyFromFk({ preferTarget: eeTargetDirty });
  }, [
    eeGizmoSyncVersion,
    jointPositions,
    endEffectorLink,
    ikEnabled,
    controlLayer,
    eeTargetDirty,
    eeTarget,
    eeTargetQuat,
    syncProxyFromFk,
  ]);

  if (controlLayer !== 'ee') return null;

  return null;
}
