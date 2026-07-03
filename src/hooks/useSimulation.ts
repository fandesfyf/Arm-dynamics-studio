import { useCallback, useEffect, useRef } from 'react';
import { CONTROLLER_OMEGA } from '../core/controller';
import { vecGet } from '../types/mujoco';
import { RobotSession, createForwardKinematics, type ForwardKinematics } from '../core/robot-session';
import { createSimulation, type IKSolver, type SimulationEngine } from '../core/simulation';
import { InverseKinematics } from '../core/inverse-kinematics';
import { createJointMapAdapter } from '../pinocchio/joint-map-adapter';
import { Trajectory } from '../core/trajectory';
import { asTrajectorySampler } from '../core/planner';
import { exportToCsv, csvToBlob } from '../export/csv-exporter';
import { useSessionStore, type RobotInfo } from '../stores/session-store';
import type { PayloadRecord, Wrench6 } from '../core/payload-editor';
import { loadDefaultBipedUpperBody } from '../utils/biped-default-loader';
import { ensureFixedBase, resolveEndEffectorJointName } from '../utils/urdf-base-fixture';
import { finalizeUrdfForMujoco } from '../utils/urdf-sanitize';
import { actuatedJointsToQpos, applyActuatedGainsToController, nvGainsToActuated, qposToActuatedJoints } from '../utils/joint-qpos';
import { ClosedChainIkBridge } from '../ik/closed-chain-ik-bridge';
import { createHybridIkSolver } from '../ik/hybrid-ik-solver';
import type { SimulationStepState } from '../types/simulation';
import type { Quat, Vec3 } from '../core/trajectory';
import { createMotionTargetId, type MotionTarget, resolveInterpProfile } from '../types/motion-target';
import type { EeIkLiveResult } from '../contexts/ee-ik-context';
import {
  applyAllJointAngles,
  readEeSceneWorldForJointAngles,
  readEeWorldFromRobot,
  readEeWorldPoseFromRobot,
  urdfTargetToWorld,
  worldToUrdfTarget,
} from '../viewer/ee-kinematics';
import {
  getIkUrdfRobot,
  getMainUrdfRobot,
  getReferenceUrdfRobot,
  onMainUrdfRobotChange,
  onReferenceUrdfRobotChange,
  readEndEffectorSceneWorld,
} from '../utils/viewer-robot-registry';

/** Max UI refresh rate during realtime / interpolation loops (~12 Hz). */
const UI_UPDATE_INTERVAL_MS = 80;

interface LastGoodSnapshot {
  urdfText: string;
  urdfFileName: string;
  robotInfo: RobotInfo;
  jointPositions: number[];
  baseLink: string;
  endEffectorLink: string;
  payloadRecords: PayloadRecord[];
  externalWrenches: Map<string, Wrench6>;
  session: RobotSession;
  engine: SimulationEngine;
  ik: InverseKinematics;
  ikSolver: IKSolver;
  fk: ForwardKinematics | null;
}

function captureSnapshot(
  session: RobotSession,
  engine: SimulationEngine,
  ik: InverseKinematics,
  ikSolver: IKSolver,
  fk: ForwardKinematics | null,
): LastGoodSnapshot {
  const state = useSessionStore.getState();
  return {
    urdfText: state.urdfText ?? '',
    urdfFileName: state.urdfFileName ?? 'robot.urdf',
    robotInfo: state.robotInfo!,
    jointPositions: [...state.jointPositions],
    baseLink: state.baseLink,
    endEffectorLink: state.endEffectorLink,
    payloadRecords: [...state.payloadRecords],
    externalWrenches: new Map(state.externalWrenches),
    session,
    engine,
    ik,
    ikSolver,
    fk,
  };
}

function restoreSnapshot(snapshot: LastGoodSnapshot, errorMessage: string): void {
  useSessionStore.setState({
    robotInfo: snapshot.robotInfo,
    jointPositions: [...snapshot.jointPositions],
    jointTargets: [...snapshot.jointPositions],
    urdfText: snapshot.urdfText,
    urdfFileName: snapshot.urdfFileName,
    baseLink: snapshot.baseLink,
    endEffectorLink: snapshot.endEffectorLink,
    payloadRecords: [...snapshot.payloadRecords],
    externalWrenches: new Map(snapshot.externalWrenches),
    loading: false,
    loadingMessage: '',
    simStatus: 'error',
    simMessage: errorMessage,
  });
}

function downloadBlob(blob: Blob, filename: string): void {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  a.click();
  URL.revokeObjectURL(url);
}

export function useSimulation() {
  const sessionRef = useRef<RobotSession | null>(null);
  const engineRef = useRef<SimulationEngine | null>(null);
  const ikRef = useRef<InverseKinematics | null>(null);
  const ikSolverRef = useRef<IKSolver | null>(null);
  const fkRef = useRef<ForwardKinematics | null>(null);
  const closedChainIkRef = useRef<ClosedChainIkBridge | null>(null);
  const cancelRef = useRef(false);
  const pauseRef = useRef(false);
  const loopActiveRef = useRef(false);
  const rafIdRef = useRef<number | null>(null);
  const meshesRef = useRef<Map<string, Uint8Array>>(new Map());
  const lastUiUpdateRef = useRef(0);
  const lastGoodSnapshotRef = useRef<LastGoodSnapshot | null>(null);
  const pendingEeSyncRef = useRef(false);

  const {
    setLoading,
    setRobotLoaded,
    setLoadError,
    setJointPositions,
    setSimStatus,
    updateRecorder,
    syncSimFrame,
    setSimRuntime,
    setPaused,
    trajectoryWaypoints,
    endEffectorLink,
    baseLink,
    controlDt,
    robotInfo,
    setBaseLink,
    setEndEffectorLink,
    setEeFkPos,
    setReferenceFromIk,
    setIkLiveStatus,
    setInterpolationActive,
  } = useSessionStore();

  const eeTarget = useSessionStore((s) => s.eeTarget);
  const eeTargetDirty = useSessionStore((s) => s.eeTargetDirty);
  const jointPositions = useSessionStore((s) => s.jointPositions);
  const controlLayer = useSessionStore((s) => s.controlLayer);
  const ikEnabled = useSessionStore((s) => s.ikEnabled);
  const ikDragActive = useSessionStore((s) => s.ikDragActive);

  const refreshEeFkPos = useCallback(() => {
    const session = sessionRef.current;
    const fk = fkRef.current;
    if (!session || !fk) {
      setEeFkPos(null);
      return;
    }
    const q = actuatedJointsToQpos(session, useSessionStore.getState().jointPositions);
    const ee = fk.compute(q);
    setEeFkPos([...ee.pos] as Vec3);
  }, [setEeFkPos]);

  const configureEndEffector = useCallback(
    (link: string, urdfText: string, session: RobotSession, ik: InverseKinematics) => {
      const eeJoint =
        resolveEndEffectorJointName(urdfText, session.jointNames, link) ??
        session.jointNames[session.jointNames.length - 1]!;
      ik.setEndEffector(eeJoint);
      fkRef.current = createForwardKinematics(
        session.pinocchioBundle,
        session.jointMap,
        urdfText,
        link,
      );
    },
    [],
  );

  const rebuildClosedChainIk = useCallback(
    (
      robot: import('urdf-loader').URDFRobot | null,
      eeLink: string,
      jointNames: string[],
      rootLink?: string,
    ) => {
      closedChainIkRef.current?.dispose();
      if (!robot) {
        closedChainIkRef.current = null;
        return;
      }
      const bridge = new ClosedChainIkBridge();
      const restoreDisplayJoints = robot === getMainUrdfRobot();
      const ok = bridge.rebuild(robot, eeLink, jointNames, {
        restoreDisplayJoints,
        rootLink: rootLink ?? useSessionStore.getState().baseLink,
      });
      closedChainIkRef.current = ok ? bridge : null;
      if (!ok) {
        console.warn('closed-chain-ik 不可用，将使用 Pinocchio IK');
      }
    },
    [],
  );

  const refreshHybridIkSolver = useCallback(() => {
    const session = sessionRef.current;
    const ik = ikRef.current;
    if (!session || !ik) return;
    ikSolverRef.current = createHybridIkSolver(session, ik, closedChainIkRef.current);
  }, []);

  const attachIkToViewerRobot = useCallback(() => {
    const session = sessionRef.current;
    if (!session) return;
    const robot = getIkUrdfRobot();
    const state = useSessionStore.getState();
    rebuildClosedChainIk(robot, state.endEffectorLink, session.jointNames, state.baseLink);
    refreshHybridIkSolver();
  }, [rebuildClosedChainIk, refreshHybridIkSolver]);

  const applyRecorderWindow = useCallback((engine: SimulationEngine) => {
    engine.recorder.setMaxDurationSec(useSessionStore.getState().recorderWindowSec);
  }, []);

  const syncRecorder = useCallback(
    (engine: SimulationEngine) => {
      applyRecorderWindow(engine);
      const times = engine.recorder.getTimes();
      updateRecorder(
        {
          sampleCount: engine.recorder.getNumFrames(),
          lastTime: times.length > 0 ? times[times.length - 1]! : null,
        },
        engine.recorder.toDict(),
      );
      setSimRuntime(engine.simTime, engine.recorder.getNumFrames());
    },
    [applyRecorderWindow, setSimRuntime, updateRecorder],
  );

  const pushSimUi = useCallback(
    (session: RobotSession, engine: SimulationEngine, force = false) => {
      const now = performance.now();
      if (!force && now - lastUiUpdateRef.current < UI_UPDATE_INTERVAL_MS) {
        return;
      }
      lastUiUpdateRef.current = now;
      applyRecorderWindow(engine);
      const times = engine.recorder.getTimes();
      syncSimFrame(
        qposToActuatedJoints(session),
        engine.simTime,
        engine.recorder.getNumFrames(),
        {
          sampleCount: engine.recorder.getNumFrames(),
          lastTime: times.length > 0 ? times[times.length - 1]! : null,
        },
        engine.recorder.toDict(),
      );
    },
    [applyRecorderWindow, syncSimFrame],
  );

  const cancelRafLoop = useCallback(() => {
    loopActiveRef.current = false;
    if (rafIdRef.current !== null) {
      cancelAnimationFrame(rafIdRef.current);
      rafIdRef.current = null;
    }
  }, []);

  const makeStepCallback = useCallback(
    (engine: SimulationEngine, stepIndex: { value: number }) => {
      return (_state: SimulationStepState) => {
        stepIndex.value += 1;
        const session = sessionRef.current;
        if (!session) return;
        pushSimUi(session, engine);
      };
    },
    [pushSimUi],
  );

  const solveEeIkLiveInternal = useCallback(
    async (
      target: Vec3,
      opts: {
        liveDrag?: boolean;
        dragEnd?: boolean;
        targetSceneWorld?: [number, number, number];
        targetSceneQuaternion?: [number, number, number, number];
      } = {},
    ): Promise<EeIkLiveResult> => {
      const session = sessionRef.current;
      const ikSolver = ikSolverRef.current;
      const fk = fkRef.current;
      const state = useSessionStore.getState();

      if (!session || !ikSolver || !fk) {
        const fallback =
          state.referenceJointPositions.length > 0
            ? state.referenceJointPositions
            : state.jointPositions;
        const message = 'IK 未就绪';
        setIkLiveStatus('failed', message);
        return { converged: false, jointAngles: [...fallback], message };
      }

      if (!state.ikEnabled) {
        const message = 'IK 未启用';
        setIkLiveStatus('failed', message);
        return {
          converged: false,
          jointAngles:
            state.referenceJointPositions.length > 0
              ? [...state.referenceJointPositions]
              : [...state.jointPositions],
          message,
        };
      }

      const t0 = performance.now();
      setIkLiveStatus('solving');

      const seedJoints =
        state.referenceJointPositions.length > 0
          ? state.referenceJointPositions
          : state.jointPositions;
      const qInit = actuatedJointsToQpos(session, seedJoints);
      const eeQuat = opts.targetSceneQuaternion
        ? (opts.targetSceneQuaternion as Quat)
        : (fk.compute(qInit).quat as Quat);

      try {
        const ik = ikSolver.solve([...target], eeQuat, qInit, {
          liveDrag: opts.liveDrag,
          dragEnd: opts.dragEnd,
          goalMode: state.ikGoalMode,
          weights: state.ikWeights,
          targetSceneWorld: opts.targetSceneWorld,
          targetSceneQuaternion: opts.targetSceneQuaternion,
        });

        if (!ik.converged) {
          const message = ik.message ?? 'IK 无解，请调整目标位置';
          const ms = performance.now() - t0;
          setIkLiveStatus('failed', message, ms);
          console.info('[IK]', {
            target,
            endEffectorLink: state.endEffectorLink,
            converged: false,
            message,
            ms: Math.round(ms),
          });
          return {
            converged: false,
            jointAngles: [...seedJoints],
            message,
          };
        }

        const joints = qposToActuatedJoints(session, ik.q);
        const ms = performance.now() - t0;
        setReferenceFromIk(joints);
        const visRobot = getReferenceUrdfRobot() ?? getMainUrdfRobot();
        if (visRobot) {
          applyAllJointAngles(visRobot, session.jointNames, joints);
        }
        setIkLiveStatus('converged', null, ms);
        console.info('[IK]', {
          target,
          endEffectorLink: state.endEffectorLink,
          converged: true,
          ms: Math.round(ms),
        });
        return { converged: true, jointAngles: joints };
      } catch (err) {
        const message = err instanceof Error ? err.message : 'IK 求解失败';
        const ms = performance.now() - t0;
        console.warn('solveEeIkLive failed:', err);
        setIkLiveStatus('failed', message, ms);
        console.info('[IK]', {
          target,
          endEffectorLink: state.endEffectorLink,
          converged: false,
          message,
          ms: Math.round(ms),
        });
        return { converged: false, jointAngles: [...seedJoints], message };
      }
    },
    [setIkLiveStatus, setReferenceFromIk],
  );

  const syncEeCommandFromFk = useCallback(async (force = false) => {
    const session = sessionRef.current;
    if (!session) return;

    const state = useSessionStore.getState();
    if (!force && (state.interpolationActive || state.eeTargetDirty)) {
      return;
    }
    const mainRobot = getMainUrdfRobot();
    if (!mainRobot) {
      pendingEeSyncRef.current = true;
      return;
    }

    const world = readEeWorldFromRobot(mainRobot, state.endEffectorLink);
    if (!world) {
      pendingEeSyncRef.current = true;
      return;
    }

    pendingEeSyncRef.current = false;
    const target = [...worldToUrdfTarget(world)] as Vec3;
    const visualWorld: [number, number, number] = [world.x, world.y, world.z];
    const worldPose = readEeWorldPoseFromRobot(mainRobot, state.endEffectorLink);
    const fk = fkRef.current ?? session.forwardKinematics;
    const eeQuat = fk.compute(actuatedJointsToQpos(session, state.jointPositions)).quat as Quat;

    useSessionStore.setState({
      eeTarget: target,
      eeTargetQuat: [...eeQuat] as Quat,
      referenceJointPositions: [...state.jointPositions],
      eeFkPos: target,
      eeTargetDirty: false,
      ikLiveError: null,
      ikLiveMessage: null,
    });
    useSessionStore.getState().bumpEeGizmoSyncVersion();

    if (state.ikEnabled && state.controlLayer === 'ee') {
      await solveEeIkLiveInternal(target, {
        liveDrag: false,
        targetSceneWorld: visualWorld,
        targetSceneQuaternion: worldPose
          ? [worldPose.quaternion.x, worldPose.quaternion.y, worldPose.quaternion.z, worldPose.quaternion.w]
          : undefined,
      });
    }
  }, [solveEeIkLiveInternal]);

  const resetReferencePose = useCallback(() => {
    void syncEeCommandFromFk(true);
  }, [syncEeCommandFromFk]);

  const resetGizmoToCurrent = resetReferencePose;

  const solveEeIkLive = solveEeIkLiveInternal;

  const resolveQDesired = useCallback((): Float64Array | null => {
    const session = sessionRef.current;
    const engine = engineRef.current;
    const ikSolver = ikSolverRef.current;
    if (!session || !engine) return null;

    const state = useSessionStore.getState();
    if (state.controlLayer === 'joint') {
      return actuatedJointsToQpos(session, state.jointTargets);
    }

    if (!ikSolver) {
      console.warn('IK 求解器未初始化，末端控制不可用');
      return null;
    }
    const qInit = vecGet(session.data.qpos, session.nq);
    const fk = fkRef.current ?? session.forwardKinematics;
    const eeQuat = fk.compute(qInit).quat;
    const ik = ikSolver.solve([...state.eeTarget], eeQuat, qInit);
    return ik.converged ? ik.q : qInit;
  }, []);

  const syncExternalWrenches = useCallback((engine: SimulationEngine) => {
    engine.externalWrenches = new Map(useSessionStore.getState().externalWrenches);
  }, []);

  const syncExternalWrenchesFromStore = useCallback(() => {
    const engine = engineRef.current;
    if (engine) {
      syncExternalWrenches(engine);
    }
  }, [syncExternalWrenches]);

  const runSimulationLoop = useCallback(
    (resolveDesired: () => Float64Array | null, statusMessage?: string) => {
      const engine = engineRef.current;
      const session = sessionRef.current;
      if (!engine || !session) return;

      cancelRafLoop();
      cancelRef.current = false;
      pauseRef.current = false;
      setPaused(false);
      loopActiveRef.current = true;
      engine.controlDt = useSessionStore.getState().controlDt;
      applyRecorderWindow(engine);
      syncExternalWrenches(engine);
      engine.isRunning = true;

      const mode = useSessionStore.getState().controlMode;
      const layer = useSessionStore.getState().controlLayer;
      const defaultMessage =
        mode === 'interpolate'
          ? '关节目标保持仿真中…'
          : layer === 'joint'
            ? '实时关节跟踪仿真中…'
            : '实时末端 IK 跟踪仿真中…';
      setSimStatus('running', statusMessage ?? defaultMessage);

      let accumulated = 0;
      let lastTs = performance.now();
      lastUiUpdateRef.current = 0;

      const tick = (ts: number) => {
        if (!loopActiveRef.current || cancelRef.current) {
          engine.isRunning = false;
          rafIdRef.current = null;
          return;
        }

        if (pauseRef.current) {
          lastTs = ts;
          rafIdRef.current = requestAnimationFrame(tick);
          return;
        }

        const frameDt = Math.min((ts - lastTs) / 1000, 0.1);
        lastTs = ts;
        accumulated += frameDt;

        const dt = engine.controlDt;
        let stepped = false;
        while (accumulated >= dt && loopActiveRef.current && !cancelRef.current) {
          accumulated -= dt;
          const qDesired = resolveDesired();
          if (!qDesired) break;

          engine.stepHoldTarget(qDesired, () => {
            stepped = true;
          });
        }

        if (stepped) {
          pushSimUi(session, engine);
        }

        if (loopActiveRef.current && !cancelRef.current) {
          rafIdRef.current = requestAnimationFrame(tick);
        } else {
          engine.isRunning = false;
          rafIdRef.current = null;
        }
      };

      rafIdRef.current = requestAnimationFrame(tick);
    },
    [
      applyRecorderWindow,
      cancelRafLoop,
      pushSimUi,
      setPaused,
      setSimStatus,
      syncExternalWrenches,
    ],
  );

  const runHoldLoop = useCallback(
    (statusMessage?: string) => {
      const session = sessionRef.current;
      if (!session) return;
      runSimulationLoop(() => {
        const state = useSessionStore.getState();
        return actuatedJointsToQpos(session, state.commandedJointPositions);
      }, statusMessage);
    },
    [runSimulationLoop],
  );

  const runContinuousLoop = useCallback(
    (statusMessage?: string) => {
      runSimulationLoop(resolveQDesired, statusMessage);
    },
    [resolveQDesired, runSimulationLoop],
  );

  const ensureSimRunning = useCallback(() => {
    if (!loopActiveRef.current) {
      const mode = useSessionStore.getState().controlMode;
      if (mode === 'interpolate') {
        runHoldLoop();
      } else {
        runContinuousLoop();
      }
    }
  }, [runContinuousLoop, runHoldLoop]);

  const loadRobot = useCallback(
    async (urdfText: string, urdfFileName: string, meshes?: Map<string, Uint8Array>, forceBaseLink?: string) => {
      setLoading(true, '正在加载 MuJoCo + Pinocchio…');
      cancelRef.current = true;
      cancelRafLoop();
      pauseRef.current = false;
      setPaused(false);

      const prevSession = sessionRef.current;
      const prevEngine = engineRef.current;
      const prevIk = ikRef.current;
      const prevIkSolver = ikSolverRef.current;
      const prevFk = fkRef.current;
      const rollbackSnapshot =
        prevSession && prevEngine && prevIk && prevIkSolver
          ? captureSnapshot(prevSession, prevEngine, prevIk, prevIkSolver, prevFk)
          : lastGoodSnapshotRef.current;

      let newSession: RobotSession | null = null;

      try {
        const prevUrdfFileName = useSessionStore.getState().urdfFileName;
        const meshMap = meshes ?? new Map();
        meshesRef.current = meshMap;

        const resolvedBase = forceBaseLink ?? baseLink;
        const fixture = ensureFixedBase(finalizeUrdfForMujoco(urdfText), resolvedBase || undefined);
        const resolvedBaseLink = fixture.baseLink;
        const resolvedEeLink =
          endEffectorLink !== 'ee_link'
            ? endEffectorLink
            : fixture.endEffectorLink ?? endEffectorLink;

        setBaseLink(resolvedBaseLink);
        if (fixture.endEffectorLink && endEffectorLink === 'ee_link') {
          setEndEffectorLink(fixture.endEffectorLink);
        }

        const session = await RobotSession.create({
          urdfXml: fixture.urdfText,
          urdfFileName,
          meshes: meshMap,
          endEffectorLink: resolvedEeLink,
          baseLink: resolvedBaseLink,
        });
        newSession = session;

        const engine = createSimulation(session);
        engine.controlDt = controlDt;
        applyRecorderWindow(engine);
        const pinData = new session.pinocchioBundle.pin.Data(session.pinocchioBundle.model);
        const jointMap = createJointMapAdapter(session.jointMap);
        const ik = new InverseKinematics(
          session.pinocchioBundle.pin,
          session.pinocchioBundle.model,
          pinData,
          jointMap,
          session.pinocchioBundle.jointNames,
        );
        if (session.jointNames.length > 0) {
          configureEndEffector(
            useSessionStore.getState().endEffectorLink,
            fixture.urdfText,
            session,
            ik,
          );
        } else {
          fkRef.current = session.forwardKinematics;
        }

        rebuildClosedChainIk(
          getIkUrdfRobot(),
          useSessionStore.getState().endEffectorLink,
          session.jointNames,
          useSessionStore.getState().baseLink,
        );
        const ikSolver = createHybridIkSolver(session, ik, closedChainIkRef.current);
        const newFk = fkRef.current;

        prevSession?.dispose();
        sessionRef.current = session;
        engineRef.current = engine;
        ikRef.current = ik;
        ikSolverRef.current = ikSolver;
        fkRef.current = newFk;
        newSession = null;

        useSessionStore.getState().setEeGizmoVisible(false);

        engine.reset();
        const qpos = qposToActuatedJoints(session);
        const qInit = vecGet(session.data.qpos, session.nq);
        const kdDamping = useSessionStore.getState().controllerKdDamping;
        engine.recomputeAutoGains(qInit, { kdDamping, omega: CONTROLLER_OMEGA });
        const fk = fkRef.current ?? session.forwardKinematics;
        const ee = fk.compute(vecGet(session.data.qpos, session.nq));
        const robotName =
          urdfText.match(/<robot\s+name="([^"]+)"/)?.[1] ?? urdfFileName.replace(/\.urdf$/i, '');

        setSimRuntime(0, 0);
        const gains = nvGainsToActuated(session, engine.getGains().kp, engine.getGains().kd);
        useSessionStore.getState().setAllJointGains(gains.kp, gains.kd);
        setRobotLoaded({
          robotInfo: {
            name: robotName,
            dof: session.jointNames.length,
            jointNames: session.jointNames,
            lowerLimits: session.pinocchioBundle.lowerLimits,
            upperLimits: session.pinocchioBundle.upperLimits,
            eePos: ee.pos as Vec3,
            eeQuat: ee.quat as Quat,
          },
          jointPositions: qpos,
          urdfText: fixture.urdfText,
          urdfFileName,
          meshAssets: meshMap,
        });

        if (prevUrdfFileName != null && prevUrdfFileName !== urdfFileName) {
          useSessionStore.getState().clearPayloadRecords();
          useSessionStore.getState().clearExternalWrenches();
        }

        if (sessionRef.current && engineRef.current && ikRef.current && ikSolverRef.current) {
          lastGoodSnapshotRef.current = captureSnapshot(
            sessionRef.current,
            engineRef.current,
            ikRef.current,
            ikSolverRef.current,
            fkRef.current,
          );
        }
      } catch (e) {
        newSession?.dispose();

        const msg = e instanceof Error ? e.message : String(e);
        if (rollbackSnapshot) {
          sessionRef.current = rollbackSnapshot.session;
          engineRef.current = rollbackSnapshot.engine;
          ikRef.current = rollbackSnapshot.ik;
          ikSolverRef.current = rollbackSnapshot.ikSolver;
          fkRef.current = rollbackSnapshot.fk;
          restoreSnapshot(rollbackSnapshot, msg);
        } else {
          sessionRef.current = null;
          engineRef.current = null;
          ikRef.current = null;
          ikSolverRef.current = null;
          fkRef.current = null;
          setLoadError(msg);
        }
        throw e instanceof Error ? e : new Error(msg);
      }
    },
    [
      applyRecorderWindow,
      baseLink,
      cancelRafLoop,
      configureEndEffector,
      controlDt,
      endEffectorLink,
      rebuildClosedChainIk,
      setBaseLink,
      setEndEffectorLink,
      setLoadError,
      setLoading,
      setPaused,
      setRobotLoaded,
      setSimRuntime,
    ],
  );

  const loadDefaultBiped = useCallback(async () => {
    const bundle = await loadDefaultBipedUpperBody();
    await loadRobot(bundle.urdfText, bundle.urdfFileName, bundle.meshes);
  }, [loadRobot]);

  const applyBaseLink = useCallback(
    async (link: string) => {
      const urdf = useSessionStore.getState().urdfText;
      const name = useSessionStore.getState().urdfFileName ?? 'robot.urdf';
      if (!urdf) return;
      setBaseLink(link);
      await loadRobot(urdf, name, meshesRef.current, link);
    },
    [loadRobot, setBaseLink],
  );

  const applyEndEffectorLink = useCallback(
    (link: string) => {
      const session = sessionRef.current;
      const ik = ikRef.current;
      const urdf = useSessionStore.getState().urdfText;
      if (!session || !ik || !urdf) {
        setEndEffectorLink(link);
        return;
      }

      setEndEffectorLink(link);
      useSessionStore.getState().setEeTargetDirty(false);
      try {
        configureEndEffector(link, urdf, session, ik);
        rebuildClosedChainIk(getIkUrdfRobot(), link, session.jointNames, useSessionStore.getState().baseLink);
        refreshHybridIkSolver();
        void syncEeCommandFromFk(true);
      } catch (e) {
        const msg = e instanceof Error ? e.message : String(e);
        setSimStatus('error', `末端 link 切换失败: ${msg}`);
      }
    },
    [configureEndEffector, rebuildClosedChainIk, refreshHybridIkSolver, setEndEffectorLink, setSimStatus, syncEeCommandFromFk],
  );

  const cancelSimulation = useCallback(() => {
    cancelRef.current = true;
    cancelRafLoop();
    pauseRef.current = false;
    setPaused(false);
    setInterpolationActive(false);
    const engine = engineRef.current;
    if (engine) {
      engine.isRunning = false;
    }
  }, [cancelRafLoop, setInterpolationActive, setPaused]);

  const pauseSimulation = useCallback(() => {
    const next = !pauseRef.current;
    pauseRef.current = next;
    setPaused(next);
  }, [setPaused]);

  const stopSimulation = useCallback(() => {
    cancelRef.current = true;
    cancelRafLoop();
    pauseRef.current = false;
    setPaused(false);
    setInterpolationActive(false);
    loopActiveRef.current = false;
    const engine = engineRef.current;
    const session = sessionRef.current;
    if (engine) {
      engine.isRunning = false;
    }
    if (engine && session) {
      const positions = qposToActuatedJoints(session);
      setJointPositions(positions);
      useSessionStore.getState().setCommandedJointPositions([...positions]);
      syncRecorder(engine);
      const times = engine.recorder.getTimes();
      const displayTime = times.length > 0 ? times[times.length - 1]! : engine.simTime;
      setSimRuntime(displayTime, engine.recorder.getNumFrames());
      setSimStatus('ready', '仿真已停止');
    }
  }, [
    cancelRafLoop,
    setInterpolationActive,
    setJointPositions,
    setPaused,
    setSimRuntime,
    setSimStatus,
    syncRecorder,
  ]);

  const resetRobotPose = useCallback(() => {
    cancelRef.current = true;
    cancelRafLoop();
    pauseRef.current = false;
    setPaused(false);
    setInterpolationActive(false);
    loopActiveRef.current = false;

    const engine = engineRef.current;
    const session = sessionRef.current;
    if (!engine || !session) return;

    engine.reset({ preserveRecorder: true });
    engine.isRunning = false;

    const positions = qposToActuatedJoints(session);
    useSessionStore.getState().setJointPositions(positions);
    useSessionStore.getState().setJointTargets([...positions]);
    useSessionStore.getState().setCommandedJointPositions([...positions]);
    useSessionStore.getState().setReferenceJointPositions([...positions]);

    const fk = fkRef.current ?? session.forwardKinematics;
    const ee = fk.compute(vecGet(session.data.qpos, session.nq));
    useSessionStore.setState({
      eeTarget: [...ee.pos] as Vec3,
      eeTargetQuat: [...ee.quat] as Quat,
      eeTargetDirty: false,
      eeFkPos: [...ee.pos] as Vec3,
    });
    useSessionStore.getState().bumpEeGizmoSyncVersion();

    syncRecorder(engine);
    setSimStatus('ready', '模型已重置到零位');
  }, [
    cancelRafLoop,
    setInterpolationActive,
    setPaused,
    setSimStatus,
    syncRecorder,
  ]);

  const resetRecorder = useCallback(() => {
    const engine = engineRef.current;
    if (!engine) return;
    engine.recorder.clear();
    engine.simTime = 0;
    useSessionStore.getState().setRecorderPaused(false);
    engine.recordingEnabled = true;
    syncRecorder(engine);
    setSimStatus('ready', '曲线数据已清空');
  }, [setSimStatus, syncRecorder]);

  const toggleRecorderPause = useCallback(() => {
    const engine = engineRef.current;
    const nextPaused = !useSessionStore.getState().recorderPaused;
    useSessionStore.getState().setRecorderPaused(nextPaused);
    if (engine) {
      engine.recordingEnabled = !nextPaused;
    }
  }, []);

  const setRecorderWindowSec = useCallback(
    (sec: number) => {
      useSessionStore.getState().setRecorderWindowSec(sec);
      const engine = engineRef.current;
      if (engine) {
        applyRecorderWindow(engine);
        syncRecorder(engine);
      }
    },
    [applyRecorderWindow, syncRecorder],
  );

  const setJointGains = useCallback((index: number, kp: number, kd: number) => {
    const session = sessionRef.current;
    const engine = engineRef.current;
    if (!session || !engine) return;

    useSessionStore.getState().setJointGainAt(index, kp, kd);
    const { jointKp, jointKd } = useSessionStore.getState();
    applyActuatedGainsToController(session, engine, jointKp, jointKd);
  }, []);

  const applyAutoJointGains = useCallback(() => {
    const session = sessionRef.current;
    const engine = engineRef.current;
    if (!session || !engine) return;

    const { controllerKdDamping } = useSessionStore.getState();
    const q = vecGet(session.data.qpos, session.nq);
    const gains = engine.recomputeAutoGains(q, {
      kdDamping: controllerKdDamping,
      omega: CONTROLLER_OMEGA,
    });
    const actuated = nvGainsToActuated(session, gains.kp, gains.kd);
    useSessionStore.getState().setAllJointGains(actuated.kp, actuated.kd);
  }, []);

  const setControllerKdDamping = useCallback(
    (value: number) => {
      useSessionStore.getState().setControllerKdDamping(value);
      applyAutoJointGains();
    },
    [applyAutoJointGains],
  );

  const runJointInterpolationTo = useCallback(
    async (
      targetJoints: number[],
      opts: { qStartJoints?: number[]; statusMessage?: string } = {},
    ): Promise<boolean> => {
      const engine = engineRef.current;
      const session = sessionRef.current;
      if (!engine || !session) return false;

      const state = useSessionStore.getState();
      const qStartJoints = opts.qStartJoints ?? [...state.commandedJointPositions];
      const maxVel = state.jointMaxVelocity;

      cancelRef.current = true;
      cancelRafLoop();
      engine.controlDt = state.controlDt;
      applyRecorderWindow(engine);
      syncExternalWrenches(engine);
      cancelRef.current = false;
      pauseRef.current = false;
      setPaused(false);
      setInterpolationActive(true);

      const statusMessage =
        opts.statusMessage ?? `关节插值 (限速 ${maxVel.toFixed(2)} rad/s)…`;
      setSimStatus('running', statusMessage);

      const qStart = actuatedJointsToQpos(session, qStartJoints);
      const qEnd = actuatedJointsToQpos(session, targetJoints);

      const ok = await engine.runVelocityLimitedInterpolationAsync(qStart, qEnd, maxVel, {
        cancelCheck: () => cancelRef.current,
        pauseCheck: () => pauseRef.current,
        yieldEvery: 20,
        onYield: () => new Promise((r) => requestAnimationFrame(() => r())),
        stepCallback: makeStepCallback(engine, { value: 0 }),
      });

      setInterpolationActive(false);
      useSessionStore.getState().setJointTargets([...targetJoints]);
      useSessionStore.getState().setCommandedJointPositions([...targetJoints]);
      setJointPositions(qposToActuatedJoints(session));
      const fk = fkRef.current ?? session.forwardKinematics;
      const ee = fk.compute(vecGet(session.data.qpos, session.nq));
      setEeFkPos([...ee.pos] as Vec3);
      syncRecorder(engine);
      setPaused(false);

      if (ok) {
        if (!loopActiveRef.current) {
          runHoldLoop('插值完成，保持目标');
        } else {
          setSimStatus('running', '插值完成，保持目标');
        }
      } else {
        setSimStatus('error', '关节插值未完成');
      }

      return ok;
    },
    [
      applyRecorderWindow,
      cancelRafLoop,
      makeStepCallback,
      runHoldLoop,
      setEeFkPos,
      setInterpolationActive,
      setJointPositions,
      setPaused,
      setSimStatus,
      syncRecorder,
      syncExternalWrenches,
    ],
  );

  const sendTargetInterpolation = useCallback(
    async (targetJoints: number[], opts?: { qStartJoints?: number[] }) => {
      ensureSimRunning();
      await runJointInterpolationTo(targetJoints, opts);
    },
    [ensureSimRunning, runJointInterpolationTo],
  );

  const computeEePoseFromJoints = useCallback(
    (joints: number[]): { pos: Vec3; quat: Quat } | null => {
      const session = sessionRef.current;
      if (!session) return null;
      const fk = fkRef.current ?? session.forwardKinematics;
      const ee = fk.compute(actuatedJointsToQpos(session, joints));
      return { pos: [...ee.pos] as Vec3, quat: [...ee.quat] as Quat };
    },
    [],
  );

  const computeEeSceneWorldFromJoints = useCallback(
    (joints: number[], endEffectorLink: string): [number, number, number] | null => {
      const session = sessionRef.current;
      if (!session) return null;
      const robot = getReferenceUrdfRobot() ?? getMainUrdfRobot();
      if (robot) {
        return readEeSceneWorldForJointAngles(
          robot,
          session.jointNames,
          joints,
          endEffectorLink,
        );
      }
      const eePose = computeEePoseFromJoints(joints);
      if (!eePose) return null;
      const w = urdfTargetToWorld(eePose.pos);
      return [w.x, w.y, w.z];
    },
    [computeEePoseFromJoints],
  );

  const addMotionTarget = useCallback(async (): Promise<{ ok: boolean; message?: string }> => {
    const session = sessionRef.current;
    const ikSolver = ikSolverRef.current;
    if (!session) {
      return { ok: false, message: '模型未加载' };
    }

    const state = useSessionStore.getState();

    if (state.controlLayer === 'joint') {
      const joints = [...state.jointTargets];
      const eePose = computeEePoseFromJoints(joints);
      const eeSceneWorld = computeEeSceneWorldFromJoints(joints, state.endEffectorLink);
      if (!eePose || !eeSceneWorld) {
        return { ok: false, message: '无法计算末端位姿' };
      }
      useSessionStore.getState().addMotionTarget({
        id: createMotionTargetId(),
        source: 'joint',
        jointPositions: joints,
        eePosition: eePose.pos,
        eeQuaternion: eePose.quat,
        eeSceneWorld,
      });
      return { ok: true };
    }

    if (!ikSolver) {
      return { ok: false, message: 'IK 求解器未初始化' };
    }

    let joints: number[] | null = null;
    if (
      state.referenceJointPositions.length > 0 &&
      (state.ikLiveStatus === 'converged' || state.eeTargetDirty)
    ) {
      joints = [...state.referenceJointPositions];
    } else {
      const target = state.eeTarget;
      const sceneWorld = state.eeTargetDirty
        ? (() => {
            const w = urdfTargetToWorld(target);
            return [w.x, w.y, w.z] as [number, number, number];
          })()
        : (readEndEffectorSceneWorld(state.endEffectorLink) ??
          (() => {
            const w = urdfTargetToWorld(target);
            return [w.x, w.y, w.z] as [number, number, number];
          })());

      const ikResult = await solveEeIkLiveInternal(target, {
        liveDrag: false,
        targetSceneWorld: sceneWorld,
      });
      if (!ikResult.converged) {
        return { ok: false, message: ikResult.message ?? 'IK 求解失败，无法添加目标' };
      }
      joints = [...ikResult.jointAngles];
    }

    const eePose = computeEePoseFromJoints(joints) ?? {
      pos: [...state.eeTarget] as Vec3,
      quat: [...state.eeTargetQuat] as Quat,
    };
    const eeSceneWorld =
      computeEeSceneWorldFromJoints(joints, state.endEffectorLink) ??
      (() => {
        const w = urdfTargetToWorld(eePose.pos);
        return [w.x, w.y, w.z] as [number, number, number];
      })();

    useSessionStore.getState().addMotionTarget({
      id: createMotionTargetId(),
      source: 'ee',
      jointPositions: joints,
      eePosition: eePose.pos,
      eeQuaternion: eePose.quat,
      eeSceneWorld,
    });
    return { ok: true };
  }, [computeEePoseFromJoints, computeEeSceneWorldFromJoints, solveEeIkLiveInternal]);

  const runMultiWaypointInterpolation = useCallback(
    async (targets: MotionTarget[]): Promise<boolean> => {
      const engine = engineRef.current;
      const session = sessionRef.current;
      if (!engine || !session || targets.length === 0) return false;

      const state = useSessionStore.getState();
      const qStartJoints = [...state.commandedJointPositions];
      const maxVel = state.jointMaxVelocity;
      const profile = state.interpProfile;

      cancelRef.current = true;
      cancelRafLoop();
      engine.controlDt = state.controlDt;
      applyRecorderWindow(engine);
      syncExternalWrenches(engine);
      cancelRef.current = false;
      pauseRef.current = false;
      setPaused(false);
      setInterpolationActive(true);

      const qWaypoints = [
        actuatedJointsToQpos(session, qStartJoints),
        ...targets.map((t) => actuatedJointsToQpos(session, t.jointPositions)),
      ];
      const effectiveProfile = resolveInterpProfile(profile, qWaypoints.length);

      const profileLabel = effectiveProfile === 'cubic' ? '三次样条' : '线性';
      setSimStatus(
        'running',
        `多路点插值 (${profileLabel}, ${targets.length} 帧, 限速 ${maxVel.toFixed(2)} rad/s)…`,
      );

      const ok = await engine.runMultiWaypointInterpolationAsync(
        qWaypoints,
        maxVel,
        effectiveProfile,
        {
        cancelCheck: () => cancelRef.current,
        pauseCheck: () => pauseRef.current,
        yieldEvery: 20,
        onYield: () => new Promise((r) => requestAnimationFrame(() => r())),
        stepCallback: makeStepCallback(engine, { value: 0 }),
      });

      const finalJoints = targets[targets.length - 1]!.jointPositions;
      setInterpolationActive(false);
      useSessionStore.getState().setJointTargets([...finalJoints]);
      useSessionStore.getState().setCommandedJointPositions([...finalJoints]);
      setJointPositions(qposToActuatedJoints(session));
      const ee = (fkRef.current ?? session.forwardKinematics).compute(
        vecGet(session.data.qpos, session.nq),
      );
      setEeFkPos([...ee.pos] as Vec3);
      syncRecorder(engine);
      setPaused(false);

      if (ok) {
        if (!loopActiveRef.current) {
          runHoldLoop('多路点插值完成，保持目标');
        } else {
          setSimStatus('running', '多路点插值完成，保持目标');
        }
      } else {
        setSimStatus('error', '多路点插值未完成');
      }

      return ok;
    },
    [
      applyRecorderWindow,
      cancelRafLoop,
      makeStepCallback,
      runHoldLoop,
      setEeFkPos,
      setInterpolationActive,
      setJointPositions,
      setPaused,
      setSimStatus,
      syncRecorder,
      syncExternalWrenches,
    ],
  );

  const runJointTarget = useCallback(async () => {
    const state = useSessionStore.getState();
    const target = [...state.jointTargets];
    await sendTargetInterpolation(target);
  }, [sendTargetInterpolation]);

  const runJointInterpolationFromReference = useCallback(async () => {
    const state = useSessionStore.getState();
    const ref = state.referenceJointPositions;
    if (ref.length === 0) {
      setSimStatus('error', '无参考关节姿态，请先拖动 Gizmo 或求解 IK');
      return;
    }
    await sendTargetInterpolation(ref);
  }, [sendTargetInterpolation, setSimStatus]);

  const commitEeGizmoDrag = useCallback(
    async (opts?: { simWasRunning?: boolean }) => {
      if (!opts?.simWasRunning) return;
      const state = useSessionStore.getState();
      if (state.controlMode !== 'realtime') return;
      await runJointInterpolationFromReference();
    },
    [runJointInterpolationFromReference],
  );

  const startSimulation = useCallback(() => {
    const state = useSessionStore.getState();
    const engine = engineRef.current;
    if (engine) {
      engine.recordingEnabled = !state.recorderPaused;
    }
    if (state.controlMode === 'interpolate') {
      useSessionStore.getState().setCommandedJointPositions([...state.jointTargets]);
      runHoldLoop('关节目标保持仿真中…');
    } else {
      runContinuousLoop();
    }
  }, [runContinuousLoop, runHoldLoop]);

  const runEeTarget = useCallback(async () => {
    const engine = engineRef.current;
    const session = sessionRef.current;
    const ikSolver = ikSolverRef.current;
    if (!engine || !session) return;
    if (!ikSolver) {
      setSimStatus('error', 'IK 求解器未初始化，请确认模型已加载且含活动关节');
      return;
    }

    const state = useSessionStore.getState();
    useSessionStore.getState().setEeGizmoVisible(true);

    if (state.eeTargetDirty && state.referenceJointPositions.length > 0) {
      await sendTargetInterpolation(state.referenceJointPositions);
      return;
    }
    cancelRafLoop();
    engine.controlDt = state.controlDt;
    applyRecorderWindow(engine);
    syncExternalWrenches(engine);
    cancelRef.current = false;
    pauseRef.current = false;
    setPaused(false);

    const target = state.eeTarget;
    setSimStatus('running', '末端 IK 求解中…');

    const sceneWorld = state.eeTargetDirty
      ? (() => {
          const w = urdfTargetToWorld(target);
          return [w.x, w.y, w.z] as [number, number, number];
        })()
      : (readEndEffectorSceneWorld(state.endEffectorLink) ??
        (() => {
          const w = urdfTargetToWorld(target);
          return [w.x, w.y, w.z] as [number, number, number];
        })());

    const ikResult = await solveEeIkLiveInternal(target, {
      liveDrag: false,
      targetSceneWorld: sceneWorld,
    });
    if (!ikResult.converged) {
      setSimStatus('error', ikResult.message ?? 'IK 求解失败');
      return;
    }

    await sendTargetInterpolation(ikResult.jointAngles);
  }, [
    applyRecorderWindow,
    cancelRafLoop,
    sendTargetInterpolation,
    setPaused,
    setSimStatus,
    solveEeIkLiveInternal,
    syncExternalWrenches,
  ]);

  const executeMotionTargets = useCallback(async () => {
    const state = useSessionStore.getState();
    if (state.motionTargets.length === 0) {
      if (state.controlLayer === 'joint') {
        await runJointTarget();
      } else {
        await runEeTarget();
      }
      return;
    }
    ensureSimRunning();
    await runMultiWaypointInterpolation(state.motionTargets);
  }, [ensureSimRunning, runEeTarget, runJointTarget, runMultiWaypointInterpolation]);

  const runTrajectorySim = useCallback(async () => {
    const engine = engineRef.current;
    const session = sessionRef.current;
    const ikSolver = ikSolverRef.current;
    if (!engine || !session || !ikSolver) return;

    if (trajectoryWaypoints.length < 2) {
      setSimStatus('error', '轨迹至少需要 2 个关键点');
      return;
    }

    cancelRafLoop();
    cancelRef.current = false;
    applyRecorderWindow(engine);
    syncExternalWrenches(engine);
    setSimStatus('running', '运行轨迹仿真…');

    const traj = new Trajectory();
    for (const wp of trajectoryWaypoints) {
      traj.addWaypoint(wp.time, wp.position, wp.quaternion);
    }
    const sampler = asTrajectorySampler(traj);

    await new Promise<void>((resolve) => {
      requestAnimationFrame(() => {
        engine.reset();
        const stepIndex = { value: 0 };
        const success = engine.runTrajectory(sampler, ikSolver, {
          cancelCheck: () => cancelRef.current,
          stepCallback: makeStepCallback(engine, stepIndex),
          progressCallback: (p) => {
            setSimStatus('running', `轨迹进度 ${(p * 100).toFixed(0)}%`);
          },
        });
        setJointPositions(qposToActuatedJoints(session));
        syncRecorder(engine);
        setSimStatus(success ? 'ready' : 'error', success ? '轨迹仿真完成' : '轨迹仿真失败');
        resolve();
      });
    });
  }, [
    applyRecorderWindow,
    cancelRafLoop,
    makeStepCallback,
    setJointPositions,
    setSimStatus,
    syncRecorder,
    syncExternalWrenches,
    trajectoryWaypoints,
  ]);

  const exportCsv = useCallback(() => {
    const engine = engineRef.current;
    if (!engine || engine.recorder.getNumFrames() === 0) {
      setSimStatus('error', '无录制数据可导出');
      return;
    }
    const csv = exportToCsv(engine.recorder, {
      jointNames: robotInfo?.jointNames,
    });
    const name = robotInfo?.name ?? 'robot';
    downloadBlob(csvToBlob(csv), `${name}_simulation.csv`);
    setSimStatus('ready', 'CSV 已导出');
  }, [robotInfo, setSimStatus]);

  const dispose = useCallback(() => {
    cancelRafLoop();
    closedChainIkRef.current?.dispose();
    closedChainIkRef.current = null;
    sessionRef.current?.dispose();
    sessionRef.current = null;
    engineRef.current = null;
    ikRef.current = null;
    ikSolverRef.current = null;
    fkRef.current = null;
  }, [cancelRafLoop]);

  const prevControlLayerRef = useRef(controlLayer);
  const prevIkEnabledRef = useRef(ikEnabled);

  useEffect(() => {
    const onViewerRobot = (robot: import('urdf-loader').URDFRobot | null) => {
      if (!robot) return;
      attachIkToViewerRobot();
      const state = useSessionStore.getState();
      if (pendingEeSyncRef.current || state.controlLayer === 'ee') {
        void syncEeCommandFromFk();
      }
    };
    const unsubMain = onMainUrdfRobotChange(onViewerRobot);
    const unsubRef = onReferenceUrdfRobotChange(() => {
      attachIkToViewerRobot();
    });
    return () => {
      unsubMain();
      unsubRef();
    };
  }, [attachIkToViewerRobot, syncEeCommandFromFk]);

  useEffect(() => {
    refreshEeFkPos();
  }, [jointPositions, endEffectorLink, refreshEeFkPos]);

  useEffect(() => {
    const enteredEe = controlLayer === 'ee' && prevControlLayerRef.current !== 'ee';
    prevControlLayerRef.current = controlLayer;
    if (enteredEe && robotInfo) {
      void syncEeCommandFromFk();
    }
  }, [controlLayer, robotInfo, syncEeCommandFromFk]);

  useEffect(() => {
    const turnedOn = ikEnabled && !prevIkEnabledRef.current;
    prevIkEnabledRef.current = ikEnabled;
    if (turnedOn && controlLayer === 'ee' && robotInfo) {
      void syncEeCommandFromFk();
    }
  }, [ikEnabled, controlLayer, robotInfo, syncEeCommandFromFk]);

  useEffect(() => {
    if (controlLayer !== 'ee' || !ikEnabled || ikDragActive) {
      return;
    }

    const timer = window.setTimeout(() => {
      const state = useSessionStore.getState();
      if (state.interpolationActive) {
        return;
      }
      const sceneWorld = state.eeTargetDirty
        ? (() => {
            const w = urdfTargetToWorld(state.eeTarget);
            return [w.x, w.y, w.z] as [number, number, number];
          })()
        : (readEndEffectorSceneWorld(state.endEffectorLink) ??
          (() => {
            const w = urdfTargetToWorld(state.eeTarget);
            return [w.x, w.y, w.z] as [number, number, number];
          })());
      const mainRobot = getMainUrdfRobot();
      const worldPose = mainRobot
        ? readEeWorldPoseFromRobot(mainRobot, state.endEffectorLink)
        : null;
      const sceneQuaternion = worldPose
        ? ([
            worldPose.quaternion.x,
            worldPose.quaternion.y,
            worldPose.quaternion.z,
            worldPose.quaternion.w,
          ] as [number, number, number, number])
        : undefined;
      void solveEeIkLive(state.eeTarget, {
        liveDrag: false,
        targetSceneWorld: sceneWorld,
        targetSceneQuaternion: sceneQuaternion,
      });
    }, 100);

    return () => window.clearTimeout(timer);
  }, [
    controlLayer,
    ikEnabled,
    ikDragActive,
    eeTarget,
    endEffectorLink,
    eeTargetDirty,
    solveEeIkLive,
  ]);

  const reloadUrdf = useCallback(
    async (urdfText: string) => {
      const name = useSessionStore.getState().urdfFileName ?? 'robot.urdf';
      await loadRobot(urdfText, name, meshesRef.current);
    },
    [loadRobot],
  );

  return {
    loadRobot,
    loadDefaultBiped,
    reloadUrdf,
    applyBaseLink,
    applyEndEffectorLink,
    startSimulation,
    runJointTarget,
    runEeTarget,
    addMotionTarget,
    executeMotionTargets,
    runJointInterpolationFromReference,
    commitEeGizmoDrag,
    runTrajectorySim,
    exportCsv,
    cancelSimulation,
    pauseSimulation,
    stopSimulation,
    resetRobotPose,
    resetRecorder,
    toggleRecorderPause,
    setRecorderWindowSec,
    setJointGains,
    applyAutoJointGains,
    setControllerKdDamping,
    solveEeIkLive,
    resetReferencePose,
    resetGizmoToCurrent,
    syncExternalWrenchesFromStore,
    dispose,
  };
}
