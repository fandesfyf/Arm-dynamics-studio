import { useCallback, useEffect, useRef } from 'react';
import { CONTROLLER_OMEGA } from '../core/controller';
import { publishChartLiveDict, clearChartLiveBuffer } from '../core/chart-live-buffer';
import { DataRecorder, type RecorderDict } from '../core/data-recorder';
import { vecGet, vecSet } from '../types/mujoco';
import { RobotSession, createForwardKinematics, type ForwardKinematics } from '../core/robot-session';
import { createSimulation, type IKSolver, type SimulationEngine } from '../core/simulation';
import { InverseKinematics } from '../core/inverse-kinematics';
import { createJointMapAdapter } from '../pinocchio/joint-map-adapter';
import { Trajectory } from '../core/trajectory';
import { asTrajectorySampler } from '../core/planner';
import { exportToCsv, csvToBlob } from '../export/csv-exporter';
import { downloadMotionTargetsCsv, parseMotionTargetsCsv } from '../export/motion-target-csv';
import { useSessionStore, type RobotInfo } from '../stores/session-store';
import type { ControlUiPreserve } from '../stores/session-store';
import type { PayloadRecord, Wrench6 } from '../core/payload-editor';
import { loadDefaultBipedUpperBody, loadDefaultBipedMeshes } from '../utils/biped-default-loader';
import { loadBundledTestArm } from '../utils/test-arm-loader';
import { detectEndEffectorLink, ensureFixedBase, parseLinkNames, resolveEndEffectorJointName } from '../utils/urdf-base-fixture';
import { prepareUrdfForMujocoLoad } from '../utils/urdf-sanitize';
import { releaseActiveMujocoHandles } from '../mujoco/loader';
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
/** 曲线缓冲刷新间隔（~60 Hz，与显示帧率对齐） */
const CHART_FLUSH_INTERVAL_MS = 16;

function waitAnimationFrames(count: number): Promise<void> {
  return new Promise((resolve) => {
    let left = count;
    const step = () => {
      left -= 1;
      if (left <= 0) resolve();
      else requestAnimationFrame(step);
    };
    requestAnimationFrame(step);
  });
}

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

function captureControlUiPreserve(
  state: ReturnType<typeof useSessionStore.getState>,
  engine: SimulationEngine | null,
): ControlUiPreserve {
  const recorderDict = engine?.recorder.toDict() ?? state.recorderDict;
  const times = recorderDict?.time ?? [];
  return {
    motionTargets: state.motionTargets.map((t) => ({
      ...t,
      jointPositions: [...t.jointPositions],
      eePosition: [...t.eePosition] as Vec3,
      eeQuaternion: [...t.eeQuaternion] as Quat,
      eeSceneWorld: [...t.eeSceneWorld] as [number, number, number],
    })),
    jointTargets: [...state.jointTargets],
    commandedJointPositions: [...state.commandedJointPositions],
    referenceJointPositions: [...state.referenceJointPositions],
    jointPositions: [...state.jointPositions],
    jointKp: [...state.jointKp],
    jointKd: [...state.jointKd],
    eeTarget: [...state.eeTarget] as Vec3,
    eeTargetQuat: [...state.eeTargetQuat] as Quat,
    eeTargetDirty: state.eeTargetDirty,
    controlLayer: state.controlLayer,
    controlMode: state.controlMode,
    jointMaxVelocity: state.jointMaxVelocity,
    interpProfile: state.interpProfile,
    trajectoryWaypoints: state.trajectoryWaypoints.map((w) => ({ ...w })),
    recorder: {
      sampleCount: times.length,
      lastTime: times.length > 0 ? times[times.length - 1]! : null,
    },
    recorderDict: recorderDict,
    simTime: engine?.simTime ?? state.simTime,
    simStepCount: state.simStepCount,
  };
}

function downloadBlob(blob: Blob, filename: string): void {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  a.click();
  URL.revokeObjectURL(url);
}

const EMPTY_RECORDER_DICT: RecorderDict = {
  time: [],
  qpos: [],
  qvel: [],
  tau: [],
  ee_pos: [],
  ee_quat: [],
};

/** MuJoCo WASM 单例 + VFS 非线程安全：串行化所有 loadRobot */
let robotLoadChain: Promise<unknown> = Promise.resolve();

function enqueueRobotLoad<T>(task: () => Promise<T>): Promise<T> {
  const run = () => task();
  const result = robotLoadChain.then(run, run);
  robotLoadChain = result.then(
    () => undefined,
    () => undefined,
  );
  return result;
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
  const chartLoopActiveRef = useRef(false);
  const chartRafIdRef = useRef<number | null>(null);
  const lastChartFlushMsRef = useRef(0);
  /** 上次刷入曲线缓冲的墙钟时间（秒） */
  const lastChartFlushTimeRef = useRef<number | null>(null);
  const chartWallRecorderRef = useRef(new DataRecorder());
  const chartWallEpochSecRef = useRef(0);
  const meshesRef = useRef<Map<string, Uint8Array>>(new Map());
  const lastUiUpdateRef = useRef(0);
  const lastGoodSnapshotRef = useRef<LastGoodSnapshot | null>(null);
  const loadGenerationRef = useRef(0);
  const pendingEeSyncRef = useRef(false);

  const {
    setLoading,
    setRobotLoaded,
    setLoadError,
    setJointPositions,
    setSimStatus,
    updateRecorder,
    syncSimUiFrame,
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
    const sec = useSessionStore.getState().recorderWindowSec;
    engine.recorder.setMaxDurationSec(sec);
    chartWallRecorderRef.current.setMaxDurationSec(sec);
  }, []);

  const resetChartWallRecorder = useCallback(() => {
    chartWallRecorderRef.current.clear();
    chartWallEpochSecRef.current = performance.now() / 1000;
    chartWallRecorderRef.current.setMaxDurationSec(useSessionStore.getState().recorderWindowSec);
  }, []);

  const syncRecorder = useCallback(
    (engine: SimulationEngine) => {
      applyRecorderWindow(engine);
      const dict = engine.recorder.toDict();
      updateRecorder(
        {
          sampleCount: engine.recorder.getNumFrames(),
          lastTime: engine.recorder.getLastTime(),
        },
        dict,
      );
      publishChartLiveDict(dict);
      setSimRuntime(engine.simTime, engine.recorder.getNumFrames());
    },
    [applyRecorderWindow, setSimRuntime, updateRecorder],
  );

  /** 将墙钟曲线缓冲同步到 store，供停止/暂停后静态显示 */
  const syncChartDisplayFromWall = useCallback(
    (engine?: SimulationEngine | null) => {
      const eng = engine ?? engineRef.current;
      if (eng) {
        applyRecorderWindow(eng);
      } else {
        chartWallRecorderRef.current.setMaxDurationSec(useSessionStore.getState().recorderWindowSec);
      }

      const wall = chartWallRecorderRef.current;
      const wallFrames = wall.getNumFrames();
      if (wallFrames > 0) {
        const dict = wall.toDictForDisplay();
        updateRecorder(
          {
            sampleCount: wallFrames,
            lastTime: wall.getLastTime(),
          },
          dict,
        );
        publishChartLiveDict(dict);
        if (eng) {
          const displayTime = wall.getLastTime() ?? eng.simTime;
          setSimRuntime(displayTime, wallFrames);
        }
        return;
      }

      if (eng) {
        syncRecorder(eng);
      }
    },
    [applyRecorderWindow, setSimRuntime, syncRecorder, updateRecorder],
  );

  const flushChartLive = useCallback(
    (engine: SimulationEngine) => {
      const wallTime = performance.now() / 1000 - chartWallEpochSecRef.current;
      if (lastChartFlushTimeRef.current !== null && wallTime <= lastChartFlushTimeRef.current) {
        return;
      }
      lastChartFlushTimeRef.current = wallTime;
      applyRecorderWindow(engine);
      chartWallRecorderRef.current.record({
        ...engine.sampleForChart(),
        time: wallTime,
      });
      publishChartLiveDict(chartWallRecorderRef.current.toDictForDisplay());
    },
    [applyRecorderWindow],
  );

  const stopChartSyncLoop = useCallback(() => {
    chartLoopActiveRef.current = false;
    if (chartRafIdRef.current !== null) {
      cancelAnimationFrame(chartRafIdRef.current);
      chartRafIdRef.current = null;
    }
    lastChartFlushTimeRef.current = null;
  }, []);

  const ensureChartSyncLoop = useCallback(() => {
    if (chartLoopActiveRef.current) return;
    chartLoopActiveRef.current = true;
    lastChartFlushMsRef.current = 0;

    const wall = chartWallRecorderRef.current;
    const lastWallTime = wall.getLastTime();
    const hasPriorWallData = wall.getNumFrames() > 0 && lastWallTime !== null;

    if (hasPriorWallData) {
      chartWallEpochSecRef.current = performance.now() / 1000 - lastWallTime;
      lastChartFlushTimeRef.current = lastWallTime;
      wall.setMaxDurationSec(useSessionStore.getState().recorderWindowSec);
      publishChartLiveDict(wall.toDictForDisplay());
    } else {
      lastChartFlushTimeRef.current = null;
      resetChartWallRecorder();
      clearChartLiveBuffer();
      updateRecorder({ sampleCount: 0, lastTime: null }, EMPTY_RECORDER_DICT);
      const eng = engineRef.current;
      if (eng) {
        eng.recorder.clear();
      }
    }

    const eng = engineRef.current;
    if (eng) {
      applyRecorderWindow(eng);
      flushChartLive(eng);
    }

    const chartTick = () => {
      if (!chartLoopActiveRef.current) {
        chartRafIdRef.current = null;
        return;
      }
      const state = useSessionStore.getState();
      if (state.simStatus === 'running' && !pauseRef.current) {
        const now = performance.now();
        if (now - lastChartFlushMsRef.current >= CHART_FLUSH_INTERVAL_MS) {
          lastChartFlushMsRef.current = now;
          const engine = engineRef.current;
          if (engine) flushChartLive(engine);
        }
      }
      chartRafIdRef.current = requestAnimationFrame(chartTick);
    };
    chartRafIdRef.current = requestAnimationFrame(chartTick);
  }, [applyRecorderWindow, flushChartLive, resetChartWallRecorder, updateRecorder]);

  const pushSimUi = useCallback(
    (session: RobotSession, engine: SimulationEngine, force = false) => {
      const now = performance.now();
      if (!force && now - lastUiUpdateRef.current < UI_UPDATE_INTERVAL_MS) {
        return;
      }
      lastUiUpdateRef.current = now;
      applyRecorderWindow(engine);
      syncSimUiFrame(
        qposToActuatedJoints(session),
        engine.simTime,
        engine.recorder.getNumFrames(),
        {
          sampleCount: engine.recorder.getNumFrames(),
          lastTime: engine.recorder.getLastTime(),
        },
      );
    },
    [applyRecorderWindow, syncSimUiFrame],
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

  /** 停止 RAF 循环并清除取消标志，供插值独占 engine 步进 */
  const prepareInterpolationRun = useCallback(
    (engine: SimulationEngine | null) => {
      cancelRafLoop();
      cancelRef.current = false;
      pauseRef.current = false;
      setPaused(false);
      const state = useSessionStore.getState();
      if (
        state.jointPositions.length > 0 &&
        state.commandedJointPositions.length !== state.jointPositions.length
      ) {
        useSessionStore.getState().setCommandedJointPositions([...state.jointPositions]);
      }
      if (engine) {
        engine.controlDt = state.controlDt;
        engine.recordingEnabled = !state.recorderPaused;
        applyRecorderWindow(engine);
        syncExternalWrenches(engine);
      }
    },
    [applyRecorderWindow, cancelRafLoop, setPaused, syncExternalWrenches],
  );

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
      ensureChartSyncLoop();

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
          if (sessionRef.current !== session) break;
          accumulated -= dt;
          const qDesired = resolveDesired();
          if (!qDesired) break;

          engine.stepHoldTarget(qDesired, () => {
            stepped = true;
          });
        }

        if (stepped && sessionRef.current === session) {
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
      ensureChartSyncLoop,
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

  const loadRobot = useCallback(
    async (
      urdfText: string,
      urdfFileName: string,
      meshes?: Map<string, Uint8Array>,
      forceBaseLink?: string,
      loadPhase: 'initial' | 'payload-reload' | 'manual' = 'manual',
      isRollbackReload = false,
    ) => {
      return enqueueRobotLoad(async () => {
        const loadId = ++loadGenerationRef.current;
        setLoading(true, '正在加载 MuJoCo + Pinocchio…');
        loopActiveRef.current = false;
        cancelRef.current = true;
        cancelRafLoop();
        if (engineRef.current) {
          engineRef.current.isRunning = false;
        }

        const prevSession = sessionRef.current;
        const prevEngine = engineRef.current;
        const prevIk = ikRef.current;
        const prevIkSolver = ikSolverRef.current;
        const prevFk = fkRef.current;
        sessionRef.current = null;
        engineRef.current = null;
        ikRef.current = null;
        ikSolverRef.current = null;
        fkRef.current = null;
        if (prevSession) {
          await waitAnimationFrames(2);
        }

        const rollbackSnapshot =
          !isRollbackReload && prevSession && prevEngine && prevIk && prevIkSolver
            ? captureSnapshot(prevSession, prevEngine, prevIk, prevIkSolver, prevFk)
            : !isRollbackReload
              ? lastGoodSnapshotRef.current
              : null;

        const preserveUi =
          loadPhase === 'payload-reload' && !isRollbackReload
            ? captureControlUiPreserve(useSessionStore.getState(), prevEngine)
            : null;

        let newSession: RobotSession | null = null;

        try {
          pauseRef.current = false;
          setPaused(false);

          if (!isRollbackReload) {
            prevSession?.dispose();
            if (prevSession) {
              await waitAnimationFrames(1);
            }
          }

          const prevUrdfFileName = useSessionStore.getState().urdfFileName;
          const meshMap = meshes ?? new Map();
          meshesRef.current = meshMap;

          const resolvedBase = forceBaseLink ?? baseLink;
          const fixture = ensureFixedBase(urdfText, resolvedBase || undefined);
          const storeUrdfText = prepareUrdfForMujocoLoad(fixture.urdfText);
          const resolvedBaseLink = fixture.baseLink;
          const urdfLinks = parseLinkNames(storeUrdfText).filter((name) => name !== 'world');
          const detectedEe = fixture.endEffectorLink ?? detectEndEffectorLink(storeUrdfText);
          const currentEe = useSessionStore.getState().endEffectorLink;
          const resolvedEeLink =
            currentEe && urdfLinks.includes(currentEe)
              ? currentEe
              : (detectedEe ?? urdfLinks[0] ?? currentEe);

          setBaseLink(resolvedBaseLink);
          if (resolvedEeLink && resolvedEeLink !== currentEe) {
            setEndEffectorLink(resolvedEeLink);
          }

          releaseActiveMujocoHandles();

          const session = await RobotSession.create({
            urdfXml: storeUrdfText,
            urdfFileName,
            meshes: meshMap,
            endEffectorLink: resolvedEeLink,
            baseLink: resolvedBaseLink,
            loadPhase,
            urdfPrepared: true,
          });
          if (loadId !== loadGenerationRef.current) {
            session.dispose();
            return;
          }
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
            storeUrdfText,
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

        sessionRef.current = session;
        engineRef.current = engine;
        ikRef.current = ik;
        ikSolverRef.current = ikSolver;
        fkRef.current = newFk;
        newSession = null;

        useSessionStore.getState().setEeGizmoVisible(false);

        engine.reset();
        if (preserveUi?.recorderDict && preserveUi.recorderDict.time.length > 0) {
          engine.recorder.loadFromDict(preserveUi.recorderDict);
          engine.simTime = preserveUi.simTime;
        }

        const qpos = preserveUi ? preserveUi.jointPositions : qposToActuatedJoints(session);
        if (preserveUi) {
          const fullQ = actuatedJointsToQpos(session, preserveUi.commandedJointPositions);
          vecSet(session.data.qpos, fullQ, session.nq);
          session.mujoco.mj_forward(session.model, session.data);
        }

        const qInit = vecGet(session.data.qpos, session.nq);
        const kdDamping = useSessionStore.getState().controllerKdDamping;
        if (preserveUi && preserveUi.jointKp.length === session.jointNames.length) {
          applyActuatedGainsToController(
            session,
            { getGains: () => engine.getGains(), setGains: (kp, kd) => engine.setGains(kp, kd) },
            preserveUi.jointKp,
            preserveUi.jointKd,
          );
        } else {
          engine.recomputeAutoGains(qInit, { kdDamping, omega: CONTROLLER_OMEGA });
        }
        const fk = fkRef.current ?? session.forwardKinematics;
        const ee = fk.compute(vecGet(session.data.qpos, session.nq));
        const robotName =
          urdfText.match(/<robot\s+name="([^"]+)"/)?.[1] ?? urdfFileName.replace(/\.urdf$/i, '');

        if (preserveUi) {
          setSimRuntime(preserveUi.simTime, preserveUi.simStepCount);
        } else {
          setSimRuntime(0, 0);
        }
        const gains = nvGainsToActuated(session, engine.getGains().kp, engine.getGains().kd);
        useSessionStore.getState().setAllJointGains(gains.kp, gains.kd);
        if (loadId !== loadGenerationRef.current) return;
        setRobotLoaded(
          {
            robotInfo: {
              name: robotName,
              dof: session.jointNames.length,
              jointNames: session.jointNames,
              lowerLimits: session.pinocchioBundle.lowerLimits,
              upperLimits: session.pinocchioBundle.upperLimits,
              eePos: preserveUi ? (preserveUi.eeTarget as Vec3) : (ee.pos as Vec3),
              eeQuat: preserveUi ? (preserveUi.eeTargetQuat as Quat) : (ee.quat as Quat),
            },
            jointPositions: preserveUi ? preserveUi.jointPositions : qpos,
            urdfText: storeUrdfText,
            urdfFileName,
            meshAssets: meshMap,
          },
          preserveUi ? { preserve: preserveUi } : undefined,
        );

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
        cancelRef.current = false;
        loopActiveRef.current = false;
        } catch (e) {
          newSession?.dispose();
          if (loadId !== loadGenerationRef.current) return;

          const msg = e instanceof Error ? e.message : String(e);
          if (rollbackSnapshot && !isRollbackReload) {
            try {
              await loadRobot(
                rollbackSnapshot.urdfText,
                rollbackSnapshot.urdfFileName,
                meshesRef.current,
                rollbackSnapshot.baseLink,
                'manual',
                true,
              );
              useSessionStore.setState({
                simStatus: 'ready',
                simMessage: `添加负载后重载失败，已恢复上一模型：${msg.split('\n')[0] ?? msg}`,
                loading: false,
                loadingMessage: '',
              });
              return;
            } catch {
              // fall through to metadata-only restore
            }
          }
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
      });
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
    await loadRobot(bundle.urdfText, bundle.urdfFileName, bundle.meshes, undefined, 'initial');
    void loadDefaultBipedMeshes().then((meshes) => {
      meshesRef.current = meshes;
      const state = useSessionStore.getState();
      if (state.urdfFileName === bundle.urdfFileName) {
        useSessionStore.setState({ meshAssets: new Map(meshes) });
      }
    });
  }, [loadRobot]);

  const loadTestArm = useCallback(async () => {
    const bundle = await loadBundledTestArm();
    await loadRobot(bundle.urdfText, bundle.urdfFileName, bundle.meshes, undefined, 'manual');
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
    if (next) {
      const engine = engineRef.current;
      if (engine) {
        flushChartLive(engine);
      }
    }
  }, [flushChartLive, setPaused]);

  const stopSimulation = useCallback(() => {
    cancelRef.current = true;
    cancelRafLoop();
    stopChartSyncLoop();
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
      syncChartDisplayFromWall(engine);
    }
    setSimStatus('ready', '仿真已停止');
    cancelRef.current = false;
  }, [
    cancelRafLoop,
    setInterpolationActive,
    setJointPositions,
    setPaused,
    setSimStatus,
    stopChartSyncLoop,
    syncChartDisplayFromWall,
  ]);

  const resetRobotPose = useCallback(() => {
    cancelRef.current = true;
    cancelRafLoop();
    stopChartSyncLoop();
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
    stopChartSyncLoop,
    syncRecorder,
  ]);

  const resetRecorder = useCallback(() => {
    const engine = engineRef.current;
    if (!engine) return;
    engine.recorder.clear();
    engine.simTime = 0;
    useSessionStore.getState().setRecorderPaused(false);
    engine.recordingEnabled = true;
    resetChartWallRecorder();
    clearChartLiveBuffer();
    syncRecorder(engine);
    setSimStatus('ready', '曲线数据已清空');
  }, [resetChartWallRecorder, setSimStatus, syncRecorder]);

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
        syncChartDisplayFromWall(engine);
      }
    },
    [applyRecorderWindow, syncChartDisplayFromWall],
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

      prepareInterpolationRun(engine);
      setInterpolationActive(true);
      ensureChartSyncLoop();

      const statusMessage =
        opts.statusMessage ?? `关节插值 (限速 ${maxVel.toFixed(2)} rad/s)…`;
      setSimStatus('running', statusMessage);

      const qStart = actuatedJointsToQpos(session, qStartJoints);
      const qEnd = actuatedJointsToQpos(session, targetJoints);

      let ok = false;
      try {
        ok = await engine.runVelocityLimitedInterpolationAsync(qStart, qEnd, maxVel, {
          cancelCheck: () => cancelRef.current,
          pauseCheck: () => pauseRef.current,
          stepCallback: makeStepCallback(engine, { value: 0 }),
        });
      } finally {
        setInterpolationActive(false);
      }

      useSessionStore.getState().setJointTargets([...targetJoints]);
      useSessionStore.getState().setCommandedJointPositions([...targetJoints]);
      setJointPositions(qposToActuatedJoints(session));
      const fk = fkRef.current ?? session.forwardKinematics;
      const ee = fk.compute(vecGet(session.data.qpos, session.nq));
      setEeFkPos([...ee.pos] as Vec3);
      syncRecorder(engine);
      setPaused(false);

      if (ok) {
        runHoldLoop('插值完成，保持目标');
      } else {
        setSimStatus('error', '关节插值未完成');
      }

      return ok;
    },
    [
      ensureChartSyncLoop,
      makeStepCallback,
      prepareInterpolationRun,
      runHoldLoop,
      setEeFkPos,
      setInterpolationActive,
      setJointPositions,
      setPaused,
      setSimStatus,
      syncRecorder,
    ],
  );

  const sendTargetInterpolation = useCallback(
    async (targetJoints: number[], opts?: { qStartJoints?: number[] }) => {
      if (!engineRef.current || !sessionRef.current) {
        setSimStatus('error', '模型未就绪，请等待加载完成');
        return;
      }
      await runJointInterpolationTo(targetJoints, opts);
    },
    [runJointInterpolationTo, setSimStatus],
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

      prepareInterpolationRun(engine);
      setInterpolationActive(true);
      ensureChartSyncLoop();

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

      let ok = false;
      try {
        ok = await engine.runMultiWaypointInterpolationAsync(
          qWaypoints,
          maxVel,
          effectiveProfile,
          {
            cancelCheck: () => cancelRef.current,
            pauseCheck: () => pauseRef.current,
            stepCallback: makeStepCallback(engine, { value: 0 }),
          },
        );
      } catch (err) {
        const msg = err instanceof Error ? err.message : String(err);
        setSimStatus('error', `多路点插值失败: ${msg.split('\n')[0] ?? msg}`);
        return false;
      } finally {
        setInterpolationActive(false);
      }

      const finalJoints = targets[targets.length - 1]!.jointPositions;
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
        runHoldLoop('多路点插值完成，保持目标');
      } else {
        setSimStatus('error', '多路点插值未完成');
      }

      return ok;
    },
    [
      ensureChartSyncLoop,
      makeStepCallback,
      prepareInterpolationRun,
      runHoldLoop,
      setEeFkPos,
      setInterpolationActive,
      setJointPositions,
      setPaused,
      setSimStatus,
      syncRecorder,
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
    if (!engineRef.current || !sessionRef.current) {
      setSimStatus('error', '模型未就绪，请等待加载完成');
      return;
    }
    if (state.motionTargets.length === 0) {
      if (state.controlLayer === 'joint') {
        await runJointTarget();
      } else {
        await runEeTarget();
      }
      return;
    }
    await runMultiWaypointInterpolation(state.motionTargets);
  }, [runEeTarget, runJointTarget, runMultiWaypointInterpolation, setSimStatus]);

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
    ensureChartSyncLoop();

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
        stopChartSyncLoop();
        setSimStatus(success ? 'ready' : 'error', success ? '轨迹仿真完成' : '轨迹仿真失败');
        resolve();
      });
    });
  }, [
    applyRecorderWindow,
    cancelRafLoop,
    ensureChartSyncLoop,
    makeStepCallback,
    setJointPositions,
    setSimStatus,
    stopChartSyncLoop,
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

  const exportMotionTargetsCsv = useCallback(() => {
    const state = useSessionStore.getState();
    if (!state.robotInfo) {
      setSimStatus('error', '请先加载模型');
      return;
    }
    const base = state.urdfFileName?.replace(/\.urdf$/i, '') ?? state.robotInfo.name;
    downloadMotionTargetsCsv(
      state.motionTargets,
      state.robotInfo.jointNames,
      `${base}_motion_targets.csv`,
    );
    setSimStatus('ready', `已导出 ${state.motionTargets.length} 个运动目标`);
  }, [setSimStatus]);

  const importMotionTargetsCsv = useCallback(
    async (file: File) => {
      const state = useSessionStore.getState();
      if (!state.robotInfo) {
        setSimStatus('error', '请先加载模型');
        return;
      }
      try {
        const text = await file.text();
        const { targets, warnings } = parseMotionTargetsCsv(text, state.robotInfo.jointNames);
        useSessionStore.getState().setMotionTargets(targets);
        const warnHint =
          warnings.length > 0 ? `（${warnings.slice(0, 2).join('；')}）` : '';
        setSimStatus('ready', `已导入 ${targets.length} 个运动目标${warnHint}`);
      } catch (e) {
        setSimStatus('error', e instanceof Error ? e.message : String(e));
      }
    },
    [setSimStatus],
  );

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
      await loadRobot(urdfText, name, meshesRef.current, undefined, 'payload-reload');
    },
    [loadRobot],
  );

  return {
    loadRobot,
    loadDefaultBiped,
    loadTestArm,
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
    exportMotionTargetsCsv,
    importMotionTargetsCsv,
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
