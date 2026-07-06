import { create } from 'zustand';
import type { RecorderDict } from '../core/data-recorder';
import type { PayloadRecord, SpherePayloadMode, Wrench6 } from '../core/payload-editor';
import type { Quat, Vec3 } from '../core/trajectory';
import {
  IK_WEIGHT_DEFAULTS,
  sanitizeIkWeights,
  type IkGoalMode,
  type IkWeights,
} from '../ik/ik-weight-config';
import { CONTROLLER_KD_DAMPING } from '../core/controller';
import type { InterpProfile, MotionTarget } from '../types/motion-target';

export type SimStatus = 'idle' | 'loading' | 'ready' | 'running' | 'error';
export type ControlLayer = 'joint' | 'ee';
export type ControlMode = 'realtime' | 'interpolate';
export type IkLiveStatus = 'idle' | 'solving' | 'converged' | 'failed';

export interface RobotInfo {
  name: string;
  dof: number;
  jointNames: string[];
  lowerLimits: number[];
  upperLimits: number[];
  eePos: Vec3;
  eeQuat: Quat;
}

/** 负载面板表单草稿（跨重载/折叠保持 link、质量等） */
export interface PayloadFormDraft {
  payloadLink: string;
  wrenchLink: string;
  sphereMass: number;
  sphereRadius: number;
  sphereMode: SpherePayloadMode;
  wrenchDraft: Record<string, number>;
}

export interface TrajectoryWaypoint {
  time: number;
  position: Vec3;
  quaternion: Quat;
}

export interface RecorderSnapshot {
  sampleCount: number;
  lastTime: number | null;
}

/** 负载重载等场景下需保留的控制/曲线状态 */
export interface ControlUiPreserve {
  motionTargets: MotionTarget[];
  jointTargets: number[];
  commandedJointPositions: number[];
  referenceJointPositions: number[];
  jointPositions: number[];
  jointKp: number[];
  jointKd: number[];
  eeTarget: Vec3;
  eeTargetQuat: Quat;
  eeTargetDirty: boolean;
  controlLayer: ControlLayer;
  controlMode: ControlMode;
  jointMaxVelocity: number;
  interpProfile: InterpProfile;
  trajectoryWaypoints: TrajectoryWaypoint[];
  recorder: RecorderSnapshot;
  recorderDict: RecorderDict | null;
  simTime: number;
  simStepCount: number;
}

interface SessionState {
  robotInfo: RobotInfo | null;
  jointPositions: number[];
  /** 当前下发给仿真的关节指令（插值 hold 跟踪；与滑条目标 jointTargets 分离） */
  commandedJointPositions: number[];
  jointTargets: number[];
  jointKp: number[];
  jointKd: number[];
  urdfText: string | null;
  urdfFileName: string | null;
  meshAssets: Map<string, Uint8Array>;
  loading: boolean;
  loadingMessage: string;
  simStatus: SimStatus;
  simMessage: string;
  eeTarget: Vec3;
  /** Commanded EE orientation (URDF FK frame, x,y,z,w). */
  eeTargetQuat: Quat;
  trajectoryWaypoints: TrajectoryWaypoint[];
  recorder: RecorderSnapshot;
  recorderDict: RecorderDict | null;
  endEffectorLink: string;
  /** 当前选中末端 link 的正运动学位置 */
  eeFkPos: Vec3 | null;
  /** IK 参考姿态（与仿真 jointPositions 分离，用于 EE 编辑可视化） */
  referenceJointPositions: number[];
  ikEnabled: boolean;
  ikGoalMode: IkGoalMode;
  ikWeights: IkWeights;
  ikLiveError: string | null;
  ikLiveStatus: IkLiveStatus;
  ikLiveMessage: string | null;
  ikLastSolveMs: number | null;
  /** True after panel XYZ edit — gizmo follows eeTarget instead of live FK. */
  eeTargetDirty: boolean;
  ikDragActive: boolean;
  eeGizmoVisible: boolean;
  /** Bumped after visual EE sync so gizmo re-reads main robot FK. */
  eeGizmoSyncVersion: number;
  baseLink: string;
  controlDt: number;
  simTime: number;
  simStepCount: number;
  isPaused: boolean;
  interpolationActive: boolean;
  recorderWindowSec: number;
  /** 仿真运行中暂停写入曲线缓冲 */
  recorderPaused: boolean;
  controlLayer: ControlLayer;
  controlMode: ControlMode;
  /** Max joint velocity for interpolate sends (rad/s). */
  jointMaxVelocity: number;
  /** Kd = controllerKdDamping · √(Kp·M_ii) when auto-computing gains. */
  controllerKdDamping: number;
  /** 插值模式下的多帧目标队列 */
  motionTargets: MotionTarget[];
  /** 多路点关节插值：线性分段 / 三次样条 */
  interpProfile: InterpProfile;
  externalWrenches: Map<string, Wrench6>;
  payloadRecords: PayloadRecord[];
  payloadFormDraft: PayloadFormDraft;

  setLoading: (loading: boolean, message?: string) => void;
  setRobotLoaded: (
    payload: {
      robotInfo: RobotInfo;
      jointPositions: number[];
      urdfText: string;
      urdfFileName: string;
      meshAssets?: Map<string, Uint8Array>;
    },
    options?: { preserve?: ControlUiPreserve },
  ) => void;
  setLoadError: (message: string) => void;
  setJointPositions: (positions: number[]) => void;
  setCommandedJointPositions: (positions: number[]) => void;
  setJointTargets: (targets: number[]) => void;
  setJointTargetAt: (index: number, value: number) => void;
  setJointGainAt: (index: number, kp?: number, kd?: number) => void;
  setAllJointGains: (kp: number[], kd: number[]) => void;
  setEeTarget: (target: Vec3) => void;
  setEeTargetQuat: (quat: Quat) => void;
  setSimStatus: (status: SimStatus, message?: string) => void;
  setTrajectoryWaypoints: (waypoints: TrajectoryWaypoint[]) => void;
  addTrajectoryWaypoint: (waypoint: TrajectoryWaypoint) => void;
  removeTrajectoryWaypoint: (index: number) => void;
  updateRecorder: (snapshot: RecorderSnapshot, dict?: RecorderDict | null) => void;
  /** Batched UI sync during simulation — one store update per tick. */
  syncSimUiFrame: (
    jointPositions: number[],
    simTime: number,
    simStepCount: number,
    recorder: RecorderSnapshot,
  ) => void;
  /** @deprecated use syncSimUiFrame + updateRecorder */
  syncSimFrame: (
    jointPositions: number[],
    simTime: number,
    simStepCount: number,
    recorder: RecorderSnapshot,
    recorderDict?: RecorderDict,
  ) => void;
  setEndEffectorLink: (link: string) => void;
  setEeFkPos: (pos: Vec3 | null) => void;
  setReferenceJointPositions: (positions: number[]) => void;
  setReferenceFromIk: (positions: number[]) => void;
  resetReferenceToCurrent: () => void;
  setIkEnabled: (enabled: boolean) => void;
  setIkGoalMode: (mode: IkGoalMode) => void;
  setIkWeights: (weights: IkWeights) => void;
  setIkLiveError: (error: string | null) => void;
  setIkLiveStatus: (status: IkLiveStatus, message?: string | null, ms?: number | null) => void;
  setEeTargetDirty: (dirty: boolean) => void;
  setIkDragActive: (active: boolean) => void;
  setEeGizmoVisible: (visible: boolean) => void;
  bumpEeGizmoSyncVersion: () => void;
  setBaseLink: (link: string) => void;
  setControlDt: (dt: number) => void;
  setSimRuntime: (simTime: number, simStepCount: number) => void;
  setPaused: (paused: boolean) => void;
  setInterpolationActive: (active: boolean) => void;
  setRecorderWindowSec: (sec: number) => void;
  setRecorderPaused: (paused: boolean) => void;
  setControlLayer: (layer: ControlLayer) => void;
  setControlMode: (mode: ControlMode) => void;
  setJointMaxVelocity: (jointMaxVelocity: number) => void;
  setControllerKdDamping: (value: number) => void;
  addMotionTarget: (target: MotionTarget) => void;
  removeMotionTarget: (id: string) => void;
  clearMotionTargets: () => void;
  setMotionTargets: (targets: MotionTarget[]) => void;
  setInterpProfile: (profile: InterpProfile) => void;
  setExternalWrench: (link: string, wrench: Wrench6) => void;
  clearExternalWrenches: () => void;
  addPayloadRecord: (record: PayloadRecord) => void;
  removePayloadRecord: (id: string) => void;
  clearPayloadRecords: () => void;
  setPayloadFormDraft: (patch: Partial<PayloadFormDraft>) => void;
  reset: () => void;
}

const initialRecorder: RecorderSnapshot = { sampleCount: 0, lastTime: null };

const initialPayloadFormDraft: PayloadFormDraft = {
  payloadLink: '',
  wrenchLink: '',
  sphereMass: 0.2,
  sphereRadius: 0.03,
  sphereMode: 'child_link',
  wrenchDraft: { fx: 0, fy: 0, fz: 0, tx: 0, ty: 0, tz: 0 },
};

const initialState = {
  robotInfo: null as RobotInfo | null,
  jointPositions: [] as number[],
  commandedJointPositions: [] as number[],
  jointTargets: [] as number[],
  jointKp: [] as number[],
  jointKd: [] as number[],
  urdfText: null as string | null,
  urdfFileName: null as string | null,
  meshAssets: new Map<string, Uint8Array>(),
  loading: false,
  loadingMessage: '',
  simStatus: 'idle' as SimStatus,
  simMessage: '',
  eeTarget: [0.3, 0, 0.4] as Vec3,
  eeTargetQuat: [0, 0, 0, 1] as Quat,
  trajectoryWaypoints: [] as TrajectoryWaypoint[],
  recorder: initialRecorder,
  recorderDict: null as RecorderDict | null,
  endEffectorLink: 'ee_link',
  eeFkPos: null as Vec3 | null,
  referenceJointPositions: [] as number[],
  ikEnabled: true,
  ikGoalMode: 'position' as IkGoalMode,
  ikWeights: sanitizeIkWeights(IK_WEIGHT_DEFAULTS),
  ikLiveError: null as string | null,
  ikLiveStatus: 'idle' as IkLiveStatus,
  ikLiveMessage: null as string | null,
  ikLastSolveMs: null as number | null,
  eeTargetDirty: false,
  ikDragActive: false,
  eeGizmoVisible: false,
  eeGizmoSyncVersion: 0,
  baseLink: 'base_link',
  controlDt: 0.002,
  simTime: 0,
  simStepCount: 0,
  isPaused: false,
  interpolationActive: false,
  recorderWindowSec: 30,
  recorderPaused: false,
  controlLayer: 'joint' as ControlLayer,
  controlMode: 'interpolate' as ControlMode,
  jointMaxVelocity: 0.6,
  controllerKdDamping: CONTROLLER_KD_DAMPING,
  motionTargets: [] as MotionTarget[],
  interpProfile: 'cubic' as InterpProfile,
  externalWrenches: new Map<string, Wrench6>(),
  payloadRecords: [] as PayloadRecord[],
  payloadFormDraft: { ...initialPayloadFormDraft, wrenchDraft: { ...initialPayloadFormDraft.wrenchDraft } },
};

export const useSessionStore = create<SessionState>((set) => ({
  ...initialState,

  setLoading: (loading, message = '') =>
    set({ loading, loadingMessage: message, simStatus: loading ? 'loading' : 'idle' }),

  setRobotLoaded: (payload, options) => {
    const base = {
      robotInfo: payload.robotInfo,
      urdfText: payload.urdfText,
      urdfFileName: payload.urdfFileName,
      meshAssets: payload.meshAssets ? new Map(payload.meshAssets) : new Map(),
      loading: false,
      loadingMessage: '',
      simStatus: 'ready' as SimStatus,
      simMessage: options?.preserve ? '模型已重载（保留控制与曲线）' : '模型已加载',
      ikLiveError: null,
      ikLiveStatus: 'idle' as IkLiveStatus,
      ikLiveMessage: null,
      ikLastSolveMs: null,
    };
    const p = options?.preserve;
    if (p) {
      set({
        ...base,
        jointPositions: [...p.jointPositions],
        commandedJointPositions: [...p.commandedJointPositions],
        jointTargets: [...p.jointTargets],
        referenceJointPositions: [...p.referenceJointPositions],
        jointKp: [...p.jointKp],
        jointKd: [...p.jointKd],
        eeTarget: [...p.eeTarget] as Vec3,
        eeTargetQuat: [...p.eeTargetQuat] as Quat,
        eeTargetDirty: p.eeTargetDirty,
        controlLayer: p.controlLayer,
        controlMode: p.controlMode,
        jointMaxVelocity: p.jointMaxVelocity,
        interpProfile: p.interpProfile,
        motionTargets: p.motionTargets.map((t) => ({ ...t, jointPositions: [...t.jointPositions] })),
        trajectoryWaypoints: p.trajectoryWaypoints.map((w) => ({ ...w })),
        recorder: { ...p.recorder },
        recorderDict: p.recorderDict,
        simTime: p.simTime,
        simStepCount: p.simStepCount,
      });
      return;
    }
    set({
      ...base,
      jointPositions: payload.jointPositions,
      commandedJointPositions: [...payload.jointPositions],
      jointTargets: [...payload.jointPositions],
      referenceJointPositions: [...payload.jointPositions],
      eeTarget: [...payload.robotInfo.eePos] as Vec3,
      eeTargetDirty: false,
      motionTargets: [],
      recorder: initialRecorder,
      recorderDict: null,
      simTime: 0,
      simStepCount: 0,
    });
  },

  setLoadError: (message) =>
    set({
      loading: false,
      simStatus: 'error',
      simMessage: message,
    }),

  setJointPositions: (positions) => set({ jointPositions: [...positions] }),

  setCommandedJointPositions: (positions) =>
    set({ commandedJointPositions: [...positions] }),

  setJointTargets: (targets) => set({ jointTargets: [...targets] }),

  setJointTargetAt: (index, value) =>
    set((state) => {
      const jointTargets = [...state.jointTargets];
      jointTargets[index] = value;
      return { jointTargets };
    }),

  setJointGainAt: (index, kp, kd) =>
    set((state) => {
      const jointKp = [...state.jointKp];
      const jointKd = [...state.jointKd];
      if (kp !== undefined) jointKp[index] = kp;
      if (kd !== undefined) jointKd[index] = kd;
      return { jointKp, jointKd };
    }),

  setAllJointGains: (jointKp, jointKd) =>
    set({ jointKp: [...jointKp], jointKd: [...jointKd] }),

  setEeTarget: (target) =>
    set((state) => {
      const next = [...target] as Vec3;
      const cur = state.eeTarget;
      const eps = 1e-5;
      if (
        Math.abs(cur[0] - next[0]!) < eps &&
        Math.abs(cur[1] - next[1]!) < eps &&
        Math.abs(cur[2] - next[2]!) < eps
      ) {
        return state;
      }
      return { eeTarget: next };
    }),

  setEeTargetQuat: (eeTargetQuat) => set({ eeTargetQuat: [...eeTargetQuat] as Quat }),

  setSimStatus: (simStatus, message = '') => set({ simStatus, simMessage: message }),

  setTrajectoryWaypoints: (waypoints) => set({ trajectoryWaypoints: waypoints }),

  addTrajectoryWaypoint: (waypoint) =>
    set((state) => ({
      trajectoryWaypoints: [...state.trajectoryWaypoints, waypoint].sort(
        (a, b) => a.time - b.time,
      ),
    })),

  removeTrajectoryWaypoint: (index) =>
    set((state) => ({
      trajectoryWaypoints: state.trajectoryWaypoints.filter((_, i) => i !== index),
    })),

  updateRecorder: (recorder, dict) =>
    set({
      recorder,
      ...(dict !== undefined ? { recorderDict: dict } : {}),
    }),

  syncSimUiFrame: (jointPositions, simTime, simStepCount, recorder) =>
    set({
      jointPositions: [...jointPositions],
      simTime,
      simStepCount,
      recorder,
    }),

  syncSimFrame: (jointPositions, simTime, simStepCount, recorder, recorderDict) =>
    set({
      jointPositions: [...jointPositions],
      simTime,
      simStepCount,
      recorder,
      ...(recorderDict !== undefined ? { recorderDict } : {}),
    }),

  setEndEffectorLink: (link) => set({ endEffectorLink: link }),

  setEeFkPos: (eeFkPos) => set({ eeFkPos }),

  setReferenceJointPositions: (referenceJointPositions) =>
    set({ referenceJointPositions: [...referenceJointPositions], ikLiveError: null }),

  setReferenceFromIk: (positions) =>
    set({
      referenceJointPositions: [...positions],
      jointTargets: [...positions],
      ikLiveError: null,
    }),

  resetReferenceToCurrent: () =>
    set((state) => ({
      referenceJointPositions: [...state.jointPositions],
      ikLiveError: null,
    })),

  setIkEnabled: (ikEnabled) => set({ ikEnabled }),

  setIkGoalMode: (ikGoalMode) => set({ ikGoalMode }),

  setIkWeights: (ikWeights) => set({ ikWeights: sanitizeIkWeights(ikWeights) }),

  setIkLiveError: (ikLiveError) =>
    set({ ikLiveError, ikLiveMessage: ikLiveError }),

  setIkLiveStatus: (ikLiveStatus, message = null, ms = null) =>
    set({
      ikLiveStatus,
      ikLiveMessage: message,
      ...(ms !== null ? { ikLastSolveMs: ms } : {}),
      ...(message !== undefined ? { ikLiveError: message } : {}),
    }),

  setEeTargetDirty: (eeTargetDirty) => set({ eeTargetDirty }),

  setIkDragActive: (ikDragActive) => set({ ikDragActive }),

  setEeGizmoVisible: (eeGizmoVisible) => set({ eeGizmoVisible }),

  bumpEeGizmoSyncVersion: () =>
    set((state) => ({ eeGizmoSyncVersion: state.eeGizmoSyncVersion + 1 })),

  setBaseLink: (link) => set({ baseLink: link }),

  setControlDt: (controlDt) => set({ controlDt }),

  setSimRuntime: (simTime, simStepCount) => set({ simTime, simStepCount }),

  setPaused: (isPaused) => set({ isPaused }),

  setInterpolationActive: (interpolationActive) => set({ interpolationActive }),

  setRecorderWindowSec: (recorderWindowSec) => set({ recorderWindowSec }),

  setRecorderPaused: (recorderPaused) => set({ recorderPaused }),

  setControlLayer: (controlLayer) =>
    set((state) => ({
      controlLayer,
      ...(controlLayer === 'joint'
        ? { eeGizmoVisible: false, ikDragActive: false, eeTargetDirty: false }
        : {
            ikEnabled: true,
            eeTargetDirty: false,
            ...(state.referenceJointPositions.length === 0 && state.jointPositions.length > 0
              ? { referenceJointPositions: [...state.jointPositions] }
              : {}),
          }),
    })),

  setControlMode: (controlMode) => set({ controlMode }),

  setJointMaxVelocity: (jointMaxVelocity) => set({ jointMaxVelocity }),

  setControllerKdDamping: (controllerKdDamping) =>
    set({ controllerKdDamping: Math.max(0.5, Math.min(10, controllerKdDamping)) }),

  addMotionTarget: (target) =>
    set((state) => ({ motionTargets: [...state.motionTargets, target] })),

  removeMotionTarget: (id) =>
    set((state) => ({
      motionTargets: state.motionTargets.filter((t) => t.id !== id),
    })),

  clearMotionTargets: () => set({ motionTargets: [] }),

  setMotionTargets: (motionTargets) =>
    set({
      motionTargets: motionTargets.map((t) => ({
        ...t,
        jointPositions: [...t.jointPositions],
        eePosition: [...t.eePosition] as Vec3,
        eeQuaternion: [...t.eeQuaternion] as Quat,
        eeSceneWorld: [...t.eeSceneWorld] as [number, number, number],
      })),
    }),

  setInterpProfile: (interpProfile) => set({ interpProfile }),

  setExternalWrench: (link, wrench) =>
    set((state) => {
      const externalWrenches = new Map(state.externalWrenches);
      externalWrenches.set(link, [...wrench] as Wrench6);
      return { externalWrenches };
    }),

  clearExternalWrenches: () => set({ externalWrenches: new Map() }),

  addPayloadRecord: (record) =>
    set((state) => ({ payloadRecords: [...state.payloadRecords, record] })),

  removePayloadRecord: (id) =>
    set((state) => ({
      payloadRecords: state.payloadRecords.filter((item) => item.id !== id),
    })),

  clearPayloadRecords: () => set({ payloadRecords: [] }),

  setPayloadFormDraft: (patch) =>
    set((state) => ({
      payloadFormDraft: {
        ...state.payloadFormDraft,
        ...patch,
        wrenchDraft: patch.wrenchDraft
          ? { ...state.payloadFormDraft.wrenchDraft, ...patch.wrenchDraft }
          : state.payloadFormDraft.wrenchDraft,
      },
    })),

  reset: () =>
    set({
      ...initialState,
      externalWrenches: new Map(),
      payloadRecords: [],
      payloadFormDraft: {
        ...initialPayloadFormDraft,
        wrenchDraft: { ...initialPayloadFormDraft.wrenchDraft },
      },
    }),
}));

if (import.meta.env.DEV && typeof window !== 'undefined') {
  (window as unknown as { __sessionStore: typeof useSessionStore }).__sessionStore = useSessionStore;
}
