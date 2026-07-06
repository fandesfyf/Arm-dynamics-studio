import type { MjData, MjModel } from '@mujoco/mujoco';
import { vecGet, vecSet } from '../types/mujoco';
import type {
  RunToEeTargetOptions,
  RunToTargetOptions,
  RunTrajectoryOptions,
  SimulationStepState,
} from '../types/simulation';
import { angleDiff, ComputedTorqueController, gainsFromMassDiagonal, type AutoGainOptions } from './controller';
import { JointMultiWaypointPlanner, type JointInterpProfile } from './joint-waypoint-planner';
import { DataRecorder, type SimulationRecord } from './data-recorder';
import {
  ConstantJointPlanner,
  JointInterpolationPlanner,
  JointVelocityLimitPlanner,
  type TrajectorySampler,
  TrajectoryPosePlanner,
} from './planner';
import type { ForwardKinematics, RobotSession } from './robot-session';
import { mjStep } from '../mujoco/step';
import { applyExternalWrenches, clearExternalWrenches } from '../mujoco/external-wrench';
import type { Wrench6 } from './payload-editor';
import type { IkGoalMode, IkWeights } from '../ik/ik-weight-config';

export interface IkSolveOptions {
  liveDrag?: boolean;
  dragEnd?: boolean;
  goalMode?: IkGoalMode;
  weights?: IkWeights;
  targetSceneWorld?: [number, number, number];
  /** Three.js world quaternion (x,y,z,w) for pose IK. */
  targetSceneQuaternion?: [number, number, number, number];
}

/** T4 逆运动学求解接口（runToEeTarget / runTrajectory 依赖） */
export interface IKSolver {
  solve(
    pos: number[],
    quat: number[],
    qInit: Float64Array,
    options?: IkSolveOptions,
  ): { q: Float64Array; converged: boolean; message?: string };
}

export const CONTROL_DT = 0.002;

interface RealtimePaceState {
  wallStartMs: number;
  stepsCompleted: number;
  /** 暂停期间累计的墙钟偏移，避免恢复后追赶步进 */
  pauseSlipMs: number;
}

function createRealtimePaceState(enabled: boolean): RealtimePaceState | null {
  if (!enabled) return null;
  return { wallStartMs: performance.now(), stepsCompleted: 0, pauseSlipMs: 0 };
}

async function awaitRealtimePace(
  controlDt: number,
  state: RealtimePaceState | null,
): Promise<void> {
  if (!state) return;
  state.stepsCompleted += 1;
  const targetMs =
    state.wallStartMs + state.pauseSlipMs + state.stepsCompleted * controlDt * 1000;
  const delay = targetMs - performance.now();
  if (delay > 0) {
    await new Promise<void>((resolve) => setTimeout(resolve, delay));
  }
}

async function waitUnlessCancelled(
  options: RunToTargetOptions,
  paceState: RealtimePaceState | null,
): Promise<boolean> {
  let pauseAnchor: number | null = null;
  while (options.pauseCheck?.()) {
    if (pauseAnchor === null) pauseAnchor = performance.now();
    await new Promise((r) => setTimeout(r, 50));
    if (options.cancelCheck?.()) return false;
  }
  if (pauseAnchor !== null && paceState) {
    paceState.pauseSlipMs += performance.now() - pauseAnchor;
  }
  return !options.cancelCheck?.();
}

async function afterSimulationStep(
  controlDt: number,
  stepIndex: number,
  options: RunToTargetOptions,
  paceState: RealtimePaceState | null,
): Promise<void> {
  if (paceState) {
    await awaitRealtimePace(controlDt, paceState);
    return;
  }
  const yieldEvery = options.yieldEvery ?? 25;
  if (options.onYield && (stepIndex + 1) % yieldEvery === 0) {
    await options.onYield();
  }
}

function arrayNorm(v: ArrayLike<number>): number {
  let sum = 0;
  for (let i = 0; i < v.length; i++) {
    const x = v[i] ?? 0;
    sum += x * x;
  }
  return Math.sqrt(sum);
}

function applyTorque(data: MjData, tau: Float64Array, nu: number, nv: number): void {
  if (nu > 0) {
    vecSet(data.ctrl, tau, nu);
  } else {
    vecSet(data.qfrc_applied, tau, nv);
  }
}

function readQposN(model: MjModel, data: MjData, nq: number): Float64Array {
  void model;
  return vecGet(data.qpos, nq);
}

function readQvelN(model: MjModel, data: MjData, nv: number): Float64Array {
  void model;
  return vecGet(data.qvel, nv);
}

function readTau(data: MjData, model: MjModel, nv: number): Float64Array {
  if (model.nu > 0) {
    return vecGet(data.actuator_force, nv);
  }
  return vecGet(data.qfrc_applied, nv);
}

function buildStepState(
  time: number,
  model: MjModel,
  data: MjData,
  nq: number,
  nv: number,
  qDesired: Float64Array,
  qvelDesired: Float64Array,
  tauCommanded: Float64Array,
  fk: ForwardKinematics,
): SimulationStepState {
  const qpos = readQposN(model, data, nq);
  const ee = fk.compute(qpos);
  return {
    time,
    qpos: new Float64Array(qpos),
    qvel: readQvelN(model, data, nv),
    tau: readTau(data, model, nv),
    q_desired: new Float64Array(qDesired),
    qvel_desired: new Float64Array(qvelDesired),
    tau_commanded: new Float64Array(tauCommanded),
    ee_pos: [...ee.pos],
    ee_quat: [...ee.quat],
  };
}

/**
 * MuJoCo 仿真循环 — runToTarget / runToEeTarget / runTrajectory。
 * 对应旧版 simulation.py / WEB_IMPLEMENTATION_PLAN.md §5.6。
 */
export class SimulationEngine {
  controlDt = CONTROL_DT;
  simTime = 0;
  isRunning = false;
  /** 为 false 时仿真继续但不再写入 recorder */
  recordingEnabled = true;
  readonly recorder = new DataRecorder();
  externalWrenches = new Map<string, Wrench6>();

  private qPrev: Float64Array | null = null;
  private lastQDesired = new Float64Array(0);
  private lastQvelDesired = new Float64Array(0);
  private lastTauCmd = new Float64Array(0);

  constructor(
    private readonly session: RobotSession,
    private readonly controller: ComputedTorqueController,
  ) {}

  get physicsDt(): number {
    return this.session.physicsDt;
  }

  getGains(): { kp: Float64Array; kd: Float64Array } {
    return this.controller.getGains();
  }

  setGains(kp: ArrayLike<number>, kd: ArrayLike<number>): void {
    this.controller.setGains(kp, kd);
  }

  /** Recompute Kp/Kd from diag(mj_fullM(q)) and apply to controller. */
  recomputeAutoGains(q: ArrayLike<number>, options?: AutoGainOptions): { kp: Float64Array; kd: Float64Array } {
    const diagM = this.controller.massMatrixDiagonal(q);
    const gains = gainsFromMassDiagonal(diagM, options);
    this.controller.setGains(gains.kp, gains.kd);
    return gains;
  }

  reset(options?: { preserveRecorder?: boolean }): void {
    this.session.mujoco.mj_resetData(this.session.model, this.session.data);
    this.session.mujoco.mj_forward(this.session.model, this.session.data);
    clearExternalWrenches(this.session.mujoco, this.session.model, this.session.data);
    this.isRunning = false;
    this.qPrev = null;
    if (options?.preserveRecorder) {
      const times = this.recorder.getTimes();
      this.simTime = times.length > 0 ? times[times.length - 1]! : this.simTime;
    } else {
      this.simTime = 0;
      this.recorder.clear();
    }
  }

  /** 实时曲线采样：用当前物理状态 + 最近指令，时间轴由调用方填入墙钟秒 */
  sampleForChart(): Omit<SimulationRecord, 'time'> {
    const { model, data, nq, nv, forwardKinematics: fk } = this.session;
    const q_d =
      this.lastQDesired.length === nq
        ? this.lastQDesired
        : readQposN(model, data, nq);
    const v_d =
      this.lastQvelDesired.length === nv
        ? this.lastQvelDesired
        : readQvelN(model, data, nv);
    const tau =
      this.lastTauCmd.length === nv ? this.lastTauCmd : readTau(data, model, nv);
    const state = buildStepState(this.simTime, model, data, nq, nv, q_d, v_d, tau, fk);
    return {
      qpos: Array.from(state.qpos),
      qvel: Array.from(state.qvel),
      tau: Array.from(state.tau),
      q_desired: Array.from(state.q_desired),
      qvel_desired: Array.from(state.qvel_desired),
      tau_commanded: Array.from(state.tau_commanded),
      ee_pos: [...state.ee_pos],
      ee_quat: [...state.ee_quat],
    };
  }

  /** 单步 hold 控制：跟踪恒定 qDesired（实时循环用） */
  stepHoldTarget(
    qDesired: ArrayLike<number>,
    stepCallback?: (state: SimulationStepState) => void,
  ): void {
    const nq = this.session.nq;
    const nv = this.session.nv;
    const q_d = Float64Array.from(qDesired);
    const v_d = new Float64Array(nv);
    const a_d = new Float64Array(nv);
    const qCurrent = readQposN(this.session.model, this.session.data, nq);
    const qvelCurrent = readQvelN(this.session.model, this.session.data, nv);
    const tau = this.controller.computeTorque(qCurrent, qvelCurrent, q_d, v_d, a_d);
    this.integrateControlStep(tau);
    this.emitStep(this.simTime, q_d, v_d, tau, stepCallback);
  }

  /** 关节空间插值：从当前 qpos 线性插值到 qEnd，固定时长 */
  async runInterpolationAsync(
    qEnd: ArrayLike<number>,
    duration: number,
    options: RunToTargetOptions = {},
  ): Promise<boolean> {
    const nq = this.session.nq;
    const nv = this.session.nv;
    const qStart = readQposN(this.session.model, this.session.data, nq);
    const planner = new JointInterpolationPlanner(qStart, qEnd, duration, nv);
    const maxSteps = Math.max(1, Math.ceil(duration / this.controlDt));
    const paceState = createRealtimePaceState(options.realtimePacing !== false);

    this.isRunning = true;

    for (let i = 0; i < maxSteps; i++) {
      if (!(await waitUnlessCancelled(options, paceState))) {
        this.isRunning = false;
        options.doneCallback?.(false, '用户取消');
        return false;
      }

      const t = i * this.controlDt;
      const { q_d, v_d, a_d } = planner.getDesired(t);
      const qCurrent = readQposN(this.session.model, this.session.data, nq);
      const qvelCurrent = readQvelN(this.session.model, this.session.data, nv);
      const tau = this.controller.computeTorque(qCurrent, qvelCurrent, q_d, v_d, a_d);
      this.integrateControlStep(tau);
      this.emitStep(this.simTime, q_d, v_d, tau, options.stepCallback);

      await afterSimulationStep(this.controlDt, i, options, paceState);
    }

    this.isRunning = false;
    options.doneCallback?.(true, `插值完成 (${duration.toFixed(2)} s)`);
    return true;
  }

  /** 关节空间插值：从 qStart 线性插值到 qEnd，关节角速度上限 maxVel (rad/s)。 */
  async runVelocityLimitedInterpolationAsync(
    qStart: ArrayLike<number>,
    qEnd: ArrayLike<number>,
    maxVel: number | number[],
    options: RunToTargetOptions = {},
  ): Promise<boolean> {
    const nq = this.session.nq;
    const nv = this.session.nv;
    const planner = new JointVelocityLimitPlanner(qStart, qEnd, maxVel, nv);
    const duration = planner.getDuration();
    const maxSteps = Math.max(1, Math.ceil(duration / this.controlDt) + 2);
    const tol = options.tol ?? 0.01;
    const paceState = createRealtimePaceState(options.realtimePacing !== false);

    this.isRunning = true;

    for (let i = 0; i < maxSteps; i++) {
      if (!(await waitUnlessCancelled(options, paceState))) {
        this.isRunning = false;
        options.doneCallback?.(false, '用户取消');
        return false;
      }

      const t = i * this.controlDt;
      const { q_d, v_d, a_d } = planner.getDesired(t);
      const qCurrent = readQposN(this.session.model, this.session.data, nq);
      const qvelCurrent = readQvelN(this.session.model, this.session.data, nv);
      const tau = this.controller.computeTorque(qCurrent, qvelCurrent, q_d, v_d, a_d);
      this.integrateControlStep(tau);
      this.emitStep(this.simTime, q_d, v_d, tau, options.stepCallback);

      if (planner.isSettled(t, tol)) {
        break;
      }

      await afterSimulationStep(this.controlDt, i, options, paceState);
    }

    this.isRunning = false;
    options.doneCallback?.(true, `插值完成 (${duration.toFixed(2)} s)`);
    return true;
  }

  /** 多路点关节插值：qWaypoints[0] 为起点，后续为各帧目标。 */
  async runMultiWaypointInterpolationAsync(
    qWaypoints: ArrayLike<number>[],
    maxVel: number | number[],
    profile: JointInterpProfile,
    options: RunToTargetOptions = {},
  ): Promise<boolean> {
    if (qWaypoints.length < 2) {
      options.doneCallback?.(false, '至少需要 2 个路点');
      return false;
    }

    const nq = this.session.nq;
    const nv = this.session.nv;
    const planner = new JointMultiWaypointPlanner(qWaypoints, maxVel, nv, profile);
    const duration = planner.getDuration();
    const maxSteps = Math.max(1, Math.ceil(duration / this.controlDt) + 2);
    const tol = options.tol ?? 0.01;
    const paceState = createRealtimePaceState(options.realtimePacing !== false);

    this.isRunning = true;

    for (let i = 0; i < maxSteps; i++) {
      if (!(await waitUnlessCancelled(options, paceState))) {
        this.isRunning = false;
        options.doneCallback?.(false, '用户取消');
        return false;
      }

      const t = i * this.controlDt;
      const { q_d, v_d, a_d } = planner.getDesired(t);
      const qCurrent = readQposN(this.session.model, this.session.data, nq);
      const qvelCurrent = readQvelN(this.session.model, this.session.data, nv);
      const tau = this.controller.computeTorque(qCurrent, qvelCurrent, q_d, v_d, a_d);
      this.integrateControlStep(tau);
      this.emitStep(this.simTime, q_d, v_d, tau, options.stepCallback);

      if (planner.isSettled(t, tol)) {
        break;
      }

      await afterSimulationStep(this.controlDt, i, options, paceState);
    }

    this.isRunning = false;
    options.doneCallback?.(true, `多路点插值完成 (${duration.toFixed(2)} s)`);
    return true;
  }

  /** 关节角目标：q_d 恒定，v_d=0，误差低于阈值后停止 */
  runToTarget(qTarget: ArrayLike<number>, options: RunToTargetOptions = {}): boolean {
    return this._runToTargetSync(qTarget, options);
  }

  async runToTargetAsync(
    qTarget: ArrayLike<number>,
    options: RunToTargetOptions = {},
  ): Promise<boolean> {
    const maxTime = options.maxTime ?? 10.0;
    const tol = options.tol ?? 0.01;
    const nq = this.session.nq;
    const nv = this.session.nv;
    const planner = new ConstantJointPlanner(qTarget, nv);
    const target = planner.getTarget();
    const paceState = createRealtimePaceState(options.realtimePacing !== false);

    this.isRunning = true;
    const maxSteps = Math.ceil(maxTime / this.controlDt);
    let reached = false;

    for (let i = 0; i < maxSteps; i++) {
      if (!(await waitUnlessCancelled(options, paceState))) {
        this.isRunning = false;
        options.doneCallback?.(false, '用户取消');
        return false;
      }

      const t = i * this.controlDt;
      const { q_d, v_d, a_d } = planner.getDesired(t);
      const qCurrent = readQposN(this.session.model, this.session.data, nq);
      const qvelCurrent = readQvelN(this.session.model, this.session.data, nv);

      const tau = this.controller.computeTorque(qCurrent, qvelCurrent, q_d, v_d, a_d);
      this.integrateControlStep(tau);

      const posError = arrayNorm(angleDiff(target, qCurrent));
      const velError = arrayNorm(qvelCurrent);
      if (posError < tol && velError < tol * 10) {
        reached = true;
        this.emitStep(this.simTime, q_d, v_d, tau, options.stepCallback);
        break;
      }

      this.emitStep(this.simTime, q_d, v_d, tau, options.stepCallback);

      await afterSimulationStep(this.controlDt, i, options, paceState);
    }

    this.isRunning = false;
    const finalQ = readQposN(this.session.model, this.session.data, nq);
    const finalError = arrayNorm(angleDiff(target, finalQ));
    if (reached) {
      options.doneCallback?.(true, `到达目标 (误差: ${finalError.toFixed(4)} rad)`);
      return true;
    }
    options.doneCallback?.(false, `超时 (剩余误差: ${finalError.toFixed(4)} rad)`);
    return false;
  }

  private _runToTargetSync(qTarget: ArrayLike<number>, options: RunToTargetOptions = {}): boolean {
    const maxTime = options.maxTime ?? 10.0;
    const tol = options.tol ?? 0.01;
    const nq = this.session.nq;
    const nv = this.session.nv;
    const planner = new ConstantJointPlanner(qTarget, nv);
    const target = planner.getTarget();

    this.isRunning = true;
    const maxSteps = Math.ceil(maxTime / this.controlDt);
    let reached = false;

    for (let i = 0; i < maxSteps; i++) {
      if (options.cancelCheck?.()) {
        this.isRunning = false;
        options.doneCallback?.(false, '用户取消');
        return false;
      }

      const t = i * this.controlDt;
      const { q_d, v_d, a_d } = planner.getDesired(t);
      const qCurrent = readQposN(this.session.model, this.session.data, nq);
      const qvelCurrent = readQvelN(this.session.model, this.session.data, nv);

      const tau = this.controller.computeTorque(qCurrent, qvelCurrent, q_d, v_d, a_d);
      this.integrateControlStep(tau);

      const posError = arrayNorm(angleDiff(target, qCurrent));
      const velError = arrayNorm(qvelCurrent);
      if (posError < tol && velError < tol * 10) {
        reached = true;
        break;
      }

      this.emitStep(this.simTime, q_d, v_d, tau, options.stepCallback);
    }

    this.isRunning = false;
    const finalQ = readQposN(this.session.model, this.session.data, nq);
    const finalError = arrayNorm(angleDiff(target, finalQ));
    if (reached) {
      options.doneCallback?.(true, `到达目标 (误差: ${finalError.toFixed(4)} rad)`);
      return true;
    }
    options.doneCallback?.(false, `超时 (剩余误差: ${finalError.toFixed(4)} rad)`);
    return false;
  }

  /** 末端 XYZ 目标：先 IK 求 q_target，再 runToTarget */
  runToEeTarget(
    eePos: number[],
    ikSolver: IKSolver,
    options: RunToEeTargetOptions = {},
  ): boolean {
    const nq = this.session.nq;
    const qInit =
      options.qInit !== undefined && options.qInit !== null
        ? Float64Array.from(options.qInit)
        : readQposN(this.session.model, this.session.data, nq);

    let eeQuat = options.eeQuat;
    if (eeQuat === undefined || eeQuat === null) {
      eeQuat = this.session.forwardKinematics.compute(qInit).quat;
    }

    const ik = ikSolver.solve(eePos, eeQuat, new Float64Array(qInit));
    if (!ik.converged) {
      options.doneCallback?.(false, ik.message ?? 'IK 求解失败，目标位置可能不可达');
      return false;
    }

    return this.runToTarget(ik.q, options);
  }

  /** 按 TrajectorySampler 播放整段轨迹 */
  runTrajectory(
    sampler: TrajectorySampler,
    ikSolver: IKSolver,
    options: RunTrajectoryOptions = {},
  ): boolean {
    const posePlanner = new TrajectoryPosePlanner(sampler);
    const duration = options.totalTime ?? posePlanner.getDuration();
    if (duration <= 0) return false;

    const nq = this.session.nq;
    const nv = this.session.nv;
    const nSteps = Math.ceil(duration / this.controlDt);

    this.isRunning = true;
    this.qPrev = readQposN(this.session.model, this.session.data, nq);

    const pos0 = posePlanner.sampleEePose(0);
    let qPrevIk = new Float64Array(this.qPrev);
    {
      const ik0 = ikSolver.solve(pos0.pos, pos0.quat, qPrevIk);
      if (ik0.converged) qPrevIk = ik0.q;
    }

    for (let i = 0; i < nSteps; i++) {
      if (options.cancelCheck?.()) {
        this.isRunning = false;
        return false;
      }

      const t = i * this.controlDt;
      const { pos, quat } = posePlanner.sampleEePose(t);
      const ik = ikSolver.solve(pos, quat, qPrevIk);
      const q_d = ik.converged ? ik.q : qPrevIk;
      qPrevIk = new Float64Array(q_d);

      const qvel_d = this.numericalDesiredVelocity(q_d, t, posePlanner, ikSolver);
      const qacc_d = this.numericalDesiredAcceleration(qvel_d, q_d);

      const qCurrent = readQposN(this.session.model, this.session.data, nq);
      const qvelCurrent = readQvelN(this.session.model, this.session.data, nv);
      const tau = this.controller.computeTorque(qCurrent, qvelCurrent, q_d, qvel_d, qacc_d);
      this.integrateControlStep(tau);

      this.emitStep(this.simTime, q_d, qvel_d, tau, options.stepCallback);
      options.progressCallback?.((i + 1) / nSteps);
    }

    this.isRunning = false;
    return true;
  }

  private integrateControlStep(tau: Float64Array): void {
    const subSteps = Math.max(1, Math.round(this.controlDt / this.physicsDt));
    for (let s = 0; s < subSteps; s++) {
      clearExternalWrenches(this.session.mujoco, this.session.model, this.session.data);
      applyTorque(this.session.data, tau, this.session.nu, this.session.nv);
      applyExternalWrenches(
        this.session.mujoco,
        this.session.model,
        this.session.data,
        this.externalWrenches,
        this.session.nv,
        {
          linkBodyBindings: this.session.linkBodyBindings,
          baseLink: this.session.baseLink,
          zeroQfrcBeforeApply: this.session.nu > 0,
        },
      );
      mjStep(this.session.mujoco, this.session.model, this.session.data);
      this.simTime += this.physicsDt;
    }
  }

  private emitStep(
    time: number,
    qDesired: Float64Array,
    qvelDesired: Float64Array,
    tauCommanded: Float64Array,
    stepCallback?: (state: SimulationStepState) => void,
  ): void {
    this.lastQDesired = new Float64Array(qDesired);
    this.lastQvelDesired = new Float64Array(qvelDesired);
    this.lastTauCmd = new Float64Array(tauCommanded);
    const state = buildStepState(
      time,
      this.session.model,
      this.session.data,
      this.session.nq,
      this.session.nv,
      qDesired,
      qvelDesired,
      tauCommanded,
      this.session.forwardKinematics,
    );
    if (this.recordingEnabled) {
      this.recorder.record({
        time: state.time,
        qpos: Array.from(state.qpos),
        qvel: Array.from(state.qvel),
        tau: Array.from(state.tau),
        q_desired: Array.from(state.q_desired),
        qvel_desired: Array.from(state.qvel_desired),
        tau_commanded: Array.from(state.tau_commanded),
        ee_pos: state.ee_pos as [number, number, number],
        ee_quat: state.ee_quat as [number, number, number, number],
      });
    }
    stepCallback?.(state);
  }

  private numericalDesiredVelocity(
    q_d: Float64Array,
    t: number,
    posePlanner: TrajectoryPosePlanner,
    ikSolver: IKSolver,
  ): Float64Array {
    const dt = 1e-4;
    const duration = posePlanner.getDuration();
    const tNext = Math.min(t + dt, duration);
    const { pos, quat } = posePlanner.sampleEePose(tNext);
    const ik = ikSolver.solve(pos, quat, q_d);
    const qNext = ik.converged ? ik.q : q_d;
    const denom = tNext - t || dt;
    const qvel = new Float64Array(q_d.length);
    for (let i = 0; i < q_d.length; i++) {
      qvel[i] = ((qNext[i] ?? 0) - (q_d[i] ?? 0)) / denom;
    }
    return qvel;
  }

  private numericalDesiredAcceleration(qvel_d: Float64Array, q_d: Float64Array): Float64Array {
    const nv = this.session.nv;
    if (this.qPrev === null) {
      return new Float64Array(nv);
    }
    const dt = this.controlDt;
    const qvelPrev = new Float64Array(nv);
    for (let i = 0; i < nv; i++) {
      qvelPrev[i] = ((q_d[i] ?? 0) - (this.qPrev[i] ?? 0)) / dt;
    }
    const qacc = new Float64Array(nv);
    for (let i = 0; i < nv; i++) {
      qacc[i] = ((qvel_d[i] ?? 0) - qvelPrev[i]) / dt;
    }
    this.qPrev = new Float64Array(q_d);
    return qacc;
  }
}

export function createSimulation(session: RobotSession): SimulationEngine {
  return new SimulationEngine(session, session.createController());
}

export function runToTarget(
  engine: SimulationEngine,
  qTarget: ArrayLike<number>,
  options?: RunToTargetOptions,
): boolean {
  return engine.runToTarget(qTarget, options);
}

export function runToEeTarget(
  engine: SimulationEngine,
  eePos: number[],
  ikSolver: IKSolver,
  options?: RunToEeTargetOptions,
): boolean {
  return engine.runToEeTarget(eePos, ikSolver, options);
}

export function runTrajectory(
  engine: SimulationEngine,
  sampler: TrajectorySampler,
  ikSolver: IKSolver,
  options?: RunTrajectoryOptions,
): boolean {
  return engine.runTrajectory(sampler, ikSolver, options);
}
