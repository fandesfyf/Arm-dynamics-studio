import { vecGet } from '../types/mujoco';
import type { RobotSession } from '../core/robot-session';

/** 将活动关节角（与 session.jointNames 同序）写入完整 MuJoCo qpos */
export function actuatedJointsToQpos(
  session: RobotSession,
  jointValues: ArrayLike<number>,
  baseQpos?: ArrayLike<number>,
): Float64Array {
  const q = baseQpos
    ? Float64Array.from(baseQpos)
    : vecGet(session.data.qpos, session.nq);
  const byName = new Map(session.jointNames.map((name, i) => [name, jointValues[i] ?? 0]));

  for (const m of session.jointMap) {
    if (m.mj_qposadr < 0) continue;
    const v = byName.get(m.name);
    if (v !== undefined) {
      q[m.mj_qposadr] = v;
    }
  }
  return q;
}

/** 从 MuJoCo qpos 读取活动关节角（与 session.jointNames 同序） */
export function qposToActuatedJoints(session: RobotSession, qpos?: ArrayLike<number>): number[] {
  const q = qpos ?? vecGet(session.data.qpos, session.nq);
  return session.jointNames.map((name) => {
    const m = session.jointMap.find((j) => j.name === name);
    if (!m || m.mj_qposadr < 0) return 0;
    return q[m.mj_qposadr] ?? 0;
  });
}

/** 将 nv 维控制器增益映射为活动关节顺序 */
export function nvGainsToActuated(
  session: RobotSession,
  kpNv: ArrayLike<number>,
  kdNv: ArrayLike<number>,
): { kp: number[]; kd: number[] } {
  return {
    kp: session.jointNames.map((name) => {
      const m = session.jointMap.find((j) => j.name === name);
      if (!m || m.mj_dofadr < 0) return 0;
      return kpNv[m.mj_dofadr] ?? 0;
    }),
    kd: session.jointNames.map((name) => {
      const m = session.jointMap.find((j) => j.name === name);
      if (!m || m.mj_dofadr < 0) return 0;
      return kdNv[m.mj_dofadr] ?? 0;
    }),
  };
}

/** 将活动关节 Kp/Kd 写回完整 nv 增益向量并应用到控制器 */
export function applyActuatedGainsToController(
  session: RobotSession,
  controller: {
    getGains(): { kp: Float64Array; kd: Float64Array };
    setGains(kp: ArrayLike<number>, kd: ArrayLike<number>): void;
  },
  jointKp: ArrayLike<number>,
  jointKd: ArrayLike<number>,
): void {
  const { kp, kd } = controller.getGains();
  for (let i = 0; i < session.jointNames.length; i++) {
    const m = session.jointMap.find((j) => j.name === session.jointNames[i]);
    if (!m || m.mj_dofadr < 0) continue;
    kp[m.mj_dofadr] = jointKp[i] ?? kp[m.mj_dofadr];
    kd[m.mj_dofadr] = jointKd[i] ?? kd[m.mj_dofadr];
  }
  controller.setGains(kp, kd);
}
