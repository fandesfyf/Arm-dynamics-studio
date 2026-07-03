import type { JointMapping } from '../types/robot';
import type { JointMap } from '../core/inverse-kinematics';

/** 将 T1 JointMapping[] 适配为 T4 InverseKinematics 所需的 JointMap */
export function createJointMapAdapter(mappings: JointMapping[]): JointMap {
  const active = mappings.filter((m) => m.pin_vidx >= 0 && m.mj_dofadr >= 0);
  const nv = active.length;
  let nq = 0;
  for (const m of active) {
    nq = Math.max(nq, m.mj_qposadr + 1);
  }

  return {
    mappings,
    mjQposToPinQ(mjQpos) {
      const out = new Float64Array(nv);
      for (const m of active) {
        out[m.pin_vidx] = mjQpos[m.mj_qposadr] ?? 0;
      }
      return out;
    },
    pinQToMjQpos(pinQ, mjQpos) {
      const out = mjQpos ? new Float64Array(mjQpos) : new Float64Array(nq);
      for (const m of active) {
        out[m.mj_qposadr] = pinQ[m.pin_vidx] ?? 0;
      }
      return out;
    },
  };
}
