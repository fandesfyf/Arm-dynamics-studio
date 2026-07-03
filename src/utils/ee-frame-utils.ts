import type { Quat, Vec3 } from '../core/trajectory';

export {
  Z_UP_TO_Y_UP,
  fkToScene,
  sceneToFk,
  urdfTargetToWorld,
  worldToUrdfTarget,
} from '../viewer/ee-kinematics';

export function vec3Near(a: Vec3, b: Vec3, eps = 1e-5): boolean {
  return (
    Math.abs(a[0] - b[0]) < eps &&
    Math.abs(a[1] - b[1]) < eps &&
    Math.abs(a[2] - b[2]) < eps
  );
}

export interface UrdfOrigin {
  xyz: Vec3;
  rpy: [number, number, number];
}

function parseVec3Attr(raw: string | undefined, fallback: Vec3 = [0, 0, 0]): Vec3 {
  if (!raw) return fallback;
  const parts = raw.trim().split(/\s+/).map(Number);
  return [parts[0] ?? 0, parts[1] ?? 0, parts[2] ?? 0];
}

export function parseUrdfJointOrigin(jointBlock: string): UrdfOrigin {
  const originBlock = jointBlock.match(/<origin\b[^>]*\/>|<origin\b[^>]*>[\s\S]*?<\/origin>/i)?.[0];
  const xyzRaw = originBlock?.match(/xyz="([^"]*)"/)?.[1];
  const rpyRaw = originBlock?.match(/rpy="([^"]*)"/)?.[1];
  const xyz = parseVec3Attr(xyzRaw);
  const rpyParts = parseVec3Attr(rpyRaw);
  return { xyz, rpy: [rpyParts[0], rpyParts[1], rpyParts[2]] };
}

function findJointBlockForChild(urdfXml: string, childLink: string): string | null {
  const blocks = urdfXml.match(/<joint\b[\s\S]*?<\/joint>/g) ?? [];
  for (const block of blocks) {
    const child = block.match(/<child\s+link="([^"]+)"/)?.[1];
    if (child === childLink) return block;
  }
  return null;
}

/**
 * Fixed-joint origins from resolved actuated joint frame down to end-effector link.
 * Order: nearest actuated joint → EE link.
 */
export function collectFixedOriginsToLink(
  urdfXml: string,
  endEffectorLink: string,
  resolvedJointName: string,
): UrdfOrigin[] {
  const origins: UrdfOrigin[] = [];
  let link = endEffectorLink;

  while (link) {
    const block = findJointBlockForChild(urdfXml, link);
    if (!block) break;

    const jointName = block.match(/name="([^"]+)"/)?.[1];
    const parentLink = block.match(/<parent\s+link="([^"]+)"/)?.[1];
    if (!jointName || !parentLink) break;

    if (jointName === resolvedJointName) break;

    const jointType = block.match(/type="([^"]+)"/)?.[1] ?? 'fixed';
    if (jointType !== 'fixed') break;

    origins.unshift(parseUrdfJointOrigin(block));
    link = parentLink;
  }

  return origins;
}

function rpyToMatrix(rpy: [number, number, number]): number[] {
  const [roll, pitch, yaw] = rpy;
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  return [
    cy * cp,
    cy * sp * sr - sy * cr,
    cy * sp * cr + sy * sr,
    sy * cp,
    sy * sp * sr + cy * cr,
    sy * sp * cr - cy * sr,
    -sp,
    cp * sr,
    cp * cr,
  ];
}

function mat3MulVec3(m: number[], v: Vec3): Vec3 {
  return [
    m[0]! * v[0]! + m[1]! * v[1]! + m[2]! * v[2]!,
    m[3]! * v[0]! + m[4]! * v[1]! + m[5]! * v[2]!,
    m[6]! * v[0]! + m[7]! * v[1]! + m[8]! * v[2]!,
  ];
}

function mat3Mul(a: number[], b: number[]): number[] {
  const out = new Array<number>(9);
  for (let row = 0; row < 3; row++) {
    for (let col = 0; col < 3; col++) {
      out[row * 3 + col] =
        a[row * 3]! * b[col]! + a[row * 3 + 1]! * b[3 + col]! + a[row * 3 + 2]! * b[6 + col]!;
    }
  }
  return out;
}

function rotationMatrixToQuat(rot: ArrayLike<number>): Quat {
  const r00 = rot[0] ?? 1;
  const r01 = rot[1] ?? 0;
  const r02 = rot[2] ?? 0;
  const r10 = rot[3] ?? 0;
  const r11 = rot[4] ?? 1;
  const r12 = rot[5] ?? 0;
  const r20 = rot[6] ?? 0;
  const r21 = rot[7] ?? 0;
  const r22 = rot[8] ?? 1;
  const trace = r00 + r11 + r22;
  let x: number;
  let y: number;
  let z: number;
  let w: number;
  if (trace > 0) {
    const s = Math.sqrt(trace + 1) * 2;
    w = 0.25 * s;
    x = (r21 - r12) / s;
    y = (r02 - r20) / s;
    z = (r10 - r01) / s;
  } else if (r00 > r11 && r00 > r22) {
    const s = Math.sqrt(1 + r00 - r11 - r22) * 2;
    w = (r21 - r12) / s;
    x = 0.25 * s;
    y = (r01 + r10) / s;
    z = (r02 + r20) / s;
  } else if (r11 > r22) {
    const s = Math.sqrt(1 + r11 - r00 - r22) * 2;
    w = (r02 - r20) / s;
    x = (r01 + r10) / s;
    y = 0.25 * s;
    z = (r12 + r21) / s;
  } else {
    const s = Math.sqrt(1 + r22 - r00 - r11) * 2;
    w = (r10 - r01) / s;
    x = (r02 + r20) / s;
    y = (r12 + r21) / s;
    z = 0.25 * s;
  }
  return [x, y, z, w];
}

/** Apply fixed-joint chain from actuated joint placement to end-effector link frame. */
export function applyFixedOriginsToPlacement(
  translation: Vec3,
  rotation: ArrayLike<number>,
  origins: UrdfOrigin[],
): { pos: Vec3; quat: Quat } {
  let pos: Vec3 = [...translation];
  let rotMat = [
    rotation[0] ?? 1,
    rotation[1] ?? 0,
    rotation[2] ?? 0,
    rotation[3] ?? 0,
    rotation[4] ?? 1,
    rotation[5] ?? 0,
    rotation[6] ?? 0,
    rotation[7] ?? 0,
    rotation[8] ?? 1,
  ];

  for (const origin of origins) {
    const localR = rpyToMatrix(origin.rpy);
    rotMat = mat3Mul(rotMat, localR);
    const offset = mat3MulVec3(rotMat, origin.xyz);
    pos = [pos[0] + offset[0], pos[1] + offset[1], pos[2] + offset[2]];
  }

  return { pos, quat: rotationMatrixToQuat(rotMat) };
}
