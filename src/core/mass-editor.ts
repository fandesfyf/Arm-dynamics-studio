import { sanitizeUrdfForMujoco } from '../utils/urdf-sanitize';

const MOVING_JOINT_TYPES = new Set([
  'revolute',
  'continuous',
  'prismatic',
  'floating',
]);

export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

export interface InertiaTensor {
  ixx: number;
  ixy: number;
  ixz: number;
  iyy: number;
  iyz: number;
  izz: number;
}

export interface LinkInertial {
  linkName: string;
  mass: number;
  com: Vec3;
  inertia: InertiaTensor;
}

export interface JointLimitInfo {
  jointName: string;
  type: string;
  lower: number;
  upper: number;
  effort: number;
  velocity: number;
}

export interface JointLimitUpdate {
  lower?: number;
  upper?: number;
  effort?: number;
  velocity?: number;
}

function parseFloatAttr(el: Element | null, attr: string, fallback = 0): number {
  if (!el) return fallback;
  const raw = el.getAttribute(attr);
  if (raw == null || raw === '') return fallback;
  const value = Number.parseFloat(raw);
  return Number.isFinite(value) ? value : fallback;
}

function parseVec3(el: Element | null): Vec3 {
  if (!el) return { x: 0, y: 0, z: 0 };
  const xyz = el.getAttribute('xyz');
  if (!xyz) return { x: 0, y: 0, z: 0 };
  const parts = xyz.trim().split(/\s+/).map(Number);
  return {
    x: parts[0] ?? 0,
    y: parts[1] ?? 0,
    z: parts[2] ?? 0,
  };
}

function readInertia(inertialEl: Element): InertiaTensor {
  const inertiaEl = inertialEl.querySelector(':scope > inertia');
  return {
    ixx: parseFloatAttr(inertiaEl, 'ixx', 0),
    ixy: parseFloatAttr(inertiaEl, 'ixy', 0),
    ixz: parseFloatAttr(inertiaEl, 'ixz', 0),
    iyy: parseFloatAttr(inertiaEl, 'iyy', 0),
    iyz: parseFloatAttr(inertiaEl, 'iyz', 0),
    izz: parseFloatAttr(inertiaEl, 'izz', 0),
  };
}

/** URDF DOM 质量/惯量/关节限位编辑器 */
export class MassEditor {
  private doc: Document;
  private robotEl: Element;

  constructor(urdfXml: string) {
    const parser = new DOMParser();
    this.doc = parser.parseFromString(urdfXml, 'application/xml');
    const parseError = this.doc.querySelector('parsererror');
    if (parseError) {
      throw new Error(`URDF 解析失败: ${parseError.textContent ?? 'unknown'}`);
    }
    const robot = this.doc.querySelector('robot');
    if (!robot) {
      throw new Error('URDF 缺少 <robot> 根元素');
    }
    this.robotEl = robot;
  }

  getLinkInertials(): LinkInertial[] {
    const result: LinkInertial[] = [];
    const links = this.robotEl.querySelectorAll(':scope > link');
    for (const link of links) {
      const linkName = link.getAttribute('name');
      if (!linkName) continue;
      const inertialEl = link.querySelector(':scope > inertial');
      if (!inertialEl) continue;

      const massEl = inertialEl.querySelector(':scope > mass');
      const originEl = inertialEl.querySelector(':scope > origin');

      result.push({
        linkName,
        mass: parseFloatAttr(massEl, 'value', 0),
        com: parseVec3(originEl),
        inertia: readInertia(inertialEl),
      });
    }
    return result;
  }

  getJointLimits(): JointLimitInfo[] {
    const result: JointLimitInfo[] = [];
    const joints = this.robotEl
      ? this.robotEl.querySelectorAll(':scope > joint')
      : [];
    for (const joint of joints) {
      const jointName = joint.getAttribute('name');
      const type = joint.getAttribute('type') ?? '';
      if (!jointName || !MOVING_JOINT_TYPES.has(type)) continue;

      const limitEl = joint.querySelector(':scope > limit');
      result.push({
        jointName,
        type,
        lower: parseFloatAttr(limitEl, 'lower', -Math.PI),
        upper: parseFloatAttr(limitEl, 'upper', Math.PI),
        effort: parseFloatAttr(limitEl, 'effort', 0),
        velocity: parseFloatAttr(limitEl, 'velocity', 0),
      });
    }
    return result;
  }

  setLinkMass(linkName: string, mass: number): void {
    if (mass <= 0) {
      throw new Error(`质量必须为正数，当前值: ${mass}`);
    }
    const massEl = this.requireInertialMass(linkName);
    massEl.setAttribute('value', String(mass));
  }

  setLinkInertia(linkName: string, ixx: number, iyy: number, izz: number): void {
    if (ixx <= 0 || iyy <= 0 || izz <= 0) {
      throw new Error('惯量对角元素必须为正数');
    }
    const inertiaEl = this.requireInertiaElement(linkName);
    inertiaEl.setAttribute('ixx', String(ixx));
    inertiaEl.setAttribute('ixy', '0');
    inertiaEl.setAttribute('ixz', '0');
    inertiaEl.setAttribute('iyy', String(iyy));
    inertiaEl.setAttribute('iyz', '0');
    inertiaEl.setAttribute('izz', String(izz));
  }

  setJointLimits(jointName: string, limits: JointLimitUpdate): void {
    const limitEl = this.requireJointLimit(jointName);
    if (limits.lower != null) {
      limitEl.setAttribute('lower', String(limits.lower));
    }
    if (limits.upper != null) {
      limitEl.setAttribute('upper', String(limits.upper));
    }
    if (limits.effort != null) {
      limitEl.setAttribute('effort', String(limits.effort));
    }
    if (limits.velocity != null) {
      limitEl.setAttribute('velocity', String(limits.velocity));
    }
  }

  serialize(): string {
    return sanitizeUrdfForMujoco(new XMLSerializer().serializeToString(this.doc));
  }

  private findLink(linkName: string): Element {
    const link = Array.from(this.robotEl.querySelectorAll(':scope > link')).find(
      (el) => el.getAttribute('name') === linkName,
    );
    if (!link) {
      throw new Error(`未找到 link: ${linkName}`);
    }
    return link;
  }

  private requireInertialMass(linkName: string): Element {
    const link = this.findLink(linkName);
    let inertialEl = link.querySelector(':scope > inertial');
    if (!inertialEl) {
      inertialEl = this.doc.createElement('inertial');
      link.appendChild(inertialEl);
    }
    let massEl = inertialEl.querySelector(':scope > mass');
    if (!massEl) {
      massEl = this.doc.createElement('mass');
      inertialEl.insertBefore(massEl, inertialEl.firstChild);
    }
    return massEl;
  }

  private requireInertiaElement(linkName: string): Element {
    const link = this.findLink(linkName);
    let inertialEl = link.querySelector(':scope > inertial');
    if (!inertialEl) {
      inertialEl = this.doc.createElement('inertial');
      link.appendChild(inertialEl);
    }
    let inertiaEl = inertialEl.querySelector(':scope > inertia');
    if (!inertiaEl) {
      inertiaEl = this.doc.createElement('inertia');
      inertialEl.appendChild(inertiaEl);
      this.ensureInertiaAttributes(inertiaEl);
    }
    return inertiaEl;
  }

  private ensureInertiaAttributes(inertiaEl: Element): void {
    for (const attr of ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'] as const) {
      if (inertiaEl.getAttribute(attr) == null) {
        inertiaEl.setAttribute(attr, '0');
      }
    }
  }

  private requireJointLimit(jointName: string): Element {
    const joint = Array.from(this.robotEl.querySelectorAll(':scope > joint')).find(
      (el) => el.getAttribute('name') === jointName,
    );
    if (!joint) {
      throw new Error(`未找到 joint: ${jointName}`);
    }
    let limitEl = joint.querySelector(':scope > limit');
    if (!limitEl) {
      limitEl = this.doc.createElement('limit');
      joint.appendChild(limitEl);
    }
    return limitEl;
  }
}
