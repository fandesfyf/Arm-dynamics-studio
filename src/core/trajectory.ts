/** 3D 向量 [x, y, z] */
export type Vec3 = [number, number, number];

/** 四元数 [x, y, z, w] */
export type Quat = [number, number, number, number];

export interface TrajectorySample {
  ee_pos: Vec3;
  ee_quat: Quat;
}

export interface Waypoint {
  time: number;
  position: Vec3;
  quaternion: Quat;
}

/** 一维自然三次样条（端点二阶导数为 0） */
export class CubicSpline1D {
  private readonly xs: number[];
  private readonly a: number[];
  private readonly b: number[];
  private readonly c: number[];
  private readonly d: number[];

  constructor(xs: number[], ys: number[]) {
    const n = xs.length;
    if (n < 2) {
      throw new Error('CubicSpline1D requires at least 2 points');
    }
    for (let i = 1; i < n; i++) {
      if (xs[i] <= xs[i - 1]) {
        throw new Error('Knot times must be strictly increasing');
      }
    }

    this.xs = xs.slice();
    const h: number[] = [];
    const alpha: number[] = new Array(n).fill(0);

    for (let i = 0; i < n - 1; i++) {
      h.push(xs[i + 1] - xs[i]);
    }

    for (let i = 1; i < n - 1; i++) {
      alpha[i] =
        (3 / h[i]) * (ys[i + 1] - ys[i]) - (3 / h[i - 1]) * (ys[i] - ys[i - 1]);
    }

    const l: number[] = new Array(n).fill(0);
    const mu: number[] = new Array(n).fill(0);
    const z: number[] = new Array(n).fill(0);
    const cArr: number[] = new Array(n).fill(0);

    l[0] = 1;
    for (let i = 1; i < n - 1; i++) {
      l[i] = 2 * (xs[i + 1] - xs[i - 1]) - h[i - 1] * mu[i - 1];
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }
    l[n - 1] = 1;
    z[n - 1] = 0;
    cArr[n - 1] = 0;

    for (let j = n - 2; j >= 0; j--) {
      cArr[j] = z[j] - mu[j] * cArr[j + 1];
    }

    this.a = ys.slice();
    this.b = new Array(n - 1);
    this.c = cArr;
    this.d = new Array(n - 1);

    for (let j = 0; j < n - 1; j++) {
      this.b[j] = (ys[j + 1] - ys[j]) / h[j] - (h[j] * (cArr[j + 1] + 2 * cArr[j])) / 3;
      this.d[j] = (cArr[j + 1] - cArr[j]) / (3 * h[j]);
    }
  }

  evaluate(t: number): number {
    const n = this.xs.length;
    if (t <= this.xs[0]) {
      return this.a[0];
    }
    if (t >= this.xs[n - 1]) {
      return this.a[n - 1];
    }

    let lo = 0;
    let hi = n - 2;
    while (lo <= hi) {
      const mid = (lo + hi) >> 1;
      if (this.xs[mid] <= t && t < this.xs[mid + 1]) {
        const dx = t - this.xs[mid];
        return this.a[mid] + this.b[mid] * dx + this.c[mid] * dx * dx + this.d[mid] * dx * dx * dx;
      }
      if (t < this.xs[mid]) {
        hi = mid - 1;
      } else {
        lo = mid + 1;
      }
    }
    return this.a[n - 1];
  }

  derivative(t: number): number {
    const n = this.xs.length;
    if (t <= this.xs[0] || t >= this.xs[n - 1]) {
      return 0;
    }

    let lo = 0;
    let hi = n - 2;
    while (lo <= hi) {
      const mid = (lo + hi) >> 1;
      if (this.xs[mid] <= t && t < this.xs[mid + 1]) {
        const dx = t - this.xs[mid];
        return this.b[mid] + 2 * this.c[mid] * dx + 3 * this.d[mid] * dx * dx;
      }
      if (t < this.xs[mid]) {
        hi = mid - 1;
      } else {
        lo = mid + 1;
      }
    }
    return 0;
  }

  secondDerivative(t: number): number {
    const n = this.xs.length;
    if (t <= this.xs[0] || t >= this.xs[n - 1]) {
      return 0;
    }

    let lo = 0;
    let hi = n - 2;
    while (lo <= hi) {
      const mid = (lo + hi) >> 1;
      if (this.xs[mid] <= t && t < this.xs[mid + 1]) {
        const dx = t - this.xs[mid];
        return 2 * this.c[mid] + 6 * this.d[mid] * dx;
      }
      if (t < this.xs[mid]) {
        hi = mid - 1;
      } else {
        lo = mid + 1;
      }
    }
    return 0;
  }
}

function clip(t: number, tMin: number, tMax: number): number {
  return Math.max(tMin, Math.min(tMax, t));
}

function normQuat(q: Quat): Quat {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  if (n < 1e-12) {
    return [0, 0, 0, 1];
  }
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

function slerp(q1: Quat, q2: Quat, alpha: number): Quat {
  let a = normQuat(q1);
  let b = normQuat(q2);

  let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (dot < 0) {
    b = [-b[0], -b[1], -b[2], -b[3]];
    dot = -dot;
  }

  dot = Math.max(-1, Math.min(1, dot));

  if (dot > 0.9995) {
    const result: Quat = [
      a[0] + alpha * (b[0] - a[0]),
      a[1] + alpha * (b[1] - a[1]),
      a[2] + alpha * (b[2] - a[2]),
      a[3] + alpha * (b[3] - a[3]),
    ];
    return normQuat(result);
  }

  const theta = Math.acos(dot);
  const sinTheta = Math.sin(theta);
  const w1 = Math.sin((1 - alpha) * theta) / sinTheta;
  const w2 = Math.sin(alpha * theta) / sinTheta;
  return normQuat([
    w1 * a[0] + w2 * b[0],
    w1 * a[1] + w2 * b[1],
    w1 * a[2] + w2 * b[2],
    w1 * a[3] + w2 * b[3],
  ]);
}

/**
 * 末端轨迹插值：位置 CubicSpline + 姿态分段 SLERP。
 * 对应旧版 trajectory_interpolator.py。
 */
export class Trajectory {
  private waypoints: Waypoint[] = [];
  private splines: { x: CubicSpline1D; y: CubicSpline1D; z: CubicSpline1D } | null = null;
  private totalTime = 0;

  addWaypoint(time: number, position: Vec3, quaternion: Quat): void {
    this.waypoints.push({
      time,
      position: [...position] as Vec3,
      quaternion: normQuat(quaternion),
    });
    this.waypoints.sort((a, b) => a.time - b.time);
    this.rebuildSplines();
  }

  removeWaypoint(index: number): void {
    if (index >= 0 && index < this.waypoints.length) {
      this.waypoints.splice(index, 1);
      this.rebuildSplines();
    }
  }

  clear(): void {
    this.waypoints = [];
    this.splines = null;
    this.totalTime = 0;
  }

  getWaypoints(): readonly Waypoint[] {
    return this.waypoints.map((w) => ({
      time: w.time,
      position: [...w.position] as Vec3,
      quaternion: [...w.quaternion] as Quat,
    }));
  }

  getDuration(): number {
    return this.totalTime;
  }

  /** 采样 t 时刻末端位姿 */
  sample(t: number): TrajectorySample {
    if (this.waypoints.length === 0) {
      throw new Error('至少需要 1 个关键点');
    }
    if (this.waypoints.length === 1) {
      const w = this.waypoints[0];
      return { ee_pos: [...w.position] as Vec3, ee_quat: [...w.quaternion] as Quat };
    }

    const t0 = this.waypoints[0].time;
    const t1 = this.waypoints[this.waypoints.length - 1].time;
    const tc = clip(t, t0, t1);

    const x = this.splines!.x.evaluate(tc);
    const y = this.splines!.y.evaluate(tc);
    const z = this.splines!.z.evaluate(tc);
    const quat = this.slerpTrajectory(tc);

    return { ee_pos: [x, y, z], ee_quat: quat };
  }

  /** 线速度（位置样条一阶导） */
  sampleVelocity(t: number): Vec3 {
    if (this.waypoints.length < 2 || !this.splines) {
      return [0, 0, 0];
    }
    const t0 = this.waypoints[0].time;
    const t1 = this.waypoints[this.waypoints.length - 1].time;
    const tc = clip(t, t0, t1);
    return [
      this.splines.x.derivative(tc),
      this.splines.y.derivative(tc),
      this.splines.z.derivative(tc),
    ];
  }

  private rebuildSplines(): void {
    if (this.waypoints.length < 2) {
      this.splines = null;
      this.totalTime = this.waypoints.length === 1 ? this.waypoints[0].time : 0;
      return;
    }

    const times = this.waypoints.map((w) => w.time);
    const xs = this.waypoints.map((w) => w.position[0]);
    const ys = this.waypoints.map((w) => w.position[1]);
    const zs = this.waypoints.map((w) => w.position[2]);

    this.splines = {
      x: new CubicSpline1D(times, xs),
      y: new CubicSpline1D(times, ys),
      z: new CubicSpline1D(times, zs),
    };
    this.totalTime = times[times.length - 1];
  }

  private slerpTrajectory(t: number): Quat {
    const times = this.waypoints.map((w) => w.time);
    const quats = this.waypoints.map((w) => w.quaternion);

    if (t <= times[0]) {
      return [...quats[0]] as Quat;
    }
    if (t >= times[times.length - 1]) {
      return [...quats[quats.length - 1]] as Quat;
    }

    let idx = 0;
    for (let i = 0; i < times.length - 1; i++) {
      if (times[i] <= t && t < times[i + 1]) {
        idx = i;
        break;
      }
      if (t >= times[i + 1]) {
        idx = i;
      }
    }

    const t0 = times[idx];
    const t1 = times[idx + 1];
    const alpha = t1 > t0 ? (t - t0) / (t1 - t0) : 0;
    return slerp(quats[idx], quats[idx + 1], alpha);
  }
}
