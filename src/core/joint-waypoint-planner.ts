import { CubicSpline1D } from './trajectory';
import { JointVelocityLimitPlanner, type JointDesired } from './planner';

export type JointInterpProfile = 'linear' | 'cubic';

function segmentDuration(
  qStart: ArrayLike<number>,
  qEnd: ArrayLike<number>,
  maxVel: number | number[],
  nv: number,
): number {
  return new JointVelocityLimitPlanner(qStart, qEnd, maxVel, nv).getDuration();
}

function buildKnotTimes(
  qWaypoints: ArrayLike<number>[],
  maxVel: number | number[],
  nv: number,
): number[] {
  const times = [0];
  for (let i = 1; i < qWaypoints.length; i++) {
    const dt = segmentDuration(qWaypoints[i - 1]!, qWaypoints[i]!, maxVel, nv);
    times.push(times[i - 1]! + dt);
  }
  return times;
}

/**
 * Multi-waypoint joint-space planner: linear segments or cubic spline (natural BC).
 * Knot times follow per-segment velocity limits (same as chained linear interp).
 */
export class JointMultiWaypointPlanner {
  private readonly qWaypoints: Float64Array[];
  private readonly knotTimes: number[];
  private readonly nv: number;
  private readonly mode: JointInterpProfile;
  private readonly segmentPlanners: JointVelocityLimitPlanner[] | null;
  private readonly cubicSplines: CubicSpline1D[] | null;

  constructor(
    qWaypoints: ArrayLike<number>[],
    maxJointVelRadPerSec: number | number[],
    nv: number,
    mode: JointInterpProfile = 'linear',
  ) {
    if (qWaypoints.length < 2) {
      throw new Error('JointMultiWaypointPlanner requires at least 2 waypoints');
    }
    this.qWaypoints = qWaypoints.map((q) => Float64Array.from(q));
    this.nv = nv;
    this.mode = mode;
    this.knotTimes = buildKnotTimes(this.qWaypoints, maxJointVelRadPerSec, nv);

    if (mode === 'linear') {
      this.segmentPlanners = [];
      for (let i = 0; i < this.qWaypoints.length - 1; i++) {
        this.segmentPlanners.push(
          new JointVelocityLimitPlanner(
            this.qWaypoints[i]!,
            this.qWaypoints[i + 1]!,
            maxJointVelRadPerSec,
            nv,
          ),
        );
      }
      this.cubicSplines = null;
    } else {
      this.segmentPlanners = null;
      const dof = this.qWaypoints[0]!.length;
      this.cubicSplines = [];
      for (let j = 0; j < dof; j++) {
        const ys = this.qWaypoints.map((q) => q[j] ?? 0);
        this.cubicSplines.push(new CubicSpline1D(this.knotTimes, ys));
      }
    }
  }

  getDuration(): number {
    return this.knotTimes[this.knotTimes.length - 1] ?? 0;
  }

  isSettled(t: number, tol = 1e-4): boolean {
    return t >= this.getDuration() - tol;
  }

  getDesired(t: number): JointDesired {
    const tc = Math.max(0, Math.min(t, this.getDuration()));
    if (this.mode === 'linear') {
      return this.getDesiredLinear(tc);
    }
    return this.getDesiredCubic(tc);
  }

  private getDesiredLinear(t: number): JointDesired {
    const planners = this.segmentPlanners!;
    let seg = planners.length - 1;
    for (let i = 0; i < planners.length; i++) {
      if (t < this.knotTimes[i + 1]! + 1e-9) {
        seg = i;
        break;
      }
    }
    const localT = t - this.knotTimes[seg]!;
    return planners[seg]!.getDesired(localT);
  }

  private getDesiredCubic(t: number): JointDesired {
    const splines = this.cubicSplines!;
    const n = splines.length;
    const q_d = new Float64Array(n);
    const v_d = new Float64Array(this.nv);
    const a_d = new Float64Array(this.nv);
    for (let j = 0; j < n; j++) {
      q_d[j] = splines[j]!.evaluate(t);
      v_d[j] = splines[j]!.derivative(t);
      a_d[j] = splines[j]!.secondDerivative(t);
    }
    return { q_d, v_d, a_d };
  }
}
