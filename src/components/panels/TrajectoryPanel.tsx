import { useSessionStore } from '../../stores/session-store';
import type { Quat, Vec3 } from '../../core/trajectory';

interface TrajectoryPanelProps {
  onRun: () => void;
  disabled?: boolean;
}

export function TrajectoryPanel({ onRun, disabled }: TrajectoryPanelProps) {
  const robotInfo = useSessionStore((s) => s.robotInfo);
  const waypoints = useSessionStore((s) => s.trajectoryWaypoints);
  const addTrajectoryWaypoint = useSessionStore((s) => s.addTrajectoryWaypoint);
  const removeTrajectoryWaypoint = useSessionStore((s) => s.removeTrajectoryWaypoint);
  const eeTarget = useSessionStore((s) => s.eeTarget);
  const simStatus = useSessionStore((s) => s.simStatus);

  if (!robotInfo) {
    return (
      <section className="panel-section">
        <h3>轨迹关键点</h3>
        <p className="hint">请先加载模型</p>
      </section>
    );
  }

  const running = simStatus === 'running';

  const handleAdd = () => {
    const t =
      waypoints.length > 0
        ? waypoints[waypoints.length - 1].time + 2
        : 0;
    addTrajectoryWaypoint({
      time: t,
      position: [...eeTarget] as Vec3,
      quaternion: [...robotInfo.eeQuat] as Quat,
    });
  };

  return (
    <section className="panel-section">
      <h3>轨迹关键点</h3>
      <div className="button-row">
        <button type="button" onClick={handleAdd} disabled={running || disabled}>
          添加关键点
        </button>
        <button
          type="button"
          className="primary"
          onClick={onRun}
          disabled={running || disabled || waypoints.length < 2}
        >
          运行轨迹
        </button>
      </div>
      {waypoints.length === 0 ? (
        <p className="hint">至少添加 2 个关键点</p>
      ) : (
        <ul className="waypoint-list">
          {waypoints.map((wp, i) => (
            <li key={`${wp.time}-${i}`}>
              <span>
                t={wp.time.toFixed(1)}s · pos=[
                {wp.position.map((v) => v.toFixed(2)).join(', ')}]
              </span>
              <button
                type="button"
                className="small"
                onClick={() => removeTrajectoryWaypoint(i)}
                disabled={running}
              >
                删除
              </button>
            </li>
          ))}
        </ul>
      )}
    </section>
  );
}
