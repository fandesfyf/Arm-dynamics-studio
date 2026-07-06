import { useSessionStore } from '../../stores/session-store';

interface SimulationPanelProps {
  disabled?: boolean;
}

export function SimulationPanel({ disabled }: SimulationPanelProps) {
  const robotInfo = useSessionStore((s) => s.robotInfo);
  const simStatus = useSessionStore((s) => s.simStatus);
  const simMessage = useSessionStore((s) => s.simMessage);
  const controlDt = useSessionStore((s) => s.controlDt);
  const controlMode = useSessionStore((s) => s.controlMode);
  const isPaused = useSessionStore((s) => s.isPaused);
  const interpolationActive = useSessionStore((s) => s.interpolationActive);
  const setControlDt = useSessionStore((s) => s.setControlDt);

  const running = simStatus === 'running';
  const realtimeMode = controlMode === 'realtime';

  if (!robotInfo) {
    return (
      <section className="panel-section">
        <p className="hint">等待默认模型加载…</p>
      </section>
    );
  }

  return (
    <section className="panel-section simulation-panel">
      <div className="sim-status-card">
        <div className="sim-status-row">
          <span className="sim-status-label">MuJoCo 状态</span>
          <span className={`sim-status-value sim-status-value--${simStatus}`}>
            {running ? (isPaused ? '已暂停' : '运行中') : simStatus === 'ready' ? '就绪' : simStatus}
          </span>
        </div>
        {simMessage && (!running || (controlMode === 'interpolate' && !interpolationActive)) && (
          <p className="sim-status-message">{simMessage}</p>
        )}
      </div>

      <label className="field-label">
        控制周期 control_dt (s)
        <input
          type="number"
          min={0.001}
          max={0.02}
          step={0.001}
          value={controlDt}
          disabled={running || disabled}
          onChange={(e) => setControlDt(Number(e.target.value))}
        />
      </label>
      <p className="hint">基于 MuJoCo 计算力矩 + 子步积分；控制周期越小轨迹越平滑。</p>

      {realtimeMode ? (
        <p className="hint">
          <strong>实时模式</strong>：左侧栏「开始仿真」进入持续循环，每 control_dt 跟踪控制面板滑条目标；暂停可挂起循环，停止重置 MuJoCo 姿态（曲线数据保留）。
        </p>
      ) : (
        <p className="hint">
          <strong>插值模式</strong>：左侧栏「开始仿真」进入关节目标保持循环；「添加目标」后执行限速插值。
        </p>
      )}
    </section>
  );
}
