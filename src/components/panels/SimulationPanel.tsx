import { useSessionStore } from '../../stores/session-store';

interface SimulationPanelProps {
  onStart: () => void;
  onPause: () => void;
  onStop: () => void;
  onResetRecorder: () => void;
  onToggleRecorderPause?: () => void;
  disabled?: boolean;
  canStart?: boolean;
}

export function SimulationPanel({
  onStart,
  onPause,
  onStop,
  onResetRecorder,
  onToggleRecorderPause,
  disabled,
  canStart = true,
}: SimulationPanelProps) {
  const robotInfo = useSessionStore((s) => s.robotInfo);
  const simStatus = useSessionStore((s) => s.simStatus);
  const simMessage = useSessionStore((s) => s.simMessage);
  const controlDt = useSessionStore((s) => s.controlDt);
  const controlMode = useSessionStore((s) => s.controlMode);
  const simStepCount = useSessionStore((s) => s.simStepCount);
  const isPaused = useSessionStore((s) => s.isPaused);
  const interpolationActive = useSessionStore((s) => s.interpolationActive);
  const recorder = useSessionStore((s) => s.recorder);
  const recorderPaused = useSessionStore((s) => s.recorderPaused);
  const setControlDt = useSessionStore((s) => s.setControlDt);

  const running = simStatus === 'running';
  const realtimeMode = controlMode === 'realtime';
  const primaryIsStop = running;
  const primaryDisabled = disabled || (!primaryIsStop && !canStart);
  const pauseDisabled = !running || disabled;

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

      <div className="sim-metrics">
        <div className="sim-metric">
          <span className="sim-metric-label">步数</span>
          <span className="sim-metric-value">{simStepCount}</span>
        </div>
        <div className="sim-metric">
          <span className="sim-metric-label">采样</span>
          <span className="sim-metric-value">{recorder.sampleCount}</span>
        </div>
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
          <strong>实时模式</strong>：「开始仿真」进入持续循环，每 control_dt 跟踪控制面板滑条目标；暂停可挂起循环，停止重置 MuJoCo 姿态（曲线数据保留）。
        </p>
      ) : (
        <p className="hint">
          <strong>插值模式</strong>：「开始仿真」进入关节目标保持循环；「发送目标」或 Gizmo 松开后执行限速插值。
        </p>
      )}

      <div className="button-row sim-transport">
        <button
          type="button"
          className={`btn ${primaryIsStop ? 'btn-danger' : 'btn-success'}`}
          disabled={primaryDisabled}
          onClick={primaryIsStop ? onStop : onStart}
          title={undefined}
        >
          {primaryIsStop ? '⏹ 停止' : '▶ 开始仿真'}
        </button>
        <button
          type="button"
          className="btn btn-warning"
          disabled={pauseDisabled}
          onClick={onPause}
        >
          {isPaused ? '▶ 继续' : '⏸ 暂停'}
        </button>
      </div>

      <button
        type="button"
        className="btn btn-ghost btn-block"
        disabled={!running || disabled || !onToggleRecorderPause}
        onClick={onToggleRecorderPause}
        title={running ? undefined : '开始仿真后可暂停/继续录制'}
      >
        {recorderPaused ? '▶ 继续录制' : '⏸ 暂停录制'}
      </button>

      <button type="button" className="btn btn-ghost btn-block" disabled={disabled} onClick={onResetRecorder}>
        清空曲线数据
      </button>
    </section>
  );
}
