import { useSessionStore } from '../../stores/session-store';

interface JointTargetPanelProps {
  onRun: () => void;
  disabled?: boolean;
}

export function JointTargetPanel({ onRun, disabled }: JointTargetPanelProps) {
  const robotInfo = useSessionStore((s) => s.robotInfo);
  const jointTargets = useSessionStore((s) => s.jointTargets);
  const jointPositions = useSessionStore((s) => s.jointPositions);
  const setJointTargetAt = useSessionStore((s) => s.setJointTargetAt);
  const simStatus = useSessionStore((s) => s.simStatus);

  if (!robotInfo) {
    return (
      <section className="panel-section">
        <h3>关节控制</h3>
        <p className="hint">请先加载模型</p>
      </section>
    );
  }

  const running = simStatus === 'running';

  const handleSlider = (index: number, value: number) => {
    setJointTargetAt(index, value);
  };

  const syncAllToCurrent = () => {
    useSessionStore.getState().setJointTargets([...jointPositions]);
  };

  return (
    <section className="panel-section">
      <h3>关节控制</h3>
      <p className="hint">拖动滑块设置目标关节角，点击「开始仿真」或下方「运行动力学」执行 MuJoCo 仿真。</p>
      <div className="joint-grid">
        {robotInfo.jointNames.map((name, i) => {
          const lo = robotInfo.lowerLimits[i] ?? -3.14;
          const hi = robotInfo.upperLimits[i] ?? 3.14;
          const val = jointTargets[i] ?? jointPositions[i] ?? 0;
          return (
            <label key={name} className="joint-row">
              <span className="joint-label" title={name}>
                {name.replace(/^zarm_[lr]\d+_joint$/, (m) => m).slice(0, 18)}
              </span>
              <input
                type="range"
                min={lo}
                max={hi}
                step={0.01}
                value={val}
                disabled={running || disabled}
                onChange={(e) => handleSlider(i, Number(e.target.value))}
              />
              <input
                type="number"
                className="joint-value"
                min={lo}
                max={hi}
                step={0.01}
                value={Number(val.toFixed(3))}
                disabled={running || disabled}
                onChange={(e) => handleSlider(i, Number(e.target.value))}
              />
            </label>
          );
        })}
      </div>
      <div className="button-row">
        <button type="button" className="btn btn-ghost" onClick={syncAllToCurrent} disabled={running || disabled}>
          目标=当前
        </button>
        <button
          type="button"
          className="primary"
          onClick={onRun}
          disabled={running || disabled}
        >
          {running ? '仿真中…' : '运行动力学'}
        </button>
      </div>
    </section>
  );
}
