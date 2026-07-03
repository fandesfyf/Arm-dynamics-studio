import { useSessionStore } from '../../stores/session-store';

interface EeTargetPanelProps {
  onRun: () => void;
  disabled?: boolean;
}

export function EeTargetPanel({ onRun, disabled }: EeTargetPanelProps) {
  const robotInfo = useSessionStore((s) => s.robotInfo);
  const eeTarget = useSessionStore((s) => s.eeTarget);
  const setEeTarget = useSessionStore((s) => s.setEeTarget);
  const simStatus = useSessionStore((s) => s.simStatus);

  if (!robotInfo) {
    return (
      <section className="panel-section">
        <h3>末端目标</h3>
        <p className="hint">请先加载模型</p>
      </section>
    );
  }

  const running = simStatus === 'running';
  const axes: Array<'x' | 'y' | 'z'> = ['x', 'y', 'z'];

  const setAxis = (axisIndex: number, value: number) => {
    const next = [...eeTarget] as [number, number, number];
    next[axisIndex] = value;
    setEeTarget(next);
  };

  return (
    <section className="panel-section">
      <h3>末端目标 (XYZ)</h3>
      <div className="xyz-grid">
        {axes.map((label, i) => (
          <label key={label} className="xyz-row">
            <span>{label.toUpperCase()}</span>
            <input
              type="number"
              step={0.01}
              value={eeTarget[i]}
              disabled={running || disabled}
              onChange={(e) => setAxis(i, Number(e.target.value))}
            />
          </label>
        ))}
      </div>
      <p className="hint">
        当前 FK: [{robotInfo.eePos.map((v) => v.toFixed(3)).join(', ')}]
      </p>
      <button
        type="button"
        className="primary"
        onClick={onRun}
        disabled={running || disabled}
      >
        运动到末端
      </button>
    </section>
  );
}
