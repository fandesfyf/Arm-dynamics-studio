import { useSessionStore } from '../../stores/session-store';
import {
  IK_WEIGHT_DEFAULTS,
  IK_WEIGHT_LIMITS,
  sanitizeIkWeights,
  type IkGoalMode,
  type PositionIkWeights,
} from '../../ik/ik-weight-config';
import { CollapsibleSection } from '../ui/CollapsibleSection';

export interface IkSolverSettingsProps {
  onResetReference?: () => void;
  disabled?: boolean;
}

const GOAL_MODES: { value: IkGoalMode; label: string }[] = [
  { value: 'position', label: '位置' },
  { value: 'pose', label: '位姿' },
  { value: 'orientation', label: '姿态' },
];

const WEIGHT_FIELDS: {
  key: keyof PositionIkWeights;
  label: string;
  format: (v: number) => string;
}[] = [
  { key: 'translationFactor', label: '平移权重 T', format: (v) => v.toFixed(3) },
  { key: 'rotationFactor', label: '旋转权重 R', format: (v) => v.toFixed(3) },
  { key: 'maxIterations', label: '最大迭代', format: (v) => String(Math.round(v)) },
  { key: 'dampingFactor', label: '阻尼', format: (v) => v.toFixed(3) },
  { key: 'translationErrorClamp', label: '步长限制 (m)', format: (v) => v.toFixed(3) },
  { key: 'divergeThreshold', label: '发散阈值', format: (v) => v.toFixed(3) },
  { key: 'convergedPositionTolerance', label: '收敛容差 (m)', format: (v) => v.toFixed(4) },
];

export function IkSolverSettings({ onResetReference, disabled }: IkSolverSettingsProps) {
  const ikEnabled = useSessionStore((s) => s.ikEnabled);
  const ikGoalMode = useSessionStore((s) => s.ikGoalMode);
  const ikWeights = useSessionStore((s) => s.ikWeights);
  const controlLayer = useSessionStore((s) => s.controlLayer);
  const setIkEnabled = useSessionStore((s) => s.setIkEnabled);
  const setIkGoalMode = useSessionStore((s) => s.setIkGoalMode);
  const setIkWeights = useSessionStore((s) => s.setIkWeights);

  const updateWeight = (key: keyof PositionIkWeights, raw: number) => {
    const next = sanitizeIkWeights({
      position: { ...ikWeights.position, [key]: raw },
    });
    setIkWeights(next);
  };

  const resetWeights = () => setIkWeights(sanitizeIkWeights(IK_WEIGHT_DEFAULTS));

  return (
    <>
      {controlLayer !== 'ee' && (
        <p className="hint">切换到「末端控制」后可拖动 Gizmo 实时求解 IK 参考姿态。</p>
      )}

      <label className="viz-toggle-row">
        <input
          type="checkbox"
          checked={ikEnabled}
          disabled={disabled}
          onChange={(e) => setIkEnabled(e.target.checked)}
        />
        <span>启用 IK</span>
      </label>

      <fieldset className="ik-goal-fieldset">
        <legend className="field-label">目标模式</legend>
        <div className="segmented-control" role="group" aria-label="IK 目标模式">
          {GOAL_MODES.map(({ value, label }) => (
            <button
              key={value}
              type="button"
              className={`btn btn-ghost segment-btn${ikGoalMode === value ? ' segment-active' : ''}`}
              disabled={disabled}
              onClick={() => setIkGoalMode(value)}
            >
              {label}
            </button>
          ))}
        </div>
      </fieldset>

      <CollapsibleIkWeights
        weights={ikWeights.position}
        disabled={disabled}
        onChange={updateWeight}
        onReset={resetWeights}
      />

      <div className="button-row">
        <button
          type="button"
          className="btn btn-ghost"
          disabled={disabled}
          onClick={() => onResetReference?.()}
        >
          重置参考姿态
        </button>
      </div>
    </>
  );
}

/** @deprecated Standalone dock panel — IK settings now live in ControlPanel. */
export interface IkPanelProps {
  onResetReference?: () => void;
  disabled?: boolean;
}

export function IkPanel({ onResetReference, disabled }: IkPanelProps) {
  const robotInfo = useSessionStore((s) => s.robotInfo);

  if (!robotInfo) {
    return (
      <section className="panel-section">
        <h3>IK</h3>
        <p className="hint">请先加载模型</p>
      </section>
    );
  }

  return (
    <section className="panel-section ik-panel">
      <h3>IK</h3>
      <IkSolverSettings onResetReference={onResetReference} disabled={disabled} />
    </section>
  );
}

function CollapsibleIkWeights({
  weights,
  disabled,
  onChange,
  onReset,
}: {
  weights: PositionIkWeights;
  disabled?: boolean;
  onChange: (key: keyof PositionIkWeights, value: number) => void;
  onReset: () => void;
}) {
  return (
    <details className="ik-weights-details">
      <summary className="field-label">求解器权重</summary>
      <div className="ik-weights-grid">
        {WEIGHT_FIELDS.map(({ key, label, format }) => {
          const lim = IK_WEIGHT_LIMITS[key as keyof typeof IK_WEIGHT_LIMITS];
          const value = weights[key];
          return (
            <label key={key} className="ik-weight-row">
              <span className="ik-weight-label">{label}</span>
              <input
                type="range"
                min={lim?.min ?? 0}
                max={lim?.max ?? 1}
                step={lim?.step ?? 0.001}
                value={value}
                disabled={disabled}
                onChange={(e) => onChange(key, Number(e.target.value))}
              />
              <span className="ik-weight-value">{format(value)}</span>
            </label>
          );
        })}
      </div>
      <button type="button" className="btn btn-ghost btn-sm" disabled={disabled} onClick={onReset}>
        恢复默认权重
      </button>
    </details>
  );
}

export function IkSolverSection({ onResetReference, disabled }: IkSolverSettingsProps) {
  return (
    <CollapsibleSection title="IK 求解器" defaultOpen={false}>
      <IkSolverSettings onResetReference={onResetReference} disabled={disabled} />
    </CollapsibleSection>
  );
}
