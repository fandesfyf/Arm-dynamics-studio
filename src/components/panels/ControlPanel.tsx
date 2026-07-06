import { useMemo, useRef, useState } from 'react';
import { useSessionStore, type IkLiveStatus } from '../../stores/session-store';
import type { ControlLayer, ControlMode } from '../../stores/session-store';
import { listEndEffectorLinkCandidates } from '../../utils/urdf-base-fixture';
import { CONTROLLER_OMEGA } from '../../core/controller';
import { CollapsibleSection } from '../ui/CollapsibleSection';
import { IkSolverSection } from './IkPanel';

export interface ControlPanelProps {
  onAddMotionTarget: () => void | Promise<void>;
  onExecuteMotionTargets: () => void;
  onEndEffectorLinkChange?: (link: string) => void;
  onSetJointGain?: (index: number, kp: number, kd: number) => void;
  onResetJointGains?: () => void;
  onControllerKdDampingChange?: (value: number) => void;
  onResetReference?: () => void;
  onResetGizmo?: () => void;
  onExportMotionTargets?: () => void;
  onImportMotionTargets?: (file: File) => void | Promise<void>;
  disabled?: boolean;
}

function ikStatusLabel(status: IkLiveStatus): string {
  switch (status) {
    case 'solving':
      return '求解中';
    case 'converged':
      return '收敛';
    case 'failed':
      return '失败';
    default:
      return '就绪';
  }
}

export function ControlPanel({
  onAddMotionTarget,
  onExecuteMotionTargets,
  onEndEffectorLinkChange,
  onSetJointGain,
  onResetJointGains,
  onControllerKdDampingChange,
  onResetReference,
  onResetGizmo,
  onExportMotionTargets,
  onImportMotionTargets,
  disabled,
}: ControlPanelProps) {
  const robotInfo = useSessionStore((s) => s.robotInfo);
  const urdfText = useSessionStore((s) => s.urdfText);
  const jointTargets = useSessionStore((s) => s.jointTargets);
  const jointPositions = useSessionStore((s) => s.jointPositions);
  const jointKp = useSessionStore((s) => s.jointKp);
  const jointKd = useSessionStore((s) => s.jointKd);
  const eeTarget = useSessionStore((s) => s.eeTarget);
  const endEffectorLink = useSessionStore((s) => s.endEffectorLink);
  const ikLiveStatus = useSessionStore((s) => s.ikLiveStatus);
  const ikLiveMessage = useSessionStore((s) => s.ikLiveMessage);
  const ikLastSolveMs = useSessionStore((s) => s.ikLastSolveMs);
  const simStatus = useSessionStore((s) => s.simStatus);
  const interpolationActive = useSessionStore((s) => s.interpolationActive);
  const controlLayer = useSessionStore((s) => s.controlLayer);
  const controlMode = useSessionStore((s) => s.controlMode);
  const jointMaxVelocity = useSessionStore((s) => s.jointMaxVelocity);
  const controllerKdDamping = useSessionStore((s) => s.controllerKdDamping);
  const motionTargets = useSessionStore((s) => s.motionTargets);
  const interpProfile = useSessionStore((s) => s.interpProfile);
  const setControlLayer = useSessionStore((s) => s.setControlLayer);
  const setControlMode = useSessionStore((s) => s.setControlMode);
  const setJointMaxVelocity = useSessionStore((s) => s.setJointMaxVelocity);
  const setInterpProfile = useSessionStore((s) => s.setInterpProfile);
  const removeMotionTarget = useSessionStore((s) => s.removeMotionTarget);
  const clearMotionTargets = useSessionStore((s) => s.clearMotionTargets);
  const setJointTargetAt = useSessionStore((s) => s.setJointTargetAt);
  const setEeTarget = useSessionStore((s) => s.setEeTarget);
  const setEeTargetDirty = useSessionStore((s) => s.setEeTargetDirty);

  const eeLinkOptions = useMemo(() => {
    if (!urdfText) return [];
    const links = listEndEffectorLinkCandidates(urdfText);
    if (endEffectorLink && !links.includes(endEffectorLink)) {
      return [endEffectorLink, ...links];
    }
    return links;
  }, [urdfText, endEffectorLink]);

  const [addTargetBusy, setAddTargetBusy] = useState(false);
  const importCsvInputRef = useRef<HTMLInputElement>(null);

  if (!robotInfo) {
    return (
      <section className="panel-section">
        <h3>控制</h3>
        <p className="hint">请先加载模型</p>
      </section>
    );
  }

  const running = simStatus === 'running';
  const modeLocked = disabled || interpolationActive || (running && controlMode === 'realtime');
  const slidersDisabled = disabled || interpolationActive;
  const axes: Array<'x' | 'y' | 'z'> = ['x', 'y', 'z'];

  const handleLayer = (layer: ControlLayer) => setControlLayer(layer);
  const handleMode = (mode: ControlMode) => setControlMode(mode);

  const handleAddTarget = async () => {
    setAddTargetBusy(true);
    try {
      await onAddMotionTarget();
    } finally {
      setAddTargetBusy(false);
    }
  };

  const syncJointTargetsToCurrent = () => {
    useSessionStore.getState().setJointTargets([...jointPositions]);
  };

  const resetTargetToCurrent = () => {
    if (controlLayer === 'joint') {
      syncJointTargetsToCurrent();
    } else {
      onResetGizmo?.();
    }
  };

  const setEeAxis = (axisIndex: number, value: number) => {
    const next = [...eeTarget] as [number, number, number];
    next[axisIndex] = value;
    setEeTarget(next);
    setEeTargetDirty(true);
  };

  return (
    <section className="panel-section control-panel">
      <h3>控制</h3>

      <div className="control-header-row">
        <div className="segmented-control control-layer-segment" role="group" aria-label="控制层">
          <button
            type="button"
            className={`btn btn-ghost segment-btn${controlLayer === 'joint' ? ' segment-active' : ''}`}
            disabled={modeLocked}
            onClick={() => handleLayer('joint')}
          >
            关节
          </button>
          <button
            type="button"
            className={`btn btn-ghost segment-btn${controlLayer === 'ee' ? ' segment-active' : ''}`}
            disabled={modeLocked}
            onClick={() => handleLayer('ee')}
          >
            末端
          </button>
        </div>
        <div className="segmented-control control-mode-segment" role="group" aria-label="控制模式">
          <button
            type="button"
            className={`btn btn-ghost segment-btn segment-btn-compact${
              controlMode === 'realtime' ? ' segment-active' : ''
            }`}
            disabled={modeLocked}
            title="实时拖动：仿真运行中松开 Gizmo 立即插值"
            onClick={() => handleMode('realtime')}
          >
            实时
          </button>
          <button
            type="button"
            className={`btn btn-ghost segment-btn segment-btn-compact${
              controlMode === 'interpolate' ? ' segment-active' : ''
            }`}
            disabled={modeLocked}
            onClick={() => handleMode('interpolate')}
            title="插值发送"
          >
            插值
          </button>
        </div>
      </div>

      {controlMode === 'interpolate' && (
        <>
          <div className="interp-send-row">
            <label className="field-label field-label-inline">
              关节限速 (rad/s)
              <input
                type="number"
                min={0.05}
                max={10}
                step={0.05}
                value={jointMaxVelocity}
                disabled={disabled || interpolationActive}
                onChange={(e) => setJointMaxVelocity(Number(e.target.value))}
              />
            </label>
            <div className="segmented-control interp-profile-segment" role="group" aria-label="插值方式">
              <button
                type="button"
                className={`btn btn-ghost segment-btn segment-btn-compact${
                  interpProfile === 'linear' ? ' segment-active' : ''
                }`}
                disabled={disabled || interpolationActive}
                onClick={() => setInterpProfile('linear')}
              >
                线性
              </button>
              <button
                type="button"
                className={`btn btn-ghost segment-btn segment-btn-compact${
                  interpProfile === 'cubic' ? ' segment-active' : ''
                }`}
                disabled={disabled || interpolationActive}
                onClick={() => setInterpProfile('cubic')}
              >
                三次样条
              </button>
            </div>
          </div>
          <div className="interp-send-row">
            <button
              type="button"
              className="btn btn-ghost"
              onClick={resetTargetToCurrent}
              disabled={slidersDisabled}
            >
              重置
            </button>
            <button
              type="button"
              className="btn btn-ghost"
              onClick={() => void handleAddTarget()}
              disabled={disabled || interpolationActive || addTargetBusy}
              title="将当前目标加入队列 (K)"
            >
              {addTargetBusy ? (
                '添加中…'
              ) : (
                <>
                  添加目标 <kbd className="btn-kbd">K</kbd>
                </>
              )}
            </button>
            <button
              type="button"
              className="primary"
              onClick={onExecuteMotionTargets}
              disabled={disabled || interpolationActive}
              title="执行插值运动 (F)"
            >
              {interpolationActive ? (
                '插值中…'
              ) : (
                <>
                  执行 <kbd className="btn-kbd">F</kbd>
                </>
              )}
            </button>
          </div>
          <CollapsibleSection
            title={`目标队列 (${motionTargets.length})`}
            defaultOpen={false}
          >
            <div className="motion-target-toolbar">
              <button
                type="button"
                className="btn btn-ghost btn-compact"
                disabled={disabled || interpolationActive || motionTargets.length === 0}
                onClick={() => clearMotionTargets()}
              >
                清空
              </button>
              <button
                type="button"
                className="btn btn-ghost btn-compact"
                disabled={disabled}
                onClick={() => importCsvInputRef.current?.click()}
              >
                导入 CSV
              </button>
              <button
                type="button"
                className="btn btn-ghost btn-compact"
                disabled={disabled || motionTargets.length === 0}
                onClick={() => onExportMotionTargets?.()}
              >
                导出 CSV
              </button>
              <input
                ref={importCsvInputRef}
                type="file"
                accept=".csv,text/csv"
                hidden
                disabled={disabled}
                onChange={(e) => {
                  const file = e.target.files?.[0];
                  e.target.value = '';
                  if (file) void onImportMotionTargets?.(file);
                }}
              />
            </div>
            {motionTargets.length === 0 ? (
              <p className="hint">
                调整关节或末端目标后点「添加目标」加入队列；「执行」按队列顺序插值（队列为空时执行当前单帧目标）。
              </p>
            ) : (
              <ul className="waypoint-list motion-target-list">
                {motionTargets.map((mt, i) => (
                  <li key={mt.id}>
                    <span className="motion-target-label">
                      #{i + 1} · 末端 [
                      {mt.eePosition.map((v) => v.toFixed(3)).join(', ')}]
                    </span>
                    <button
                      type="button"
                      className="small"
                      disabled={disabled || interpolationActive}
                      onClick={() => removeMotionTarget(mt.id)}
                    >
                      删除
                    </button>
                  </li>
                ))}
              </ul>
            )}
          </CollapsibleSection>
        </>
      )}

      {controlLayer === 'joint' ? (
        <>
          <div className="joint-grid">
            {robotInfo.jointNames.map((name, i) => {
              const lo = robotInfo.lowerLimits[i] ?? -3.14;
              const hi = robotInfo.upperLimits[i] ?? 3.14;
              const val = jointTargets[i] ?? jointPositions[i] ?? 0;
              return (
                <label key={name} className="joint-row">
                  <span className="joint-label" title={name}>
                    {name.length > 18 ? `${name.slice(0, 16)}…` : name}
                  </span>
                  <input
                    type="range"
                    min={lo}
                    max={hi}
                    step={0.01}
                    value={val}
                    disabled={slidersDisabled}
                    onChange={(e) => setJointTargetAt(i, Number(e.target.value))}
                  />
                  <input
                    type="number"
                    className="joint-value"
                    min={lo}
                    max={hi}
                    step={0.01}
                    value={Number(val.toFixed(3))}
                    disabled={slidersDisabled}
                    onChange={(e) => setJointTargetAt(i, Number(e.target.value))}
                  />
                </label>
              );
            })}
          </div>
        </>
      ) : (
        <>
          <label className="field-label">
            末端 Link
            <select
              value={endEffectorLink}
              disabled={disabled || eeLinkOptions.length === 0}
              onChange={(e) => onEndEffectorLinkChange?.(e.target.value)}
            >
              {eeLinkOptions.map((name) => (
                <option key={name} value={name}>
                  {name}
                </option>
              ))}
            </select>
          </label>

          <div className="xyz-grid">
            {axes.map((label, i) => (
              <label key={label} className="xyz-row">
                <span>{label.toUpperCase()}</span>
                <input
                  type="number"
                  step={0.01}
                  value={eeTarget[i]}
                  disabled={slidersDisabled}
                  onChange={(e) => setEeAxis(i, Number(e.target.value))}
                />
              </label>
            ))}
          </div>

          <div className={`ik-status-line ik-status-line--${ikLiveStatus}`}>
            <span className="ik-status-line-label">IK {ikStatusLabel(ikLiveStatus)}</span>
            {ikLastSolveMs != null && ikLiveStatus !== 'idle' && (
              <span className="ik-status-line-ms">{ikLastSolveMs.toFixed(0)} ms</span>
            )}
            {ikLiveMessage && (
              <span className="ik-status-line-msg" title={ikLiveMessage}>
                {ikLiveMessage}
              </span>
            )}
          </div>

          <div className="button-row">
            <button
              type="button"
              className="btn btn-ghost"
              onClick={onResetGizmo}
              disabled={slidersDisabled}
            >
              重置 Gizmo 到当前位置
            </button>
          </div>

          <p className="hint">
            {controlMode === 'realtime'
              ? '实时模式：仿真运行中拖动 Gizmo 松开后立即插值到目标。'
              : '插值模式：调整目标后「添加目标」入队，点「执行」开始仿真并插值。'}
          </p>
        </>
      )}

      <CollapsibleSection title="关节增益 PD" defaultOpen={false}>
        <div className="controller-gain-doc">
          <p className="hint">
            逆动力学前馈 + 关节 PD 反馈；自动增益由 <code>diag(mj_fullM)</code> 计算，可逐关节覆盖。
          </p>
          <pre className="controller-gain-formula">{`Kp = max(ω²·M_ii, 0.5)   Kd = ζ·√(Kp·M_ii)
ω = ${CONTROLLER_OMEGA} rad/s   ζ ≈ 阻尼系数 / 2`}</pre>
        </div>
        <div className="controller-gain-toolbar">
          <label className="field-label field-label-inline">
            阻尼系数
            <input
              type="number"
              min={0.5}
              max={10}
              step={0.1}
              value={controllerKdDamping}
              disabled={disabled}
              title="Kd = 阻尼系数 × √(Kp·M_ii)，增大可抑制超调"
              onChange={(e) => onControllerKdDampingChange?.(Number(e.target.value))}
            />
          </label>
          <button
            type="button"
            className="btn btn-ghost"
            disabled={disabled}
            onClick={() => onResetJointGains?.()}
          >
            重置关节增益
          </button>
        </div>
        <div className="joint-gains-grid">
          {robotInfo.jointNames.map((name, i) => {
            const kp = jointKp[i] ?? 0;
            const kd = jointKd[i] ?? 0;
            return (
              <div key={`gain-${name}`} className="joint-gain-row">
                <span className="joint-label" title={name}>
                  {name.length > 16 ? `${name.slice(0, 14)}…` : name}
                </span>
                <label className="joint-gain-field">
                  Kp
                  <input
                    type="number"
                    min={0}
                    step={1}
                    value={Number(kp.toFixed(2))}
                    disabled={disabled}
                    onChange={(e) => onSetJointGain?.(i, Number(e.target.value), kd)}
                  />
                </label>
                <label className="joint-gain-field">
                  Kd
                  <input
                    type="number"
                    min={0}
                    step={0.1}
                    value={Number(kd.toFixed(2))}
                    disabled={disabled}
                    onChange={(e) => onSetJointGain?.(i, kp, Number(e.target.value))}
                  />
                </label>
              </div>
            );
          })}
        </div>
      </CollapsibleSection>

      <IkSolverSection onResetReference={onResetReference} disabled={disabled} />
    </section>
  );
}
