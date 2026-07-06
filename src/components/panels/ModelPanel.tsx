import { useSessionStore } from '../../stores/session-store';
import { CollapsibleSection } from '../ui/CollapsibleSection';
import { RobotUpload, type RobotUploadResult } from './RobotUpload';
import { BaseLinkSelector } from './BaseLinkSelector';
import { MassEditorPanel } from './MassEditorPanel';

export interface ModelPanelProps {
  onRobotLoaded: (result: RobotUploadResult) => Promise<void>;
  onLoadTestArm: () => Promise<void>;
  onApplyBaseLink: (link: string) => void;
  onUrdfChanged: (xml: string) => void;
  onResetRobotPose?: () => void;
  /** True while model is loading */
  disabled?: boolean;
}

function JointPreviewSection({ disabled }: { disabled?: boolean }) {
  const robotInfo = useSessionStore((s) => s.robotInfo);
  const jointPositions = useSessionStore((s) => s.jointPositions);
  const setJointPositions = useSessionStore((s) => s.setJointPositions);
  const simStatus = useSessionStore((s) => s.simStatus);

  if (!robotInfo) {
    return <p className="hint">加载模型后可预览关节姿态</p>;
  }

  const previewDisabled = disabled || simStatus === 'running';

  const handleSlider = (index: number, value: number) => {
    const next = [...jointPositions];
    next[index] = value;
    setJointPositions(next);
  };

  return (
    <section className="panel-section joint-preview-section">
      <p className="hint">仅运动学预览，仿真运行中不可用</p>
      <div className="joint-grid">
        {robotInfo.jointNames.map((name, i) => {
          const lo = robotInfo.lowerLimits[i] ?? -3.14;
          const hi = robotInfo.upperLimits[i] ?? 3.14;
          const val = jointPositions[i] ?? 0;
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
                disabled={previewDisabled}
                onChange={(e) => handleSlider(i, Number(e.target.value))}
              />
              <input
                type="number"
                className="joint-value"
                min={lo}
                max={hi}
                step={0.01}
                value={Number(val.toFixed(3))}
                disabled={previewDisabled}
                onChange={(e) => handleSlider(i, Number(e.target.value))}
              />
            </label>
          );
        })}
      </div>
    </section>
  );
}

export default function ModelPanel({
  onRobotLoaded,
  onLoadTestArm,
  onApplyBaseLink,
  onUrdfChanged,
  onResetRobotPose,
  disabled = false,
}: ModelPanelProps) {
  const urdfText = useSessionStore((s) => s.urdfText);
  const robotInfo = useSessionStore((s) => s.robotInfo);
  const simStatus = useSessionStore((s) => s.simStatus);
  const baseLinkDisabled = disabled || simStatus === 'running';
  const resetDisabled = disabled || !robotInfo || simStatus === 'running';

  return (
    <>
      <RobotUpload onLoadTestArm={onLoadTestArm} onRobotLoaded={onRobotLoaded} />
      {robotInfo && (
        <section className="panel-section panel-section--compact">
          <div className="button-row">
            <button
              type="button"
              className="btn btn-ghost"
              disabled={resetDisabled}
              title="将仿真状态与关节目标重置到零位"
              onClick={() => onResetRobotPose?.()}
            >
              重置模型零位
            </button>
          </div>
        </section>
      )}
      <section className="panel-section panel-section--compact">
        <h3>固定基座</h3>
        <BaseLinkSelector onApply={onApplyBaseLink} disabled={baseLinkDisabled} />
      </section>
      {urdfText && (
        <CollapsibleSection title="质量编辑" icon="⚖️" defaultOpen={false}>
          <MassEditorPanel urdfXml={urdfText} onUrdfChanged={onUrdfChanged} />
        </CollapsibleSection>
      )}
      <CollapsibleSection title="关节预览" icon="🦾" defaultOpen={false}>
        <JointPreviewSection disabled={disabled} />
      </CollapsibleSection>
    </>
  );
}
