import {
  REFERENCE_TF_FRAME_SIZE_MAX,
  REFERENCE_TF_FRAME_SIZE_MIN,
  useVizStore,
  type ReferencePoseStyle,
} from '../../stores/viz-store';

const REFERENCE_STYLE_OPTIONS: { value: ReferencePoseStyle; label: string }[] = [
  { value: 'tf_frames', label: 'TF 坐标系（默认）' },
  { value: 'ghost', label: '绿色幽灵模型' },
  { value: 'off', label: '关闭' },
];

export function VisualizationPanel() {
  const showCollision = useVizStore((s) => s.showCollision);
  const showInertia = useVizStore((s) => s.showInertia);
  const modelOpacity = useVizStore((s) => s.modelOpacity);
  const showJointAxes = useVizStore((s) => s.showJointAxes);
  const referencePoseStyle = useVizStore((s) => s.referencePoseStyle);
  const referenceTfFrameSize = useVizStore((s) => s.referenceTfFrameSize);
  const referenceTfShowChainLines = useVizStore((s) => s.referenceTfShowChainLines);
  const setShowCollision = useVizStore((s) => s.setShowCollision);
  const setShowInertia = useVizStore((s) => s.setShowInertia);
  const setModelOpacity = useVizStore((s) => s.setModelOpacity);
  const setShowJointAxes = useVizStore((s) => s.setShowJointAxes);
  const setReferencePoseStyle = useVizStore((s) => s.setReferencePoseStyle);
  const setReferenceTfFrameSize = useVizStore((s) => s.setReferenceTfFrameSize);
  const setReferenceTfShowChainLines = useVizStore((s) => s.setReferenceTfShowChainLines);

  const tfControlsVisible = referencePoseStyle === 'tf_frames';

  return (
    <section className="panel-section visualization-panel">
      <h3>可视化</h3>

      <label className="field-label">
        参考位姿显示
        <select
          value={referencePoseStyle}
          onChange={(e) => setReferencePoseStyle(e.target.value as ReferencePoseStyle)}
        >
          {REFERENCE_STYLE_OPTIONS.map((opt) => (
            <option key={opt.value} value={opt.value}>
              {opt.label}
            </option>
          ))}
        </select>
      </label>

      {tfControlsVisible && (
        <>
          <label className="field-label">
            参考 TF 坐标轴尺寸
            <div className="viz-opacity-row">
              <input
                type="range"
                min={REFERENCE_TF_FRAME_SIZE_MIN}
                max={REFERENCE_TF_FRAME_SIZE_MAX}
                step={0.01}
                value={referenceTfFrameSize}
                onChange={(e) => setReferenceTfFrameSize(Number(e.target.value))}
              />
              <span className="viz-opacity-value">{referenceTfFrameSize.toFixed(2)}</span>
            </div>
          </label>

          <label className="viz-toggle-row">
            <input
              type="checkbox"
              checked={referenceTfShowChainLines}
              onChange={(e) => setReferenceTfShowChainLines(e.target.checked)}
            />
            <span>显示运动学链连线</span>
          </label>
        </>
      )}

      <label className="viz-toggle-row">
        <input
          type="checkbox"
          checked={showCollision}
          onChange={(e) => setShowCollision(e.target.checked)}
        />
        <span>显示碰撞体</span>
      </label>

      <label className="viz-toggle-row">
        <input
          type="checkbox"
          checked={showInertia}
          onChange={(e) => setShowInertia(e.target.checked)}
        />
        <span>显示惯量</span>
      </label>

      <label className="viz-toggle-row">
        <input
          type="checkbox"
          checked={showJointAxes}
          onChange={(e) => setShowJointAxes(e.target.checked)}
        />
        <span>显示关节轴</span>
      </label>

      <label className="field-label">
        模型透明度
        <div className="viz-opacity-row">
          <input
            type="range"
            min={0.1}
            max={1}
            step={0.05}
            value={modelOpacity}
            onChange={(e) => setModelOpacity(Number(e.target.value))}
          />
          <span className="viz-opacity-value">{modelOpacity.toFixed(2)}</span>
        </div>
      </label>

      <p className="hint">
        TF 模式显示全身参考姿态（所有关节与 link 的 RGB 坐标轴；可选高亮当前末端运动学链连线）。
        关节模式参考目标角，末端模式参考 IK 解；幽灵模式为绿色半透明整模。
      </p>
    </section>
  );
}
