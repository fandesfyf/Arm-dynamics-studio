import { useSessionStore } from '../../stores/session-store';

interface ExportPanelProps {
  onExport: () => void;
  disabled?: boolean;
}

export function ExportPanel({ onExport, disabled }: ExportPanelProps) {
  const recorder = useSessionStore((s) => s.recorder);
  const robotInfo = useSessionStore((s) => s.robotInfo);
  const simStatus = useSessionStore((s) => s.simStatus);

  const running = simStatus === 'running';
  const hasData = recorder.sampleCount > 0;

  return (
    <section className="panel-section">
      <h3>数据导出</h3>
      <p className="hint">
        {hasData
          ? `已录制 ${recorder.sampleCount} 条` +
            (recorder.lastTime != null ? `，末时刻 ${recorder.lastTime.toFixed(2)}s` : '')
          : '运行仿真后导出 CSV'}
      </p>
      <button
        type="button"
        className="primary"
        onClick={onExport}
        disabled={!robotInfo || !hasData || running || disabled}
      >
        导出 CSV
      </button>
    </section>
  );
}
