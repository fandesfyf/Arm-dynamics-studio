import { memo, useState } from 'react';
import { useSessionStore } from '../../stores/session-store';
import { SimCharts, type SimChartsProps } from './SimCharts';
import { ExportModal } from './ExportModal';
import './charts.css';

export interface ChartPanelProps extends Omit<SimChartsProps, 'recorderDict'> {
  title?: string;
  /** CSV 文件名前缀 */
  filenameBase?: string;
  /** 录制窗口秒数，传给 ExportModal「最近 N 秒」选项 */
  windowSeconds?: number;
  /** 环形缓冲保留窗口（秒） */
  recorderWindowSec?: number;
  onRecorderWindowChange?: (sec: number) => void;
  onToggleRecorderPause?: () => void;
  onResetRecorder?: () => void;
  recorderControlsDisabled?: boolean;
}

/** 可独立挂载的底部曲线面板（§6.1） */
export const ChartPanel = memo(function ChartPanel({
  title = '实时曲线',
  className,
  jointNames,
  filenameBase,
  windowSeconds,
  recorderWindowSec,
  onRecorderWindowChange,
  onToggleRecorderPause,
  onResetRecorder,
  recorderControlsDisabled,
  ...chartProps
}: ChartPanelProps) {
  const [exportOpen, setExportOpen] = useState(false);
  const recorderDict = useSessionStore((s) => s.recorderDict);
  const hasData = Boolean(recorderDict && recorderDict.time.length > 0);
  const simStatus = useSessionStore((s) => s.simStatus);
  const recorderPaused = useSessionStore((s) => s.recorderPaused);
  const running = simStatus === 'running';
  const showRecorderControls = Boolean(onToggleRecorderPause || onResetRecorder);

  return (
    <section className={['chart-panel', className].filter(Boolean).join(' ')}>
      {(title || showRecorderControls) && (
        <div className="chart-panel-toolbar">
          {title ? <h3 className="chart-panel-title">{title}</h3> : <span />}
          {showRecorderControls && (
            <div className="chart-panel-recorder-actions">
              {onToggleRecorderPause && (
                <button
                  type="button"
                  className="btn btn-ghost btn-compact"
                  disabled={!running || recorderControlsDisabled}
                  onClick={onToggleRecorderPause}
                  title={running ? undefined : '开始仿真后可暂停/继续录制'}
                >
                  {recorderPaused ? '▶ 继续录制' : '⏸ 暂停录制'}
                </button>
              )}
              {onResetRecorder && (
                <button
                  type="button"
                  className="btn btn-ghost btn-compact"
                  disabled={recorderControlsDisabled}
                  onClick={onResetRecorder}
                >
                  清空曲线数据
                </button>
              )}
            </div>
          )}
        </div>
      )}
      <SimCharts
        {...chartProps}
        recorderDict={recorderDict}
        jointNames={jointNames}
        recorderWindowSec={recorderWindowSec}
        onRecorderWindowChange={onRecorderWindowChange}
        onExportClick={() => setExportOpen(true)}
        exportDisabled={!hasData}
        exportTitle={hasData ? '导出 CSV' : '运行仿真后可导出'}
      />
      <ExportModal
        open={exportOpen}
        onClose={() => setExportOpen(false)}
        recorderDict={recorderDict ?? null}
        jointNames={jointNames}
        filenameBase={filenameBase}
        windowSeconds={windowSeconds}
      />
    </section>
  );
});
