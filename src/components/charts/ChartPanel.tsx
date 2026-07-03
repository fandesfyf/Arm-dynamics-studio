import { useState } from 'react';
import { SimCharts, type SimChartsProps } from './SimCharts';
import { ExportModal } from './ExportModal';
import './charts.css';

export interface ChartPanelProps extends SimChartsProps {
  title?: string;
  /** CSV 文件名前缀 */
  filenameBase?: string;
  /** 录制窗口秒数，传给 ExportModal「最近 N 秒」选项 */
  windowSeconds?: number;
  /** 环形缓冲保留窗口（秒） */
  recorderWindowSec?: number;
  onRecorderWindowChange?: (sec: number) => void;
}

/** 可独立挂载的底部曲线面板（§6.1） */
export function ChartPanel({
  title = '实时曲线',
  className,
  recorderDict,
  jointNames,
  filenameBase,
  windowSeconds,
  recorderWindowSec,
  onRecorderWindowChange,
  ...chartProps
}: ChartPanelProps) {
  const [exportOpen, setExportOpen] = useState(false);
  const hasData = Boolean(recorderDict && recorderDict.time.length > 0);

  return (
    <section className={['chart-panel', className].filter(Boolean).join(' ')}>
      {title ? (
        <div className="chart-panel-toolbar">
          <h3 className="chart-panel-title">{title}</h3>
        </div>
      ) : null}
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
}
