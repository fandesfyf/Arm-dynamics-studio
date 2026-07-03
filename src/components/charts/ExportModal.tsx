import { useEffect, useMemo, useState } from 'react';
import type { RecorderDict } from '../../core/data-recorder';
import {
  downloadCsv,
  exportDictToCsv,
  type CsvExportMetric,
  type CsvTimeRange,
} from '../../export/csv-exporter';
import { chartMetricLabel, type ChartMetric } from './chart-types';
import './charts.css';

const METRIC_OPTIONS: ChartMetric[] = ['position', 'velocity', 'torque', 'ee'];

export interface ExportModalProps {
  open: boolean;
  onClose: () => void;
  recorderDict: RecorderDict | null;
  jointNames?: string[];
  /** 下载文件名（不含扩展名） */
  filenameBase?: string;
  /** 录制窗口秒数，用于「窗口」选项说明 */
  windowSeconds?: number;
}

function defaultFilenameBase(): string {
  return 'robot_simulation';
}

export function ExportModal({
  open,
  onClose,
  recorderDict,
  jointNames,
  filenameBase = defaultFilenameBase(),
  windowSeconds = 30,
}: ExportModalProps) {
  const [metrics, setMetrics] = useState<Set<CsvExportMetric>>(
    () => new Set(['position', 'velocity', 'torque']),
  );
  const [selectedJoints, setSelectedJoints] = useState<Set<number>>(() => new Set());
  const [timeRange, setTimeRange] = useState<CsvTimeRange['mode']>('all');
  const [error, setError] = useState<string | null>(null);

  const names = jointNames ?? [];
  const hasData = Boolean(recorderDict && recorderDict.time.length > 0);

  useEffect(() => {
    if (!open) return;
    setError(null);
    setSelectedJoints(new Set(names.map((_, i) => i)));
  }, [open, names]);

  useEffect(() => {
    if (!open) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') onClose();
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [open, onClose]);

  const windowHint = useMemo(() => {
    if (!recorderDict || recorderDict.time.length === 0) return '';
    const last = recorderDict.time[recorderDict.time.length - 1] ?? 0;
    const start = Math.max(0, last - windowSeconds);
    return `${start.toFixed(2)}s – ${last.toFixed(2)}s`;
  }, [recorderDict, windowSeconds]);

  const toggleMetric = (metric: CsvExportMetric) => {
    setMetrics((prev) => {
      const next = new Set(prev);
      if (next.has(metric)) next.delete(metric);
      else next.add(metric);
      if (next.size === 0) next.add(metric);
      return next;
    });
  };

  const toggleJoint = (index: number) => {
    setSelectedJoints((prev) => {
      const next = new Set(prev);
      if (next.has(index)) next.delete(index);
      else next.add(index);
      if (next.size === 0) next.add(index);
      return next;
    });
  };

  const selectAllJoints = () => setSelectedJoints(new Set(names.map((_, i) => i)));
  const clearAllJoints = () => {
    if (names.length > 0) setSelectedJoints(new Set([0]));
  };

  const handleExport = () => {
    if (!recorderDict || !hasData) {
      setError('无录制数据可导出');
      return;
    }
    if (metrics.size === 0) {
      setError('请至少选择一种曲线类型');
      return;
    }

    const needsJoints = [...metrics].some((m) => m !== 'ee');
    if (needsJoints && selectedJoints.size === 0) {
      setError('请至少选择一个关节');
      return;
    }

    const csv = exportDictToCsv(recorderDict, {
      metrics: [...metrics],
      jointIndices: needsJoints ? [...selectedJoints].sort((a, b) => a - b) : undefined,
      timeRange:
        timeRange === 'window'
          ? { mode: 'window', windowSeconds }
          : { mode: 'all' },
      jointNames: names.length > 0 ? names : undefined,
    });

    if (!csv.trim() || csv.trim() === 'time') {
      setError('所选条件下无数据');
      return;
    }

    downloadCsv(csv, `${filenameBase}.csv`);
    onClose();
  };

  if (!open) return null;

  return (
    <div className="export-modal-overlay" role="presentation" onClick={onClose}>
      <div
        className="export-modal"
        role="dialog"
        aria-modal="true"
        aria-labelledby="export-modal-title"
        onClick={(e) => e.stopPropagation()}
      >
        <header className="export-modal-header">
          <h2 id="export-modal-title">导出 CSV</h2>
          <button type="button" className="export-modal-close" onClick={onClose} aria-label="关闭">
            ×
          </button>
        </header>

        <div className="export-modal-body">
          <fieldset className="export-modal-fieldset">
            <legend>曲线类型</legend>
            <div className="export-modal-checks">
              {METRIC_OPTIONS.map((metric) => (
                <label key={metric} className="export-modal-check">
                  <input
                    type="checkbox"
                    checked={metrics.has(metric)}
                    onChange={() => toggleMetric(metric)}
                  />
                  {chartMetricLabel(metric)}
                </label>
              ))}
            </div>
          </fieldset>

          {names.length > 0 && (
            <fieldset className="export-modal-fieldset">
              <legend>
                关节
                <span className="export-modal-legend-actions">
                  <button type="button" className="export-modal-link" onClick={selectAllJoints}>
                    全选
                  </button>
                  <button type="button" className="export-modal-link" onClick={clearAllJoints}>
                    清空
                  </button>
                </span>
              </legend>
              <div className="export-modal-joints">
                {names.map((name, i) => (
                  <label key={name} className="joint-pick-chip">
                    <input
                      type="checkbox"
                      checked={selectedJoints.has(i)}
                      onChange={() => toggleJoint(i)}
                    />
                    {name.replace(/_joint$/, '')}
                  </label>
                ))}
              </div>
            </fieldset>
          )}

          <fieldset className="export-modal-fieldset">
            <legend>时间范围</legend>
            <div className="export-modal-radios">
              <label className="export-modal-radio">
                <input
                  type="radio"
                  name="time-range"
                  checked={timeRange === 'all'}
                  onChange={() => setTimeRange('all')}
                />
                全部录制数据
              </label>
              <label className="export-modal-radio">
                <input
                  type="radio"
                  name="time-range"
                  checked={timeRange === 'window'}
                  onChange={() => setTimeRange('window')}
                />
                最近 {windowSeconds}s 窗口
                {windowHint ? <span className="export-modal-hint">（{windowHint}）</span> : null}
              </label>
            </div>
          </fieldset>

          <p className="export-modal-format">格式：CSV</p>

          {error && <p className="export-modal-error">{error}</p>}
        </div>

        <footer className="export-modal-footer">
          <button type="button" className="export-modal-btn" onClick={onClose}>
            取消
          </button>
          <button
            type="button"
            className="export-modal-btn export-modal-btn--primary"
            onClick={handleExport}
            disabled={!hasData}
          >
            导出
          </button>
        </footer>
      </div>
    </div>
  );
}
