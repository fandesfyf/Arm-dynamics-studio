import { memo, useCallback, useEffect, useMemo, useRef, useState } from 'react';
import type uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';
import {
  bundleFromRecorderDict,
  chartMetricLabel,
  pickActiveSeries,
  type ChartMetric,
  type ChartSeriesBundle,
  type ChartSeriesInput,
} from './chart-types';
import type { RecorderDict } from '../../core/data-recorder';
import { chartLiveBuffer } from '../../core/chart-live-buffer';
import { useSessionStore } from '../../stores/session-store';
import './charts.css';

const JOINT_COLORS = [
  '#1976d2',
  '#d32f2f',
  '#388e3c',
  '#f57c00',
  '#7b1fa2',
  '#0097a7',
  '#c2185b',
  '#5d4037',
];

const PLOT_MIN_HEIGHT = 80;

function yRangePad(min: number, max: number): [number, number] {
  if (!Number.isFinite(min) || !Number.isFinite(max)) {
    return [-1, 1];
  }
  const span = max - min;
  const pad = span > 0 ? span * 0.08 : Math.abs(max || min || 1) * 0.1 + 0.001;
  return [min - pad, max + pad];
}

function xRangePad(min: number, max: number): [number, number] {
  if (!Number.isFinite(min) || !Number.isFinite(max)) {
    return [0, 1];
  }
  if (min === max) {
    const pad = Math.abs(min) * 0.05 + 0.001;
    return [min - pad, max + pad];
  }
  const span = max - min;
  const pad = span * 0.02;
  return [min - pad, max + pad];
}

function createInteractionPlugins(getCtx: () => {
  onUserViewChange: () => void;
  onRestoreFollow: (u: uPlot) => void;
  isProgrammatic: () => boolean;
}): uPlot.Plugin[] {
  return [
    {
      hooks: {
        ready: (u) => {
          const over = u.over;

          over.addEventListener(
            'wheel',
            (e) => {
              e.preventDefault();
              getCtx().onUserViewChange();
              const { left, width } = u.bbox;
              const leftPx = e.clientX - over.getBoundingClientRect().left - left;
              const pct = Math.min(1, Math.max(0, leftPx / width));
              const xVal = u.posToVal(leftPx, 'x');
              const oxMin = u.scales.x.min ?? 0;
              const oxMax = u.scales.x.max ?? 1;
              const oxRange = oxMax - oxMin;
              const factor = e.deltaY < 0 ? 0.85 : 1.15;
              const nxRange = oxRange * factor;
              const nxMin = xVal - pct * nxRange;
              u.setScale('x', { min: nxMin, max: nxMin + nxRange });
            },
            { passive: false },
          );

          let panning = false;
          let panStartX = 0;
          let panStartY = 0;
          let panXMin = 0;
          let panXMax = 0;
          let panYMin = 0;
          let panYMax = 0;

          const onPanMove = (e: MouseEvent) => {
            if (!panning) return;
            const dx = e.clientX - panStartX;
            const dy = e.clientY - panStartY;
            const xRange = panXMax - panXMin;
            const yRange = panYMax - panYMin;
            const shiftX = (-dx / u.bbox.width) * xRange;
            const shiftY = (dy / u.bbox.height) * yRange;
            u.setScale('x', { min: panXMin + shiftX, max: panXMax + shiftX });
            u.setScale('y', { min: panYMin + shiftY, max: panYMax + shiftY });
          };

          const endPan = () => {
            panning = false;
            window.removeEventListener('mousemove', onPanMove);
            window.removeEventListener('mouseup', endPan);
          };

          over.addEventListener('mousedown', (e) => {
            if (e.button !== 1 && !e.shiftKey) return;
            e.preventDefault();
            getCtx().onUserViewChange();
            panning = true;
            panStartX = e.clientX;
            panStartY = e.clientY;
            panXMin = u.scales.x.min ?? 0;
            panXMax = u.scales.x.max ?? 1;
            panYMin = u.scales.y.min ?? 0;
            panYMax = u.scales.y.max ?? 1;
            window.addEventListener('mousemove', onPanMove);
            window.addEventListener('mouseup', endPan);
          });

          over.addEventListener('dblclick', () => {
            getCtx().onRestoreFollow(u);
          });
        },
        /** 仅用户主动改 X 轴时暂停跟随；Y 轴 auto 与 setData 触发的 setScale 忽略 */
        setScale: (_u, scaleKey) => {
          if (scaleKey === 'x' && !getCtx().isProgrammatic()) {
            getCtx().onUserViewChange();
          }
        },
      },
    },
  ];
}

export interface SimChartsProps {
  /** 便捷：从 DataRecorder.toDict() 推导三组曲线 */
  recorderDict?: RecorderDict | null;
  /** 或直接传入已适配的三组曲线 */
  series?: Partial<ChartSeriesBundle> | null;
  jointNames?: string[];
  height?: number;
  className?: string;
  onExportClick?: () => void;
  exportDisabled?: boolean;
  exportTitle?: string;
  recorderWindowSec?: number;
  onRecorderWindowChange?: (sec: number) => void;
}

function chartDataFingerprint(data: ChartSeriesInput, metricKey = ''): string {
  const n = data.times.length;
  if (n === 0) return '0';
  const rowN = data.actual[n - 1];
  const tail = rowN ? rowN.join(',') : '';
  const t0 = data.times[0] ?? 0;
  const t1 = data.times[n - 1]!;
  return `${metricKey}:${n}:${t0}:${t1}:${data.jointNames.join(',')}:${tail}`;
}

function buildUPlotPayload(
  data: ChartSeriesInput,
): {
  aligned: uPlot.AlignedData;
  series: uPlot.Series[];
} {
  const { times, jointNames, actual, desired } = data;
  const n = times.length;
  const aligned: uPlot.AlignedData = [times];
  const series: uPlot.Series[] = [{}];

  for (let j = 0; j < jointNames.length; j++) {
    const color = JOINT_COLORS[j % JOINT_COLORS.length];
    const actualCol = new Array<number>(n);
    for (let i = 0; i < n; i++) {
      actualCol[i] = actual[i]?.[j] ?? 0;
    }
    aligned.push(actualCol);
    series.push({
      label: desired ? `${jointNames[j]} 实际` : jointNames[j],
      stroke: color,
      width: 2,
    });

    if (desired) {
      const desiredCol = new Array<number>(n);
      for (let i = 0; i < n; i++) {
        desiredCol[i] = desired[i]?.[j] ?? 0;
      }
      aligned.push(desiredCol);
      series.push({
        label: `${jointNames[j]} 指令`,
        stroke: color,
        width: 1.5,
        dash: [6, 4],
      });
    }
  }

  return { aligned, series };
}

function resolvePlotSeries(
  data: ChartSeriesInput | null,
  liveChart: boolean,
  metric: ChartMetric,
  jointNames: string[] | undefined,
  selectedJoints: ReadonlySet<number>,
): ChartSeriesInput | null {
  if (liveChart) {
    const dict = chartLiveBuffer.dict;
    if (dict && dict.time.length > 0) {
      const bundle = bundleFromRecorderDict(dict, jointNames);
      return pickActiveSeries(bundle[metric], metric, jointNames, selectedJoints);
    }
  }
  if (data && data.times.length > 0) {
    return data;
  }
  const buf = chartLiveBuffer.dict;
  if (buf && buf.time.length > 0) {
    const bundle = bundleFromRecorderDict(buf, jointNames);
    return pickActiveSeries(bundle[metric], metric, jointNames, selectedJoints);
  }
  return data;
}

interface UPlotPaneProps {
  data: ChartSeriesInput | null;
  minHeight?: number;
  /** 滑动时间窗口（秒）；有值时 X 轴锁定最近 N 秒 */
  windowSec?: number;
  simRunning?: boolean;
  liveChart?: boolean;
  metric?: ChartMetric;
  jointNames?: string[];
  selectedJoints?: ReadonlySet<number>;
}

const PROGRAMMATIC_SCALE_MS = 100;

function UPlotPane({
  data,
  minHeight = PLOT_MIN_HEIGHT,
  windowSec,
  simRunning = false,
  liveChart = false,
  metric = 'position',
  jointNames,
  selectedJoints = new Set<number>(),
}: UPlotPaneProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const plotRef = useRef<uPlot | null>(null);
  const uPlotCtorRef = useRef<typeof uPlot | null>(null);
  const seriesKeyRef = useRef('');
  const dataFingerprintRef = useRef('');
  const liveRevisionRef = useRef(-1);
  const rafRef = useRef<number | null>(null);
  const followLatestRef = useRef(true);
  const jointNamesRef = useRef(jointNames);
  const selectedJointsRef = useRef(selectedJoints);
  jointNamesRef.current = jointNames;
  selectedJointsRef.current = selectedJoints;
  const programmaticUntilRef = useRef(0);
  const prevSimRunningRef = useRef(false);
  const interactionCtxRef = useRef({
    onUserViewChange: () => {
      followLatestRef.current = false;
    },
    onRestoreFollow: (_u: uPlot) => {},
    isProgrammatic: () => performance.now() < programmaticUntilRef.current,
  });
  const [uPlotReady, setUPlotReady] = useState(false);

  const plotData = resolvePlotSeries(data, liveChart, metric, jointNames, selectedJoints);
  const hasData = Boolean(plotData && plotData.times.length > 0);

  const markProgrammaticScale = useCallback(() => {
    programmaticUntilRef.current = performance.now() + PROGRAMMATIC_SCALE_MS;
  }, []);

  const applyFollowWindow = useCallback(
    (plot: uPlot, times: number[]) => {
      if (times.length === 0) return;
      const tMax = times[times.length - 1]!;
      const span =
        windowSec != null && windowSec > 0
          ? windowSec
          : Math.max(0.001, tMax - (times[0] ?? 0));
      const [xMin, xMax] = xRangePad(Math.max(0, tMax - span), tMax);
      markProgrammaticScale();
      plot.setScale('x', { min: xMin, max: xMax });
    },
    [markProgrammaticScale, windowSec],
  );

  interactionCtxRef.current.onRestoreFollow = (u: uPlot) => {
    followLatestRef.current = true;
    const times = u.data[0] as number[];
    applyFollowWindow(u, times);
  };

  useEffect(() => {
    if (simRunning && !prevSimRunningRef.current) {
      followLatestRef.current = true;
    }
    prevSimRunningRef.current = simRunning;
  }, [simRunning]);

  useEffect(() => {
    let cancelled = false;
    void import('uplot').then((mod) => {
      if (cancelled) return;
      uPlotCtorRef.current = mod.default;
      setUPlotReady(true);
    });
    return () => {
      cancelled = true;
    };
  }, []);

  useEffect(() => {
    return () => {
      if (rafRef.current !== null) {
        cancelAnimationFrame(rafRef.current);
      }
      plotRef.current?.destroy();
      plotRef.current = null;
    };
  }, []);

  const measure = useCallback(() => {
    const el = containerRef.current;
    if (!el) return { width: 320, height: minHeight };
    const width = Math.max(el.clientWidth, 320);
    const height = Math.max(el.clientHeight, minHeight);
    return { width, height };
  }, [minHeight]);

  const ensurePlotMounted = useCallback(
    (series: ChartSeriesInput): uPlot | null => {
      const el = containerRef.current;
      const uPlotCtor = uPlotCtorRef.current;
      if (!el || !uPlotCtor) return null;

      const { aligned, series: uSeries } = buildUPlotPayload(series);
      const { width, height } = measure();
      const seriesKey = `${uSeries.length}:${series.jointNames.join(',')}`;
      const seriesChanged = seriesKey !== seriesKeyRef.current;
      if (seriesChanged) {
        followLatestRef.current = true;
        seriesKeyRef.current = seriesKey;
        dataFingerprintRef.current = '';
        liveRevisionRef.current = -1;
      }

      const opts: uPlot.Options = {
        width,
        height,
        series: uSeries,
        padding: [12, 12, 8, 6],
        scales: {
          x: { time: false, auto: true },
          y: {
            auto: true,
            range: (_u, dataMin, dataMax) => yRangePad(dataMin, dataMax),
          },
        },
        axes: [
          {
            stroke: '#6b6b88',
            grid: { show: true, stroke: '#242436' },
            size: 28,
            gap: 4,
            values: (_u, splits) => splits.map((v) => v.toFixed(2)),
          },
          {
            stroke: '#6b6b88',
            grid: { show: true, stroke: '#242436' },
            size: 58,
            gap: 6,
            values: (_u, splits) => splits.map((v) => (Math.abs(v) >= 100 ? v.toExponential(1) : v.toFixed(3))),
          },
        ],
        legend: { show: true },
        cursor: {
          drag: { x: true, y: true, setScale: true },
        },
        plugins: createInteractionPlugins(() => interactionCtxRef.current),
      };

      if (!plotRef.current || seriesChanged) {
        plotRef.current?.destroy();
        plotRef.current = new uPlotCtor(opts, aligned, el);
        dataFingerprintRef.current = chartDataFingerprint(series, metric);
        applyFollowWindow(plotRef.current, series.times);
      }
      return plotRef.current;
    },
    [applyFollowWindow, measure, metric],
  );

  const showPlotShell = hasData || liveChart;

  useEffect(() => {
    if (!uPlotReady || !plotData || !hasData) {
      if (!liveChart && !hasData) {
        plotRef.current?.destroy();
        plotRef.current = null;
        seriesKeyRef.current = '';
        dataFingerprintRef.current = '';
        liveRevisionRef.current = -1;
      }
      return;
    }
    ensurePlotMounted(plotData);
  }, [ensurePlotMounted, hasData, liveChart, plotData, uPlotReady]);

  useEffect(() => {
    if (liveChart) return;
    const plot = plotRef.current;
    if (!plot || !hasData || !plotData) return;

    const fp = chartDataFingerprint(plotData, metric);
    if (fp === dataFingerprintRef.current) return;

    if (rafRef.current !== null) {
      cancelAnimationFrame(rafRef.current);
    }

    rafRef.current = requestAnimationFrame(() => {
      rafRef.current = null;
      const live = plotRef.current;
      if (!live || !plotData) return;
      const { aligned } = buildUPlotPayload(plotData);
      live.setData(aligned, false);
      dataFingerprintRef.current = fp;
      if (followLatestRef.current) {
        applyFollowWindow(live, plotData.times);
      }
    });

    return () => {
      if (rafRef.current !== null) {
        cancelAnimationFrame(rafRef.current);
        rafRef.current = null;
      }
    };
  }, [applyFollowWindow, hasData, liveChart, metric, plotData]);

  useEffect(() => {
    if (!liveChart) return;

    let liveRaf = 0;
    const tick = () => {
      const buf = chartLiveBuffer;
      if (buf.revision !== liveRevisionRef.current && buf.dict && buf.dict.time.length > 0) {
        liveRevisionRef.current = buf.revision;
        const series = resolvePlotSeries(
          data,
          true,
          metric,
          jointNamesRef.current,
          selectedJointsRef.current,
        );
        if (series && series.times.length > 0) {
          const plot = plotRef.current ?? ensurePlotMounted(series);
          if (plot) {
            const fp = chartDataFingerprint(series, metric);
            if (fp !== dataFingerprintRef.current) {
              const { aligned } = buildUPlotPayload(series);
              plot.setData(aligned, false);
              dataFingerprintRef.current = fp;
              if (followLatestRef.current) {
                applyFollowWindow(plot, series.times);
              }
            }
          }
        }
      }
      liveRaf = requestAnimationFrame(tick);
    };
    liveRaf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(liveRaf);
  }, [applyFollowWindow, data, ensurePlotMounted, liveChart, metric]);

  useEffect(() => {
    if (liveChart) {
      liveRevisionRef.current = -1;
    }
  }, [liveChart]);

  useEffect(() => {
    const el = containerRef.current;
    if (!el || !showPlotShell) return;

    const ro = new ResizeObserver(() => {
      if (!plotRef.current || !containerRef.current) return;
      const { width, height } = measure();
      plotRef.current.setSize({ width, height });
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, [measure, showPlotShell]);

  if (!showPlotShell) {
    return <div className="sim-charts-empty">暂无仿真数据 — 运行仿真后曲线将实时更新</div>;
  }

  return <div ref={containerRef} className="sim-charts-plot" />;
}

export const SimCharts = memo(function SimCharts({
  recorderDict,
  series,
  jointNames,
  height,
  className,
  onExportClick,
  exportDisabled = false,
  exportTitle = '导出 CSV',
  recorderWindowSec,
  onRecorderWindowChange,
}: SimChartsProps) {
  const [tab, setTab] = useState<ChartMetric>('position');
  const [selectedJoints, setSelectedJoints] = useState<Set<number>>(new Set([0, 1, 2]));
  const simRunning = useSessionStore((s) => s.simStatus === 'running');

  const bundle = useMemo((): ChartSeriesBundle => {
    if (series) {
      const base = recorderDict
        ? bundleFromRecorderDict(recorderDict, jointNames)
        : bundleFromRecorderDict({ time: [], qpos: [], qvel: [], tau: [], ee_pos: [], ee_quat: [] }, jointNames);
      return {
        position: series.position ?? base.position,
        velocity: series.velocity ?? base.velocity,
        torque: series.torque ?? base.torque,
        ee: series.ee ?? base.ee,
      };
    }
    if (recorderDict) {
      return bundleFromRecorderDict(recorderDict, jointNames);
    }
    return bundleFromRecorderDict(
      { time: [], qpos: [], qvel: [], tau: [], ee_pos: [], ee_quat: [] },
      jointNames,
    );
  }, [recorderDict, series, jointNames]);

  useEffect(() => {
    if (!jointNames?.length) return;
    setSelectedJoints((prev) => {
      const valid = [...prev].filter((i) => i < jointNames.length);
      if (valid.length > 0) return new Set(valid);
      return new Set(jointNames.slice(0, Math.min(3, jointNames.length)).map((_, i) => i));
    });
  }, [jointNames]);

  const activeData = useMemo(() => {
    const raw = bundle[tab];
    return pickActiveSeries(raw, tab, jointNames, selectedJoints);
  }, [bundle, tab, jointNames, selectedJoints]);

  const toggleJoint = (index: number) => {
    setSelectedJoints((prev) => {
      const next = new Set(prev);
      if (next.has(index)) next.delete(index);
      else next.add(index);
      if (next.size === 0) next.add(index);
      return next;
    });
  };

  const plotMinHeight = height ?? PLOT_MIN_HEIGHT;
  const showJointPicks = tab !== 'ee' && jointNames && jointNames.length > 0;
  const showEeLabels = tab === 'ee';
  const showFilterRow =
    Boolean(onExportClick) ||
    Boolean(onRecorderWindowChange) ||
    showJointPicks ||
    showEeLabels;

  return (
    <div className={['sim-charts', className].filter(Boolean).join(' ')}>
      <div className="sim-charts-tabs" role="tablist" aria-label="曲线类型">
        {(['position', 'velocity', 'torque', 'ee'] as const).map((metric) => (
          <button
            key={metric}
            type="button"
            role="tab"
            aria-selected={tab === metric}
            className={['sim-charts-tab', tab === metric ? 'active' : ''].filter(Boolean).join(' ')}
            onClick={() => setTab(metric)}
          >
            {chartMetricLabel(metric)}
          </button>
        ))}
      </div>
      {showFilterRow && (
        <div className="sim-charts-joint-picks" role="group" aria-label="曲线筛选">
          {onExportClick && (
            <>
              <button
                type="button"
                className="sim-charts-export-btn"
                onClick={onExportClick}
                disabled={exportDisabled}
                title={exportTitle}
              >
                导出
              </button>
              {(onRecorderWindowChange || showJointPicks || showEeLabels) && (
                <span className="sim-charts-joint-divider" aria-hidden="true" />
              )}
            </>
          )}
          {onRecorderWindowChange && recorderWindowSec != null && (
            <label className="sim-charts-window-input">
              保留窗口 (s)
              <input
                type="number"
                min={1}
                max={300}
                step={1}
                value={recorderWindowSec}
                onChange={(e) => onRecorderWindowChange(Number(e.target.value))}
              />
            </label>
          )}
          {onExportClick && onRecorderWindowChange && (showJointPicks || showEeLabels) && (
            <span className="sim-charts-joint-divider" aria-hidden="true" />
          )}
          {showJointPicks &&
            jointNames!.map((name, i) => (
              <label
                key={name}
                className={['joint-pick-chip', selectedJoints.has(i) ? 'active' : ''].filter(Boolean).join(' ')}
              >
                <input
                  type="checkbox"
                  checked={selectedJoints.has(i)}
                  onChange={() => toggleJoint(i)}
                />
                {name.replace(/_joint$/, '').slice(-16)}
              </label>
            ))}
          {showEeLabels &&
            ['ee_x', 'ee_y', 'ee_z'].map((label) => (
              <span key={label} className="joint-pick-chip active ee-chip">
                {label}
              </span>
            ))}
        </div>
      )}
      <div className="sim-charts-legend-hint">
        {tab === 'ee'
          ? '末端位置 ee_x / ee_y / ee_z · 滚轮缩放 · Shift+拖移平移 · 框选缩放 · 双击恢复跟随'
          : '实线 = 实际 · 虚线 = 指令 · 滚轮缩放 · Shift+拖移平移 · 框选缩放 · 双击恢复跟随'}
      </div>
      <div className="sim-charts-plot-wrap">
        <UPlotPane
          data={activeData}
          minHeight={plotMinHeight}
          windowSec={recorderWindowSec}
          simRunning={simRunning}
          liveChart={simRunning}
          metric={tab}
          jointNames={jointNames}
          selectedJoints={selectedJoints}
        />
      </div>
    </div>
  );
});
