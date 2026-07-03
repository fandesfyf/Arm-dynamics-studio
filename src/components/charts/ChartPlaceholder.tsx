/**
 * 图表占位 / 导出入口 — 实际渲染 SimCharts。
 * App 或面板可直接 `import { SimCharts } from './ChartPlaceholder'`。
 */
export { SimCharts, type SimChartsProps } from './SimCharts';
export { ChartPanel, type ChartPanelProps } from './ChartPanel';
export {
  bundleFromDataRecorder,
  bundleFromRecorderDict,
  chartMetricLabel,
  fromDataRecorder,
  fromRecorderDict,
  type ChartMetric,
  type ChartSeriesBundle,
  type ChartSeriesInput,
} from './chart-types';

import { SimCharts, type SimChartsProps } from './SimCharts';

/** @deprecated 使用 SimCharts */
export function ChartPlaceholder(props: SimChartsProps) {
  return <SimCharts {...props} />;
}
