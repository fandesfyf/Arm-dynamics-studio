import type { RecorderDict } from './data-recorder';

/** 仿真运行时曲线直绘缓冲，绕过 React store 以降低卡顿 */
export interface ChartLiveBuffer {
  revision: number;
  dict: RecorderDict | null;
}

export const chartLiveBuffer: ChartLiveBuffer = {
  revision: 0,
  dict: null,
};

export function publishChartLiveDict(dict: RecorderDict): void {
  chartLiveBuffer.dict = dict;
  chartLiveBuffer.revision += 1;
}

export function clearChartLiveBuffer(): void {
  chartLiveBuffer.dict = null;
  chartLiveBuffer.revision += 1;
}
