import { create } from 'zustand';

export type DockSide = 'left' | 'right' | 'bottom';

export interface PanelDef {
  id: string;
  title: string;
  side: DockSide;
  order: number;
  defaultOpen: boolean;
}

export const PANEL_REGISTRY: PanelDef[] = [
  { id: 'model', title: '模型', side: 'left', order: 0, defaultOpen: true },
  { id: 'simulation', title: '仿真', side: 'left', order: 1, defaultOpen: true },
  { id: 'control', title: '控制', side: 'right', order: 0, defaultOpen: true },
  { id: 'payload', title: '负载', side: 'right', order: 1, defaultOpen: true },
  { id: 'visualization', title: '可视化', side: 'right', order: 2, defaultOpen: true },
  { id: 'charts', title: '曲线', side: 'bottom', order: 0, defaultOpen: true },
];

interface PanelState {
  open: boolean;
  collapsed: boolean;
}

export const SIDEBAR_WIDTH_MIN = 220;
export const SIDEBAR_WIDTH_MAX = 480;
export const BOTTOM_HEIGHT_MIN = 120;
export const BOTTOM_HEIGHT_MAX = 600;

const STORAGE_KEY = 'arm-dynamics-sim-ui-dimensions';

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function loadStoredDimensions(): Partial<Pick<UiState, 'leftWidth' | 'rightWidth' | 'bottomHeight'>> {
  if (typeof localStorage === 'undefined') return {};
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return {};
    const parsed = JSON.parse(raw) as Record<string, unknown>;
    const out: Partial<Pick<UiState, 'leftWidth' | 'rightWidth' | 'bottomHeight'>> = {};
    if (typeof parsed.leftWidth === 'number') {
      out.leftWidth = clamp(parsed.leftWidth, SIDEBAR_WIDTH_MIN, SIDEBAR_WIDTH_MAX);
    }
    if (typeof parsed.rightWidth === 'number') {
      out.rightWidth = clamp(parsed.rightWidth, SIDEBAR_WIDTH_MIN, SIDEBAR_WIDTH_MAX);
    }
    if (typeof parsed.bottomHeight === 'number') {
      out.bottomHeight = clamp(parsed.bottomHeight, BOTTOM_HEIGHT_MIN, BOTTOM_HEIGHT_MAX);
    }
    return out;
  } catch {
    return {};
  }
}

function persistDimensions(state: Pick<UiState, 'leftWidth' | 'rightWidth' | 'bottomHeight'>) {
  if (typeof localStorage === 'undefined') return;
  try {
    localStorage.setItem(
      STORAGE_KEY,
      JSON.stringify({
        leftWidth: state.leftWidth,
        rightWidth: state.rightWidth,
        bottomHeight: state.bottomHeight,
      }),
    );
  } catch {
    // ignore quota / private mode errors
  }
}

interface UiState {
  panels: Record<string, PanelState>;
  leftWidth: number;
  rightWidth: number;
  bottomHeight: number;

  isPanelOpen: (id: string) => boolean;
  isPanelCollapsed: (id: string) => boolean;
  setPanelOpen: (id: string, open: boolean) => void;
  togglePanel: (id: string) => void;
  setPanelCollapsed: (id: string, collapsed: boolean) => void;
  togglePanelCollapsed: (id: string) => void;
  openAllOnSide: (side: DockSide) => void;
  closeAllOnSide: (side: DockSide) => void;
  setLeftWidth: (width: number) => void;
  setRightWidth: (width: number) => void;
  setBottomHeight: (height: number) => void;
  persistDimensions: () => void;
}

function initialPanels(): Record<string, PanelState> {
  const out: Record<string, PanelState> = {};
  for (const p of PANEL_REGISTRY) {
    out[p.id] = { open: p.defaultOpen, collapsed: false };
  }
  return out;
}

const storedDimensions = loadStoredDimensions();

export const useUiStore = create<UiState>((set, get) => ({
  panels: initialPanels(),
  leftWidth: storedDimensions.leftWidth ?? 300,
  rightWidth: storedDimensions.rightWidth ?? 300,
  bottomHeight: storedDimensions.bottomHeight ?? 300,

  isPanelOpen: (id) => get().panels[id]?.open ?? false,
  isPanelCollapsed: (id) => get().panels[id]?.collapsed ?? false,

  setPanelOpen: (id, open) =>
    set((s) => ({
      panels: { ...s.panels, [id]: { ...s.panels[id], open } },
    })),

  togglePanel: (id) => {
    const cur = get().panels[id];
    if (!cur) return;
    get().setPanelOpen(id, !cur.open);
  },

  setPanelCollapsed: (id, collapsed) =>
    set((s) => ({
      panels: { ...s.panels, [id]: { ...s.panels[id], collapsed } },
    })),

  togglePanelCollapsed: (id) => {
    const cur = get().panels[id];
    if (!cur) return;
    get().setPanelCollapsed(id, !cur.collapsed);
  },

  openAllOnSide: (side) =>
    set((s) => {
      const panels = { ...s.panels };
      for (const p of PANEL_REGISTRY.filter((x) => x.side === side)) {
        panels[p.id] = { ...panels[p.id], open: true, collapsed: false };
      }
      return { panels };
    }),

  closeAllOnSide: (side) =>
    set((s) => {
      const panels = { ...s.panels };
      for (const p of PANEL_REGISTRY.filter((x) => x.side === side)) {
        panels[p.id] = { ...panels[p.id], open: false };
      }
      return { panels };
    }),

  setLeftWidth: (width) =>
    set({ leftWidth: clamp(width, SIDEBAR_WIDTH_MIN, SIDEBAR_WIDTH_MAX) }),

  setRightWidth: (width) =>
    set({ rightWidth: clamp(width, SIDEBAR_WIDTH_MIN, SIDEBAR_WIDTH_MAX) }),

  setBottomHeight: (height) =>
    set({ bottomHeight: clamp(height, BOTTOM_HEIGHT_MIN, BOTTOM_HEIGHT_MAX) }),

  persistDimensions: () => {
    const { leftWidth, rightWidth, bottomHeight } = get();
    persistDimensions({ leftWidth, rightWidth, bottomHeight });
  },
}));
