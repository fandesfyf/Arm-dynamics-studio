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
  /** 左右侧栏当前展开的面板 id；空字符串表示全部折叠 */
  expandedOnSide: Record<'left' | 'right', string>;
  leftWidth: number;
  rightWidth: number;
  bottomHeight: number;

  isPanelOpen: (id: string) => boolean;
  isPanelCollapsed: (id: string) => boolean;
  isPanelExpandedOnSide: (side: 'left' | 'right', id: string) => boolean;
  getExpandedOnSide: (side: 'left' | 'right') => string;
  setExpandedOnSide: (side: 'left' | 'right', id: string) => void;
  setPanelOpen: (id: string, open: boolean) => void;
  togglePanel: (id: string) => void;
  setPanelCollapsed: (id: string, collapsed: boolean) => void;
  togglePanelCollapsed: (id: string) => void;
  focusPanelOnSide: (side: 'left' | 'right', id: string) => void;
  openAllOnSide: (side: DockSide) => void;
  closeAllOnSide: (side: DockSide) => void;
  setLeftWidth: (width: number) => void;
  setRightWidth: (width: number) => void;
  setBottomHeight: (height: number) => void;
  persistDimensions: () => void;
}

function defaultExpandedOnSide(): Record<'left' | 'right', string> {
  const left = PANEL_REGISTRY.filter((p) => p.side === 'left').sort((a, b) => a.order - b.order)[0]?.id ?? 'model';
  const right = PANEL_REGISTRY.filter((p) => p.side === 'right').sort((a, b) => a.order - b.order)[0]?.id ?? 'control';
  return { left, right };
}

function panelSide(id: string): DockSide | undefined {
  return PANEL_REGISTRY.find((p) => p.id === id)?.side;
}

function initialPanels(): Record<string, PanelState> {
  const out: Record<string, PanelState> = {};
  for (const p of PANEL_REGISTRY) {
    const open = p.side === 'bottom' ? p.defaultOpen : true;
    out[p.id] = { open, collapsed: false };
  }
  return out;
}

const storedDimensions = loadStoredDimensions();

export const useUiStore = create<UiState>((set, get) => ({
  panels: initialPanels(),
  expandedOnSide: defaultExpandedOnSide(),
  leftWidth: storedDimensions.leftWidth ?? 300,
  rightWidth: storedDimensions.rightWidth ?? 300,
  bottomHeight: storedDimensions.bottomHeight ?? 300,

  isPanelOpen: (id) => {
    const side = panelSide(id);
    if (side === 'left' || side === 'right') return true;
    return get().panels[id]?.open ?? false;
  },

  isPanelCollapsed: (id) => {
    const side = panelSide(id);
    if (side === 'left' || side === 'right') {
      return get().expandedOnSide[side] !== id;
    }
    return get().panels[id]?.collapsed ?? false;
  },

  isPanelExpandedOnSide: (side, id) => get().expandedOnSide[side] === id,

  getExpandedOnSide: (side) => get().expandedOnSide[side],

  setExpandedOnSide: (side, id) =>
    set((s) => ({
      expandedOnSide: { ...s.expandedOnSide, [side]: id },
    })),

  focusPanelOnSide: (side, id) => {
    get().setExpandedOnSide(side, id);
    set((s) => ({
      panels: { ...s.panels, [id]: { ...s.panels[id], open: true } },
    }));
  },

  setPanelOpen: (id, open) => {
    const side = panelSide(id);
    if (side === 'left' || side === 'right') {
      if (open) get().setExpandedOnSide(side, id);
      else if (get().expandedOnSide[side] === id) get().setExpandedOnSide(side, '');
      return;
    }
    set((s) => ({
      panels: { ...s.panels, [id]: { ...s.panels[id], open } },
    }));
  },

  togglePanel: (id) => {
    const side = panelSide(id);
    if (side === 'left' || side === 'right') {
      const expanded = get().expandedOnSide[side];
      get().setExpandedOnSide(side, expanded === id ? '' : id);
      return;
    }
    const cur = get().panels[id];
    if (!cur) return;
    get().setPanelOpen(id, !cur.open);
  },

  setPanelCollapsed: (id, collapsed) => {
    const side = panelSide(id);
    if (side === 'left' || side === 'right') {
      if (collapsed) {
        if (get().expandedOnSide[side] === id) get().setExpandedOnSide(side, '');
      } else {
        get().setExpandedOnSide(side, id);
      }
      return;
    }
    set((s) => ({
      panels: { ...s.panels, [id]: { ...s.panels[id], collapsed } },
    }));
  },

  togglePanelCollapsed: (id) => {
    const side = panelSide(id);
    if (side === 'left' || side === 'right') {
      get().togglePanel(id);
      return;
    }
    const cur = get().panels[id];
    if (!cur) return;
    get().setPanelCollapsed(id, !cur.collapsed);
  },

  openAllOnSide: (side) => {
    if (side === 'left' || side === 'right') {
      const first = PANEL_REGISTRY.filter((x) => x.side === side).sort((a, b) => a.order - b.order)[0];
      if (first) get().setExpandedOnSide(side, first.id);
      return;
    }
    set((s) => {
      const panels = { ...s.panels };
      for (const p of PANEL_REGISTRY.filter((x) => x.side === side)) {
        panels[p.id] = { ...panels[p.id], open: true, collapsed: false };
      }
      return { panels };
    });
  },

  closeAllOnSide: (side) => {
    if (side === 'left' || side === 'right') {
      set((s) => ({
        expandedOnSide: { ...s.expandedOnSide, [side]: '' },
      }));
      return;
    }
    set((s) => {
      const panels = { ...s.panels };
      for (const p of PANEL_REGISTRY.filter((x) => x.side === side)) {
        panels[p.id] = { ...panels[p.id], open: false };
      }
      return { panels };
    });
  },

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
