import { useCallback, useRef, type ReactNode } from 'react';
import { PANEL_REGISTRY, type DockSide, useUiStore } from '../../stores/ui-store';
import { DockPanel } from './DockPanel';
import { DockResizeHandle } from './DockResizeHandle';

export interface DockSidebarTransportProps {
  running: boolean;
  disabled: boolean;
  title: string;
  onTransport: () => void;
  pauseDisabled?: boolean;
  pauseLabel?: string;
  isPaused?: boolean;
  onPause?: () => void;
}

interface DockSidebarProps {
  side: DockSide;
  panels: Record<string, ReactNode>;
  transport?: DockSidebarTransportProps;
}

export function DockSidebar({ side, panels, transport }: DockSidebarProps) {
  const expandedId = useUiStore((s) => s.getExpandedOnSide(side as 'left' | 'right'));
  const togglePanel = useUiStore((s) => s.togglePanel);
  const width = useUiStore((s) => (side === 'left' ? s.leftWidth : s.rightWidth));
  const setLeftWidth = useUiStore((s) => s.setLeftWidth);
  const setRightWidth = useUiStore((s) => s.setRightWidth);
  const persistDimensions = useUiStore((s) => s.persistDimensions);

  const isDraggingRef = useRef(false);
  const startWidthRef = useRef(width);

  const setWidth = side === 'left' ? setLeftWidth : setRightWidth;

  const handleDragStart = useCallback(() => {
    isDraggingRef.current = true;
    startWidthRef.current = width;
  }, [width]);

  const handleDrag = useCallback(
    (delta: number) => {
      setWidth(startWidthRef.current + delta);
    },
    [setWidth],
  );

  const handleDragEnd = useCallback(() => {
    isDraggingRef.current = false;
    persistDimensions();
  }, [persistDimensions]);

  if (side === 'bottom') return null;

  const defs = PANEL_REGISTRY.filter((p) => p.side === side).sort((a, b) => a.order - b.order);

  return (
    <aside className={`dock-sidebar dock-sidebar--${side}`} style={{ width }}>
      {side === 'left' && transport && (
        <div className="dock-sidebar-transport">
          <button
            type="button"
            className={`header-transport-btn dock-transport-btn${
              transport.running ? ' header-transport-btn--stop' : ''
            }${transport.disabled ? ' header-transport-btn--disabled' : ''}`}
            disabled={transport.disabled}
            onClick={transport.onTransport}
            title={transport.title}
            aria-label={transport.title}
          >
            {transport.running ? '⏹ 停止仿真' : '▶ 开始仿真'}
          </button>
          {transport.onPause && transport.running && (
            <button
              type="button"
              className={`header-transport-btn dock-pause-btn${
                transport.pauseDisabled ? ' header-transport-btn--disabled' : ''
              }${transport.isPaused ? ' dock-pause-btn--paused' : ''}`}
              disabled={transport.pauseDisabled}
              onClick={transport.onPause}
              title={transport.pauseLabel}
              aria-label={transport.pauseLabel}
            >
              {transport.pauseLabel}
            </button>
          )}
        </div>
      )}
      <div className="dock-sidebar-stack">
        {defs.map((def) => (
          <DockPanel
            key={def.id}
            id={def.id}
            title={def.title}
            expanded={expandedId === def.id}
            onActivate={() => togglePanel(def.id)}
          >
            {panels[def.id]}
          </DockPanel>
        ))}
      </div>
      <DockResizeHandle
        axis="vertical"
        edge={side === 'left' ? 'end' : 'start'}
        onDragStart={handleDragStart}
        onDrag={handleDrag}
        onDragEnd={handleDragEnd}
      />
    </aside>
  );
}

interface DockBottomProps {
  panelId: string;
  title: string;
  children: ReactNode;
}

export function DockBottom({ panelId, title, children }: DockBottomProps) {
  const open = useUiStore((s) => s.isPanelOpen(panelId));
  const setPanelOpen = useUiStore((s) => s.setPanelOpen);
  const height = useUiStore((s) => s.bottomHeight);
  const setBottomHeight = useUiStore((s) => s.setBottomHeight);
  const persistDimensions = useUiStore((s) => s.persistDimensions);
  const collapsed = useUiStore((s) => s.isPanelCollapsed(panelId));
  const toggleCollapsed = useUiStore((s) => s.togglePanelCollapsed);

  const isDraggingRef = useRef(false);
  const startHeightRef = useRef(height);

  const handleDragStart = useCallback(() => {
    isDraggingRef.current = true;
    startHeightRef.current = height;
  }, [height]);

  const handleDrag = useCallback((delta: number) => {
    setBottomHeight(startHeightRef.current + delta);
  }, [setBottomHeight]);

  const handleDragEnd = useCallback(() => {
    isDraggingRef.current = false;
    persistDimensions();
  }, [persistDimensions]);

  if (!open) return null;

  return (
    <footer
      className={`dock-bottom${collapsed ? ' dock-bottom--collapsed' : ''}`}
      style={{ height: collapsed ? 40 : height }}
    >
      {!collapsed && (
        <DockResizeHandle
          axis="horizontal"
          edge="start"
          onDragStart={handleDragStart}
          onDrag={handleDrag}
          onDragEnd={handleDragEnd}
        />
      )}
      <DockPanel
        id={panelId}
        title={title}
        expanded={!collapsed}
        onActivate={() => toggleCollapsed(panelId)}
        onClose={() => setPanelOpen(panelId, false)}
      >
        {children}
      </DockPanel>
    </footer>
  );
}
