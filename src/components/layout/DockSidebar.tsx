import { useCallback, useRef, type ReactNode } from 'react';
import { PANEL_REGISTRY, type DockSide, useUiStore } from '../../stores/ui-store';
import { DockPanel } from './DockPanel';
import { DockResizeHandle } from './DockResizeHandle';

interface DockSidebarProps {
  side: DockSide;
  panels: Record<string, ReactNode>;
}

export function DockSidebar({ side, panels }: DockSidebarProps) {
  const panelState = useUiStore((s) => s.panels);
  const setPanelOpen = useUiStore((s) => s.setPanelOpen);
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

  const defs = PANEL_REGISTRY.filter((p) => p.side === side && panelState[p.id]?.open)
    .sort((a, b) => a.order - b.order);

  if (defs.length === 0) return null;

  return (
    <aside className={`dock-sidebar dock-sidebar--${side}`} style={{ width }}>
      <div className="dock-sidebar-scroll">
        {defs.map((def) => (
          <DockPanel
            key={def.id}
            id={def.id}
            title={def.title}
            onClose={() => setPanelOpen(def.id, false)}
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
      <DockPanel id={panelId} title={title} onClose={() => setPanelOpen(panelId, false)}>
        {children}
      </DockPanel>
    </footer>
  );
}
