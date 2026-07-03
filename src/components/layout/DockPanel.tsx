import type { ReactNode } from 'react';
import { useUiStore } from '../../stores/ui-store';

interface DockPanelProps {
  id: string;
  title: string;
  children: ReactNode;
  onClose?: () => void;
}

export function DockPanel({ id, title, children, onClose }: DockPanelProps) {
  const collapsed = useUiStore((s) => s.isPanelCollapsed(id));
  const toggleCollapsed = useUiStore((s) => s.togglePanelCollapsed);

  return (
    <section className={`dock-panel${collapsed ? ' dock-panel--collapsed' : ''}`}>
      <header className="dock-panel-header">
        <button
          type="button"
          className="dock-panel-toggle"
          onClick={() => toggleCollapsed(id)}
          aria-expanded={!collapsed}
        >
          {collapsed ? '▸' : '▾'}
        </button>
        <h3 className="dock-panel-title">{title}</h3>
        {onClose && (
          <button type="button" className="dock-panel-close" onClick={onClose} aria-label="关闭面板">
            ×
          </button>
        )}
      </header>
      {!collapsed && <div className="dock-panel-body">{children}</div>}
    </section>
  );
}
