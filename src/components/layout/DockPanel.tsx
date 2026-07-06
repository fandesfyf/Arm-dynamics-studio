import type { ReactNode } from 'react';

interface DockPanelProps {
  id: string;
  title: string;
  children: ReactNode;
  expanded: boolean;
  onActivate: () => void;
  /** 底部面板可关闭；左右侧栏始终显示 */
  onClose?: () => void;
}

export function DockPanel({ id, title, children, expanded, onActivate, onClose }: DockPanelProps) {
  return (
    <section
      className={`dock-panel${expanded ? ' dock-panel--expanded' : ' dock-panel--collapsed'}`}
      data-panel-id={id}
    >
      <header className="dock-panel-header">
        <button
          type="button"
          className="dock-panel-toggle"
          onClick={onActivate}
          aria-expanded={expanded}
        >
          {expanded ? '▾' : '▸'}
        </button>
        <button type="button" className="dock-panel-title-btn" onClick={onActivate}>
          <h3 className="dock-panel-title">{title}</h3>
        </button>
        {onClose && (
          <button type="button" className="dock-panel-close" onClick={onClose} aria-label="关闭面板">
            ×
          </button>
        )}
      </header>
      {expanded && <div className="dock-panel-body">{children}</div>}
    </section>
  );
}
