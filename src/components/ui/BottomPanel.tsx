import { useState, type ReactNode } from 'react';

interface BottomPanelProps {
  children: ReactNode;
  defaultHeight?: number;
}

export function BottomPanel({ children, defaultHeight = 200 }: BottomPanelProps) {
  const [collapsed, setCollapsed] = useState(false);
  const [height, setHeight] = useState(defaultHeight);

  return (
    <footer
      className={`app-bottom-panel${collapsed ? ' is-collapsed' : ''}`}
      style={collapsed ? undefined : { height }}
    >
      <div className="bottom-panel-toolbar">
        <span className="bottom-panel-title">实时曲线</span>
        <div className="bottom-panel-controls">
          {!collapsed && (
            <button
              type="button"
              className="btn-icon"
              onClick={() => setHeight((h) => Math.min(h + 60, 480))}
              title="增高"
              aria-label="增高图表区"
            >
              ▲
            </button>
          )}
          {!collapsed && (
            <button
              type="button"
              className="btn-icon"
              onClick={() => setHeight((h) => Math.max(h - 60, 120))}
              title="降低"
              aria-label="降低图表区"
            >
              ▼
            </button>
          )}
          <button
            type="button"
            className="btn-icon"
            onClick={() => setCollapsed((c) => !c)}
            title={collapsed ? '展开' : '折叠'}
            aria-label={collapsed ? '展开图表区' : '折叠图表区'}
          >
            {collapsed ? '▲' : '▼'}
          </button>
        </div>
      </div>
      {!collapsed && <div className="bottom-panel-content">{children}</div>}
    </footer>
  );
}
