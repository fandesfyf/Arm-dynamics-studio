import { PANEL_REGISTRY, useUiStore } from '../../stores/ui-store';

export function AppMenuBar() {
  const panels = useUiStore((s) => s.panels);
  const togglePanel = useUiStore((s) => s.togglePanel);
  const openAllOnSide = useUiStore((s) => s.openAllOnSide);

  return (
    <nav className="app-menu-bar" aria-label="主菜单">
      <div className="menu-dropdown">
        <button type="button" className="menu-trigger">
          视图
        </button>
        <div className="menu-dropdown-content">
          <div className="menu-section-label">左侧面板</div>
          {PANEL_REGISTRY.filter((p) => p.side === 'left').map((p) => (
            <label key={p.id} className="menu-check-item">
              <input
                type="checkbox"
                checked={panels[p.id]?.open ?? false}
                onChange={() => togglePanel(p.id)}
              />
              {p.title}
            </label>
          ))}
          <button type="button" className="menu-action" onClick={() => openAllOnSide('left')}>
            展开全部左侧
          </button>
          <div className="menu-divider" />
          <div className="menu-section-label">右侧面板</div>
          {PANEL_REGISTRY.filter((p) => p.side === 'right').map((p) => (
            <label key={p.id} className="menu-check-item">
              <input
                type="checkbox"
                checked={panels[p.id]?.open ?? false}
                onChange={() => togglePanel(p.id)}
              />
              {p.title}
            </label>
          ))}
          <button type="button" className="menu-action" onClick={() => openAllOnSide('right')}>
            展开全部右侧
          </button>
          <div className="menu-divider" />
          <div className="menu-section-label">底部面板</div>
          {PANEL_REGISTRY.filter((p) => p.side === 'bottom').map((p) => (
            <label key={p.id} className="menu-check-item">
              <input
                type="checkbox"
                checked={panels[p.id]?.open ?? false}
                onChange={() => togglePanel(p.id)}
              />
              {p.title}
            </label>
          ))}
          <button type="button" className="menu-action" onClick={() => openAllOnSide('bottom')}>
            展开全部底部
          </button>
        </div>
      </div>
    </nav>
  );
}
