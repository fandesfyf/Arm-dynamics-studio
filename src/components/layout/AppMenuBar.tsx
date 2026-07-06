import { PANEL_REGISTRY, useUiStore } from '../../stores/ui-store';

export function AppMenuBar() {
  const expandedOnSide = useUiStore((s) => s.expandedOnSide);
  const focusPanelOnSide = useUiStore((s) => s.focusPanelOnSide);
  const panels = useUiStore((s) => s.panels);
  const togglePanel = useUiStore((s) => s.togglePanel);
  const openAllOnSide = useUiStore((s) => s.openAllOnSide);
  const closeAllOnSide = useUiStore((s) => s.closeAllOnSide);

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
                type="radio"
                name="dock-left-focus"
                checked={expandedOnSide.left === p.id}
                onChange={() => focusPanelOnSide('left', p.id)}
              />
              {p.title}
            </label>
          ))}
          <button type="button" className="menu-action" onClick={() => openAllOnSide('left')}>
            展开左侧首个面板
          </button>
          <button type="button" className="menu-action" onClick={() => closeAllOnSide('left')}>
            折叠左侧全部
          </button>
          <div className="menu-divider" />
          <div className="menu-section-label">右侧面板</div>
          {PANEL_REGISTRY.filter((p) => p.side === 'right').map((p) => (
            <label key={p.id} className="menu-check-item">
              <input
                type="radio"
                name="dock-right-focus"
                checked={expandedOnSide.right === p.id}
                onChange={() => focusPanelOnSide('right', p.id)}
              />
              {p.title}
            </label>
          ))}
          <button type="button" className="menu-action" onClick={() => openAllOnSide('right')}>
            展开右侧首个面板
          </button>
          <button type="button" className="menu-action" onClick={() => closeAllOnSide('right')}>
            折叠右侧全部
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
            展开底部曲线
          </button>
        </div>
      </div>
    </nav>
  );
}
