import { useState, useCallback, type ReactNode } from 'react';
import { FloatingPanel } from './FloatingPanel';
import './FloatingPanel.css';

interface PanelConfig {
  id: string;
  title: string;
  icon: string;
  accentColor: string;
  content: ReactNode;
  defaultOpen?: boolean;
  defaultMinimized?: boolean;
  position?: { x: number; y: number };
}

interface FloatingPanelManagerProps {
  panels: PanelConfig[];
}

type PanelState = 'expanded' | 'collapsed' | 'minimized';

interface PanelStateRecord {
  state: PanelState;
  position: { x: number; y: number };
}

export function FloatingPanelManager({ panels }: FloatingPanelManagerProps) {
  const [panelStates, setPanelStates] = useState<Record<string, PanelStateRecord>>(() =>
    panels.reduce(
      (acc, panel) => ({
        ...acc,
        [panel.id]: {
          state: panel.defaultMinimized ? 'minimized' : panel.defaultOpen ? 'expanded' : 'collapsed',
          position: panel.position ?? { x: 0, y: 0 },
        },
      }),
      {} as Record<string, PanelStateRecord>,
    ),
  );

  const updatePanelState = useCallback((panelId: string, newState: PanelState) => {
    setPanelStates((prev) => ({
      ...prev,
      [panelId]: { ...prev[panelId]!, state: newState },
    }));
  }, []);

  const updatePanelPosition = useCallback((panelId: string, position: { x: number; y: number }) => {
    setPanelStates((prev) => ({
      ...prev,
      [panelId]: { ...prev[panelId]!, position },
    }));
  }, []);

  const toggleAllPanels = useCallback((state: PanelState) => {
    setPanelStates((prev) => {
      const next = { ...prev };
      for (const id of Object.keys(next)) {
        next[id] = { ...next[id]!, state };
      }
      return next;
    });
  }, []);

  const autoSortPanels = useCallback(() => {
    const panelWidth = 320;
    const panelHeight = 500;
    const gap = 20;
    const startX = 20;
    const startY = 72;
    const maxColumns = Math.max(1, Math.floor((window.innerWidth - startX) / (panelWidth + gap)));

    setPanelStates((prev) => {
      const next = { ...prev };
      let x = startX;
      let y = startY;
      let col = 0;
      for (const panel of panels) {
        if (next[panel.id]?.state === 'collapsed') continue;
        next[panel.id] = { ...next[panel.id]!, position: { x, y } };
        col++;
        if (col >= maxColumns) {
          col = 0;
          x = startX;
          y += panelHeight + gap;
        } else {
          x += panelWidth + gap;
        }
      }
      return next;
    });
  }, [panels]);

  const minimized = panels.filter((p) => panelStates[p.id]?.state === 'minimized');
  const visible = panels.filter(
    (p) => panelStates[p.id]?.state !== 'minimized' && panelStates[p.id]?.state !== 'collapsed',
  );

  return (
    <>
      {visible.map((panel) => {
        const panelState = panelStates[panel.id];
        if (!panelState) return null;
        return (
          <FloatingPanel
            key={panel.id}
            title={panel.title}
            icon={panel.icon}
            accentColor={panel.accentColor}
            defaultOpen={panelState.state === 'expanded'}
            position={panelState.position}
            onStateChange={(s) => updatePanelState(panel.id, s)}
            onPositionChange={(pos) => updatePanelPosition(panel.id, pos)}
          >
            {panel.content}
          </FloatingPanel>
        );
      })}

      {minimized.length > 0 && (
        <div className="panel-dock">
          {minimized.map((panel) => (
            <div
              key={panel.id}
              className="docked-panel"
              style={{ borderLeftColor: panel.accentColor }}
              onClick={() => updatePanelState(panel.id, 'expanded')}
            >
              <div className="docked-title">
                <span className="docked-icon">{panel.icon}</span>
                <span>{panel.title}</span>
              </div>
              <div className="docked-controls">
                <button
                  type="button"
                  className="docked-btn"
                  onClick={(e) => {
                    e.stopPropagation();
                    updatePanelState(panel.id, 'expanded');
                  }}
                  title="展开"
                >
                  ⤢
                </button>
                <button
                  type="button"
                  className="docked-btn"
                  onClick={(e) => {
                    e.stopPropagation();
                    updatePanelState(panel.id, 'collapsed');
                  }}
                  title="关闭"
                >
                  ×
                </button>
              </div>
            </div>
          ))}
        </div>
      )}

      <div className="panel-launcher">
        <div className="panel-header">
          <div className="panel-title">
            <span className="panel-icon">⚙️</span>
            <span className="panel-title-text">面板</span>
          </div>
        </div>
        <div className="panel-content-inner panel-launcher-body">
          <button
            type="button"
            className="btn btn-ghost btn-block"
            onClick={() => {
              toggleAllPanels('expanded');
              autoSortPanels();
            }}
          >
            全部展开
          </button>
          <button type="button" className="btn btn-ghost btn-block" onClick={() => toggleAllPanels('minimized')}>
            全部最小化
          </button>
          <button type="button" className="btn btn-ghost btn-block" onClick={autoSortPanels}>
            自动排列
          </button>
        </div>
      </div>
    </>
  );
}
