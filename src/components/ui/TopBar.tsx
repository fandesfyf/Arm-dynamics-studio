import { StatusBadge } from './StatusBadge';

interface TopBarProps {
  robotName?: string;
  dof?: number;
  simStatus: string;
  statusLabel: string;
  simMessage?: string;
  busy: boolean;
  sidebarOpen: boolean;
  onToggleSidebar: () => void;
  onLoadTestArm: () => void;
  onExport: () => void;
  onCancel?: () => void;
  showCancel: boolean;
}

export function TopBar({
  robotName,
  dof,
  simStatus,
  statusLabel,
  simMessage,
  busy,
  sidebarOpen,
  onToggleSidebar,
  onLoadTestArm,
  onExport,
  onCancel,
  showCancel,
}: TopBarProps) {
  return (
    <header className="app-topbar">
      <div className="app-topbar-left">
        <button
          type="button"
          className="btn-icon sidebar-toggle"
          onClick={onToggleSidebar}
          aria-label={sidebarOpen ? '收起侧栏' : '展开侧栏'}
          title={sidebarOpen ? '收起侧栏' : '展开侧栏'}
        >
          <span className="hamburger" aria-hidden />
        </button>
        <h1 className="app-brand">
          <span className="app-brand-gradient">Arm Dynamics</span>
          <span className="app-brand-sub">Sim</span>
        </h1>
      </div>

      <div className="app-topbar-center">
        {robotName && (
          <span className="model-badge">
            <span className="model-badge-label">模型</span>
            {robotName}
          </span>
        )}
        {dof != null && (
          <span className="dof-badge">{dof} DOF</span>
        )}
        <StatusBadge status={simStatus} label={statusLabel} />
        {simMessage && <span className="sim-message">{simMessage}</span>}
      </div>

      <div className="app-topbar-actions">
        {showCancel && onCancel && (
          <button type="button" className="btn btn-secondary" onClick={onCancel}>
            取消仿真
          </button>
        )}
        <button
          type="button"
          className="btn btn-ghost"
          onClick={onExport}
          disabled={busy}
          title="导出 CSV"
        >
          ↓ 导出
        </button>
        <button
          type="button"
          className="btn btn-primary"
          onClick={onLoadTestArm}
          disabled={busy}
        >
          加载 test_arm
        </button>
      </div>
    </header>
  );
}
