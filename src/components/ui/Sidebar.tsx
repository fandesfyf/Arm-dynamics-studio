import type { ReactNode } from 'react';

interface SidebarProps {
  open: boolean;
  onClose: () => void;
  children: ReactNode;
}

export function Sidebar({ open, onClose, children }: SidebarProps) {
  return (
    <>
      {open && (
        <div
          className="sidebar-backdrop"
          onClick={onClose}
          aria-hidden
        />
      )}
      <aside className={`app-sidebar${open ? ' is-open' : ''}`}>
        <div className="sidebar-scroll">{children}</div>
      </aside>
    </>
  );
}
