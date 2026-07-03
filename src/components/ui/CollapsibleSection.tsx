import { useState, type ReactNode } from 'react';

interface CollapsibleSectionProps {
  title: string;
  icon?: string;
  defaultOpen?: boolean;
  children: ReactNode;
}

export function CollapsibleSection({
  title,
  icon,
  defaultOpen = false,
  children,
}: CollapsibleSectionProps) {
  const [open, setOpen] = useState(defaultOpen);

  return (
    <section className={`collapsible-section${open ? ' is-open' : ''}`}>
      <button
        type="button"
        className="collapsible-section-header"
        aria-expanded={open}
        onClick={() => setOpen((v) => !v)}
      >
        {icon && <span className="collapsible-section-icon" aria-hidden>{icon}</span>}
        <span className="collapsible-section-title">{title}</span>
        <span className="collapsible-section-chevron" aria-hidden />
      </button>
      {open && <div className="collapsible-section-body">{children}</div>}
    </section>
  );
}
