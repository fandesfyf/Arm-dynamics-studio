interface StatusBadgeProps {
  status: string;
  label: string;
}

export function StatusBadge({ status, label }: StatusBadgeProps) {
  const pulse = status === 'running';
  return (
    <span
      className={[
        'status-pill',
        `status-pill--${status}`,
        pulse ? 'status-pill--pulse' : '',
      ]
        .filter(Boolean)
        .join(' ')}
    >
      <span className="status-pill-dot" aria-hidden />
      {label}
    </span>
  );
}
