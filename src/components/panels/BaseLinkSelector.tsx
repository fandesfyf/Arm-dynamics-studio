import { useMemo } from 'react';
import { useSessionStore } from '../../stores/session-store';
import { parseLinkNames, BASE_LINK_CANDIDATES } from '../../utils/urdf-base-fixture';

export { BASE_LINK_CANDIDATES };

interface BaseLinkSelectorProps {
  onApply: (link: string) => void;
  disabled?: boolean;
}

export function BaseLinkSelector({ onApply, disabled }: BaseLinkSelectorProps) {
  const urdfText = useSessionStore((s) => s.urdfText);
  const baseLink = useSessionStore((s) => s.baseLink);
  const setBaseLink = useSessionStore((s) => s.setBaseLink);

  const linkOptions = useMemo(() => {
    if (!urdfText) return [];
    const all = parseLinkNames(urdfText).filter((n) => n !== 'world');
    const preferred = BASE_LINK_CANDIDATES.filter((c) =>
      all.some((n) => n.toLowerCase() === c.toLowerCase()),
    );
    const rest = all.filter((n) => !preferred.some((p) => p.toLowerCase() === n.toLowerCase()));
    return [...preferred.map((p) => all.find((n) => n.toLowerCase() === p.toLowerCase())!), ...rest];
  }, [urdfText]);

  if (!urdfText) {
    return <p className="hint">加载模型后可选择固定基座 link</p>;
  }

  return (
    <div className="base-link-selector">
      <label className="field-label">
        固定基座 Link
        <select
          value={baseLink}
          disabled={disabled}
          onChange={(e) => setBaseLink(e.target.value)}
        >
          {linkOptions.map((name) => (
            <option key={name} value={name}>
              {name}
            </option>
          ))}
        </select>
      </label>
      <p className="hint">仿真时将把所选 link 通过 world 固定关节焊接到地面。</p>
      <button
        type="button"
        className="btn btn-secondary"
        disabled={disabled}
        onClick={() => onApply(baseLink)}
      >
        应用并重新加载
      </button>
    </div>
  );
}
