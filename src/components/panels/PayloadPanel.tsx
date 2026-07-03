import { useCallback, useMemo, useState } from 'react';
import { parseLinkNames } from '../../utils/urdf-base-fixture';
import {
  appendSpherePayloadWithRecord,
  attachUrdfSnippet,
  listSpherePayloadLinks,
  parseWrenchValues,
  removeLastSpherePayloadOnLink,
  removeSpherePayloads,
  revertModifyInertialPayload,
  solidSphereInertia,
  type SpherePayloadMode,
  type PayloadRecord,
} from '../../core/payload-editor';
import { finalizeUrdfForMujoco } from '../../utils/urdf-sanitize';
import { useSessionStore } from '../../stores/session-store';

export interface PayloadPanelProps {
  urdfText: string;
  onUrdfChanged: (xml: string) => void | Promise<void>;
  disabled?: boolean;
  onExternalWrenchChange?: () => void;
}

const WRENCH_FIELDS = ['fx', 'fy', 'fz', 'tx', 'ty', 'tz'] as const;

const WRENCH_LABELS: { key: (typeof WRENCH_FIELDS)[number]; label: string }[] = [
  { key: 'fx', label: 'Fx (N)' },
  { key: 'fy', label: 'Fy (N)' },
  { key: 'fz', label: 'Fz (N)' },
  { key: 'tx', label: 'Tx (N·m)' },
  { key: 'ty', label: 'Ty (N·m)' },
  { key: 'tz', label: 'Tz (N·m)' },
];

export function PayloadPanel({
  urdfText,
  onUrdfChanged,
  disabled,
  onExternalWrenchChange,
}: PayloadPanelProps) {
  const externalWrenches = useSessionStore((s) => s.externalWrenches);
  const payloadRecords = useSessionStore((s) => s.payloadRecords);
  const simStatus = useSessionStore((s) => s.simStatus);
  const setExternalWrench = useSessionStore((s) => s.setExternalWrench);
  const clearExternalWrenches = useSessionStore((s) => s.clearExternalWrenches);
  const addPayloadRecord = useSessionStore((s) => s.addPayloadRecord);
  const removePayloadRecord = useSessionStore((s) => s.removePayloadRecord);
  const clearPayloadRecords = useSessionStore((s) => s.clearPayloadRecords);

  const linkOptions = useMemo(
    () => parseLinkNames(urdfText).filter((name) => name !== 'world'),
    [urdfText],
  );

  const spherePayloadCount = useMemo(() => listSpherePayloadLinks(urdfText).length, [urdfText]);

  const [selectedLink, setSelectedLink] = useState(linkOptions[0] ?? '');
  const [sphereMass, setSphereMass] = useState(0.2);
  const [sphereRadius, setSphereRadius] = useState(0.03);
  const [sphereMode, setSphereMode] = useState<SpherePayloadMode>('child_link');
  const [wrenchDraft, setWrenchDraft] = useState<Record<string, number>>({
    fx: 0,
    fy: 0,
    fz: 0,
    tx: 0,
    ty: 0,
    tz: 0,
  });
  const [snippetText, setSnippetText] = useState(
    `<link name="payload_tool">\n  <inertial>\n    <mass value="0.1"/>\n    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>\n  </inertial>\n</link>`,
  );
  const [error, setError] = useState<string | null>(null);
  const [notice, setNotice] = useState<string | null>(null);
  const [urdfViewerOpen, setUrdfViewerOpen] = useState(false);

  const robotInfo = useSessionStore((s) => s.robotInfo);
  const urdfFileName = useSessionStore((s) => s.urdfFileName);

  const exportFileName = useMemo(() => {
    const name =
      robotInfo?.name ??
      urdfText.match(/<robot\s+name="([^"]+)"/)?.[1] ??
      urdfFileName?.replace(/\.urdf$/i, '') ??
      'robot';
    return `${name}_with_payload.urdf`;
  }, [robotInfo, urdfFileName, urdfText]);

  const activeLink = selectedLink || linkOptions[0] || '';

  const previewInertia = useMemo(() => {
    try {
      const inertia = solidSphereInertia(sphereMass, sphereRadius);
      return { ixx: inertia.ixx, iyy: inertia.iyy, izz: inertia.izz };
    } catch {
      return { ixx: 0, iyy: 0, izz: 0 };
    }
  }, [sphereMass, sphereRadius]);

  const applyUrdfChange = useCallback(
    async (xml: string, message: string, record?: PayloadRecord) => {
      try {
        const finalized = finalizeUrdfForMujoco(xml);
        setError(null);
        await onUrdfChanged(finalized);
        if (record) addPayloadRecord(record);
        setNotice(message);
      } catch (err) {
        setNotice(null);
        setError(err instanceof Error ? err.message : String(err));
      }
    },
    [addPayloadRecord, onUrdfChanged],
  );

  const handleAddSphere = useCallback(() => {
    void (async () => {
      try {
        if (!activeLink) throw new Error('请选择目标 link');
        const { urdfText: xml, record } = appendSpherePayloadWithRecord(urdfText, {
          parentLink: activeLink,
          mass: sphereMass,
          radius: sphereRadius,
          mode: sphereMode,
        });
        await applyUrdfChange(
          xml,
          sphereMode === 'child_link'
            ? `已在 ${activeLink} 添加球体负载，请重新运行仿真`
            : `已更新 ${activeLink} 惯量，请重新运行仿真`,
          record,
        );
      } catch (err) {
        setNotice(null);
        setError(err instanceof Error ? err.message : String(err));
      }
    })();
  }, [
    activeLink,
    applyUrdfChange,
    sphereMass,
    sphereMode,
    sphereRadius,
    urdfText,
  ]);

  const handleRemoveSphereOnLink = useCallback(() => {
    void (async () => {
      try {
        if (!activeLink) throw new Error('请选择目标 link');

        const modifyRecords = [...payloadRecords]
          .reverse()
          .filter((r) => r.parentLink === activeLink && r.kind === 'modify_inertial');

        if (modifyRecords.length > 0) {
          const record = modifyRecords[0]!;
          const xml = revertModifyInertialPayload(urdfText, record);
          await applyUrdfChange(xml, `已还原 ${activeLink} 的惯量修改`);
          removePayloadRecord(record.id);
          return;
        }

        const childRecords = payloadRecords.filter(
          (r) => r.parentLink === activeLink && r.kind === 'child_link',
        );
        const xml = removeLastSpherePayloadOnLink(urdfText, activeLink);
        const removed = childRecords[childRecords.length - 1];
        await applyUrdfChange(xml, `已移除 ${activeLink} 上的球体负载`);
        if (removed) removePayloadRecord(removed.id);
      } catch (err) {
        setNotice(null);
        setError(err instanceof Error ? err.message : String(err));
      }
    })();
  }, [
    activeLink,
    applyUrdfChange,
    payloadRecords,
    removePayloadRecord,
    urdfText,
  ]);

  const handleApplyWrench = useCallback(() => {
    try {
      if (!activeLink) throw new Error('请选择目标 link');
      const wrench = parseWrenchValues(wrenchDraft);
      setExternalWrench(activeLink, wrench);
      onExternalWrenchChange?.();
      setError(null);
      setNotice(
        simStatus === 'running'
          ? `已对 ${activeLink} 施加 6D 外力，外力已生效`
          : `已对 ${activeLink} 设置 6D 外力，开始仿真后生效`,
      );
    } catch (err) {
      setNotice(null);
      setError(err instanceof Error ? err.message : String(err));
    }
  }, [activeLink, onExternalWrenchChange, setExternalWrench, simStatus, wrenchDraft]);

  const handleClearWrenches = useCallback(() => {
    clearExternalWrenches();
    onExternalWrenchChange?.();
    setWrenchDraft({ fx: 0, fy: 0, fz: 0, tx: 0, ty: 0, tz: 0 });
    setNotice(simStatus === 'running' ? '已清除全部 6D 外力，变更已生效' : '已清除全部 6D 外力');
    setError(null);
  }, [clearExternalWrenches, onExternalWrenchChange, simStatus]);

  const handleAttachSnippet = useCallback(() => {
    void (async () => {
      try {
        if (!activeLink) throw new Error('请选择目标 link');
        const xml = attachUrdfSnippet(urdfText, {
          parentLink: activeLink,
          snippetXml: snippetText,
        });
        await applyUrdfChange(xml, `已将 URDF 片段 fixed 拼接到 ${activeLink}，请重新运行仿真`);
      } catch (err) {
        setNotice(null);
        setError(err instanceof Error ? err.message : String(err));
      }
    })();
  }, [activeLink, applyUrdfChange, snippetText, urdfText]);

  const handleClearAllPayloadsAndWrenches = useCallback(() => {
    void (async () => {
      try {
        let xml = urdfText;

        const modifyRecords = payloadRecords.filter((r) => r.kind === 'modify_inertial');
        for (const record of [...modifyRecords].reverse()) {
          xml = revertModifyInertialPayload(xml, record);
        }

        xml = removeSpherePayloads(xml);
        clearPayloadRecords();
        clearExternalWrenches();
        onExternalWrenchChange?.();
        setWrenchDraft({ fx: 0, fy: 0, fz: 0, tx: 0, ty: 0, tz: 0 });
        await applyUrdfChange(xml, '已清除全部负载与外力，请重新运行仿真');
      } catch (err) {
        setNotice(null);
        setError(err instanceof Error ? err.message : String(err));
      }
    })();
  }, [
    applyUrdfChange,
    clearExternalWrenches,
    clearPayloadRecords,
    onExternalWrenchChange,
    payloadRecords,
    urdfText,
  ]);

  const handleSnippetFile = useCallback(
    (event: React.ChangeEvent<HTMLInputElement>) => {
      const file = event.target.files?.[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = () => {
        const text = typeof reader.result === 'string' ? reader.result : '';
        setSnippetText(text);
      };
      reader.readAsText(file);
      event.target.value = '';
    },
    [],
  );

  const handleDownloadUrdf = useCallback(() => {
    if (!urdfText) return;
    const blob = new Blob([urdfText], { type: 'application/xml;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = exportFileName;
    a.click();
    URL.revokeObjectURL(url);
    setError(null);
    setNotice(`已下载 ${exportFileName}`);
  }, [exportFileName, urdfText]);

  const handleCopyUrdf = useCallback(async () => {
    if (!urdfText) return;
    try {
      await navigator.clipboard.writeText(urdfText);
      setError(null);
      setNotice('URDF 已复制到剪贴板');
    } catch {
      setNotice(null);
      setError('复制失败，请手动选择文本复制');
    }
  }, [urdfText]);

  const activeWrench = activeLink ? externalWrenches.get(activeLink) : undefined;
  const hasPayloadOnLink =
    payloadRecords.some((r) => r.parentLink === activeLink) ||
    listSpherePayloadLinks(urdfText).some((name) => name.startsWith(`${activeLink}_payload`));

  return (
    <section className="mass-editor-panel payload-panel">
      <h3>负载 / 外力</h3>
      {error && <p className="mass-editor-error">{error}</p>}
      {notice && <p className="hint">{notice}</p>}

      <label className="field-label">
        目标 Link
        <select
          value={activeLink}
          disabled={disabled}
          onChange={(e) => setSelectedLink(e.target.value)}
        >
          {linkOptions.map((name) => (
            <option key={name} value={name}>
              {name}
            </option>
          ))}
        </select>
      </label>

      <h4>球体负载</h4>
      <div className="payload-grid">
        <label className="field-label">
          质量 (kg)
          <input
            type="number"
            min={0}
            step="any"
            disabled={disabled}
            value={sphereMass}
            onChange={(e) => setSphereMass(Number(e.target.value))}
          />
        </label>
        <label className="field-label">
          半径 (m)
          <input
            type="number"
            min={0}
            step="any"
            disabled={disabled}
            value={sphereRadius}
            onChange={(e) => setSphereRadius(Number(e.target.value))}
          />
        </label>
      </div>

      <div className="payload-mode-row">
        <label>
          <input
            type="radio"
            name="sphereMode"
            checked={sphereMode === 'child_link'}
            disabled={disabled}
            onChange={() => setSphereMode('child_link')}
          />
          追加子 link + 球体
        </label>
        <label>
          <input
            type="radio"
            name="sphereMode"
            checked={sphereMode === 'modify_inertial'}
            disabled={disabled}
            onChange={() => setSphereMode('modify_inertial')}
          />
          叠加到 link 惯量
        </label>
      </div>

      <p className="hint">
        实心球惯量 I = 2/5 m r² — 预览: ixx={previewInertia.ixx.toExponential(3)}, iyy=
        {previewInertia.iyy.toExponential(3)}, izz={previewInertia.izz.toExponential(3)}
      </p>

      <div className="payload-actions">
        <button type="button" className="primary" disabled={disabled} onClick={handleAddSphere}>
          添加球体负载
        </button>
        <button
          type="button"
          className="btn btn-secondary"
          disabled={disabled || !hasPayloadOnLink}
          onClick={handleRemoveSphereOnLink}
        >
          移除球体负载
        </button>
      </div>
      {spherePayloadCount > 0 && (
        <p className="hint">当前 URDF 中共有 {spherePayloadCount} 个球体负载 link</p>
      )}

      <hr className="payload-divider" />

      <h4>6D 外力（link 坐标系）</h4>
      <div className="payload-grid payload-grid--wrench">
        {WRENCH_LABELS.map(({ key, label }) => (
          <label key={key} className="field-label">
            {label}
            <input
              type="number"
              step="any"
              disabled={disabled}
              value={wrenchDraft[key] ?? 0}
              onChange={(e) =>
                setWrenchDraft((prev) => ({ ...prev, [key]: Number(e.target.value) }))
              }
            />
          </label>
        ))}
      </div>
      {activeWrench && (
        <p className="hint">
          当前 {activeLink}: [{activeWrench.map((v) => v.toFixed(3)).join(', ')}]
        </p>
      )}
      <div className="payload-actions">
        <button type="button" className="primary" disabled={disabled} onClick={handleApplyWrench}>
          应用外力
        </button>
        <button
          type="button"
          className="btn btn-secondary"
          disabled={disabled || externalWrenches.size === 0}
          onClick={handleClearWrenches}
        >
          清除全部外力
        </button>
      </div>
      <p className="hint">
        仿真运行中施加外力会立即写入 MuJoCo xfrc_applied；body 未找到时回退为 qfrc 常值偏置。
      </p>

      <hr className="payload-divider" />

      <h4>URDF 片段拼接 (MVP)</h4>
      <textarea
        className="payload-snippet"
        rows={6}
        disabled={disabled}
        value={snippetText}
        onChange={(e) => setSnippetText(e.target.value)}
      />
      <label className="field-label payload-file">
        导入 .urdf / .xml 片段
        <input type="file" accept=".urdf,.xml,text/xml" disabled={disabled} onChange={handleSnippetFile} />
      </label>
      <button type="button" className="primary" disabled={disabled} onClick={handleAttachSnippet}>
        拼接到所选 link
      </button>

      <hr className="payload-divider" />

      <button
        type="button"
        className="btn btn-secondary btn-block"
        disabled={!urdfText}
        onClick={() => setUrdfViewerOpen((open) => !open)}
      >
        {urdfViewerOpen ? '收起 URDF 视图' : '查看当前 URDF'}
      </button>
      {urdfViewerOpen && urdfText && (
        <>
          <div className="payload-actions">
            <button type="button" className="primary" onClick={handleDownloadUrdf}>
              下载模型文件
            </button>
            <button type="button" className="btn btn-secondary" onClick={() => void handleCopyUrdf()}>
              复制 XML
            </button>
          </div>
          <p className="hint">当前合并后的 URDF（含负载），文件名：{exportFileName}</p>
          <textarea className="payload-snippet" rows={14} readOnly value={urdfText} />
        </>
      )}

      <button
        type="button"
        className="btn btn-danger btn-block"
        disabled={
          disabled ||
          (spherePayloadCount === 0 &&
            payloadRecords.length === 0 &&
            externalWrenches.size === 0)
        }
        onClick={handleClearAllPayloadsAndWrenches}
      >
        一键移除所有负载与外力
      </button>
    </section>
  );
}
