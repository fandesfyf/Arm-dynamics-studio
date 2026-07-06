import { useCallback, useMemo, useState } from 'react';
import { parseLinkNames } from '../../utils/urdf-base-fixture';
import {
  appendSpherePayloadWithRecord,
  attachUrdfSnippet,
  listSpherePayloadDisplayItems,
  listSpherePayloadLinks,
  parseWrenchValues,
  removeLastSpherePayloadOnLink,
  removeSpherePayloads,
  revertModifyInertialPayload,
  solidSphereInertia,
  SPHERE_PAYLOAD_LINK_PATTERN,
  type PayloadRecord,
} from '../../core/payload-editor';
import { useSessionStore } from '../../stores/session-store';

export interface PayloadPanelProps {
  urdfText: string;
  onUrdfChanged: (xml: string) => void | Promise<void>;
  /** 禁用添加/移除负载（仿真运行中、加载中） */
  payloadDisabled?: boolean;
  /** 禁用表单输入（仅加载中） */
  payloadFormDisabled?: boolean;
  /** 禁用外力编辑（仅加载中） */
  wrenchDisabled?: boolean;
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
  payloadDisabled = false,
  payloadFormDisabled = false,
  wrenchDisabled = false,
  onExternalWrenchChange,
}: PayloadPanelProps) {
  const externalWrenches = useSessionStore((s) => s.externalWrenches);
  const payloadRecords = useSessionStore((s) => s.payloadRecords);
  const simStatus = useSessionStore((s) => s.simStatus);
  const payloadFormDraft = useSessionStore((s) => s.payloadFormDraft);
  const setPayloadFormDraft = useSessionStore((s) => s.setPayloadFormDraft);
  const setExternalWrench = useSessionStore((s) => s.setExternalWrench);
  const clearExternalWrenches = useSessionStore((s) => s.clearExternalWrenches);
  const addPayloadRecord = useSessionStore((s) => s.addPayloadRecord);
  const removePayloadRecord = useSessionStore((s) => s.removePayloadRecord);
  const clearPayloadRecords = useSessionStore((s) => s.clearPayloadRecords);

  const linkOptions = useMemo(
    () =>
      parseLinkNames(urdfText).filter(
        (name) => name !== 'world' && !SPHERE_PAYLOAD_LINK_PATTERN.test(name),
      ),
    [urdfText],
  );

  const spherePayloadItems = useMemo(
    () => listSpherePayloadDisplayItems(urdfText, payloadRecords),
    [payloadRecords, urdfText],
  );

  const spherePayloadCount = spherePayloadItems.length;

  const {
    payloadLink,
    wrenchLink,
    sphereMass,
    sphereRadius,
    sphereMode,
    wrenchDraft,
  } = payloadFormDraft;

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

  const activePayloadLink =
    payloadLink && linkOptions.includes(payloadLink)
      ? payloadLink
      : linkOptions[0] ?? '';
  const activeWrenchLink =
    wrenchLink && linkOptions.includes(wrenchLink) ? wrenchLink : linkOptions[0] ?? '';

  const payloadSelectValue =
    payloadLink && linkOptions.includes(payloadLink) ? payloadLink : activePayloadLink;
  const wrenchSelectValue =
    wrenchLink && linkOptions.includes(wrenchLink) ? wrenchLink : activeWrenchLink;

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
        setError(null);
        await onUrdfChanged(xml);
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
        if (!activePayloadLink) throw new Error('请选择目标 link');
        if (import.meta.env.DEV) {
          console.log('[payload] 添加球体负载', { link: activePayloadLink, mass: sphereMass, radius: sphereRadius, mode: sphereMode });
        }
        const { urdfText: xml, record } = appendSpherePayloadWithRecord(urdfText, {
          parentLink: activePayloadLink,
          mass: sphereMass,
          radius: sphereRadius,
          mode: sphereMode,
        });
        await applyUrdfChange(
          xml,
          sphereMode === 'child_link'
            ? `已在 ${activePayloadLink} 添加球体负载，请重新运行仿真`
            : `已更新 ${activePayloadLink} 惯量，请重新运行仿真`,
          record,
        );
      } catch (err) {
        setNotice(null);
        setError(err instanceof Error ? err.message : String(err));
      }
    })();
  }, [
    activePayloadLink,
    applyUrdfChange,
    sphereMass,
    sphereMode,
    sphereRadius,
    urdfText,
  ]);

  const handleRemoveSphereOnLink = useCallback(() => {
    void (async () => {
      try {
        if (!activePayloadLink) throw new Error('请选择目标 link');

        const modifyRecords = [...payloadRecords]
          .reverse()
          .filter((r) => r.parentLink === activePayloadLink && r.kind === 'modify_inertial');

        if (modifyRecords.length > 0) {
          const record = modifyRecords[0]!;
          const xml = revertModifyInertialPayload(urdfText, record);
          await applyUrdfChange(xml, `已还原 ${activePayloadLink} 的惯量修改`);
          removePayloadRecord(record.id);
          return;
        }

        const childRecords = payloadRecords.filter(
          (r) => r.parentLink === activePayloadLink && r.kind === 'child_link',
        );
        const xml = removeLastSpherePayloadOnLink(urdfText, activePayloadLink);
        const removed = childRecords[childRecords.length - 1];
        await applyUrdfChange(xml, `已移除 ${activePayloadLink} 上的球体负载`);
        if (removed) removePayloadRecord(removed.id);
      } catch (err) {
        setNotice(null);
        setError(err instanceof Error ? err.message : String(err));
      }
    })();
  }, [
    activePayloadLink,
    applyUrdfChange,
    payloadRecords,
    removePayloadRecord,
    urdfText,
  ]);

  const handleRemovePayloadItem = useCallback(
    (itemId: string) => {
      void (async () => {
        try {
          const record = payloadRecords.find((r) => r.id === itemId);
          const item = spherePayloadItems.find((entry) => entry.id === itemId);
          if (!item) return;

          if (record?.kind === 'modify_inertial') {
            const xml = revertModifyInertialPayload(urdfText, record);
            await applyUrdfChange(xml, `已还原 ${record.parentLink} 的惯量修改`);
            removePayloadRecord(record.id);
            return;
          }

          const payloadLinkName = record?.payloadLink ?? item.payloadLink;
          if (!payloadLinkName) {
            throw new Error('未找到可移除的球体负载 link');
          }

          const xml = removeSpherePayloads(urdfText, [payloadLinkName]);
          await applyUrdfChange(xml, `已移除 ${item.parentLink} 上的球体负载`);
          if (record) {
            removePayloadRecord(record.id);
          }
        } catch (err) {
          setNotice(null);
          setError(err instanceof Error ? err.message : String(err));
        }
      })();
    },
    [
      applyUrdfChange,
      payloadRecords,
      removePayloadRecord,
      spherePayloadItems,
      urdfText,
    ],
  );

  const handleApplyWrench = useCallback(() => {
    try {
      if (!activeWrenchLink) throw new Error('请选择外力目标 link');
      const wrench = parseWrenchValues(wrenchDraft);
      setExternalWrench(activeWrenchLink, wrench);
      onExternalWrenchChange?.();
      setError(null);
      setNotice(
        simStatus === 'running'
          ? `已对 ${activeWrenchLink} 施加 6D 外力（仿真中实时生效）`
          : `已对 ${activeWrenchLink} 设置 6D 外力，开始仿真后生效`,
      );
    } catch (err) {
      setNotice(null);
      setError(err instanceof Error ? err.message : String(err));
    }
  }, [activeWrenchLink, onExternalWrenchChange, setExternalWrench, simStatus, wrenchDraft]);

  const handleClearWrenches = useCallback(() => {
    clearExternalWrenches();
    onExternalWrenchChange?.();
    setPayloadFormDraft({
      wrenchDraft: { fx: 0, fy: 0, fz: 0, tx: 0, ty: 0, tz: 0 },
    });
    setNotice(simStatus === 'running' ? '已清除全部 6D 外力，变更已生效' : '已清除全部 6D 外力');
    setError(null);
  }, [clearExternalWrenches, onExternalWrenchChange, setPayloadFormDraft, simStatus]);

  const handleAttachSnippet = useCallback(() => {
    void (async () => {
      try {
        if (!activePayloadLink) throw new Error('请选择目标 link');
        const xml = attachUrdfSnippet(urdfText, {
          parentLink: activePayloadLink,
          snippetXml: snippetText,
        });
        await applyUrdfChange(xml, `已将 URDF 片段 fixed 拼接到 ${activePayloadLink}，请重新运行仿真`);
      } catch (err) {
        setNotice(null);
        setError(err instanceof Error ? err.message : String(err));
      }
    })();
  }, [activePayloadLink, applyUrdfChange, snippetText, urdfText]);

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
        setPayloadFormDraft({
          wrenchDraft: { fx: 0, fy: 0, fz: 0, tx: 0, ty: 0, tz: 0 },
        });
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
    setPayloadFormDraft,
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

  const activeWrench = activeWrenchLink ? externalWrenches.get(activeWrenchLink) : undefined;
  const hasPayloadOnLink =
    payloadRecords.some((r) => r.parentLink === activePayloadLink) ||
    listSpherePayloadLinks(urdfText).some((name) => name.startsWith(`${activePayloadLink}_payload`));

  return (
    <section className="mass-editor-panel payload-panel">
      <h3>负载 / 外力</h3>
      {error && <p className="mass-editor-error">{error}</p>}
      {notice && <p className="hint">{notice}</p>}

      <h4>球体负载</h4>
      <label className="field-label">
        负载目标 Link
        <select
          value={payloadSelectValue}
          disabled={payloadFormDisabled}
          onChange={(e) => setPayloadFormDraft({ payloadLink: e.target.value })}
        >
          {linkOptions.map((name) => (
            <option key={name} value={name}>
              {name}
            </option>
          ))}
        </select>
      </label>
      <div className="payload-grid">
        <label className="field-label">
          质量 (kg)
          <input
            type="number"
            min={0}
            step="any"
            disabled={payloadFormDisabled}
            value={sphereMass}
            onChange={(e) => setPayloadFormDraft({ sphereMass: Number(e.target.value) })}
          />
        </label>
        <label className="field-label">
          半径 (m)
          <input
            type="number"
            min={0}
            step="any"
            disabled={payloadFormDisabled}
            value={sphereRadius}
            onChange={(e) => setPayloadFormDraft({ sphereRadius: Number(e.target.value) })}
          />
        </label>
      </div>

      <div className="payload-mode-row">
        <label>
          <input
            type="radio"
            name="sphereMode"
            checked={sphereMode === 'child_link'}
            disabled={payloadFormDisabled}
            onChange={() => setPayloadFormDraft({ sphereMode: 'child_link' })}
          />
          追加子 link + 球体
        </label>
        <label>
          <input
            type="radio"
            name="sphereMode"
            checked={sphereMode === 'modify_inertial'}
            disabled={payloadFormDisabled}
            onChange={() => setPayloadFormDraft({ sphereMode: 'modify_inertial' })}
          />
          叠加到 link 惯量
        </label>
      </div>

      <p className="hint">
        实心球惯量 I = 2/5 m r² — 预览: ixx={previewInertia.ixx.toExponential(3)}, iyy=
        {previewInertia.iyy.toExponential(3)}, izz={previewInertia.izz.toExponential(3)}
      </p>

      <div className="payload-actions">
        <button type="button" className="primary" disabled={payloadDisabled} onClick={handleAddSphere}>
          添加球体负载
        </button>
        <button
          type="button"
          className="btn btn-secondary"
          disabled={payloadDisabled || !hasPayloadOnLink}
          onClick={handleRemoveSphereOnLink}
        >
          移除球体负载
        </button>
      </div>
      {spherePayloadItems.length > 0 && (
        <div className="payload-list-wrap">
          <p className="payload-list-title">已添加负载 ({spherePayloadItems.length})</p>
          <ul className="payload-list">
            {spherePayloadItems.map((item) => {
              const isActive = item.parentLink === activePayloadLink;
              const kindLabel = item.kind === 'modify_inertial' ? '惯量叠加' : '子 link 球体';
              return (
                <li
                  key={item.id}
                  className={`payload-list-item${isActive ? ' payload-list-item--active' : ''}`}
                >
                  <button
                    type="button"
                    className="payload-list-main"
                    disabled={payloadFormDisabled}
                    onClick={() => setPayloadFormDraft({ payloadLink: item.parentLink })}
                  >
                    <span className="payload-list-dot" aria-hidden />
                    <span className="payload-list-text">
                      <strong>{item.parentLink}</strong>
                      <span className="payload-list-meta">
                        {kindLabel} · {item.mass.toFixed(3)} kg · r={item.radius.toFixed(3)} m
                        {item.payloadLink ? ` · ${item.payloadLink}` : ''}
                      </span>
                    </span>
                  </button>
                  <button
                    type="button"
                    className="btn btn-secondary payload-list-remove"
                    disabled={payloadDisabled}
                    title="移除此负载"
                    onClick={() => handleRemovePayloadItem(item.id)}
                  >
                    移除
                  </button>
                </li>
              );
            })}
          </ul>
        </div>
      )}

      <hr className="payload-divider" />

      <h4>6D 外力（link 坐标系）</h4>
      <label className="field-label">
        外力目标 Link
        <select
          value={wrenchSelectValue}
          disabled={wrenchDisabled}
          onChange={(e) => setPayloadFormDraft({ wrenchLink: e.target.value })}
        >
          {linkOptions.map((name) => (
            <option key={name} value={name}>
              {name}
            </option>
          ))}
        </select>
      </label>
      <div className="payload-grid payload-grid--wrench">
        {WRENCH_LABELS.map(({ key, label }) => (
          <label key={key} className="field-label">
            {label}
            <input
              type="number"
              step="any"
              disabled={wrenchDisabled}
              value={wrenchDraft[key] ?? 0}
              onChange={(e) =>
                setPayloadFormDraft({
                  wrenchDraft: { ...wrenchDraft, [key]: Number(e.target.value) },
                })
              }
            />
          </label>
        ))}
      </div>
      {activeWrench && (
        <p className="hint">
          当前 {activeWrenchLink}: [{activeWrench.map((v) => v.toFixed(3)).join(', ')}]
        </p>
      )}
      <div className="payload-actions">
        <button type="button" className="primary" disabled={wrenchDisabled} onClick={handleApplyWrench}>
          应用外力
        </button>
        <button
          type="button"
          className="btn btn-secondary"
          disabled={wrenchDisabled || externalWrenches.size === 0}
          onClick={handleClearWrenches}
        >
          清除全部外力
        </button>
      </div>
      <p className="hint">
        力/力矩在目标 link 坐标系下定义；fixed 子 link（如 end_effector）会变换到父 body 施加。
        仿真运行中可实时修改外力；负载编辑需停止仿真。
      </p>

      <hr className="payload-divider" />

      <h4>URDF 片段拼接 (MVP)</h4>
      <textarea
        className="payload-snippet"
        rows={6}
        disabled={payloadFormDisabled}
        value={snippetText}
        onChange={(e) => setSnippetText(e.target.value)}
      />
      <label className="field-label payload-file">
        导入 .urdf / .xml 片段
        <input type="file" accept=".urdf,.xml,text/xml" disabled={payloadDisabled} onChange={handleSnippetFile} />
      </label>
      <button type="button" className="primary" disabled={payloadDisabled} onClick={handleAttachSnippet}>
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

      <p className="hint payload-sim-hint">
        {payloadDisabled && simStatus === 'running'
          ? '仿真运行中不可添加/移除负载（表单可继续编辑）；外力可实时调整。'
          : null}
      </p>

      <button
        type="button"
        className="btn btn-danger btn-block"
        disabled={
          payloadDisabled ||
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
