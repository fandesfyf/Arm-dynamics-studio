import { useCallback, useRef, useState } from 'react';
import { useSessionStore } from '../../stores/session-store';
import {
  extractRobotFromFileList,
  extractRobotFromFiles,
  extractRobotFromZip,
  listUrdfPathsFromZip,
  listUrdfPathsFromFiles,
  type RobotAssetExtract,
} from '../../utils/robot-asset-loader';
import {
  collectFilesFromDataTransfer,
  hasFileTransfer,
} from '../../utils/file-drop-handler';

export interface RobotUploadResult {
  urdfText: string;
  urdfFileName: string;
  meshes: Map<string, Uint8Array>;
}

interface RobotUploadProps {
  onLoadTestArm: () => Promise<void>;
  onRobotLoaded: (result: RobotUploadResult) => Promise<void>;
}

interface UrdfPickState {
  label: string;
  candidates: string[];
  zipFile?: File;
  folderFiles?: File[];
}

function folderUrdfCandidates(files: File[]): string[] {
  return listUrdfPathsFromFiles(files, { includeSkippedDirs: true });
}

export function RobotUpload({ onLoadTestArm, onRobotLoaded }: RobotUploadProps) {
  const loading = useSessionStore((s) => s.loading);
  const loadingMessage = useSessionStore((s) => s.loadingMessage);
  const robotInfo = useSessionStore((s) => s.robotInfo);
  const simMessage = useSessionStore((s) => s.simMessage);
  const simStatus = useSessionStore((s) => s.simStatus);
  const baseLink = useSessionStore((s) => s.baseLink);
  const fileInputRef = useRef<HTMLInputElement>(null);
  const folderInputRef = useRef<HTMLInputElement>(null);
  const [dragOver, setDragOver] = useState(false);
  const [urdfPick, setUrdfPick] = useState<UrdfPickState | null>(null);
  const [selectedUrdf, setSelectedUrdf] = useState('');

  const applyBundle = useCallback(
    async (bundle: RobotAssetExtract) => {
      setUrdfPick(null);
      await onRobotLoaded({
        urdfText: bundle.urdfText,
        urdfFileName: bundle.urdfFileName,
        meshes: bundle.meshes,
      });
    },
    [onRobotLoaded],
  );

  const handleError = useCallback((error: unknown) => {
    const msg = error instanceof Error ? error.message : String(error);
    useSessionStore.getState().setLoadError(msg);
  }, []);

  const loadZipWithUrdf = useCallback(
    async (file: File, urdfRelPath: string) => {
      await applyBundle(await extractRobotFromZip(file, urdfRelPath));
    },
    [applyBundle],
  );

  const loadFolderWithUrdf = useCallback(
    async (files: File[], urdfRelPath: string) => {
      await applyBundle(await extractRobotFromFiles(files, urdfRelPath));
    },
    [applyBundle],
  );

  const promptUrdfPickIfNeeded = useCallback(
    (label: string, candidates: string[], zipFile?: File, folderFiles?: File[]) => {
      if (candidates.length <= 1) {
        return candidates[0] ?? null;
      }
      setUrdfPick({ label, candidates, zipFile, folderFiles });
      setSelectedUrdf(candidates[0]!);
      return null;
    },
    [],
  );

  const ingestZip = useCallback(
    async (file: File) => {
      const candidates = await listUrdfPathsFromZip(file, { includeSkippedDirs: true });
      const picked = promptUrdfPickIfNeeded(file.name, candidates, file);
      if (picked) {
        await loadZipWithUrdf(file, picked);
      }
    },
    [loadZipWithUrdf, promptUrdfPickIfNeeded],
  );

  const ingestFiles = useCallback(
    async (files: File[]) => {
      if (files.length === 0) return;
      try {
        if (files.length === 1) {
          const only = files[0]!;
          const lower = only.name.toLowerCase();
          if (lower.endsWith('.zip')) {
            await ingestZip(only);
            return;
          }
          if (lower.endsWith('.urdf')) {
            await applyBundle(await extractRobotFromFileList(only));
            return;
          }
        }

        const candidates = folderUrdfCandidates(files);
        const picked = promptUrdfPickIfNeeded('文件夹', candidates, undefined, files);
        if (picked) {
          await applyBundle(await extractRobotFromFiles(files, picked));
        }
      } catch (e) {
        handleError(e);
      }
    },
    [applyBundle, handleError, ingestZip, promptUrdfPickIfNeeded],
  );

  const confirmUrdfPick = useCallback(async () => {
    if (!urdfPick || !selectedUrdf) return;
    try {
      if (urdfPick.zipFile) {
        await loadZipWithUrdf(urdfPick.zipFile, selectedUrdf);
      } else if (urdfPick.folderFiles) {
        await loadFolderWithUrdf(urdfPick.folderFiles, selectedUrdf);
      }
    } catch (e) {
      handleError(e);
    }
  }, [handleError, loadFolderWithUrdf, loadZipWithUrdf, selectedUrdf, urdfPick]);

  const onDrop = useCallback(
    async (e: React.DragEvent) => {
      e.preventDefault();
      setDragOver(false);
      if (!hasFileTransfer(e.dataTransfer)) return;
      try {
        const files = await collectFilesFromDataTransfer(e.dataTransfer);
        await ingestFiles(files);
      } catch (err) {
        handleError(err);
      }
    },
    [ingestFiles, handleError],
  );

  return (
    <section
      className={`panel-section upload-dropzone${dragOver ? ' is-dragover' : ''}`}
      onDragEnter={(e) => {
        e.preventDefault();
        if (hasFileTransfer(e.dataTransfer)) setDragOver(true);
      }}
      onDragOver={(e) => {
        e.preventDefault();
        if (e.dataTransfer) e.dataTransfer.dropEffect = 'copy';
      }}
      onDragLeave={(e) => {
        if (!e.currentTarget.contains(e.relatedTarget as Node)) {
          setDragOver(false);
        }
      }}
      onDrop={(e) => void onDrop(e)}
    >
      <h3>模型加载</h3>
      <p className="hint upload-hint">
        拖放文件夹 / ZIP / URDF，或选择 ZIP 文件 / 文件夹
      </p>
      <div className="button-row">
        <button type="button" onClick={() => void onLoadTestArm().catch(handleError)} disabled={loading}>
          加载 test_arm
        </button>
        <button
          type="button"
          onClick={() => fileInputRef.current?.click()}
          disabled={loading}
        >
          选择 ZIP
        </button>
        <button
          type="button"
          onClick={() => folderInputRef.current?.click()}
          disabled={loading}
        >
          选择文件夹
        </button>
        <input
          ref={fileInputRef}
          type="file"
          accept=".zip,application/zip"
          hidden
          onChange={(e) => {
            const file = e.target.files?.[0];
            if (file) void ingestZip(file);
            e.target.value = '';
          }}
        />
        <input
          ref={folderInputRef}
          type="file"
          // @ts-expect-error webkitdirectory 非标准属性
          webkitdirectory=""
          directory=""
          multiple
          hidden
          onChange={(e) => {
            const list = e.target.files;
            if (list?.length) void ingestFiles(Array.from(list));
            e.target.value = '';
          }}
        />
      </div>
      {urdfPick && (
        <div className="urdf-pick-panel" role="group" aria-label="选择 URDF">
          <p className="hint">
            「{urdfPick.label}」内含 {urdfPick.candidates.length} 个 URDF，请选择要加载的模型：
          </p>
          <label className="urdf-pick-row">
            <span>URDF 文件</span>
            <select
              value={selectedUrdf}
              disabled={loading}
              onChange={(e) => setSelectedUrdf(e.target.value)}
            >
              {urdfPick.candidates.map((path) => (
                <option key={path} value={path}>
                  {path}
                </option>
              ))}
            </select>
          </label>
          <div className="button-row">
            <button type="button" disabled={loading} onClick={() => void confirmUrdfPick()}>
              加载所选 URDF
            </button>
            <button
              type="button"
              className="btn btn-ghost"
              disabled={loading}
              onClick={() => setUrdfPick(null)}
            >
              取消
            </button>
          </div>
        </div>
      )}
      {loading && <p className="hint">{loadingMessage || '加载中…'}</p>}
      {simStatus === 'error' && simMessage && (
        <p className="upload-error" role="alert">{simMessage}</p>
      )}
      {robotInfo && (
        <dl className="info-dl">
          <dt>模型</dt>
          <dd>{robotInfo.name}</dd>
          <dt>固定基座</dt>
          <dd>{baseLink}</dd>
          <dt>DOF</dt>
          <dd>{robotInfo.dof}</dd>
          <dt>末端位置</dt>
          <dd>
            [{robotInfo.eePos.map((v) => v.toFixed(3)).join(', ')}]
          </dd>
        </dl>
      )}
    </section>
  );
}
