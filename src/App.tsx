import { useCallback, useEffect, useMemo, useRef } from 'react';
import { useSessionStore } from './stores/session-store';
import { useSimulation } from './hooks/useSimulation';
import { EeIkProvider } from './contexts/ee-ik-context';
import { RobotViewer } from './components/Viewer/RobotViewer';
import { ChartPanel } from './components/charts';
import ModelPanel from './components/panels/ModelPanel';
import { ControlPanel } from './components/panels/ControlPanel';
import { PayloadPanel } from './components/panels/PayloadPanel';
import { VisualizationPanel } from './components/panels/VisualizationPanel';
import { SimulationPanel } from './components/panels/SimulationPanel';
import { StatusBadge } from './components/ui/StatusBadge';
import { AppMenuBar } from './components/layout/AppMenuBar';
import { DockSidebar, DockBottom } from './components/layout/DockSidebar';

function simStatusLabel(status: string): string {
  switch (status) {
    case 'idle':
      return '空闲';
    case 'loading':
      return '加载中';
    case 'ready':
      return '就绪';
    case 'running':
      return '仿真中';
    case 'error':
      return '错误';
    default:
      return status;
  }
}

export default function App() {
  const robotInfo = useSessionStore((s) => s.robotInfo);
  const recorderDict = useSessionStore((s) => s.recorderDict);
  const recorderWindowSec = useSessionStore((s) => s.recorderWindowSec);
  const urdfText = useSessionStore((s) => s.urdfText);
  const simStatus = useSessionStore((s) => s.simStatus);
  const simMessage = useSessionStore((s) => s.simMessage);
  const simTime = useSessionStore((s) => s.simTime);
  const isPaused = useSessionStore((s) => s.isPaused);
  const recorder = useSessionStore((s) => s.recorder);
  const loading = useSessionStore((s) => s.loading);
  const baseLink = useSessionStore((s) => s.baseLink);
  const controlMode = useSessionStore((s) => s.controlMode);
  const controlLayer = useSessionStore((s) => s.controlLayer);
  const ikLiveStatus = useSessionStore((s) => s.ikLiveStatus);
  const ikLiveMessage = useSessionStore((s) => s.ikLiveMessage);
  const ikLastSolveMs = useSessionStore((s) => s.ikLastSolveMs);

  const sim = useSimulation();
  const {
    loadRobot,
    loadDefaultBiped,
    reloadUrdf,
    applyBaseLink,
    applyEndEffectorLink,
    startSimulation,
    addMotionTarget,
    executeMotionTargets,
    commitEeGizmoDrag,
    pauseSimulation,
    stopSimulation,
    resetRobotPose,
    resetRecorder,
    toggleRecorderPause,
    setRecorderWindowSec,
    setJointGains,
    applyAutoJointGains,
    setControllerKdDamping,
    solveEeIkLive,
    resetReferencePose,
    resetGizmoToCurrent,
    syncExternalWrenchesFromStore,
    dispose,
  } = sim;

  const eeIkApi = useMemo(
    () => ({ solveEeIkLive, resetReferencePose, onEeDragCommit: commitEeGizmoDrag }),
    [solveEeIkLive, resetReferencePose, commitEeGizmoDrag],
  );

  useEffect(() => () => dispose(), [dispose]);

  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      if (e.code !== 'Space' && e.key !== ' ') return;

      const target = e.target as HTMLElement | null;
      if (target) {
        const tag = target.tagName;
        if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || tag === 'BUTTON') return;
        if (target.isContentEditable) return;
      }

      const state = useSessionStore.getState();
      if (state.simStatus !== 'running') return;

      e.preventDefault();
      pauseSimulation();
    };

    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [pauseSimulation]);

  const defaultLoadedRef = useRef(false);
  useEffect(() => {
    if (defaultLoadedRef.current) return;
    defaultLoadedRef.current = true;
    void loadDefaultBiped().catch((e) => {
      const msg = e instanceof Error ? e.message : String(e);
      useSessionStore.getState().setLoadError(`默认模型加载失败: ${msg}`);
    });
  }, [loadDefaultBiped]);

  const panelDisabled = loading || simStatus === 'running';
  const controlPanelDisabled = loading;

  const running = simStatus === 'running';
  const primaryIsStop = running;
  const transportDisabled = loading || !robotInfo || (!running && simStatus !== 'ready');
  const transportTitle = running
    ? '停止仿真'
    : '开始仿真';

  const handleTransport = () => {
    if (transportDisabled) return;
    if (primaryIsStop) {
      stopSimulation();
    } else {
      void startSimulation();
    }
  };

  const leftPanels = useMemo(
    () => ({
      model: (
        <ModelPanel
          onRobotLoaded={(result) => loadRobot(result.urdfText, result.urdfFileName, result.meshes)}
          onLoadDefault={loadDefaultBiped}
          onApplyBaseLink={(link) => void applyBaseLink(link)}
          onUrdfChanged={(xml) => void reloadUrdf(xml)}
          onResetRobotPose={() => resetRobotPose()}
          disabled={loading}
        />
      ),
      simulation: (
        <SimulationPanel
          onStart={() => void startSimulation()}
          onPause={pauseSimulation}
          onStop={stopSimulation}
          onResetRecorder={resetRecorder}
          onToggleRecorderPause={toggleRecorderPause}
          disabled={loading || !robotInfo}
          canStart={!!robotInfo && simStatus !== 'running'}
        />
      ),
    }),
    [
      applyBaseLink,
      loadDefaultBiped,
      loadRobot,
      loading,
      pauseSimulation,
      reloadUrdf,
      resetRecorder,
      resetRobotPose,
      toggleRecorderPause,
      robotInfo,
      simStatus,
      startSimulation,
      stopSimulation,
    ],
  );

  const handleAddMotionTarget = useCallback(async () => {
    const result = await addMotionTarget();
    if (!result.ok) {
      useSessionStore.getState().setSimStatus('error', result.message ?? '添加目标失败');
    }
  }, [addMotionTarget]);

  const rightPanels = useMemo(
    () => ({
      control: (
        <ControlPanel
          onAddMotionTarget={handleAddMotionTarget}
          onExecuteMotionTargets={() => void executeMotionTargets()}
          onEndEffectorLinkChange={applyEndEffectorLink}
          onSetJointGain={setJointGains}
          onResetJointGains={applyAutoJointGains}
          onControllerKdDampingChange={setControllerKdDamping}
          onResetReference={resetReferencePose}
          onResetGizmo={resetGizmoToCurrent}
          disabled={controlPanelDisabled || !robotInfo}
        />
      ),
      payload: urdfText ? (
        <PayloadPanel
          urdfText={urdfText}
          onUrdfChanged={(xml) => reloadUrdf(xml)}
          onExternalWrenchChange={syncExternalWrenchesFromStore}
          disabled={panelDisabled}
        />
      ) : (
        <section className="panel-section">
          <p className="hint">请先加载模型</p>
        </section>
      ),
      visualization: <VisualizationPanel />,
    }),
    [
      applyEndEffectorLink,
      controlPanelDisabled,
      panelDisabled,
      reloadUrdf,
      resetReferencePose,
      resetGizmoToCurrent,
      robotInfo,
      handleAddMotionTarget,
      executeMotionTargets,
      setJointGains,
      applyAutoJointGains,
      setControllerKdDamping,
      syncExternalWrenchesFromStore,
      urdfText,
    ],
  );

  return (
    <EeIkProvider value={eeIkApi}>
    <div className="app-studio">
      <header className="app-studio-header">
        <h1 className="app-brand">
          <span className="app-brand-gradient">Arm Dynamics</span>
          <span className="app-brand-sub">Studio</span>
        </h1>
        <AppMenuBar />
        {robotInfo && (
          <div className="app-studio-meta">
            <span>{robotInfo.name}</span>
            <span className="header-dof-badge">{robotInfo.dof} DOF</span>
            <span className="header-base-badge">基座 {baseLink}</span>
          </div>
        )}
        <div className="app-studio-status">
          {robotInfo && (
            <button
              type="button"
              className={`header-transport-btn${primaryIsStop ? ' header-transport-btn--stop' : ''}${
                transportDisabled ? ' header-transport-btn--disabled' : ''
              }`}
              disabled={transportDisabled}
              onClick={handleTransport}
              title={transportTitle}
              aria-label={transportTitle}
            >
              {primaryIsStop ? '⏹' : '▶'}
            </button>
          )}
          <StatusBadge status={simStatus} label={simStatusLabel(simStatus)} />
          {running && <span className="header-sim-time">t = {simTime.toFixed(3)} s</span>}
          {recorder.sampleCount > 0 && (
            <span className="header-sample-count">采样 {recorder.sampleCount}</span>
          )}
          {robotInfo && running && (
            <span className="header-robot-short" title={robotInfo.name}>
              {robotInfo.name.replace(/\.(urdf|xml)$/i, '').split(/[/\\]/).pop()}
            </span>
          )}
          {running && isPaused && <span className="header-paused-badge">已暂停</span>}
          {controlLayer === 'ee' && robotInfo && (
            <span className={`ik-status-badge ik-status-badge--${ikLiveStatus}`}>
              IK{' '}
              {ikLiveStatus === 'solving'
                ? '求解中'
                : ikLiveStatus === 'converged'
                  ? '收敛'
                  : ikLiveStatus === 'failed'
                    ? '失败'
                    : '就绪'}
              {ikLastSolveMs != null && ikLiveStatus !== 'idle' && ` ${ikLastSolveMs.toFixed(0)}ms`}
              {ikLiveMessage && ikLiveStatus === 'failed' && ` · ${ikLiveMessage}`}
            </span>
          )}
          {running && controlMode === 'interpolate' && simMessage && (
            <span className="header-sim-msg">{simMessage}</span>
          )}
          {!running && (loading ? '正在加载…' : simMessage) && (
            <span className="header-sim-msg">{loading ? '正在加载…' : simMessage}</span>
          )}
        </div>
      </header>

      <div className="app-studio-body">
        <DockSidebar side="left" panels={leftPanels} />
        <main className="app-studio-viewer">
          <RobotViewer />
        </main>
        <DockSidebar side="right" panels={rightPanels} />
      </div>

      <DockBottom panelId="charts" title="曲线">
        <ChartPanel
          recorderDict={recorderDict}
          jointNames={robotInfo?.jointNames}
          windowSeconds={recorderWindowSec}
          recorderWindowSec={recorderWindowSec}
          onRecorderWindowChange={setRecorderWindowSec}
          filenameBase={robotInfo?.name}
          title=""
        />
      </DockBottom>
    </div>
    </EeIkProvider>
  );
}
