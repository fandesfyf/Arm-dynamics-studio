import { useState, useCallback, useRef, useEffect, type ReactNode, type CSSProperties } from 'react';

interface FloatingPanelProps {
  title: string;
  icon: string;
  accentColor: string;
  children: ReactNode;
  defaultOpen?: boolean;
  position?: { x: number; y: number };
  onStateChange?: (state: 'expanded' | 'collapsed' | 'minimized') => void;
  onPositionChange?: (position: { x: number; y: number }) => void;
}

type PanelState = 'expanded' | 'collapsed' | 'minimized';

export function FloatingPanel({
  title,
  icon,
  accentColor,
  children,
  defaultOpen = true,
  position: initialPosition = { x: 0, y: 0 },
  onStateChange,
  onPositionChange,
}: FloatingPanelProps) {
  const [state, setState] = useState<PanelState>(defaultOpen ? 'expanded' : 'collapsed');
  const [position, setPosition] = useState(initialPosition);
  const [isDragging, setIsDragging] = useState(false);
  const dragOffsetRef = useRef({ x: 0, y: 0 });
  const panelRef = useRef<HTMLDivElement>(null);
  const isDraggingRef = useRef(false);
  const currentPositionRef = useRef(initialPosition);

  useEffect(() => {
    currentPositionRef.current = position;
    if (panelRef.current && !isDragging) {
      panelRef.current.style.transform = `translate(${position.x}px, ${position.y}px)`;
    }
  }, [position, isDragging]);

  const handleMouseDown = useCallback((e: React.MouseEvent) => {
    if (e.target instanceof HTMLElement && e.target.closest('.panel-controls')) {
      return;
    }
    isDraggingRef.current = true;
    setIsDragging(true);
    dragOffsetRef.current = {
      x: e.clientX - currentPositionRef.current.x,
      y: e.clientY - currentPositionRef.current.y,
    };
    e.preventDefault();
  }, []);

  const handleMouseMove = useCallback((e: MouseEvent) => {
    if (!isDraggingRef.current) return;
    const panelWidth = panelRef.current?.offsetWidth || 320;
    const panelHeight = panelRef.current?.offsetHeight || 500;
    const boundedX = Math.max(0, Math.min(e.clientX - dragOffsetRef.current.x, window.innerWidth - panelWidth));
    const boundedY = Math.max(0, Math.min(e.clientY - dragOffsetRef.current.y, window.innerHeight - panelHeight));
    currentPositionRef.current = { x: boundedX, y: boundedY };
    if (panelRef.current) {
      panelRef.current.style.transform = `translate(${boundedX}px, ${boundedY}px)`;
    }
  }, []);

  const handleMouseUp = useCallback(() => {
    if (!isDraggingRef.current) return;
    isDraggingRef.current = false;
    setIsDragging(false);
    setPosition(currentPositionRef.current);
    onPositionChange?.(currentPositionRef.current);
  }, [onPositionChange]);

  useEffect(() => {
    if (!isDragging) return;
    document.addEventListener('mousemove', handleMouseMove, { passive: true });
    document.addEventListener('mouseup', handleMouseUp);
    return () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isDragging, handleMouseMove, handleMouseUp]);

  const setPanelState = useCallback(
    (newState: PanelState) => {
      setState(newState);
      onStateChange?.(newState);
    },
    [onStateChange],
  );

  const panelStyle = {
    transform: `translate(${position.x}px, ${position.y}px)`,
    '--panel-accent': accentColor,
  } as CSSProperties;

  return (
    <div ref={panelRef} className={`floating-panel floating-panel-${state}`} style={panelStyle}>
      <div className="panel-header" onMouseDown={handleMouseDown}>
        <div className="panel-title">
          <span className="panel-icon">{icon}</span>
          <span className="panel-title-text">{title}</span>
        </div>
        <div className="panel-controls">
          <button
            type="button"
            className="panel-control-btn minimize-btn"
            onClick={() => setPanelState(state === 'minimized' ? 'expanded' : 'minimized')}
            title={state === 'minimized' ? '展开' : '最小化'}
          >
            {state === 'minimized' ? '⤢' : '🗕'}
          </button>
          <button
            type="button"
            className="panel-control-btn expand-btn"
            onClick={() => setPanelState(state === 'expanded' ? 'collapsed' : 'expanded')}
            title={state === 'expanded' ? '折叠' : '展开'}
          >
            {state === 'expanded' ? '−' : '+'}
          </button>
          <button type="button" className="panel-control-btn close-btn" onClick={() => setPanelState('collapsed')} title="关闭">
            ×
          </button>
        </div>
      </div>
      {state !== 'minimized' && state === 'expanded' && (
        <div className="panel-content">
          <div className="panel-content-inner">{children}</div>
        </div>
      )}
    </div>
  );
}
