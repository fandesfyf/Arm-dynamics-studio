import { useCallback, useRef } from 'react';

export type DockResizeAxis = 'vertical' | 'horizontal';
export type DockResizeEdge = 'start' | 'end';

interface DockResizeHandleProps {
  axis: DockResizeAxis;
  edge: DockResizeEdge;
  onDragStart?: () => void;
  onDrag: (deltaPx: number) => void;
  onDragEnd?: () => void;
}

function deltaForMove(
  axis: DockResizeAxis,
  edge: DockResizeEdge,
  start: number,
  current: number,
): number {
  const raw = current - start;
  if (axis === 'vertical') {
    return edge === 'end' ? raw : -raw;
  }
  return edge === 'start' ? -raw : raw;
}

export function DockResizeHandle({ axis, edge, onDragStart, onDrag, onDragEnd }: DockResizeHandleProps) {
  const startRef = useRef(0);
  const isDraggingRef = useRef(false);

  const handlePointerDown = useCallback(
    (e: React.PointerEvent<HTMLDivElement>) => {
      e.preventDefault();
      e.stopPropagation();

      isDraggingRef.current = true;
      startRef.current = axis === 'vertical' ? e.clientX : e.clientY;
      onDragStart?.();

      const onPointerMove = (ev: PointerEvent) => {
        const current = axis === 'vertical' ? ev.clientX : ev.clientY;
        onDrag(deltaForMove(axis, edge, startRef.current, current));
      };

      const onPointerUp = () => {
        document.removeEventListener('pointermove', onPointerMove);
        document.removeEventListener('pointerup', onPointerUp);
        document.body.style.cursor = '';
        document.body.style.userSelect = '';
        isDraggingRef.current = false;
        onDragEnd?.();
      };

      document.body.style.cursor = axis === 'vertical' ? 'col-resize' : 'row-resize';
      document.body.style.userSelect = 'none';
      document.addEventListener('pointermove', onPointerMove);
      document.addEventListener('pointerup', onPointerUp);
    },
    [axis, edge, onDragStart, onDrag, onDragEnd],
  );

  const className = [
    'dock-resize-handle',
    axis === 'vertical' ? 'dock-resize-handle--vertical' : 'dock-resize-handle--horizontal',
    edge === 'start' ? 'dock-resize-handle--start' : 'dock-resize-handle--end',
  ].join(' ');

  return (
    <div
      className={className}
      role="separator"
      aria-orientation={axis === 'vertical' ? 'vertical' : 'horizontal'}
      onPointerDown={handlePointerDown}
    />
  );
}
