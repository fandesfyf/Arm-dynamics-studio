import type { ReferencePoseStyle } from '../../stores/viz-store';

/** Ghost mesh only — no TF axes. */
export function overlayShowsGhostMesh(style: ReferencePoseStyle): boolean {
  return style === 'ghost';
}

/** TF axes + chain lines only — no ghost mesh. */
export function overlayShowsTfMarkers(style: ReferencePoseStyle): boolean {
  return style === 'tf_frames';
}

export function overlayIsActive(style: ReferencePoseStyle, jointCount: number): boolean {
  return style !== 'off' && jointCount > 0;
}
