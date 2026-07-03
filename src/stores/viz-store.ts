import { create } from 'zustand';

export type ReferencePoseStyle = 'tf_frames' | 'ghost' | 'off';

export interface VizState {
  showCollision: boolean;
  showInertia: boolean;
  modelOpacity: number;
  showJointAxes: boolean;
  referencePoseStyle: ReferencePoseStyle;
  referenceTfFrameSize: number;
  referenceTfShowChainLines: boolean;

  setShowCollision: (value: boolean) => void;
  setShowInertia: (value: boolean) => void;
  setModelOpacity: (value: number) => void;
  setShowJointAxes: (value: boolean) => void;
  setReferencePoseStyle: (style: ReferencePoseStyle) => void;
  setReferenceTfFrameSize: (size: number) => void;
  setReferenceTfShowChainLines: (show: boolean) => void;
}

function clampOpacity(value: number): number {
  return Math.min(1, Math.max(0.1, value));
}

export const REFERENCE_TF_FRAME_SIZE_MIN = 0.03;
export const REFERENCE_TF_FRAME_SIZE_MAX = 0.5;
export const DEFAULT_REFERENCE_TF_FRAME_SIZE = 0.12;

function clampReferenceTfFrameSize(value: number): number {
  return Math.min(
    REFERENCE_TF_FRAME_SIZE_MAX,
    Math.max(REFERENCE_TF_FRAME_SIZE_MIN, value),
  );
}

/** Default main robot opacity (matches viz-store initial state). */
export const DEFAULT_MODEL_OPACITY = 0.7;

/** At default opacity 0.7, preview ghost renders at 0.2. */
export const TARGET_GHOST_OPACITY_RATIO = 0.2 / DEFAULT_MODEL_OPACITY;

export function computeGhostOpacity(modelOpacity: number): number {
  return clampOpacity(modelOpacity * TARGET_GHOST_OPACITY_RATIO);
}

export const useVizStore = create<VizState>((set) => ({
  showCollision: false,
  showInertia: false,
  modelOpacity: DEFAULT_MODEL_OPACITY,
  showJointAxes: false,
  referencePoseStyle: 'tf_frames' as ReferencePoseStyle,
  referenceTfFrameSize: DEFAULT_REFERENCE_TF_FRAME_SIZE,
  referenceTfShowChainLines: true,

  setShowCollision: (showCollision) => set({ showCollision }),
  setShowInertia: (showInertia) => set({ showInertia }),
  setModelOpacity: (modelOpacity) => set({ modelOpacity: clampOpacity(modelOpacity) }),
  setShowJointAxes: (showJointAxes) => set({ showJointAxes }),
  setReferencePoseStyle: (referencePoseStyle) => set({ referencePoseStyle }),
  setReferenceTfFrameSize: (referenceTfFrameSize) =>
    set({ referenceTfFrameSize: clampReferenceTfFrameSize(referenceTfFrameSize) }),
  setReferenceTfShowChainLines: (referenceTfShowChainLines) => set({ referenceTfShowChainLines }),
}));
