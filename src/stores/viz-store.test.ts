import { describe, expect, it } from 'vitest';
import {
  DEFAULT_MODEL_OPACITY,
  DEFAULT_REFERENCE_TF_FRAME_SIZE,
  REFERENCE_TF_FRAME_SIZE_MAX,
  REFERENCE_TF_FRAME_SIZE_MIN,
  TARGET_GHOST_OPACITY_RATIO,
  computeGhostOpacity,
  useVizStore,
} from './viz-store';

describe('viz-store', () => {
  it('has expected defaults', () => {
    const state = useVizStore.getState();
    expect(state.showCollision).toBe(false);
    expect(state.showInertia).toBe(false);
    expect(state.modelOpacity).toBe(DEFAULT_MODEL_OPACITY);
    expect(state.showJointAxes).toBe(false);
    expect(state.referencePoseStyle).toBe('tf_frames');
    expect(state.referenceTfFrameSize).toBe(DEFAULT_REFERENCE_TF_FRAME_SIZE);
    expect(state.referenceTfShowChainLines).toBe(true);
  });

  it('clamps model opacity to [0.1, 1]', () => {
    useVizStore.getState().setModelOpacity(0.01);
    expect(useVizStore.getState().modelOpacity).toBe(0.1);
    useVizStore.getState().setModelOpacity(2);
    expect(useVizStore.getState().modelOpacity).toBe(1);
    useVizStore.getState().setModelOpacity(0.55);
    expect(useVizStore.getState().modelOpacity).toBe(0.55);
  });

  it('scales ghost opacity with model opacity', () => {
    expect(TARGET_GHOST_OPACITY_RATIO).toBeCloseTo(0.2 / 0.7);
    expect(computeGhostOpacity(DEFAULT_MODEL_OPACITY)).toBeCloseTo(0.2);
    expect(computeGhostOpacity(1)).toBeCloseTo(1 * TARGET_GHOST_OPACITY_RATIO);
    expect(computeGhostOpacity(0.1)).toBe(0.1);
  });

  it('clamps reference TF frame size to [0.03, 0.5]', () => {
    useVizStore.getState().setReferenceTfFrameSize(0.01);
    expect(useVizStore.getState().referenceTfFrameSize).toBe(REFERENCE_TF_FRAME_SIZE_MIN);
    useVizStore.getState().setReferenceTfFrameSize(1);
    expect(useVizStore.getState().referenceTfFrameSize).toBe(REFERENCE_TF_FRAME_SIZE_MAX);
    useVizStore.getState().setReferenceTfFrameSize(0.2);
    expect(useVizStore.getState().referenceTfFrameSize).toBe(0.2);
  });
});
