import { describe, expect, it } from 'vitest';
import {
  overlayIsActive,
  overlayShowsGhostMesh,
  overlayShowsTfMarkers,
} from './reference-overlay-mode';

describe('reference-overlay-mode', () => {
  it('ghost and tf_frames are mutually exclusive', () => {
    for (const style of ['ghost', 'tf_frames', 'off'] as const) {
      expect(overlayShowsGhostMesh(style) && overlayShowsTfMarkers(style)).toBe(false);
    }
    expect(overlayShowsGhostMesh('ghost')).toBe(true);
    expect(overlayShowsTfMarkers('ghost')).toBe(false);
    expect(overlayShowsGhostMesh('tf_frames')).toBe(false);
    expect(overlayShowsTfMarkers('tf_frames')).toBe(true);
  });

  it('overlayIsActive requires joints and non-off style', () => {
    expect(overlayIsActive('off', 17)).toBe(false);
    expect(overlayIsActive('ghost', 0)).toBe(false);
    expect(overlayIsActive('tf_frames', 17)).toBe(true);
  });
});
