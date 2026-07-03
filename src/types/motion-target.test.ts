import { describe, expect, it } from 'vitest';
import { resolveInterpProfile } from '../types/motion-target';

describe('resolveInterpProfile', () => {
  it('uses cubic when profile is cubic and at least 3 waypoints', () => {
    expect(resolveInterpProfile('cubic', 3)).toBe('cubic');
    expect(resolveInterpProfile('cubic', 4)).toBe('cubic');
  });

  it('falls back to linear when fewer than 3 waypoints', () => {
    expect(resolveInterpProfile('cubic', 2)).toBe('linear');
    expect(resolveInterpProfile('cubic', 1)).toBe('linear');
  });

  it('respects linear profile', () => {
    expect(resolveInterpProfile('linear', 5)).toBe('linear');
  });
});
