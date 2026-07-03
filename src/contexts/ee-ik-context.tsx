import { createContext, useContext, type ReactNode } from 'react';
import type { Vec3 } from '../core/trajectory';

export interface EeIkLiveResult {
  converged: boolean;
  jointAngles: number[];
  message?: string;
}

export interface EeIkApi {
  solveEeIkLive: (
    target: Vec3,
    opts?: {
      liveDrag?: boolean;
      dragEnd?: boolean;
      targetSceneWorld?: [number, number, number];
      targetSceneQuaternion?: [number, number, number, number];
    },
  ) => Promise<EeIkLiveResult>;
  resetReferencePose: () => void;
  /** Gizmo drag release: joint-space interpolation when sim was already running. */
  onEeDragCommit?: (opts?: { simWasRunning?: boolean }) => void | Promise<void>;
}

const EeIkContext = createContext<EeIkApi | null>(null);

export function EeIkProvider({ value, children }: { value: EeIkApi; children: ReactNode }) {
  return <EeIkContext.Provider value={value}>{children}</EeIkContext.Provider>;
}

export function useEeIk(): EeIkApi {
  const ctx = useContext(EeIkContext);
  if (!ctx) {
    throw new Error('useEeIk must be used within EeIkProvider');
  }
  return ctx;
}

export function useEeIkOptional(): EeIkApi | null {
  return useContext(EeIkContext);
}
