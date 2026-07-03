/// <reference types="vite/client" />

declare module 'pinocchio-js' {
  type PinocchioFactory = (opts?: {
    locateFile?: (path: string) => string;
  }) => Promise<Record<string, unknown>>;
  const loadPinocchio: PinocchioFactory;
  export default loadPinocchio;
}
