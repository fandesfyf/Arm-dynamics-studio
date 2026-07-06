import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import { bipedAssetsPlugin } from './vite-biped-assets';
import { debugDumpPlugin } from './vite-debug-dump';

export default defineConfig({
  plugins: [react(), bipedAssetsPlugin(), debugDumpPlugin()],
  assetsInclude: ['**/*.wasm'],
  optimizeDeps: {
    exclude: ['@mujoco/mujoco'],
    include: ['pinocchio-js'],
    needsInterop: ['pinocchio-js'],
  },
  build: {
    commonjsOptions: {
      transformMixedEsModules: true,
    },
    chunkSizeWarningLimit: 1600,
    rollupOptions: {
      output: {
        manualChunks: {
          mujoco: ['@mujoco/mujoco'],
          three: ['three', '@react-three/fiber', '@react-three/drei'],
        },
      },
    },
  },
  server: {
    host: true,
    headers: {
      // 仅在使用 @mujoco/mujoco/mt 时需要：
      // 'Cross-Origin-Opener-Policy': 'same-origin',
      // 'Cross-Origin-Embedder-Policy': 'require-corp',
    },
  },
});
