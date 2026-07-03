import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import { bipedAssetsPlugin } from './vite-biped-assets';

export default defineConfig({
  plugins: [react(), bipedAssetsPlugin()],
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
