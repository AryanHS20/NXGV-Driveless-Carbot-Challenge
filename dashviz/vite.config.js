import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import { viteSingleFile } from 'vite-plugin-singlefile';

// base './' keeps the bundle portable: served from /viz/ on the robot,
// opened from dist/ locally, or embedded in an iframe. Single-file output
// lets reviewers double-click dist/index.html with no server.
export default defineConfig({
  plugins: [react(), viteSingleFile()],
  base: './',
  build: {
    outDir: 'dist',
    emptyOutDir: true,
    chunkSizeWarningLimit: 1200,
  },
  server: {
    proxy: {
      '/data': 'http://192.168.137.161:8080',
      '/lidar_data': 'http://192.168.137.161:8080',
    },
  },
});
