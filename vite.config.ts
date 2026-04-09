import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import tailwindcss from '@tailwindcss/vite';
import { pwaPlugin } from './vite.pwa-plugin';

export default defineConfig({
  plugins: [react(), tailwindcss(), pwaPlugin()],
});
