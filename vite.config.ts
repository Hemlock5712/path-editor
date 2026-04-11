import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import tailwindcss from '@tailwindcss/vite';
import { VitePWA } from 'vite-plugin-pwa';

export default defineConfig({
  plugins: [
    react(),
    tailwindcss(),
    VitePWA({
      injectRegister: false,
      registerType: 'autoUpdate',
      includeAssets: ['field-2026.png'],
      manifest: {
        name: 'FRC Path Editor',
        short_name: 'Path Editor',
        description:
          'Create and tune FRC robot paths directly in your browser.',
        start_url: '/',
        scope: '/',
        display: 'standalone',
        background_color: '#0f172a',
        theme_color: '#0f172a',
        icons: [
          {
            src: '/field-2026.png',
            sizes: '512x512',
            type: 'image/png',
            purpose: 'any',
          },
          {
            src: '/field-2026.png',
            sizes: '512x512',
            type: 'image/png',
            purpose: 'maskable',
          },
        ],
      },
      workbox: {
        navigateFallback: '/index.html',
        runtimeCaching: [
          {
            urlPattern: ({ url, request }) =>
              url.origin === self.location.origin &&
              request.method === 'GET' &&
              request.destination !== 'document',
            handler: 'NetworkFirst',
            options: {
              cacheName: 'path-editor-runtime-assets',
            },
          },
        ],
      },
    }),
  ],
});
