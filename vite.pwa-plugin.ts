import type { IndexHtmlTransformContext, Plugin } from 'vite';

const CACHE_NAME = 'path-editor-v2';

const manifest = {
  name: 'FRC Path Editor',
  short_name: 'Path Editor',
  description: 'Create and tune FRC robot paths directly in your browser.',
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
};

const buildServiceWorker = (assetPaths: string[]) => `
const CACHE_NAME = '${CACHE_NAME}';
const APP_SHELL_CACHE = ${JSON.stringify(assetPaths, null, 2)};

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) => cache.addAll(APP_SHELL_CACHE))
  );
  self.skipWaiting();
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches
      .keys()
      .then((keys) =>
        Promise.all(
          keys
            .filter((key) => key !== CACHE_NAME)
            .map((key) => caches.delete(key))
        )
      )
      .then(() => self.clients.claim())
  );
});

self.addEventListener('fetch', (event) => {
  const { request } = event;
  const url = new URL(request.url);

  if (request.mode === 'navigate') {
    event.respondWith(
      fetch(request)
        .then((response) => {
          const responseClone = response.clone();
          caches.open(CACHE_NAME).then((cache) => cache.put('/index.html', responseClone));
          return response;
        })
        .catch(async () => {
          const cachedResponse = await caches.match('/index.html');
          return cachedResponse || Response.error();
        })
    );
    return;
  }

  if (url.origin === self.location.origin && request.method === 'GET') {
    event.respondWith(
      caches.match(request).then((cachedResponse) => {
        if (cachedResponse) {
          return cachedResponse;
        }

        return fetch(request).then((networkResponse) => {
          if (networkResponse.ok) {
            const responseClone = networkResponse.clone();
            caches.open(CACHE_NAME).then((cache) => cache.put(request, responseClone));
          }
          return networkResponse;
        });
      })
    );
  }
});
`;

const pwaTags = [
  {
    tag: 'meta',
    attrs: { name: 'theme-color', content: '#0f172a' },
    injectTo: 'head' as const,
  },
  {
    tag: 'link',
    attrs: { rel: 'manifest', href: '/manifest.webmanifest' },
    injectTo: 'head' as const,
  },
  {
    tag: 'link',
    attrs: { rel: 'apple-touch-icon', href: '/field-2026.png' },
    injectTo: 'head' as const,
  },
];

export const pwaPlugin = (): Plugin => {
  return {
    name: 'path-editor-pwa',
    transformIndexHtml(html: string, ctx?: IndexHtmlTransformContext) {
      if (!ctx || !ctx.bundle) {
        return {
          html,
          tags: pwaTags,
        };
      }

      return {
        html,
        tags: pwaTags,
      };
    },
    generateBundle(_outputOptions, bundle) {
      const bundleAssets = Object.values(bundle)
        .map((chunkOrAsset) => `/${chunkOrAsset.fileName}`)
        .filter((fileName) => fileName !== '/sw.js' && fileName !== '/manifest.webmanifest');

      const appShellAssets = [
        '/',
        '/index.html',
        '/field-2026.png',
        ...bundleAssets,
      ];

      this.emitFile({
        type: 'asset',
        fileName: 'manifest.webmanifest',
        source: `${JSON.stringify(manifest, null, 2)}\n`,
      });

      this.emitFile({
        type: 'asset',
        fileName: 'sw.js',
        source: buildServiceWorker(appShellAssets),
      });
    },
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        if (req.url === '/manifest.webmanifest') {
          res.setHeader('Content-Type', 'application/manifest+json');
          res.end(`${JSON.stringify(manifest, null, 2)}\n`);
          return;
        }

        if (req.url === '/sw.js') {
          res.setHeader('Content-Type', 'application/javascript');
          res.end(buildServiceWorker(['/', '/index.html', '/field-2026.png']));
          return;
        }

        next();
      });
    },
  };
};
