// ── Service Worker — IR Remote PWA ──────────────────────────
const CACHE_NAME = 'ir-remote-v4';
const CACHE_IRDB = 'ir-remote-irdb-v1';

// Asset inti yang di-cache saat install
const CORE_ASSETS = [
  '/',
  '/index.html',
  '/manifest.json',
  '/icon-192.png',
  '/icon-512.png',
  '/ir_db/meta.json',
  '/ir_db/index.json',
  'https://cdn.jsdelivr.net/npm/@tabler/icons-webfont@3.44.0/tabler-icons.min.css',
];

// ── Install: cache asset inti ────────────────────────────────
self.addEventListener('install', e => {
  e.waitUntil(
    caches.open(CACHE_NAME).then(cache => cache.addAll(CORE_ASSETS))
  );
  self.skipWaiting();
});

// ── Activate: hapus cache lama ───────────────────────────────
self.addEventListener('activate', e => {
  e.waitUntil(
    caches.keys().then(keys =>
      Promise.all(
        keys.filter(k => k !== CACHE_NAME && k !== CACHE_IRDB)
            .map(k => caches.delete(k))
      )
    )
  );
  self.clients.claim();
});

// ── Fetch strategy ───────────────────────────────────────────
self.addEventListener('fetch', e => {
  const url = new URL(e.request.url);

  // File .ir dari flipper_irdb → cache-first (on-demand)
  if (url.pathname.startsWith('/flipper_irdb/')) {
    e.respondWith(
      caches.open(CACHE_IRDB).then(async cache => {
        const cached = await cache.match(e.request);
        if (cached) return cached;
        const res = await fetch(e.request);
        if (res.ok) cache.put(e.request, res.clone());
        return res;
      }).catch(() => new Response('Not found', { status: 404 }))
    );
    return;
  }

  // Asset inti & ir_db JSON → cache-first, fallback network
  if (
    url.pathname === '/' ||
    url.pathname.startsWith('/ir_db/') ||
    url.pathname === '/index.html' ||
    url.pathname === '/manifest.json' ||
    url.pathname.startsWith('/icon-') ||
    url.origin === 'https://cdn.jsdelivr.net'
  ) {
    e.respondWith(
      caches.match(e.request).then(cached => {
        const networkFetch = fetch(e.request).then(res => {
          if (res.ok) {
            caches.open(CACHE_NAME).then(c => c.put(e.request, res.clone()));
          }
          return res;
        });
        return cached || networkFetch;
      })
    );
    return;
  }

  // Lainnya → network only
  e.respondWith(fetch(e.request));
});
