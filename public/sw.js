// ── Service Worker — IR Remote PWA ──────────────────────────
const CACHE_NAME = 'ir-remote-v11';
const CACHE_IRDB = 'ir-remote-irdb-v1';

// FIX: path dulu di-hardcode absolut dari root ("/ir_db/...") — cuma benar
// kalau di-deploy di root domain (Vercel). Di GitHub Pages app-nya jalan di
// subpath ("/ir-remote-app/..."), jadi semua path dihitung relatif terhadap
// scope registrasi SW ini sendiri supaya portable di hosting manapun.
const BASE = new URL(self.registration.scope).pathname;

// Asset inti yang di-cache saat install
const CORE_ASSETS = [
  BASE,
  BASE + 'index.html',
  BASE + 'manifest.json',
  BASE + 'icon-192.png',
  BASE + 'icon-512.png',
  BASE + 'ir_db/meta.json',
  BASE + 'ir_db/index.json',
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
  if (url.pathname.startsWith(BASE + 'flipper_irdb/')) {
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

  // FIX: app shell (index.html/root/manifest) dulu cache-first — begitu
  // ke-cache sekali, HP user SELALU pakai versi itu sampai... kapan aja,
  // gak ada mekanisme yang memaksa refresh selama sw.js sendiri gak
  // berubah byte-nya (dan sw.js jarang ikut ke-edit tiap kali index.html
  // di-update). Ini akar masalah kenapa fix yang udah di-push berkali-kali
  // "kelihatan belum jalan" di HP user — bukan fix-nya gagal, tapi App
  // shell-nya beneran masih yang lama. Sekarang network-first: coba versi
  // terbaru dulu tiap load (kalau online), baru fallback ke cache kalau
  // offline. ir_db/icon tetap cache-first karena jarang berubah & besar.
  if (
    url.pathname === BASE ||
    url.pathname === BASE + 'index.html' ||
    url.pathname === BASE + 'manifest.json'
  ) {
    e.respondWith(
      fetch(e.request).then(res => {
        if (res.ok) caches.open(CACHE_NAME).then(c => c.put(e.request, res.clone()));
        return res;
      }).catch(() => caches.match(e.request))
    );
    return;
  }

  // Asset yang jarang berubah (data IR, icon, font) → cache-first tetap aman
  if (
    url.pathname.startsWith(BASE + 'ir_db/') ||
    url.pathname.startsWith(BASE + 'icon-') ||
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
