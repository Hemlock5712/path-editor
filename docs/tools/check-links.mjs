// Checks every internal link and anchor in the built site (run `npm run build` first).
// Usage: node tools/check-links.mjs
import fs from 'node:fs';
import path from 'node:path';

const dist = 'dist';
const base = (process.env.DOCS_BASE ?? '/docs').replace(/\/$/, '');
const pages = [];
(function walk(dir) {
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    const p = path.join(dir, entry.name);
    if (entry.isDirectory()) walk(p);
    else if (entry.name.endsWith('.html')) pages.push(p);
  }
})(dist);

const html = new Map(pages.map((p) => [p, fs.readFileSync(p, 'utf8')]));
const ids = new Map(pages.map((p) => [p, new Set([...html.get(p).matchAll(/ id="([^"]+)"/g)].map((m) => m[1]))]));
const show = (p) => p.split(path.sep).join('/');

let bad = 0;
const seen = new Set();
for (const p of pages) {
  for (const m of html.get(p).matchAll(/href="(\/[^"]*)"/g)) {
    const [full, fragment] = m[1].split('#');
    if (full === '/') continue; // the editor, outside this site
    if (base && !full.startsWith(base + '/')) {
      console.log('NO BASE ', show(p) + ' -> ' + m[1]);
      bad++;
      continue;
    }
    const url = full.slice(base.length);
    if (/^\/(_astro|pagefind)\//.test(url) || /\.(svg|xml|png|webp)$/.test(url)) continue;
    const target = path.join(dist, url, 'index.html');
    const key = show(p) + ' -> ' + m[1];
    if (seen.has(key)) continue;
    seen.add(key);
    if (!fs.existsSync(target)) {
      console.log('MISSING ', key);
      bad++;
    } else if (fragment && !ids.get(target)?.has(fragment)) {
      console.log('ANCHOR  ', key);
      bad++;
    }
  }
}
console.log(`${pages.length} pages, ${bad} broken link(s)`);
process.exit(bad ? 1 : 0);
