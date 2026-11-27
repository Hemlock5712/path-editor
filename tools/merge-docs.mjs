// Copies the built documentation site (docs/dist) into the editor's build (editor/app/dist/docs), so one
// Vercel deployment serves the editor at / and the docs at /docs. Run after both builds (see vercel.json).
import { cpSync, existsSync, rmSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const from = join(root, "docs", "dist");
const to = join(root, "editor", "app", "dist", "docs");

if (!existsSync(join(from, "index.html"))) {
  console.error("docs/dist is missing: run `npm --prefix docs run build` first");
  process.exit(1);
}
if (!existsSync(join(root, "editor", "app", "dist", "index.html"))) {
  console.error("editor/app/dist is missing: run `npm --prefix editor/app run build` first");
  process.exit(1);
}
rmSync(to, { recursive: true, force: true });
cpSync(from, to, { recursive: true });
console.log(`docs copied to ${to}`);
