import { createHash } from "node:crypto";
import { defineConfig } from "@playwright/test";

// Runs against the production build (vite preview). Uses the installed Edge so no browser download is
// needed; set PW_CHANNEL=chromium (after `npx playwright install chromium`) to use Playwright's own.
//
// Each checkout gets its own port (from its path; PW_PORT overrides), and every run builds and serves
// its own app: two checkouts or worktrees testing at once never share, or reuse, each other's server.
const PORT = Number(process.env.PW_PORT ?? 5713 + 2 * (createHash("sha1").update(import.meta.dirname).digest().readUInt16BE(0) % 400));
const URL = `http://localhost:${PORT}`;

export default defineConfig({
  testDir: "e2e",
  timeout: 60_000,
  use: {
    baseURL: URL,
    channel: process.env.PW_CHANNEL ?? "msedge",
    viewport: { width: 1280, height: 800 },
  },
  webServer: {
    command: `npm run build && npx vite preview --port ${PORT} --strictPort`,
    url: URL,
    reuseExistingServer: false,
    timeout: 180_000,
  },
});
