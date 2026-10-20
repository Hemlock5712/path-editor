import { defineConfig } from "@playwright/test";

// Runs against the production build (vite preview). Uses the installed Edge so no browser download is
// needed; set PW_CHANNEL=chromium (after `npx playwright install chromium`) to use Playwright's own.
export default defineConfig({
  testDir: "e2e",
  timeout: 60_000,
  use: {
    baseURL: "http://localhost:5713",
    channel: process.env.PW_CHANNEL ?? "msedge",
    viewport: { width: 1280, height: 800 },
  },
  webServer: {
    command: "npm run build && npx vite preview --port 5713 --strictPort",
    url: "http://localhost:5713",
    reuseExistingServer: !process.env.CI,
    timeout: 180_000,
  },
});
