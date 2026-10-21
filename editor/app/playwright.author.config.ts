import { defineConfig } from "@playwright/test";
import base from "./playwright.config.ts";

// Phase 6 authoring (e2e-author): writes the end-to-end suite to <repo>/e2e/out. Run with
// `npm run e2e:author`, or the whole chain with `npm run e2e:full`.
export default defineConfig({ ...base, testDir: "e2e-author", timeout: 300_000, use: { ...base.use, actionTimeout: 10_000, screenshot: "only-on-failure" } });
