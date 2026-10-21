import { defineConfig } from "@playwright/test";
import base from "./playwright.config.ts";

// Review screenshots of the app at two window sizes, into screenshots/: `npm run screenshots`.
export default defineConfig({ ...base, testDir: "e2e-screens", timeout: 120_000 });
