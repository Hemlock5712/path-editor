import { defineConfig } from "@playwright/test";
import base from "./playwright.config.ts";

// The Telemetry page against lib's real ntcore harness (it starts the harness through gradle, so it is
// slow and needs the WPILib 2027 install): npm run e2e:live. Saves review screenshots to screenshots/.
export default defineConfig({ ...base, testDir: "e2e-live", timeout: 300_000, use: { ...base.use, viewport: { width: 1440, height: 900 } } });
