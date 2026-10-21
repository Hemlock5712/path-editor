// The deployed editor is served over https; the robot speaks ws://. This serves the production build at
// the real site's origin (every request answered from dist/, nothing goes to the network) to check what
// the Live panel tells the user there, before and after the browser's Local Network Access permission.

import { readFileSync } from "node:fs";
import { extname, join } from "node:path";
import { expect, test } from "@playwright/test";

const ORIGIN = "https://paths.frc5712.com";
const DIST = join(import.meta.dirname, "..", "dist");
const TYPES: Record<string, string> = { ".html": "text/html", ".js": "text/javascript", ".css": "text/css", ".json": "application/json", ".png": "image/png" };

test.beforeEach(async ({ page }) => {
  await page.addInitScript(() => localStorage.setItem("pathEditor.quickStartSeen", "1"));
  await page.route(`${ORIGIN}/**`, async (route) => {
    const path = new URL(route.request().url()).pathname;
    const file = path === "/" || !extname(path) ? "index.html" : path.slice(1);
    try {
      await route.fulfill({ body: readFileSync(join(DIST, file)), contentType: TYPES[extname(file)] ?? "application/octet-stream" });
    } catch {
      await route.fulfill({ status: 404, body: "" });
    }
  });
});

async function openLive(page: import("@playwright/test").Page) {
  await page.goto(`${ORIGIN}/`);
  await expect(page.getByTestId("path-stats")).toContainText(" s,");
  await page.getByTestId("page-telemetry").click();
}

test("from the https site, the Live panel says to allow local network access", async ({ page }) => {
  await openLive(page);
  expect(await page.evaluate(() => isSecureContext && location.protocol)).toBe("https:");
  const note = page.getByTestId("live-https");
  // localhost (a simulation): this computer
  await expect(note).toContainText("reach apps on this computer: allow it");
  await page.getByTestId("live-address").fill("5712");
  await page.getByTestId("live-address").press("Enter");
  await expect(note).toContainText("reach devices on your local network: allow it");
  await expect(note).toContainText("npm --prefix editor/app run dev");
  await page.getByTestId("live-address").fill("roborio-5712-frc");
  await page.getByTestId("live-address").press("Enter");
  await expect(note).toContainText("blocks ws://");
});

test("once local network access is allowed, the note goes", async ({ page, context, browser }) => {
  const session = await context.newCDPSession(page);
  const { targetInfo } = (await session.send("Target.getTargetInfo")) as { targetInfo: { browserContextId: string } };
  const cdp = await browser.newBrowserCDPSession();
  await cdp.send("Browser.setPermission", {
    permission: { name: "local-network" },
    setting: "granted",
    origin: ORIGIN,
    browserContextId: targetInfo.browserContextId,
  });
  await openLive(page);
  await page.getByTestId("live-address").fill("5712");
  await page.getByTestId("live-address").press("Enter");
  await expect(page.getByText("Connects to 10.57.12.2")).toBeVisible();
  await expect(page.getByTestId("live-https")).toHaveCount(0);
});
