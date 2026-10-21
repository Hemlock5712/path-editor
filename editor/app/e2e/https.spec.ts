// The deployed editor is served over https; the robot speaks ws://. This serves the production build at
// the real site's origin (every request answered by the test's own preview server; nothing goes to the
// network) to check what the Telemetry page tells the user there, before and after the browser's Local
// Network Access permission.

import { extname } from "node:path";
import { expect, test } from "@playwright/test";

const ORIGIN = "https://paths.frc5712.com";

test.beforeEach(async ({ page, baseURL }) => {
  await page.addInitScript(() => localStorage.setItem("pathEditor.quickStartSeen", "1"));
  // every request to the site is answered by this run's own preview server (the build under test)
  await page.route(`${ORIGIN}/**`, async (route) => {
    const path = new URL(route.request().url()).pathname;
    const response = await route.fetch({ url: `${baseURL}${path === "/" || !extname(path) ? "/" : path}` });
    await route.fulfill({ response });
  });
});

// Waits are for state, with room for a cold, busy machine (the first run on a fresh clone builds the app
// and compiles every module on first load).
const SLOW = { timeout: 30_000 };

async function openLive(page: import("@playwright/test").Page) {
  await page.goto(`${ORIGIN}/`, { waitUntil: "domcontentloaded" });
  await expect(page.getByTestId("path-stats")).toContainText(" s,", SLOW);
  await page.getByTestId("page-telemetry").click();
  await expect(page.getByTestId("live-address")).toBeVisible(SLOW);
}

test("from the https site, the Live panel says to allow local network access", async ({ page }) => {
  await openLive(page);
  expect(await page.evaluate(() => isSecureContext && location.protocol)).toBe("https:");
  const note = page.getByTestId("live-https");
  // localhost (a simulation): this computer
  await expect(note).toContainText("reach apps on this computer: allow it", SLOW);
  await page.getByTestId("live-address").fill("5712");
  await page.getByTestId("live-address").press("Enter");
  await expect(note).toContainText("reach devices on your local network: allow it", SLOW);
  await expect(note).toContainText("npm --prefix editor/app run dev");
  await page.getByTestId("live-address").fill("roborio-5712-frc");
  await page.getByTestId("live-address").press("Enter");
  await expect(note).toContainText("blocks ws://", SLOW);
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
  await expect(page.getByText("Connects to 10.57.12.2")).toBeVisible(SLOW);
  // the permission check is asynchronous: wait for its answer, not for time
  await expect(page.getByTestId("live-https")).toHaveCount(0, SLOW);
});
