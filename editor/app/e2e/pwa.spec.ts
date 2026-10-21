// The installable app: the browser finds the manifest installable, the service worker caches the app,
// and the app opens with the network off.

import { expect, test } from "@playwright/test";

// the app skips the service worker under automation (navigator.webdriver); this test turns that off
test.use({ launchOptions: { args: ["--disable-blink-features=AutomationControlled"] } });

test("installable, and opens offline once visited", async ({ page, context }) => {
  await page.goto("/");
  await expect(page.getByTestId("path-stats")).toContainText("no path yet"); // the app is up (a new, empty path)
  expect(await page.evaluate(() => navigator.webdriver)).toBe(false);

  // the browser's own verdict on the manifest and icons
  const cdp = await context.newCDPSession(page);
  type Manifest = { errors: unknown[]; url: string };
  await expect.poll(async () => ((await cdp.send("Page.getAppManifest")) as Manifest).url).toContain("/manifest.webmanifest");
  const manifest = (await cdp.send("Page.getAppManifest")) as Manifest;
  expect(manifest.errors).toEqual([]);
  const installable = (await cdp.send("Page.getInstallabilityErrors")) as { installabilityErrors: { errorId: string }[] };
  // (a test's browser profile is incognito, where nothing installs: the only error allowed)
  expect(installable.installabilityErrors.map((e) => e.errorId).filter((id) => id !== "in-incognito")).toEqual([]);

  // the worker installs and takes over; then the network goes away
  await page.evaluate(() => navigator.serviceWorker.ready.then(() => undefined));
  await expect.poll(() => page.evaluate(async () => (await caches.keys()).some((k) => k.startsWith("path-editor-")))).toBe(true);
  await context.setOffline(true);
  await page.reload();
  await expect(page.getByTestId("path-stats")).toContainText("no path yet"); // the app is up (a new, empty path)
  await expect(page.getByTestId("tool-select")).toBeVisible();
  await context.setOffline(false);
});
