// Screenshots for review, at the window sizes the editor is built for: the empty editor (first run),
// a drawn path, a selected waypoint, the settings page, the telemetry page and the bench page. Saved to screenshots/.

import { join } from "node:path";
import { expect, test, type Page } from "@playwright/test";

const OUT = join(import.meta.dirname, "..", "screenshots");
const SIZES = [
  { width: 1440, height: 900 },
  { width: 1280, height: 800 },
];

async function fieldPoint(page: Page, x: number, y: number): Promise<{ x: number; y: number }> {
  const box = (await page.getByTestId("field").boundingBox())!;
  // through the view the field was last drawn with
  const [px, py] = await page.evaluate(
    ([fx, fy]) => (window as unknown as { pathEditorTest: { toCanvas(x: number, y: number): [number, number] } }).pathEditorTest.toCanvas(fx!, fy!),
    [x, y],
  );
  return { x: box.x + px, y: box.y + py };
}

async function dblField(page: Page, x: number, y: number): Promise<void> {
  const p = await fieldPoint(page, x, y);
  await page.mouse.dblclick(p.x, p.y);
}

async function settle(page: Page): Promise<void> {
  await page.mouse.move(2, 2);
  await page.waitForTimeout(400);
}

for (const size of SIZES) {
  test(`screens at ${size.width}x${size.height}`, async ({ page }) => {
    const shot = (name: string) => page.screenshot({ path: join(OUT, `${size.width}x${size.height}-${name}.png`) });
    await page.setViewportSize(size);
    await page.addInitScript(() => localStorage.removeItem("pathEditor.layout"));
    await page.goto("/");
    await expect(page.getByTestId("path-stats")).toContainText("no path yet");
    await settle(page);
    await shot("1-empty-editor");

    await page.getByTestId("quick-start-dismiss").click();
    // double-clicks: the start, the end, then a point after the last each time
    for (const [x, y] of [
      [-6.5, -1.5],
      [-4, -1.5],
      [-2, 1.5],
      [1, 0.5],
      [4, 2],
    ]) {
      await dblField(page, x!, y!);
    }
    await page.keyboard.press("Escape");
    await page.getByTestId("charts-toggle").click();
    await settle(page);
    await shot("2-drawn-path");

    await page.getByTestId("charts-toggle").click();

    // the sim preview: its switch and numbers in the playback bar, its card in the inspector
    await page.getByTestId("sim-toggle").click();
    await expect(page.getByTestId("sim-time")).toBeVisible();
    await page.getByTestId("inspector-tab-optimise").click();
    await page.getByTestId("play").click();
    await page.waitForTimeout(1500);
    await page.getByTestId("play").click();
    await settle(page);
    await shot("2b-sim-preview");
    await page.getByTestId("sim-toggle").click();
    await page.getByTestId("inspector-tab-path").click();

    await page.getByRole("list", { name: "Waypoints" }).getByRole("listitem").nth(2).getByRole("button").first().click();
    await settle(page);
    await shot("3-selected-waypoint");

    await page.getByTestId("settings").click();
    await settle(page);
    await shot("4-settings");

    await page.getByTestId("page-telemetry").click();
    await settle(page);
    await shot("5-telemetry");

    await page.getByTestId("page-bench").click();
    await expect(page.getByTestId("bench-benchmark")).toBeVisible();
    await settle(page);
    await shot("6-bench");
    await page.getByTestId("bench-charts").scrollIntoViewIfNeeded();
    await settle(page);
    await shot("6b-bench-charts");
    await page.getByRole("radio", { name: "Per target" }).click();
    await settle(page);
    await shot("6c-bench-per-target");
    await page.getByRole("radio", { name: "All targets" }).click();
    await page.getByTestId("bench-optimised").scrollIntoViewIfNeeded();
    await settle(page);
    await shot("6d-bench-optimised");
    await page.locator("#bench-detail").scrollIntoViewIfNeeded();
    await settle(page);
    await shot("6e-bench-detail");
  });
}
