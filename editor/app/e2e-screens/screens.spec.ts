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
  const scale = Math.min((box.width - 48) / 16.54, (box.height - 48) / 8.07);
  return { x: box.x + box.width / 2 + x * scale, y: box.y + box.height / 2 - y * scale };
}

async function clickField(page: Page, x: number, y: number): Promise<void> {
  const p = await fieldPoint(page, x, y);
  await page.mouse.click(p.x, p.y);
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
    await expect(page.getByTestId("path-stats")).toContainText(" s,");
    await settle(page);
    await shot("1-empty-editor");

    await page.getByTestId("quick-start-dismiss").click();
    await page.keyboard.press("w");
    for (const [x, y] of [
      [-2, 1.5],
      [1, 0.5],
      [4, 2],
    ]) {
      await clickField(page, x!, y!);
    }
    await page.keyboard.press("Enter");
    await page.keyboard.press("Escape");
    await page.getByTestId("charts-toggle").click();
    await settle(page);
    await shot("2-drawn-path");

    await page.getByTestId("charts-toggle").click();
    await page.getByRole("list", { name: "Waypoints" }).getByRole("button").nth(2).click();
    await settle(page);
    await shot("3-selected-waypoint");

    await page.getByTestId("settings").click();
    await settle(page);
    await shot("4-settings");

    await page.getByTestId("page-telemetry").click();
    await settle(page);
    await shot("5-telemetry");

    await page.getByTestId("page-bench").click();
    await expect(page.getByTestId("bench-scoreboard")).toBeVisible();
    await settle(page);
    await shot("6-bench");
  });
}
