// A short recording of the editor for the documentation site: a path is drawn point by point, then the sim
// preview drives it. Saved to docs/public/clips/draw-path.webm.
//
//   npx playwright test --config playwright.screens.config.ts clip

import { join } from "node:path";
import { expect, test, type Page } from "@playwright/test";

const OUT = join(import.meta.dirname, "..", "..", "..", "docs", "public", "clips", "draw-path.webm");
const SIZE = { width: 1280, height: 720 };

test.use({ viewport: SIZE, video: { mode: "on", size: SIZE } });

async function fieldPoint(page: Page, x: number, y: number): Promise<{ x: number; y: number }> {
  const box = (await page.getByTestId("field").boundingBox())!;
  const [px, py] = await page.evaluate(
    ([fx, fy]) => (window as unknown as { pathEditorTest: { toCanvas(x: number, y: number): [number, number] } }).pathEditorTest.toCanvas(fx!, fy!),
    [x, y],
  );
  return { x: box.x + px, y: box.y + py };
}

test("the documentation clip: draw a path and preview it", async ({ page }) => {
  await page.addInitScript(() => {
    localStorage.removeItem("pathEditor.layout");
    localStorage.setItem("pathEditor.quickStartSeen", "1");
  });
  await page.goto("/");
  await expect(page.getByTestId("path-stats")).toContainText("no path yet");
  await page.waitForTimeout(700);

  // the path the documentation's animations drive, placed at a pace a reader can follow
  for (const [x, y] of [
    [6.9, -3.0],
    [5.3, -1.6],
    [3.65, -1.52],
    [2.3, -2.3],
    [1.2, -0.9],
  ]) {
    const p = await fieldPoint(page, x!, y!);
    await page.mouse.move(p.x, p.y, { steps: 24 });
    await page.waitForTimeout(180);
    await page.mouse.dblclick(p.x, p.y);
    await page.waitForTimeout(420);
  }
  await page.keyboard.press("Escape");
  await page.waitForTimeout(600);

  // the sim preview drives it
  await page.getByTestId("sim-toggle").click();
  await expect(page.getByTestId("sim-time")).toBeVisible();
  await page.waitForTimeout(500);
  await page.getByTestId("play").click();
  await page.waitForTimeout(4200);

  await page.close();
  await page.video()!.saveAs(OUT);
});
