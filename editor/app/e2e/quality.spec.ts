// Regressions from the quality pass: things a new student ran into.

import { expect, test, type Page } from "@playwright/test";

const doc = (page: Page) =>
  page.evaluate(() => (window as unknown as { pathEditorTest: { doc(): { file: { waypoints: { x: number; y: number; headingDegrees?: number }[] } | null; selection: unknown } } }).pathEditorTest.doc());

async function fieldPoint(page: Page, x: number, y: number): Promise<{ x: number; y: number }> {
  const box = (await page.getByTestId("field").boundingBox())!;
  const scale = Math.min((box.width - 48) / 16.54, (box.height - 48) / 8.07);
  return { x: box.x + box.width / 2 + x * scale, y: box.y + box.height / 2 - y * scale };
}

async function dblField(page: Page, x: number, y: number): Promise<void> {
  const p = await fieldPoint(page, x, y);
  await page.keyboard.down("Shift");
  await page.mouse.dblclick(p.x, p.y);
  await page.keyboard.up("Shift");
}

async function draw(page: Page, points: [number, number][]): Promise<void> {
  await page.goto("/");
  let last = "";
  await expect
    .poll(async () => {
      const box = JSON.stringify(await page.getByTestId("field").boundingBox());
      const same = box === last;
      last = box;
      return same;
    }, { intervals: [150] })
    .toBe(true);
  for (const [x, y] of points) await dblField(page, x, y);
}

test.beforeEach(async ({ page }) => {
  await page.addInitScript(() => localStorage.setItem("pathEditor.quickStartSeen", "1"));
});

test("a loop auto: double-clicking the start's robot puts the end there", async ({ page }) => {
  await draw(page, [
    [-6, -2],
    [-2, -2.5],
    [-1, 1.5],
    [-5, 2],
    [-6, -2],
  ]);
  const ws = (await doc(page)).file!.waypoints;
  expect(ws.map((w) => [w.x, w.y])).toEqual([
    [-6, -2],
    [-2, -2.5],
    [-1, 1.5],
    [-5, 2],
    [-6, -2],
  ]);
});

test("Tab moves on from the field: it steps through the waypoints, then leaves (no keyboard trap)", async ({ page }) => {
  await draw(page, [
    [-6, -2],
    [-3, -2],
  ]);
  const field = page.getByTestId("field");
  await field.focus();
  await page.keyboard.press("Escape");
  await page.keyboard.press("Tab");
  expect((await doc(page)).selection).toEqual({ kind: "waypoint", index: 0 });
  await page.keyboard.press("Tab");
  expect((await doc(page)).selection).toEqual({ kind: "waypoint", index: 1 });
  await page.keyboard.press("Tab");
  // past the last waypoint the focus moves on
  await expect(field).not.toBeFocused();
  expect((await doc(page)).selection).toBeNull();
});

test("a typed heading of 190° is kept as -170°", async ({ page }) => {
  await draw(page, [
    [-6, -2],
    [-3, -2],
  ]);
  const heading = page.getByTestId("waypoint-heading");
  await heading.fill("190");
  await heading.press("Enter");
  await expect(heading).toHaveValue("-170");
  expect((await doc(page)).file!.waypoints[1]!.headingDegrees).toBeCloseTo(-170, 6);
});

test("two waypoints on the same spot: the plan says it can't, and offers to delete one", async ({ page }) => {
  await draw(page, [
    [-6, -2],
    [-4, -2],
    [-2, -2],
  ]);
  await page.keyboard.press("Escape");
  await page.getByRole("list", { name: "Waypoints" }).getByRole("button").nth(1).click();
  const x = page.getByRole("group", { name: "Waypoint 1", exact: true }).getByLabel("x", { exact: true });
  await x.fill("-2");
  await x.press("Enter");
  await expect(page.getByTestId("path-stats")).toContainText("can't plan");
  await page.getByTestId("checks-badge").click();
  await expect(page.locator(".checks-popover")).toContainText("waypoints 1 and 2 are at the same place");
  await page.getByTestId("fix-same-place").click();
  await expect(page.getByTestId("path-stats")).toContainText(" s,");
  expect((await doc(page)).file!.waypoints).toHaveLength(2);
});

test("a tooltip by the window's edge stays inside it", async ({ page }) => {
  await page.goto("/");
  await page.getByTestId("settings").hover();
  const tip = page.getByRole("tooltip");
  await expect(tip).toBeVisible();
  const box = (await tip.boundingBox())!;
  const width = await page.evaluate(() => window.innerWidth);
  expect(box.x + box.width).toBeLessThanOrEqual(width);
});

test("leaving with unsaved changes asks first", async ({ page }) => {
  await draw(page, [
    [-6, -2],
    [-3, -2],
  ]);
  let asked = "";
  page.once("dialog", async (d) => {
    asked = d.type();
    await d.dismiss();
  });
  await page.reload({ timeout: 3000 }).catch(() => undefined);
  expect(asked).toBe("beforeunload");
});
