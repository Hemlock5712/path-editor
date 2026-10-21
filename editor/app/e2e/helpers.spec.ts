// Fix-it buttons on the checks, and the small helpers: nudging and turning from the keyboard, Tab,
// Measure, facing the next waypoint, copy and paste, and the pose offered beside a field element.

import { expect, test, type Page } from "@playwright/test";

type Doc = {
  file: {
    waypoints: { x: number; y: number; headingDegrees?: number }[];
    constraintZones: { fromWaypoint: number; toWaypoint: number; maxMetersPerSecond?: number }[];
    events: { name: string; atWaypoint?: number; atMeters?: number }[];
  } | null;
  paths: { name: string }[];
  selection: { kind: string; index?: number } | null;
};

const doc = (page: Page): Promise<Doc> => page.evaluate(() => (window as unknown as { pathEditorTest: { doc(): Doc } }).pathEditorTest.doc());

async function fieldPoint(page: Page, x: number, y: number): Promise<{ x: number; y: number }> {
  const box = (await page.getByTestId("field").boundingBox())!;
  const scale = Math.min((box.width - 48) / 16.54, (box.height - 48) / 8.07);
  return { x: box.x + box.width / 2 + x * scale, y: box.y + box.height / 2 - y * scale };
}

async function clickField(page: Page, x: number, y: number): Promise<void> {
  const p = await fieldPoint(page, x, y);
  await page.mouse.click(p.x, p.y);
}

async function dblField(page: Page, x: number, y: number, shift = true): Promise<void> {
  const p = await fieldPoint(page, x, y);
  if (shift) await page.keyboard.down("Shift");
  await page.mouse.dblclick(p.x, p.y);
  if (shift) await page.keyboard.up("Shift");
}

async function fieldSettled(page: Page): Promise<void> {
  let last = "";
  await expect
    .poll(async () => {
      const box = JSON.stringify(await page.getByTestId("field").boundingBox());
      const same = box === last;
      last = box;
      return same;
    }, { intervals: [150] })
    .toBe(true);
}

async function draw(page: Page, points: [number, number][]): Promise<void> {
  await page.goto("/");
  await fieldSettled(page);
  for (const [x, y] of points) await dblField(page, x, y);
  await expect(page.getByTestId("path-stats")).toContainText(" s,");
}

test.beforeEach(async ({ page }) => {
  await page.addInitScript(() => localStorage.setItem("pathEditor.quickStartSeen", "1"));
});

test("checks offer fixes: Route around for a hit, Snap for an event off the path", async ({ page }) => {
  // across the blue hub
  await draw(page, [
    [-6.5, -1.5],
    [-4, -1.5],
    [-3.65, 0.3],
    [-0.5, 0],
  ]);
  await page.keyboard.press("Escape");
  await page.getByTestId("checks-badge").click();
  const checks = page.getByTestId("checks");
  await expect(checks).toContainText("Bumpers hit blue hub");
  await page.screenshot({ path: "test-results/fixes.png" });
  await checks.getByRole("button", { name: "Route around" }).first().click();
  await expect(page.getByTestId("optimise-apply")).toBeVisible({ timeout: 30_000 });
  await page.getByTestId("optimise-apply").click();
  await expect(page.getByTestId("checks-badge")).not.toHaveClass(/error/);

  // an event moved off the end: Snap to the path brings it back
  await page.keyboard.press("e");
  await clickField(page, -5, -1.5);
  await page.getByTestId("flow-name").fill("score");
  await page.getByTestId("flow-name").press("Enter");
  const at = page.getByTestId("event-at-0");
  await at.fill("7");
  await at.press("Enter");
  await page.getByTestId("checks-badge").click();
  await page.getByTestId("checks").getByRole("button", { name: "Snap to the path" }).click();
  expect((await doc(page)).file!.events[0]!.atWaypoint).toBe((await doc(page)).file!.waypoints.length - 1);
});

test("keys: arrows nudge 1 cm (5 cm with Shift), Q and E turn 5°, Tab steps through the waypoints", async ({ page }) => {
  await draw(page, [
    [-6.5, -1.5],
    [-4, -1.5],
    [-1, -1],
  ]);
  await clickField(page, -4, -1.5);
  await page.keyboard.press("ArrowRight");
  await page.keyboard.press("Shift+ArrowUp");
  let w = (await doc(page)).file!.waypoints[1]!;
  expect([w.x, w.y]).toEqual([-3.99, -1.45]);
  await page.keyboard.press("Tab");
  expect((await doc(page)).selection).toEqual({ kind: "waypoint", index: 2 });
  await page.keyboard.press("q");
  await page.keyboard.press("q");
  await page.keyboard.press("e");
  w = (await doc(page)).file!.waypoints[2]!;
  expect(w.headingDegrees).toBeCloseTo(5, 6);
  await page.keyboard.press("Shift+Tab");
  expect((await doc(page)).selection).toEqual({ kind: "waypoint", index: 1 });
});

test("Measure, Face next, copy and paste, and the pose offered beside the hub", async ({ page }) => {
  await draw(page, [
    [-6.5, -1.5],
    [-4, -1.5],
  ]);
  // Measure: a drag between the two robots reads 2.50 m, and changes nothing
  await page.keyboard.press("m");
  const a = await fieldPoint(page, -6.5, -1.5);
  const b = await fieldPoint(page, -4, -1.5);
  await page.mouse.move(a.x, a.y);
  await page.mouse.down();
  await page.mouse.move(b.x, b.y, { steps: 5 });
  await page.mouse.up();
  await page.screenshot({ path: "test-results/measure.png" });
  await expect(page.getByTestId("undo")).toBeEnabled(); // (the drawing itself)
  await page.keyboard.press("v");

  // Face next: the start turns toward the end
  await clickField(page, -6.5, -1.5);
  await page.getByTestId("face-next").click();
  expect((await doc(page)).file!.waypoints[0]!.headingDegrees).toBeCloseTo(0, 6);

  // copy the end, paste after it: a new end 30 cm on
  await clickField(page, -4, -1.5);
  await page.keyboard.press("Control+c");
  await page.keyboard.press("Control+v");
  const ws = (await doc(page)).file!.waypoints;
  expect(ws).toHaveLength(3);
  expect([ws[2]!.x, ws[2]!.y]).toEqual([-3.7, -1.5]);
  // copy the path (nothing selected), paste: a new path
  await page.keyboard.press("Escape");
  await page.keyboard.press("Control+c");
  await page.keyboard.press("Control+v");
  await expect.poll(async () => (await doc(page)).paths.map((p) => p.name)).toEqual(["Path1", "Path1Copy"]);

  // a waypoint double-clicked next to the hub: snap it in front, facing it
  await clickField(page, -3.7, -1.5);
  await dblField(page, -3.6, -1.0, false);
  const offer = page.getByTestId("toast-action");
  await expect(offer).toBeVisible();
  await expect(page.locator(".toast", { has: offer })).toContainText("in front of the blue hub, facing it");
  await offer.click();
  const end = (await doc(page)).file!.waypoints.at(-1)!;
  expect(end.y).toBeCloseTo(-0.5969 - 0.425 - 0.45, 3);
  expect(end.headingDegrees).toBeCloseTo(90, 6);
});
