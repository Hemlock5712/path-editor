// The sim preview in a real browser: switch it on and the worker runs the follower on a short path
// and reports when it finished against the plan; a shove changes the run.

import { expect, test, type Page } from "@playwright/test";

async function dblField(page: Page, x: number, y: number): Promise<void> {
  const box = (await page.getByTestId("field").boundingBox())!;
  const [px, py] = await page.evaluate(
    ([fx, fy]) => (window as unknown as { pathEditorTest: { toCanvas(x: number, y: number): [number, number] } }).pathEditorTest.toCanvas(fx!, fy!),
    [x, y],
  );
  await page.mouse.dblclick(box.x + px, box.y + py);
}

test.beforeEach(async ({ page }) => {
  await page.addInitScript(() => localStorage.setItem("pathEditor.quickStartSeen", "1"));
});

test("the sim preview runs and reports a time", async ({ page }) => {
  await page.goto("/");
  // a new project's path is empty: double-click its start and end
  await dblField(page, -6.5, -1.5);
  await dblField(page, -4, -1.5);
  await page.keyboard.press("Escape"); // the inspector's path overview, with its cards
  await expect(page.getByTestId("time")).toContainText(/\/ \d+(\.\d+)? s/);
  // the switch is in the playback bar; the results take a row above it
  await page.getByTestId("sim-toggle").click();
  await expect(page.getByTestId("sim-toggle")).toHaveAttribute("aria-checked", "true");
  const time = page.getByTestId("sim-time");
  await expect(time).toContainText(/Sim \d+(\.\d+)? s vs plan \d+(\.\d+)? s \([+−]\d+(\.\d+)? s\)/);
  // the shove (1.5 m/s sideways at 1 s by default) pushes the robot off the path
  const bar = page.getByRole("group", { name: "Sim preview" });
  await expect(bar).toContainText(/cross-track max \d+(\.\d+)? cm/);
  const calm = await bar.textContent();
  // the shove is in the inspector's Sim preview card
  await page.getByTestId("inspector-tab-optimise").click();
  await page.getByTestId("sim-shove").click();
  await expect(bar).not.toHaveText(calm!);
  await expect(time).toContainText(/Sim \d+(\.\d+)? s vs plan/);
});
