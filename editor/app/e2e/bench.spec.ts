// The Bench page on the committed snapshot (public/bench/matched.json): the short version, why they differ,
// speed, when something goes wrong, without vision, course by course, and how it was tested.

import { expect, test } from "@playwright/test";

test.beforeEach(async ({ page }) => {
  await page.addInitScript(() => localStorage.setItem("pathEditor.quickStartSeen", "1"));
});

test("the bench page tells the comparison in plain words", async ({ page }) => {
  await page.setViewportSize({ width: 1440, height: 1000 });
  await page.goto("/");
  await page.getByTestId("page-bench").click();
  await expect(page.getByRole("heading", { name: "Path followers, head to head" })).toBeVisible();
  // tip or grip, the robot's traction rule: Choreo finishes none of its runs
  await expect(page.getByTestId("bench-tile-finish")).toContainText("Choreo 0");
  await expect(page.getByTestId("bench-differences-table").locator("tbody tr")).toHaveCount(4);
  await expect(page.getByTestId("bench-differences-caveats")).toContainText("Where it isn't ahead");
  await expect(page.getByTestId("bench-speed-bars").locator("li")).toHaveCount(6);
  await expect(page.getByTestId("bench-slip-bars").locator("li")).toHaveCount(6);
  await expect(page.getByTestId("bench-tip-bars").locator("li")).toHaveCount(6);
  await expect(page.getByTestId("bench-disturbed-table").locator("tbody tr")).toHaveCount(3);
  await expect(page.getByTestId("bench-slipaware-bars").locator("li")).toHaveCount(6);
  await expect(page.getByTestId("bench-course-table").locator("tbody tr")).toHaveCount(10);
  await expect(page.getByTestId("bench-course-table")).toContainText("never finished");
  await expect(page.getByRole("radio", { name: "Equal push" })).toHaveCount(0);
  await page.screenshot({ path: "test-results/bench-page.png" });
  for (const id of ["bench-differences", "bench-speed","bench-disturbed", "bench-odometry", "bench-courses"]) {
    await page.getByTestId(id).screenshot({ path: `test-results/${id}.png` });
  }
  await page.getByTestId("bench-how").locator("summary").click();
  await expect(page.getByTestId("bench-how")).toContainText("95 % of grip or tipping");
  await expect(page.getByTestId("bench-hold-note")).toContainText("+ final hold");
});
