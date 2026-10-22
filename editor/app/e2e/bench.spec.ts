// The Bench page on the committed snapshot: the harder courses' own measures (in the main table and the
// course detail, best the right way round) and the optimised-paths table's labels.

import { expect, test } from "@playwright/test";

test.beforeEach(async ({ page }) => {
  await page.addInitScript(() => localStorage.setItem("pathEditor.quickStartSeen", "1"));
});

test("course measures head the drive-through's row, sit under the wall course's, and fill the course detail", async ({ page }) => {
  await page.setViewportSize({ width: 1440, height: 1000 });
  await page.goto("/");
  await page.getByTestId("page-bench").click();
  await expect(page.getByText(/results-lite\.json \(snapshot\): \d+ courses/)).toBeVisible();
  const table = page.getByTestId("bench-benchmark");
  // the drive-through never settles by design: its miss distance instead of "never", lowest in bold
  const intake = table.locator('tr[data-row="intake_drive_through"]');
  await expect(intake.locator("th")).toContainText("miss at the piece, cm");
  await expect(intake).not.toContainText("never");
  await expect(intake.locator("td.best")).toHaveCount(1);
  // the wall course: time settled, then its closest-bumper measure (higher is better) below
  const wall = table.locator('tr[data-row="wall_hug:wall_clear_cm"]');
  await expect(wall.locator("th")).toContainText("closest bumper to a wall, cm ▲");
  const values = await wall.locator("td").evaluateAll((tds) => tds.map((td) => ({ v: Number(td.textContent), best: td.classList.contains("best") })));
  const best = values.filter((x) => Number.isFinite(x.v)).reduce((a, b) => (b.v > a.v ? b : a));
  expect(values.find((x) => x.best)?.v).toBe(best.v);
  await table.locator('tr[data-row="wall_hug"]').scrollIntoViewIfNeeded();
  await page.screenshot({ path: "test-results/bench-measures-table.png" });

  // the course detail
  await intake.locator("button.link-row").click();
  const measures = page.getByTestId("bench-measures");
  await expect(measures).toContainText("miss at the piece");
  await expect(measures).toContainText("speed at the piece");
  await measures.scrollIntoViewIfNeeded();
  await page.screenshot({ path: "test-results/bench-measures-detail.png" });

  // the optimised table: curves-only on both sides, said so; our search on path2 only with more methods
  const opt = page.getByTestId("bench-optimised");
  await expect(page.getByTestId("bench-optimised-caption")).toContainText("PathPlanner's own genetic algorithm on the handles");
  await expect(opt).toContainText("PathPlanner Bezier, its own optimiser (handles only)");
  await expect(opt).not.toContainText("path2, our optimiser's search");
  await page.getByTestId("bench-more-methods").click();
  await expect(opt).toContainText("path2, our optimiser's search (path2 has none)");
  await opt.scrollIntoViewIfNeeded();
  await page.screenshot({ path: "test-results/bench-optimised.png" });
});
