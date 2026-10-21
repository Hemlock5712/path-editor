// The sim preview in a real browser: switch it on and the worker runs the follower on the starter path
// and reports when it finished against the plan; a shove changes the run.

import { expect, test } from "@playwright/test";

test("the sim preview runs and reports a time", async ({ page }) => {
  await page.goto("/");
  await expect(page.getByTestId("time")).toContainText(/\/ \d+(\.\d+)? s/);
  await page.getByTestId("sim-toggle").check();
  const time = page.getByTestId("sim-time");
  await expect(time).toContainText(/Sim \d+(\.\d+)? s vs plan \d+(\.\d+)? s \([+−]\d+(\.\d+)? s\)/);
  // the shove (1.5 m/s sideways at 1 s by default) pushes the robot off the path
  const bar = page.getByRole("group", { name: "Sim preview" });
  await expect(bar).toContainText(/cross-track max \d+(\.\d+)? cm/);
  const calm = await bar.textContent();
  await page.getByTestId("sim-shove").check();
  await expect(bar).not.toHaveText(calm!);
  await expect(time).toContainText(/Sim \d+(\.\d+)? s vs plan/);
});
