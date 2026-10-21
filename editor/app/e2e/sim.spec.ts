// The sim preview in a real browser: switch it on and the worker runs the follower on the starter path
// and reports when it finished against the plan; a shove changes the run.

import { expect, test } from "@playwright/test";

test("the sim preview runs and reports a time", async ({ page }) => {
  await page.goto("/");
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
  await page.locator("button.card-head", { hasText: /^Sim preview/ }).click();
  await page.getByTestId("sim-shove").click();
  await expect(bar).not.toHaveText(calm!);
  await expect(time).toContainText(/Sim \d+(\.\d+)? s vs plan/);
});
