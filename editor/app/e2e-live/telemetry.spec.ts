// The Telemetry page against a real robot stand-in: lib's Nt4TelemetryServer (WPILib's ntcore) runs the
// S-curve with a real PathFollower in real time. The page connects, records the runs, and shows the
// tracking; screenshots during and after a run go to screenshots/ for review.

import { spawn, spawnSync, type ChildProcess } from "node:child_process";
import { join } from "node:path";
import { expect, test } from "@playwright/test";

const REPO = join(import.meta.dirname, "..", "..", "..");
const SHOTS = join(import.meta.dirname, "..", "screenshots");
const PORT = 5899;
const windows = process.platform === "win32";
let server: ChildProcess;

test.beforeAll(async () => {
  test.setTimeout(300_000);
  const gradlew = join(REPO, "lib", windows ? "gradlew.bat" : "gradlew");
  server = spawn(gradlew, ["-p", "lib", "nt4Server", "--no-daemon", "-q", `-Pnt4Port=${PORT}`, "-Pnt4Seconds=180"], { cwd: REPO, shell: windows });
  let output = "";
  await new Promise<void>((resolve, reject) => {
    const onData = (chunk: Buffer) => {
      output += chunk.toString();
      if (output.includes("NT4 READY")) resolve();
    };
    server.stdout!.on("data", onData);
    server.stderr!.on("data", onData);
    server.on("exit", (code) => reject(new Error(`the NT4 server exited (${String(code)}):\n${output}`)));
  });
});

test.afterAll(() => {
  if (!server?.pid) return;
  if (windows) spawnSync("taskkill", ["/pid", String(server.pid), "/T", "/F"]);
  else server.kill("SIGTERM");
});

test("the Telemetry page records runs and shows how well the robot tracks", async ({ page }) => {
  await page.addInitScript(() => localStorage.setItem("pathEditor.quickStartSeen", "1"));
  await page.goto("/");
  await page.getByTestId("page-telemetry").click();
  await page.getByTestId("live-address").fill("127.0.0.1");
  await page.getByTestId("live-address").press("Enter");
  await page.getByLabel("Port", { exact: true }).fill(String(PORT));
  await page.getByLabel("Port", { exact: true }).press("Enter");
  await page.getByTestId("live-connect").click();
  await expect(page.getByTestId("live-status")).toContainText("connected", { timeout: 20_000 });

  // during a run: the live run, part way along
  await expect(page.getByTestId("tracking-field")).toBeVisible({ timeout: 20_000 });
  await expect(page.getByTestId("run-summary")).toContainText("running…", { timeout: 20_000 });
  await page.waitForTimeout(1500);
  await page.mouse.move(2, 2);
  await page.screenshot({ path: join(SHOTS, "telemetry-during-run.png") });

  // after a run: pick a finished one; its summary, its graphs, the whole trail
  await expect.poll(async () => page.getByTestId("run-item").count(), { timeout: 30_000 }).toBeGreaterThan(1);
  await page.getByTestId("run-item").nth(1).click(); // newest first: the one before the live run
  const summary = page.getByTestId("run-summary");
  await expect(summary).not.toContainText("running…");
  await expect(page.getByTestId("summary-events")).toContainText("3/3");
  const rms = Number((await page.getByTestId("summary-ct-rms").textContent())!.replace(/[^\d.]/g, ""));
  expect(rms).toBeGreaterThan(0);
  expect(rms).toBeLessThan(5); // cm: the follower keeps to the S-curve
  for (const id of ["chart-cross-track", "chart-along-track", "chart-heading", "chart-speed", "chart-omega"]) await expect(page.getByTestId(id)).toBeVisible();
  // the cursor at the end shows the whole run
  await page.getByTestId("run-scrub").press("End");
  await page.mouse.move(2, 2);
  await page.screenshot({ path: join(SHOTS, "telemetry-after-run.png") });
  // against distance along the path too
  await page.getByRole("radio", { name: "Distance along" }).click();
  await expect(page.getByTestId("chart-cross-track")).toContainText("m");

  await page.getByTestId("live-disconnect").click();
});
