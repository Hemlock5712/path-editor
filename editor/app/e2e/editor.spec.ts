// End to end in a real browser: open a (mocked) robot project folder, add waypoints and a named
// event on the canvas, save, reload, open the folder again, and save the path afresh: the file must
// come back byte for byte. The mock folder lives in localStorage, so it survives the reload.

import { expect, test, type Page } from "@playwright/test";

const PATH_FILE = "src/main/deploy/paths/Path1.path.json";

/** Runs in the page before the app: window.showDirectoryPicker returns a folder kept in localStorage. */
function installMockFolder(): void {
  const KEY = "mockFolder";
  const load = (): Record<string, string> => JSON.parse(localStorage.getItem(KEY) ?? "{}");
  const store = (files: Record<string, string>) => localStorage.setItem(KEY, JSON.stringify(files));
  const notFound = () => new DOMException("not found", "NotFoundError");

  const fileHandle = (path: string, name: string) => ({
    kind: "file" as const,
    name,
    async getFile() {
      const text = load()[path];
      if (text === undefined) throw notFound();
      return new File([text], name);
    },
    async createWritable() {
      let text = "";
      return {
        async write(data: string) {
          text += data;
        },
        async close() {
          const files = load();
          files[path] = text;
          store(files);
        },
      };
    },
  });

  const dirHandle = (prefix: string, name: string): unknown => ({
    kind: "directory" as const,
    name,
    async getDirectoryHandle(child: string, options?: { create?: boolean }) {
      const p = `${prefix}${child}/`;
      const exists = Object.keys(load()).some((k) => k.startsWith(p));
      if (!exists && !options?.create) throw notFound();
      return dirHandle(p, child);
    },
    async getFileHandle(child: string, options?: { create?: boolean }) {
      const p = `${prefix}${child}`;
      if (load()[p] === undefined && !options?.create) throw notFound();
      return fileHandle(p, child);
    },
    async removeEntry(child: string) {
      const files = load();
      if (files[`${prefix}${child}`] === undefined) throw notFound();
      delete files[`${prefix}${child}`];
      store(files);
    },
    async *entries() {
      const seen = new Set<string>();
      for (const k of Object.keys(load())) {
        if (!k.startsWith(prefix)) continue;
        const rest = k.slice(prefix.length);
        const part = rest.split("/")[0]!;
        if (seen.has(part)) continue;
        seen.add(part);
        yield [part, rest.includes("/") ? dirHandle(`${prefix}${part}/`, part) : fileHandle(k, part)];
      }
    },
  });

  (window as unknown as { showDirectoryPicker: () => Promise<unknown> }).showDirectoryPicker = async () => dirHandle("", "robot-project");
}

async function folderFile(page: Page, path: string): Promise<string | undefined> {
  return page.evaluate((p) => (JSON.parse(localStorage.getItem("mockFolder") ?? "{}") as Record<string, string>)[p], path);
}

/** Canvas pixels for a field point on the blue view at the default zoom (the FieldView maths). */
async function fieldPoint(page: Page, x: number, y: number): Promise<{ x: number; y: number }> {
  const box = (await page.getByTestId("field").boundingBox())!;
  const scale = Math.min((box.width - 48) / 16.54, (box.height - 48) / 8.07);
  return { x: box.x + box.width / 2 + x * scale, y: box.y + box.height / 2 - y * scale };
}

async function clickField(page: Page, x: number, y: number): Promise<void> {
  const p = await fieldPoint(page, x, y);
  await page.mouse.click(p.x, p.y);
}

test("add waypoints and an event, save, reload, and get the same file back", async ({ page }) => {
  await page.addInitScript(installMockFolder);
  await page.goto("/");
  await page.evaluate(() => localStorage.removeItem("mockFolder"));

  // an empty robot project folder: the editor starts a path in it
  await page.getByTestId("open-folder").click();
  await expect(page.getByTestId("status")).toContainText("No path files");
  await expect(page.getByTestId("project-name")).toContainText("robot-project");

  // an event on the starter path (a straight run along y = -1 from x = -6.5 to -4)
  await page.keyboard.press("e");
  await clickField(page, -5.25, -1.0);
  const name = page.getByTestId("event-name-0");
  await expect(name).toHaveValue("event");
  await name.fill("intakeDown");
  await name.press("Enter");

  // two more waypoints, appended with the waypoint tool
  await page.keyboard.press("w");
  await clickField(page, -2.0, 1.0);
  await clickField(page, 1.0, -0.5);
  await expect(page.getByRole("list", { name: "Waypoints" }).getByRole("listitem")).toHaveCount(4);
  await expect(page.getByTestId("path-stats")).toContainText(" s");

  // undo and redo the last waypoint: the document comes back the same
  await page.keyboard.press("Control+z");
  await expect(page.getByRole("list", { name: "Waypoints" }).getByRole("listitem")).toHaveCount(3);
  await page.keyboard.press("Control+y");
  await expect(page.getByRole("list", { name: "Waypoints" }).getByRole("listitem")).toHaveCount(4);

  await page.getByTestId("save").click();
  await expect(page.getByTestId("status")).toContainText("Saved to robot-project");
  const saved = (await folderFile(page, PATH_FILE))!;
  expect(saved).toContain('"schema": 2');
  expect(saved).toContain('{ "name": "intakeDown", "atWaypoint": 0.5 }');
  expect(saved.match(/"x":/g)).toHaveLength(4);
  expect(await folderFile(page, "src/main/java/frc/robot/PathName.java")).toContain('PATH1("Path1")');
  expect(await folderFile(page, "src/main/deploy/paths/.pathproject.json")).toContain('"name": "Path1"');

  // reload, open the folder again: the path and its event are back
  await page.reload();
  await page.getByTestId("open-folder").click();
  await expect(page.getByTestId("status")).toContainText("Opened robot-project: 1 path");
  await expect(page.getByTestId("event-name-0")).toHaveValue("intakeDown");
  await expect(page.getByRole("list", { name: "Waypoints" }).getByRole("listitem")).toHaveCount(4);

  // the red view only changes the picture, never the file
  await page.keyboard.press("a");
  await expect(page.getByTestId("alliance")).toHaveText("Red view");

  // delete the file and save: the editor writes it again from what it loaded
  await page.evaluate((p) => {
    const files = JSON.parse(localStorage.getItem("mockFolder")!) as Record<string, string>;
    delete files[p];
    localStorage.setItem("mockFolder", JSON.stringify(files));
  }, PATH_FILE);
  await page.keyboard.press("Control+s");
  // two files: the path, and the sidecar, which keeps the red view (editor-only, not in the path)
  await expect(page.getByTestId("status")).toContainText("2 written");
  expect(await folderFile(page, PATH_FILE)).toBe(saved);
  expect(await folderFile(page, "src/main/deploy/paths/.pathproject.json")).toContain('"alliance": "red"');
});

test("the layout holds at 1280 px and the checks see an obstacle", async ({ page }) => {
  await page.setViewportSize({ width: 1280, height: 720 });
  await page.goto("/");
  const width = await page.evaluate(() => document.documentElement.scrollWidth);
  expect(width).toBeLessThanOrEqual(1280);
  // draw an obstacle across the starter path: the bumper check reports the hit
  await page.keyboard.press("o");
  for (const [x, y] of [
    [-5.5, -1.5],
    [-5.0, -1.5],
    [-5.0, -0.5],
  ]) {
    await clickField(page, x!, y!);
  }
  await page.keyboard.press("Enter");
  await expect(page.getByTestId("checks")).toContainText("Bumpers hit obstacle 1");
  await page.keyboard.press("Control+z");
  await expect(page.getByTestId("checks")).not.toContainText("Bumpers hit");
});

/** The plan's total time as the Path panel shows it ("…, 4.38 s, top …"). */
async function planSeconds(page: Page): Promise<number> {
  const text = (await page.getByTestId("path-stats").textContent()) ?? "";
  return Number(/([\d.]+) s,/.exec(text)![1]);
}

test("the optimiser shortens a weave, and applying it is one undo step", async ({ page }) => {
  await page.goto("/");
  await expect(page.getByTestId("path-stats")).toContainText(" s,"); // the app is up and planning
  await page.keyboard.press("w");
  for (const [x, y] of [
    [-2, 0.2],
    [0, -2],
    [2, 0.5],
    [4, -1],
  ]) {
    await clickField(page, x!, y!);
  }
  await page.keyboard.press("v");
  await expect(page.getByRole("list", { name: "Waypoints" }).getByRole("listitem")).toHaveCount(6);
  await expect(page.getByTestId("path-stats")).toContainText(" s,");
  const before = await planSeconds(page);

  await page.locator("summary", { hasText: "Optimise waypoints" }).click();
  await page.getByLabel("Time budget", { exact: true }).fill("3");
  await page.getByLabel("Time budget", { exact: true }).press("Enter");
  await page.getByTestId("optimise-start").click();
  await expect(page.getByTestId("optimise-apply")).toBeVisible({ timeout: 20_000 });
  await expect(page.getByTestId("optimise-status")).toContainText("faster");
  await page.getByTestId("optimise-apply").click();
  await expect.poll(() => planSeconds(page)).toBeLessThan(before * 0.95);

  await page.keyboard.press("Control+z");
  await expect.poll(() => planSeconds(page)).toBe(before);
});

test("the bench view shows an export, read-only", async ({ page }) => {
  await page.goto("/");
  const stat = (mean: number) => ({ mean, worst: mean * 1.5, nan: 0 });
  const metrics = { t_done: stat(2.1), t_settle: stat(2.3), ct_rms_cm: stat(0.4), ct_max_cm: stat(1), err_end_cm: stat(0.5), yaw_end_deg: stat(0.2), overshoot_cm: stat(0), wheel_slip_ms: stat(0) };
  const exportJson = {
    schema: 1,
    source: "out/results.json",
    meta: { seeds: [0, 1], courses: ["straight_3m"], poses: ["odom"], tracePose: "odom", traceSeeds: [0] },
    metrics: [{ key: "t_done", label: "time to done", unit: "s" }],
    methods: [
      { key: "ours_v2", label: "Ours v2" },
      { key: "choreo", label: "Choreo" },
    ],
    courses: [{ key: "straight_3m", description: "3 m straight" }],
    runs: { odom: { straight_3m: { ours_v2: { n: 2, ...metrics }, choreo: { n: 2, ...metrics, t_done: stat(2.6) } } } },
    geometry: { straight_3m: { neutral: [[0, 0], [3, 0]], ours_v2: [[0, 0], [3, 0]], choreo: [[0, 0], [3, 0]] } },
    traces: { straight_3m: { "0": { ours_v2: [[0, 0, 0, 0], [1, 1.5, 0.01, 0], [2, 3, 0, 0]], choreo: [[0, 0, 0, 0], [2.6, 3, 0.02, 0]] } } },
  };
  await page.getByTestId("bench-toggle").click();
  await page.getByTestId("bench-file").setInputFiles({ name: "editor_bench.json", mimeType: "application/json", buffer: Buffer.from(JSON.stringify(exportJson)) });
  const table = page.getByTestId("bench-table");
  await expect(table).toContainText("Ours v2");
  await expect(table.locator("td.best")).toHaveText("2.1 (3.15)");
  // editing shortcuts are off while it's open, and nothing in the project changed
  await page.keyboard.press("w");
  await page.getByTestId("bench-toggle").click();
  await expect(page.getByTestId("tool-select")).toHaveAttribute("aria-checked", "true");
  await expect(page.getByTestId("undo")).toBeDisabled();
});
