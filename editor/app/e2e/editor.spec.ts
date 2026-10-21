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

async function dragField(page: Page, from: { x: number; y: number }, to: { x: number; y: number }): Promise<void> {
  const a = await fieldPoint(page, from.x, from.y);
  const b = await fieldPoint(page, to.x, to.y);
  await page.mouse.move(a.x, a.y);
  await page.mouse.down();
  await page.mouse.move(b.x, b.y, { steps: 6 });
  await page.mouse.up();
}

interface Poses {
  waypoints: { x: number; y: number; heading: number; set: boolean; knobX: number; knobY: number }[];
  targets: { s: number; x: number; y: number; heading: number; knobX: number; knobY: number }[];
  plan: { s: number; x: number; y: number; heading: number; travel: number }[];
}

/** The active path's robots and plan, once the plan matches the document (the app's test hook). */
function poses(page: Page): Promise<Poses> {
  return page.evaluate(() => (window as unknown as { pathEditorTest: { poses(): Promise<Poses> } }).pathEditorTest.poses());
}

const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

/** How many waypoints the active path has, once the plan has caught up. */
async function waypointCount(page: Page): Promise<number> {
  return (await poses(page)).waypoints.length;
}

/** Back to the inspector's path overview, and open one of its cards. */
async function openCard(page: Page, title: string): Promise<void> {
  const back = page.getByTestId("inspector-back");
  if (await back.isVisible()) await back.click();
  const head = page.locator("button.card-head", { hasText: new RegExp(`^${title}`) });
  if ((await head.getAttribute("aria-expanded")) !== "true") await head.click();
}
const DEG = Math.PI / 180;

// The quick start shows on a first run; the tests that aren't about it start with it dismissed
test.beforeEach(async ({ page }, info) => {
  if (!info.title.includes("first run")) await page.addInitScript(() => localStorage.setItem("pathEditor.quickStartSeen", "1"));
});

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
  await expect.poll(() => waypointCount(page)).toBe(4);
  await expect(page.getByTestId("path-stats")).toContainText(" s");

  // undo and redo the last waypoint: the document comes back the same
  await page.keyboard.press("Control+z");
  await expect.poll(() => waypointCount(page)).toBe(3);
  await page.keyboard.press("Control+y");
  await expect.poll(() => waypointCount(page)).toBe(4);

  await page.getByTestId("save").click();
  await expect(page.getByTestId("status")).toContainText("Saved to robot-project");
  const saved = (await folderFile(page, PATH_FILE))!;
  expect(saved).toContain('"schema": 3');
  expect(saved).toContain('"headingMode": "hold"');
  expect(saved).toContain('{ "name": "intakeDown", "atWaypoint": 0.5 }');
  expect(saved.match(/"x":/g)).toHaveLength(4);
  expect(await folderFile(page, "src/main/java/frc/robot/PathName.java")).toContain('PATH1("Path1")');
  expect(await folderFile(page, "src/main/deploy/paths/.pathproject.json")).toContain('"name": "Path1"');

  // reload, open the folder again: the path and its event are back
  await page.reload();
  await page.getByTestId("open-folder").click();
  await expect(page.getByTestId("status")).toContainText("Opened robot-project: 1 path");
  await expect(page.getByRole("list", { name: "Waypoints" }).getByRole("listitem")).toHaveCount(4);
  await openCard(page, "Event markers");
  await page.getByTestId("event-item-0").click();
  await expect(page.getByTestId("event-name-0")).toHaveValue("intakeDown");

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
  // the checks are a badge on the field; its popover lists them
  await expect(page.getByTestId("checks-badge")).toContainText("issue");
  await page.getByTestId("checks-badge").click();
  await expect(page.getByTestId("checks")).toContainText("Bumpers hit obstacle 1");
  await page.keyboard.press("Escape");
  await page.keyboard.press("Control+z");
  await expect(page.getByTestId("checks-badge")).toContainText("No issues");
});

/** The plan's total time as the Path panel shows it ("…, 4.38 s, top …"). */
async function planSeconds(page: Page): Promise<number> {
  const text = (await page.getByTestId("path-stats").textContent()) ?? "";
  return Number(/([\d.]+) s,/.exec(text)![1]);
}

test("Optimise path shortens a weave, and applying it is one undo step", async ({ page }) => {
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
  await page.keyboard.press("Escape"); // nothing selected: the inspector shows the path overview
  await expect.poll(() => waypointCount(page)).toBe(6);
  const before = await planSeconds(page);

  // the Path panel's Optimise path, with its default budget; the new shape shows dashed until applied
  await page.getByTestId("optimise-path").click();
  await expect(page.getByTestId("optimise-apply")).toBeVisible({ timeout: 20_000 });
  expect(await planSeconds(page)).toBe(before);
  await expect(page.getByTestId("optimise-status")).toContainText("faster");
  await page.getByTestId("optimise-apply").click();
  await expect.poll(() => planSeconds(page)).toBeLessThan(before * 0.95);

  await page.keyboard.press("Control+z");
  await expect.poll(() => planSeconds(page)).toBe(before);
});

test("the bench view opens with the snapshot and shows an export, read-only", async ({ page }) => {
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
  await page.getByTestId("page-bench").click();
  // it opens with the committed snapshot; a file replaces it
  await expect(page.getByText(/results-lite\.json \(snapshot\): \d+ courses/)).toBeVisible();
  await expect(page.getByTestId("bench-table")).toContainText("Ours v2");
  // read fairly: perfect pose, ours ticked first, a "what to look at" line, a ranked scoreboard
  await expect(page.getByRole("radio", { name: "Perfect" })).toHaveAttribute("aria-checked", "true");
  await expect(page.getByTestId("show-ours_v2")).toHaveAttribute("aria-pressed", "true");
  await expect(page.getByTestId("bench-look")).toContainText("What to look at");
  const board = page.getByTestId("bench-scoreboard");
  await expect(board.locator("tbody tr").first()).toHaveAttribute("data-method", "ours_v2");
  // every method listed: BLine's README gains is fast to "done" but never settles, so it is void
  await page.getByTestId("bench-more-methods").click();
  const bline = board.locator('tr[data-method="bline_readme"]');
  await expect(bline).toHaveAttribute("data-valid", "false");
  await expect(bline).toContainText("never settled");
  await expect(bline.locator("td.rank")).toHaveText("-");
  await expect(page.getByTestId("bench-table").locator('tr[data-method="bline_readme"] td.best')).toHaveCount(0);
  await expect(page.getByTestId("bench-table").locator('tr[data-method="bline_readme"] td.void')).not.toHaveCount(0);
  // the valid rows come first, ranked 1, 2, 3 by time settled
  const order = await board.locator("tbody tr").evaluateAll((rows) =>
    rows.map((r) => ({ valid: r.getAttribute("data-valid") === "true", rank: r.querySelector("td.rank")?.textContent ?? "", t: Number(r.children[2]?.textContent) })),
  );
  const firstInvalid = order.findIndex((r) => !r.valid);
  expect(order.slice(firstInvalid).every((r) => !r.valid)).toBe(true);
  expect(order.slice(0, firstInvalid).map((r) => r.rank)).toEqual(order.slice(0, firstInvalid).map((_, i) => String(i + 1)));
  const times = order.slice(0, firstInvalid).map((r) => r.t);
  expect(times).toEqual([...times].sort((a, b) => a - b));
  await page.getByTestId("bench-file").setInputFiles({ name: "editor_bench.json", mimeType: "application/json", buffer: Buffer.from(JSON.stringify(exportJson)) });
  const table = page.getByTestId("bench-table");
  await expect(table).toContainText("Ours v2");
  await expect(table.locator("td.best")).toHaveText("2.1 (3.15)");
  // editing shortcuts are off while it's open, and nothing in the project changed
  await page.keyboard.press("w");
  await page.getByTestId("page-editor").click();
  await expect(page.getByTestId("tool-select")).toHaveAttribute("aria-checked", "true");
  await expect(page.getByTestId("undo")).toBeDisabled();
});

test("usability, first run: draw a typical swerve auto in about ten actions", async ({ page }) => {
  await page.goto("/");
  await expect(page.getByTestId("path-stats")).toContainText(" s,");
  // a first run shows the quick start; it goes for good once dismissed (not counted: it's a one-off)
  await expect(page.getByTestId("quick-start")).toBeVisible();
  await page.getByTestId("quick-start-dismiss").click();
  await expect(page.getByTestId("quick-start")).toHaveCount(0);

  let actions = 0;
  const act = async (what: () => Promise<void>) => {
    actions++;
    await what();
  };

  // 1. the start pose: drag the starter's first robot to the start line, and
  // 2. turn it by its knob to face 180° (the 15° snap lands it exactly)
  let p = await poses(page);
  await act(() => dragField(page, p.waypoints[0]!, { x: -6, y: -2 }));
  p = await poses(page);
  await act(() => dragField(page, { x: p.waypoints[0]!.knobX, y: p.waypoints[0]!.knobY }, { x: -7, y: -2.02 }));
  // 3-6. Draw: three clicks add points at the end (the starter's end becomes the first pass-through point)
  await act(() => page.keyboard.press("w"));
  await expect(page.getByTestId("canvas-hint")).toContainText("Click to add a point");
  for (const [x, y] of [
    [-2, 1.5],
    [1, 0.5],
    [4, 2],
  ]) {
    await act(() => clickField(page, x!, y!));
  }
  // 7. the scoring pose: turn the end robot's knob to 30°
  p = await poses(page);
  const end = p.waypoints[4]!;
  await act(() => dragField(page, { x: end.knobX, y: end.knobY }, { x: end.x + Math.cos(30 * DEG), y: end.y + Math.sin(30 * DEG) }));
  // 8-10. Heading: click the path between the 2nd and 3rd pass-through points, and turn that robot to 90°
  await act(() => page.keyboard.press("r"));
  p = await poses(page);
  const nearest = (w: { x: number; y: number }) =>
    p.plan.reduce((best, q, i) => (Math.hypot(q.x - w.x, q.y - w.y) < Math.hypot(p.plan[best]!.x - w.x, p.plan[best]!.y - w.y) ? i : best), 0);
  const mid = p.plan[Math.round((nearest(p.waypoints[2]!) + nearest(p.waypoints[3]!)) / 2)]!;
  await act(() => clickField(page, mid.x, mid.y));
  p = await poses(page);
  expect(p.targets).toHaveLength(1);
  const t = p.targets[0]!;
  await act(() => dragField(page, { x: t.knobX, y: t.knobY }, { x: t.x, y: t.y + 1 }));

  // what was drawn: a start pose, three pass-through points, a scoring pose, one heading change between
  p = await poses(page);
  console.log(`typical swerve auto: ${actions} actions`);
  expect(actions).toBeLessThanOrEqual(10);
  expect(p.waypoints.map((w) => w.set)).toEqual([true, false, false, false, true]);
  expect(Math.abs(wrap(p.waypoints[0]!.heading - Math.PI))).toBeLessThan(1e-9);
  expect(Math.abs(wrap(p.waypoints[4]!.heading - 30 * DEG))).toBeLessThan(1e-9);
  expect(p.targets).toHaveLength(1);
  expect(Math.abs(wrap(p.targets[0]!.heading - 90 * DEG))).toBeLessThan(1e-9);
  await page.keyboard.press("Escape");
  await openCard(page, "Rotation targets");
  await expect(page.getByTestId("heading-mode")).toHaveAttribute("data-value", "hold");

  // the robot holds and blends the set headings: it turns 180° → 90° → 30° and no more, wherever the
  // path bends; it never just faces where it's going
  const plan = p.plan;
  expect(Math.abs(wrap(plan[0]!.heading - Math.PI))).toBeLessThan(1 * DEG);
  expect(Math.abs(wrap(plan.at(-1)!.heading - 30 * DEG))).toBeLessThan(1 * DEG);
  let turned = 0;
  for (let i = 1; i < plan.length; i++) turned += Math.abs(wrap(plan[i]!.heading - plan[i - 1]!.heading));
  expect(turned).toBeLessThan((90 + 60 + 2) * DEG);
  const facingTravel = plan.filter((q) => Math.abs(wrap(q.heading - q.travel)) < 10 * DEG).length;
  expect(facingTravel / plan.length).toBeLessThan(0.25);
});

test("right-click a waypoint to set its heading; the panel row has the heading and pass-through", async ({ page }) => {
  await page.goto("/");
  await expect(page.getByTestId("path-stats")).toContainText(" s,");
  await page.keyboard.press("w");
  await clickField(page, -1, 1);
  await page.keyboard.press("Enter");
  await expect(page.getByTestId("tool-select")).toHaveAttribute("aria-checked", "true");
  // the starter's end is now a pass-through point in the middle
  let p = await poses(page);
  expect(p.waypoints.map((w) => w.set)).toEqual([true, false, true]);
  const mid = await fieldPoint(page, p.waypoints[1]!.x, p.waypoints[1]!.y);
  await page.mouse.click(mid.x, mid.y, { button: "right" });
  await page.getByRole("menuitem", { name: "Set heading…" }).click();
  const heading = page.getByTestId("waypoint-heading");
  await expect(heading).toBeFocused();
  await heading.fill("45");
  await heading.press("Enter");
  await expect(page.getByTestId("waypoint-pass-through")).not.toBeChecked();
  p = await poses(page);
  expect(p.waypoints[1]!.set).toBe(true);
  expect(Math.abs(wrap(p.waypoints[1]!.heading - 45 * DEG))).toBeLessThan(1e-9);
  // and back to pass-through from the checkbox; an end can't be made pass-through
  await page.getByTestId("waypoint-pass-through").check();
  await expect.poll(async () => (await poses(page)).waypoints[1]!.set).toBe(false);
  await page.getByTestId("next-waypoint").click();
  await expect(page.getByTestId("waypoint-pass-through")).toBeDisabled();
  // the ? button shows the quick start again
  await page.getByTestId("help").click();
  await expect(page.getByRole("dialog", { name: "Quick start" })).toContainText("drag the knob on its ring");
});

test("Optimise path leaves pinned and set-heading waypoints alone; Discard keeps the path", async ({ page }) => {
  await page.goto("/");
  await expect(page.getByTestId("path-stats")).toContainText(" s,");
  await page.keyboard.press("w");
  for (const [x, y] of [
    [-2, 0.2],
    [0, -2],
    [2, 0.5],
  ]) {
    await clickField(page, x!, y!);
  }
  await page.keyboard.press("Enter");
  // pin waypoint 2 from its menu, and give waypoint 3 a heading: only 1 may move
  const p = await poses(page);
  const w2 = await fieldPoint(page, p.waypoints[2]!.x, p.waypoints[2]!.y);
  await page.mouse.click(w2.x, w2.y, { button: "right" });
  await page.getByRole("menuitem", { name: /^Pin/ }).click();
  const w3 = await fieldPoint(page, p.waypoints[3]!.x, p.waypoints[3]!.y);
  await page.mouse.click(w3.x, w3.y, { button: "right" });
  await page.getByRole("menuitem", { name: "Hold a heading here" }).click();
  await page.keyboard.press("Escape");
  await expect(page.getByTestId("optimise-path")).toHaveAttribute("data-tip", /Move the 1 pass-through point /);
  const before = (await poses(page)).waypoints;
  await page.getByTestId("optimise-path").click();
  await expect(page.getByTestId("optimise-discard")).toBeVisible({ timeout: 20_000 });
  await page.getByTestId("optimise-discard").click();
  const after = (await poses(page)).waypoints;
  expect(after.map((w) => [w.x, w.y])).toEqual(before.map((w) => [w.x, w.y]));
  await expect(page.getByTestId("optimise-status")).toHaveCount(0);
});
