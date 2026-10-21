// Guided creation, removal on the field, and flip/mirror with linked twins, in a real browser. Each
// flow is a few clicks and a small form; Esc cancels; the whole flow is one undo step.

import { expect, test, type Page } from "@playwright/test";

type Doc = {
  active: string;
  file: {
    waypoints: { x: number; y: number; headingDegrees?: number }[];
    constraintZones: { fromWaypoint: number; toWaypoint: number; maxMetersPerSecond?: number; maxMetersPerSecondSquared?: number }[];
    rotationTargets: { waypoint?: number; meters?: number; degrees: number }[];
    facePointZones: { fromWaypoint: number; toWaypoint: number; x: number; y: number }[];
    events: { name: string; atWaypoint?: number; atMeters?: number }[];
  } | null;
  links: (string | null)[];
  paths: { name: string; mirrorOf: string | null; file: Doc["file"] }[];
  namedPoints: { id: string; name: string; x: number; y: number; heading: number | null; twin: string | null }[];
  selection: { kind: string; index?: number; id?: string } | null;
};

const doc = (page: Page): Promise<Doc> => page.evaluate(() => (window as unknown as { pathEditorTest: { doc(): Doc } }).pathEditorTest.doc());

async function fieldPoint(page: Page, x: number, y: number): Promise<{ x: number; y: number }> {
  const box = (await page.getByTestId("field").boundingBox())!;
  const scale = Math.min((box.width - 48) / 16.54, (box.height - 48) / 8.07);
  return { x: box.x + box.width / 2 + x * scale, y: box.y + box.height / 2 - y * scale };
}

async function clickField(page: Page, x: number, y: number, button: "left" | "right" = "left"): Promise<void> {
  const p = await fieldPoint(page, x, y);
  await page.mouse.click(p.x, p.y, { button });
}

async function dblField(page: Page, x: number, y: number): Promise<void> {
  const p = await fieldPoint(page, x, y);
  await page.keyboard.down("Shift");
  await page.mouse.dblclick(p.x, p.y);
  await page.keyboard.up("Shift");
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

/** A straight run along y = -1.5 from x = -6.5 to 0 (three waypoints), nothing selected. */
async function drawRun(page: Page): Promise<void> {
  await page.goto("/");
  await fieldSettled(page);
  await dblField(page, -6.5, -1.5);
  await dblField(page, -4, -1.5);
  await dblField(page, 0, -1.5);
  await expect(page.getByTestId("path-stats")).toContainText(" s,");
  await page.keyboard.press("Escape");
}

test.beforeEach(async ({ page }) => {
  await page.addInitScript(() => localStorage.setItem("pathEditor.quickStartSeen", "1"));
});

test("a speed zone in three steps (about five actions), and one undo takes it away", async ({ page }) => {
  await drawRun(page);
  let actions = 0;
  const act = async (what: () => Promise<void>) => {
    actions++;
    await what();
  };
  await act(() => page.keyboard.press("z"));
  const step = page.getByTestId("flow-step");
  await expect(step).toHaveText("Step 1 of 3: click where the zone starts on the path");
  await expect(page.getByTestId("flow-bar").locator("li.active")).toHaveText("1");
  await act(() => clickField(page, -5.5, -1.5));
  await expect(step).toHaveText("Step 2 of 3: click where the zone ends");
  await act(() => clickField(page, -2, -1.5));
  await expect(step).toHaveText("Step 3 of 3: set its speed limit");
  await expect(page.getByTestId("flow-speed")).toBeFocused();
  await page.screenshot({ path: "test-results/flow-zone.png" });
  await act(() => page.getByTestId("flow-speed").fill("1.2"));
  await act(() => page.keyboard.press("Enter"));
  await expect(page.getByTestId("flow-bar")).toHaveCount(0);
  const d = await doc(page);
  expect(d.file!.constraintZones).toHaveLength(1);
  const z = d.file!.constraintZones[0]!;
  expect(z.maxMetersPerSecond).toBe(1.2);
  expect(z.maxMetersPerSecondSquared).toBeUndefined(); // blank: none
  expect(z.fromWaypoint).toBeGreaterThan(0.2);
  expect(z.toWaypoint).toBeGreaterThan(1);
  expect(d.selection).toEqual({ kind: "constraint", index: 0 });
  console.log(`make a speed zone: ${actions} actions`);
  expect(actions).toBeLessThanOrEqual(5);
  await page.mouse.move(2, 2);
  await page.screenshot({ path: "test-results/flow-zone-done.png" });
  // one flow, one undo step
  await page.keyboard.press("Control+z");
  await expect.poll(async () => (await doc(page)).file!.constraintZones.length).toBe(0);
  // Esc part-way through adds nothing
  await page.keyboard.press("z");
  await clickField(page, -5.5, -1.5);
  await page.keyboard.press("Escape");
  await expect(page.getByTestId("flow-bar")).toHaveCount(0);
  expect((await doc(page)).file!.constraintZones).toHaveLength(0);
});

test("a heading: click the path, turn the ghost robot's knob or type an angle", async ({ page }) => {
  await drawRun(page);
  await page.getByTestId("flow-rotation").click();
  await clickField(page, -2, -1.5);
  await expect(page.getByTestId("flow-step")).toContainText("Step 2 of 2");
  await page.screenshot({ path: "test-results/flow-heading.png" });
  await page.getByTestId("flow-heading").fill("90");
  await page.getByTestId("flow-done").click();
  const d = await doc(page);
  expect(d.file!.rotationTargets).toHaveLength(1);
  expect(d.file!.rotationTargets[0]!.degrees).toBeCloseTo(90, 6);
  expect(d.file!.rotationTargets[0]!.waypoint).toBeGreaterThan(1);
});

test("an event: click the path, then a name (the project's names are offered)", async ({ page }) => {
  await drawRun(page);
  await page.keyboard.press("e");
  await clickField(page, -5, -1.5);
  await page.getByTestId("flow-name").fill("intakeDown");
  await page.getByTestId("flow-name").press("Enter");
  await page.keyboard.press("e");
  await clickField(page, -2, -1.5);
  // the name already used is one click away
  await page.screenshot({ path: "test-results/flow-event.png" });
  await page.getByTestId("flow-form").getByRole("button", { name: "intakeDown" }).click();
  expect((await doc(page)).file!.events.map((e) => e.name)).toEqual(["intakeDown", "intakeDown"]);
});

test("a point-towards zone: start, end, then the point to face, snapping to the hub's centre", async ({ page }) => {
  await drawRun(page);
  await page.keyboard.press("t");
  await clickField(page, -6, -1.5);
  await clickField(page, -1, -1.5);
  await expect(page.getByTestId("flow-step")).toContainText("Step 3 of 3: click the point to face");
  await clickField(page, -3.5, 0.15);
  const z = (await doc(page)).file!.facePointZones[0]!;
  expect([z.x, z.y]).toEqual([-3.645, 0]);
});

test("a named point: click, then its name and heading", async ({ page }) => {
  await drawRun(page);
  await page.keyboard.press("n");
  await clickField(page, -1, -2.5);
  await page.getByTestId("flow-name").fill("Score L");
  await page.getByTestId("flow-heading").fill("90");
  await page.getByTestId("flow-heading").press("Enter");
  const p = (await doc(page)).namedPoints;
  expect(p.map((q) => q.name)).toEqual(["Score L"]);
  expect(p[0]!.heading).toBeCloseTo(Math.PI / 2, 6);
});

test("removing things: the × on the field, right-click Delete, the Delete key and the row's trash, each undoable", async ({ page }) => {
  await drawRun(page);
  // an event and a speed zone
  await page.keyboard.press("e");
  await clickField(page, -5, -1.5);
  await page.getByTestId("flow-name").fill("intake");
  await page.getByTestId("flow-name").press("Enter");
  await page.keyboard.press("z");
  await clickField(page, -3, -1.5);
  await clickField(page, -1, -1.5);
  await expect(page.getByTestId("flow-speed")).toBeFocused();
  await page.keyboard.press("Enter");
  await page.keyboard.press("Escape");
  expect((await doc(page)).file!.events).toHaveLength(1);
  expect((await doc(page)).file!.constraintZones).toHaveLength(1);

  // hover the event's flag: a × shows by it; clicking the × deletes it
  const flag = await fieldPoint(page, -5, -1.5);
  await page.mouse.move(flag.x + 4, flag.y - 18);
  await page.mouse.move(flag.x + 5, flag.y - 17);
  await page.screenshot({ path: "test-results/remove-hover.png" });
  await page.mouse.move(flag.x - 10, flag.y - 26);
  await page.mouse.click(flag.x - 10, flag.y - 26);
  await expect.poll(async () => (await doc(page)).file!.events.length).toBe(0);
  await page.keyboard.press("Control+z");
  await expect.poll(async () => (await doc(page)).file!.events.length).toBe(1);

  // right-click the zone's chip: Delete
  const chip = page.getByTestId("field");
  const z = await fieldPoint(page, -2, -1.5);
  await page.mouse.move(z.x, z.y);
  // the chip sits beside the path's middle; find it by hit-testing a short column above it
  let found = false;
  for (let dy = 12; dy <= 32 && !found; dy += 2) {
    await page.mouse.click(z.x, z.y - dy, { button: "right" });
    found = (await page.getByRole("menuitem", { name: "Delete" }).count()) > 0;
    if (!found) await page.keyboard.press("Escape");
  }
  expect(found).toBe(true);
  await page.getByRole("menuitem", { name: "Delete" }).click();
  await expect.poll(async () => (await doc(page)).file!.constraintZones.length).toBe(0);
  void chip;

  // the event again: select it from its row and press Delete; then the row's trash
  await page.keyboard.press("Control+z");
  await page.keyboard.press("Escape");
  await page.locator("button.card-head", { hasText: /^Event markers/ }).click();
  await page.getByTestId("event-item-0").click();
  await page.keyboard.press("Delete");
  await expect.poll(async () => (await doc(page)).file!.events.length).toBe(0);
  await page.keyboard.press("Control+z");
  await page.keyboard.press("Escape");
  await page.locator("button.card-head", { hasText: /^Constraint zones/ }).click();
  await page.getByRole("button", { name: "Delete constraint zone 1" }).click();
  await expect.poll(async () => (await doc(page)).file!.constraintZones.length).toBe(0);
});

test("mirror a path with a named point as a linked copy (three actions): the twin and the copy stay in step", async ({ page }) => {
  await drawRun(page);
  // a named point at the end: the run's end double-clicked onto it links it
  await page.keyboard.press("n");
  await clickField(page, 2, -2.5);
  await page.getByTestId("flow-name").fill("Score L");
  await page.getByTestId("flow-name").press("Enter");
  await clickField(page, 0, -1.5);
  await dblField(page, 2, -2.5);
  let d = await doc(page);
  const score = d.namedPoints[0]!;
  expect(d.links.at(-1)).toBe(score.id);
  await page.keyboard.press("Escape");

  let actions = 0;
  const act = async (what: () => Promise<void>) => {
    actions++;
    await what();
  };
  await act(() => page.getByTestId("transform-path").click());
  await act(() => page.getByTestId("transform-mirrorTopBottom").check());
  // (a linked copy is the default)
  await expect(page.getByTestId("transform-mode-linked")).toBeChecked();
  await page.screenshot({ path: "test-results/mirror-dialog.png" });
  await act(() => page.getByTestId("transform-apply").click());
  console.log(`mirror a path with named points: ${actions} actions`);
  expect(actions).toBeLessThanOrEqual(3);

  d = await doc(page);
  expect(d.active).toBe("Path1Mirror");
  const copy = d.paths.find((p) => p.name === "Path1Mirror")!;
  expect(copy.mirrorOf).toBe("Path1");
  expect(copy.file!.waypoints.map((w) => [w.x, w.y])).toEqual([
    [-6.5, 1.5],
    [-4, 1.5],
    [0, 1.5],
    [2, 2.5],
  ]);
  const twin = d.namedPoints.find((p) => p.name === "Score L (mirror)")!;
  expect(twin.twin).toBe(score.id);
  expect(d.links.at(-1)).toBe(twin.id);
  await expect(page.getByTestId("link-Path1Mirror")).toContainText("follows Path1");
  await expect(page.getByTestId("link-Path1")).toContainText("followed by Path1Mirror");
  await page.mouse.move(2, 2);
  await page.screenshot({ path: "test-results/mirror-linked.png" });

  // the copy can't be edited on its own
  const end = await fieldPoint(page, -4, 1.5);
  await page.mouse.move(end.x, end.y);
  await page.mouse.down();
  await page.mouse.move(end.x + 30, end.y + 10, { steps: 4 });
  await page.mouse.up();
  await expect(page.getByTestId("status")).toContainText("is a linked copy of Path1");
  expect((await doc(page)).paths.find((p) => p.name === "Path1Mirror")!.file!.waypoints[1]).toMatchObject({ x: -4, y: 1.5 });

  // live sync: move the twin on the copy's side, and the original point (and Path1's end) follow
  const t = await fieldPoint(page, 2, 2.5);
  await page.mouse.move(t.x, t.y);
  await page.mouse.down();
  const to = await fieldPoint(page, 2.5, 3);
  await page.keyboard.down("Shift");
  await page.mouse.move(to.x, to.y, { steps: 6 });
  await page.keyboard.up("Shift");
  await page.mouse.up();
  d = await doc(page);
  expect(d.namedPoints.find((p) => p.name === "Score L")).toMatchObject({ x: 2.5, y: -3 });
  expect(d.paths.find((p) => p.name === "Path1")!.file!.waypoints.at(-1)).toMatchObject({ x: 2.5, y: -3 });
  expect(d.paths.find((p) => p.name === "Path1Mirror")!.file!.waypoints.at(-1)).toMatchObject({ x: 2.5, y: 3 });

  // edit the source: the copy follows
  await page.getByTestId("tab-Path1").click();
  const mid = await fieldPoint(page, -4, -1.5);
  await page.mouse.move(mid.x, mid.y);
  await page.mouse.down();
  const there = await fieldPoint(page, -4, -2);
  await page.keyboard.down("Shift");
  await page.mouse.move(there.x, there.y, { steps: 6 });
  await page.keyboard.up("Shift");
  await page.mouse.up();
  d = await doc(page);
  expect(d.paths.find((p) => p.name === "Path1Mirror")!.file!.waypoints[1]).toMatchObject({ x: -4, y: 2 });

  // unlink: the copy is its own path
  await page.getByTestId("unlink-Path1").click();
  await expect(page.getByTestId("link-Path1")).toHaveCount(0);
  expect((await doc(page)).paths.find((p) => p.name === "Path1Mirror")!.mirrorOf).toBeNull();
});
