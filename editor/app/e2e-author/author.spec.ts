// Phase 6, step 1: author the end-to-end suite in the real editor. Every path, waypoint, target,
// zone, named point and event is made through the UI (canvas clicks and the side panels), and saved
// with the editor's own save flow into a folder on disk (a directory handle whose writes go to the
// file system through Playwright). The editor's preview numbers for each path go next to it, in
// preview.json, for the Java suite to compare with what the robot code does.
//
// Output: <repo>/e2e/out/robot-project/src/main/deploy/paths/*  and  <repo>/e2e/out/preview.json

import { mkdirSync, readdirSync, readFileSync, rmSync, statSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { expect, test, type Locator, type Page } from "@playwright/test";

const REPO = join(import.meta.dirname, "..", "..", "..");
const OUT = process.env.E2E_OUT ?? join(REPO, "e2e", "out");
const PROJECT = join(OUT, "robot-project");

// ------------------------------------------------------------------------------ the suite

type At = { m: number } | { wp: number };

interface PathPlan {
  readonly name: string;
  /** x, y and optionally the tangent in degrees. */
  readonly waypoints: readonly (readonly [number, number, number?])[];
  /** Headings set on waypoints (a swerve's waypoints are robot poses); the ends always have one. */
  readonly headings: readonly { readonly wp: number; readonly deg: number }[];
  /** Headings partway along the path (rotation targets), at a waypoint position. */
  readonly rotationTargets?: readonly { readonly wp: number; readonly deg: number }[];
  readonly events: readonly { readonly name: string; readonly at: At }[];
  readonly startSpeed?: number;
  readonly endSpeed?: number;
  readonly faceZone?: { readonly from: number; readonly to: number; readonly x: number; readonly y: number; readonly blend: number };
  readonly constraintZone?: { readonly from: number; readonly to: number; readonly maxSpeed: number };
  /** Link this waypoint to a new named point, then move the point to (x, y). */
  readonly namedPoint?: { readonly wp: number; readonly x: number; readonly y: number };
}

/** Events every path gets: at the start, mid-path, two 7 cm apart, and right at the end. */
function standardEvents(last: number, close: number): PathPlan["events"] {
  return [
    { name: "start", at: { m: 0 } },
    { name: "mid", at: { wp: last / 2 } },
    { name: "closeA", at: { m: close } },
    { name: "closeB", at: { m: close + 0.07 } },
    { name: "end", at: { wp: last } },
  ];
}

const held = (last: number, deg = 0) => [
  { wp: 0, deg },
  { wp: last, deg },
];

/** The bench courses (bench/courses.py), moved onto the field, plus the special cases. */
const hairpin: [number, number][] = [
  [0, 0],
  [1, 0],
  ...[-90, -60, -30, 0, 30, 60, 90].map((a): [number, number] => [2 + Math.cos((a * Math.PI) / 180), 1 + Math.sin((a * Math.PI) / 180)]),
  [1, 2],
  [0, 2],
];
const shift = (pts: readonly (readonly [number, number])[], dx: number, dy: number) =>
  pts.map(([x, y]) => [Math.round((x + dx) * 1e5) / 1e5, Math.round((y + dy) * 1e5) / 1e5] as [number, number]);
const withEnds = (pts: [number, number][], first: number, last: number): [number, number, number?][] =>
  pts.map((p, i) => (i === 0 ? [...p, first] : i === pts.length - 1 ? [...p, last] : p));
const chord = (a: readonly [number, number], b: readonly [number, number]) => Math.round(((Math.atan2(b[1] - a[1], b[0] - a[0]) * 180) / Math.PI) * 1e5) / 1e5;
const course = (pts: [number, number][]) => withEnds(pts, chord(pts[0]!, pts[1]!), chord(pts[pts.length - 2]!, pts[pts.length - 1]!));

const SUITE: PathPlan[] = [
  {
    name: "Straight",
    waypoints: course(shift([[0, 0], [3, 0]], -6, -1)),
    headings: held(1),
    events: standardEvents(1, 2.0),
  },
  {
    name: "SCurve",
    waypoints: course(shift([[0, 0], [1.25, 0], [2.5, 0.8], [3.75, 1.6], [5, 1.6]], -6, -1)),
    headings: held(4),
    events: standardEvents(4, 3.5),
  },
  {
    name: "Hairpin",
    waypoints: course(shift(hairpin, -6, -2.5)),
    headings: held(10),
    events: standardEvents(10, 5.0),
  },
  {
    name: "Slalom",
    waypoints: course(shift([[0, 0], [1.75, 0.4], [3.5, -0.4], [5.25, 0.4], [7, 0]], -6, 0.5)),
    headings: held(4),
    events: standardEvents(4, 5.0),
  },
  {
    name: "LongAuto",
    waypoints: course(shift([[0, 0], [3, 0.5], [5.5, 2], [7.5, 3.2], [9, 2.2], [9.3, 0.3]], -7, -2)),
    // a set heading on a waypoint, and one partway along the path
    headings: [
      { wp: 0, deg: 0 },
      { wp: 2, deg: 90 },
      { wp: 5, deg: 180 },
    ],
    rotationTargets: [{ wp: 3.5, deg: 135 }],
    events: [...standardEvents(5, 8.0), { name: "turning", at: { wp: 1.5 } }],
  },
  {
    name: "MovingStart",
    waypoints: course(shift([[0, 0], [1.5, 0.4], [2.6, 1.4], [3, 3]], -6, -3)),
    headings: held(3),
    startSpeed: 1.5,
    events: standardEvents(3, 2.5),
  },
  {
    name: "ShortHop",
    waypoints: [
      [-4, 2.5, 0],
      [-3.8, 2.5, 0],
    ],
    headings: [
      { wp: 0, deg: 0 },
      { wp: 1, deg: 80 },
    ],
    events: standardEvents(1, 0.12),
  },
  {
    name: "TurnInPlace",
    waypoints: [
      [-5, 3],
      [-5, 3],
    ],
    headings: [
      { wp: 0, deg: 0 },
      { wp: 1, deg: 90 },
    ],
    events: [
      { name: "start", at: { m: 0 } },
      { name: "mid", at: { wp: 0.5 } },
      { name: "end", at: { wp: 1 } },
    ],
  },
  {
    name: "Zones",
    waypoints: course([
      [-7, 1],
      [-4, 1.5],
      [-1, 1],
      [1, 2],
    ]),
    headings: held(3),
    faceZone: { from: 1, to: 2, x: -2.5, y: 3.2, blend: 1 },
    constraintZone: { from: 2, to: 3, maxSpeed: 1.5 },
    events: standardEvents(3, 6.0),
  },
  {
    name: "ViaPoint",
    waypoints: course([
      [-6, -2],
      [-3, -2.8],
      [0, -2],
    ]),
    headings: held(2),
    namedPoint: { wp: 1, x: -2.9, y: -2.7 },
    events: standardEvents(2, 4.0),
  },
  {
    name: "RollA",
    waypoints: [
      [-6, 3.2, 0],
      [-3, 3.2, 0],
    ],
    headings: held(1),
    endSpeed: 1.5,
    events: standardEvents(1, 2.0),
  },
  {
    name: "RollB",
    waypoints: [
      [-3, 3.2, 0],
      [0, 2.6, 0],
    ],
    headings: held(1),
    startSpeed: 1.5,
    events: standardEvents(1, 2.0),
  },
];

/** E2E_ONLY=Name,Name authors just those (for debugging one path). */
const chosen = process.env.E2E_ONLY ? SUITE.filter((p) => process.env.E2E_ONLY!.split(",").includes(p.name)) : SUITE;

/** RollA ends moving and hands its speed to RollB. */
const CHAINS = [["RollA", "RollB"]];

// ------------------------------------------------------------------------------ a folder on disk

/** The page side: window.showDirectoryPicker returns a handle whose reads and writes go to disk. */
function installDiskFolder(): void {
  type Bridge = {
    diskList(dir: string): Promise<{ name: string; dir: boolean }[]>;
    diskRead(path: string): Promise<string | null>;
    diskWrite(path: string, text: string): Promise<void>;
    diskRemove(path: string): Promise<boolean>;
  };
  const bridge = window as unknown as Bridge;
  const notFound = () => new DOMException("not found", "NotFoundError");
  const fileHandle = (path: string, name: string) => ({
    kind: "file" as const,
    name,
    async getFile() {
      const text = await bridge.diskRead(path);
      if (text === null) throw notFound();
      return new File([text], name);
    },
    async createWritable() {
      let text = "";
      return {
        async write(data: string) {
          text += data;
        },
        async close() {
          await bridge.diskWrite(path, text);
        },
      };
    },
  });
  const dirHandle = (prefix: string, name: string): unknown => ({
    kind: "directory" as const,
    name,
    async getDirectoryHandle(child: string, options?: { create?: boolean }) {
      const entries = await bridge.diskList(prefix);
      if (!entries.some((e) => e.name === child && e.dir) && !options?.create) throw notFound();
      return dirHandle(`${prefix}${child}/`, child);
    },
    async getFileHandle(child: string, options?: { create?: boolean }) {
      if ((await bridge.diskRead(`${prefix}${child}`)) === null && !options?.create) throw notFound();
      return fileHandle(`${prefix}${child}`, child);
    },
    async removeEntry(child: string) {
      if (!(await bridge.diskRemove(`${prefix}${child}`))) throw notFound();
    },
    async *entries() {
      for (const e of await bridge.diskList(prefix)) {
        yield [e.name, e.dir ? dirHandle(`${prefix}${e.name}/`, e.name) : fileHandle(`${prefix}${e.name}`, e.name)];
      }
    },
  });
  (window as unknown as { showDirectoryPicker: () => Promise<unknown> }).showDirectoryPicker = async () => dirHandle("", "robot-project");
}

async function exposeDisk(page: Page): Promise<void> {
  const abs = (p: string) => join(PROJECT, ...p.split("/").filter((s) => s !== ""));
  await page.exposeFunction("diskList", (dir: string) => {
    try {
      return readdirSync(abs(dir)).map((name) => ({ name, dir: statSync(join(abs(dir), name)).isDirectory() }));
    } catch {
      return [];
    }
  });
  await page.exposeFunction("diskRead", (path: string) => {
    try {
      return readFileSync(abs(path), "utf8");
    } catch {
      return null;
    }
  });
  await page.exposeFunction("diskWrite", (path: string, text: string) => {
    mkdirSync(dirname(abs(path)), { recursive: true });
    writeFileSync(abs(path), text, "utf8");
  });
  await page.exposeFunction("diskRemove", (path: string) => {
    try {
      rmSync(abs(path));
      return true;
    } catch {
      return false;
    }
  });
}

// ------------------------------------------------------------------------------ driving the UI

/** Types a number into a field and commits it (Enter blurs, which commits). */
async function setNumber(field: Locator, value: number): Promise<void> {
  await field.fill(String(value));
  await field.press("Enter");
}

/** The inspector's path overview (back from a selected item). */
async function overview(page: Page): Promise<void> {
  const back = page.getByTestId("inspector-back");
  if (await back.isVisible()) await back.click();
  await expect(page.getByTestId("optimise-path")).toBeVisible();
}

/** Opens one of the overview's cards. */
async function openSection(page: Page, title: string): Promise<void> {
  await overview(page);
  const head = page.locator("button.card-head", { hasText: new RegExp(`^${title}`) });
  if ((await head.getAttribute("aria-expanded")) !== "true") await head.click();
}

/** Picks an option in one of the app's selects. */
async function choose(page: Page, select: Locator, option: string): Promise<void> {
  await select.click();
  await page.getByRole("option", { name: option, exact: true }).click();
}

/** Canvas pixels for a blue-frame field point at the default view. */
async function fieldPoint(page: Page, x: number, y: number): Promise<{ x: number; y: number }> {
  const box = (await page.getByTestId("field").boundingBox())!;
  const scale = Math.min((box.width - 48) / 16.54, (box.height - 48) / 8.07);
  return { x: box.x + box.width / 2 + x * scale, y: box.y + box.height / 2 - y * scale };
}

const waypointGroup = (page: Page, i: number) => page.getByRole("group", { name: `Waypoint ${i}`, exact: true });

async function selectWaypoint(page: Page, i: number): Promise<void> {
  await openSection(page, "Waypoints");
  await page.getByRole("list", { name: "Waypoints" }).getByRole("button").nth(i).click();
}

async function setWaypoint(page: Page, i: number, [x, y, tangent]: readonly [number, number, number?], heading?: number): Promise<void> {
  await selectWaypoint(page, i);
  const g = waypointGroup(page, i);
  await setNumber(g.getByLabel("x", { exact: true }), x);
  await setNumber(g.getByLabel("y", { exact: true }), y);
  // a blank heading makes a middle waypoint pass-through (the ends always keep theirs)
  if (heading !== undefined) await setNumber(g.getByTestId("waypoint-heading"), heading);
  else if (i > 0 && i < (await page.getByRole("list", { name: "Waypoints" }).getByRole("listitem").count()) - 1) {
    await expect(g.getByTestId("waypoint-pass-through")).toBeChecked();
  }
  if (tangent !== undefined) {
    // the travel direction is set by hand behind "Shape the curve here"
    const shape = page.getByTestId("shape-curve");
    if ((await shape.getAttribute("aria-checked")) !== "true") await shape.click();
    await setNumber(g.getByLabel("Travel direction", { exact: true }), tangent);
  }
}

async function author(page: Page, plan: PathPlan, first: boolean): Promise<void> {
  if (!first) await page.getByTestId("new-path").click();
  await page.getByTestId("rename-path").click();
  const name = page.getByTestId("path-name");
  await name.fill(plan.name);
  await name.press("Enter");
  await expect(page.getByRole("tab", { name: plan.name })).toHaveAttribute("aria-selected", "true");

  // waypoints: a new project's path is empty, so two double-clicks make its start and end; a new path
  // starts with just its start (selected), and Add after makes its end. Add after on the last one adds
  // the rest, then every one is typed in
  if (first) {
    for (const x of [-1, 1]) {
      const p = await fieldPoint(page, x, 0);
      await page.mouse.dblclick(p.x, p.y);
    }
    await expect(waypointGroup(page, 1)).toBeVisible();
  } else {
    await waypointGroup(page, 0).getByTestId("add-after").click();
  }
  for (let i = 2; i < plan.waypoints.length; i++) await waypointGroup(page, i - 1).getByTestId("add-after").click();
  for (let i = 0; i < plan.waypoints.length; i++) await setWaypoint(page, i, plan.waypoints[i]!, plan.headings.find((h) => h.wp === i)?.deg);

  if (plan.startSpeed !== undefined || plan.endSpeed !== undefined) await openSection(page, "Start and end state");
  if (plan.startSpeed !== undefined) await setNumber(page.getByLabel("Start speed"), plan.startSpeed);
  if (plan.endSpeed !== undefined) await setNumber(page.getByLabel("End speed"), plan.endSpeed);

  if (plan.namedPoint) {
    const { wp, x, y } = plan.namedPoint;
    await selectWaypoint(page, wp);
    await page.getByRole("button", { name: `New point at waypoint ${wp}` }).click();
    const card = page.getByRole("group", { name: /^Point / });
    await setNumber(card.getByLabel("x", { exact: true }), x);
    await setNumber(card.getByLabel("y", { exact: true }), y);
    // the linked waypoint followed the point
    await openSection(page, "Waypoints");
    await expect(page.getByRole("list", { name: "Waypoints" }).getByRole("button").nth(wp)).toContainText(`(${x}, ${y})`);
  }

  // the guided flows click the path: at its first two waypoints (on the path, and apart)
  const onPath = async (wp: number) => {
    const w = plan.waypoints[wp]!;
    const p = await fieldPoint(page, w[0], w[1]);
    await page.mouse.click(p.x, p.y);
  };
  // events: E, a click near the path, a name; then positioned exactly
  const near = plan.waypoints[Math.min(1, plan.waypoints.length - 1)]!;
  for (const [i, e] of plan.events.entries()) {
    await page.keyboard.press("m");
    const p = await fieldPoint(page, near[0], near[1]);
    await page.mouse.click(p.x, p.y + 18);
    await page.getByTestId("flow-name").fill(e.name);
    await page.getByTestId("flow-name").press("Enter");
    const card = page.getByTestId(`event-${i}`);
    await expect(card).toBeVisible();
    await expect(page.getByTestId(`event-name-${i}`)).toHaveValue(e.name);
    await choose(page, card.getByRole("combobox", { name: "At measured by" }), "m" in e.at ? "by metres" : "by waypoint");
    await setNumber(page.getByTestId(`event-at-${i}`), "m" in e.at ? e.at.m : e.at.wp);
  }
  await page.keyboard.press("v");

  if (plan.rotationTargets) {
    for (const [i, t] of plan.rotationTargets.entries()) {
      await openSection(page, "Rotation targets");
      await page.getByTestId("add-rotation").click();
      await onPath(0);
      await page.getByTestId("flow-heading").fill(String(t.deg));
      await page.getByTestId("flow-heading").press("Enter");
      const card = page.getByRole("group", { name: `Heading ${i + 1}`, exact: true });
      await setNumber(card.getByLabel("At", { exact: true }), t.wp);
      await setNumber(card.getByLabel("Heading", { exact: true }), t.deg);
    }
  }
  if (plan.faceZone) {
    const z = plan.faceZone;
    await openSection(page, "Point-towards zones");
    await page.getByTestId("add-face").click();
    await onPath(0);
    await onPath(1);
    const target = await fieldPoint(page, z.x, z.y);
    await page.keyboard.down("Shift"); // (no snapping to field elements)
    await page.mouse.click(target.x, target.y);
    await page.keyboard.up("Shift");
    const card = page.getByRole("group", { name: "Face-point zone 1" });
    // (made from waypoint 0 to 1: the end first, so the zone never ends before it starts)
    await setNumber(card.getByLabel("To", { exact: true }), z.to);
    await setNumber(card.getByLabel("From", { exact: true }), z.from);
    await setNumber(card.getByLabel("Point x"), z.x);
    await setNumber(card.getByLabel("Point y"), z.y);
    await setNumber(card.getByLabel("Blend"), z.blend);
  }
  if (plan.constraintZone) {
    const z = plan.constraintZone;
    await openSection(page, "Constraint zones");
    await page.getByTestId("add-constraint").click();
    await onPath(0);
    await onPath(1);
    await expect(page.getByTestId("flow-speed")).toBeFocused();
    await page.getByTestId("flow-speed").fill(String(z.maxSpeed));
    await page.getByTestId("flow-speed").press("Enter");
    const card = page.getByRole("group", { name: "Constraint zone 1" });
    await setNumber(card.getByLabel("To", { exact: true }), z.to);
    await setNumber(card.getByLabel("From", { exact: true }), z.from);
    await setNumber(card.getByLabel("Top speed", { exact: true }), z.maxSpeed);
  }
  await expect(page.getByTestId("status")).not.toHaveClass(/error/);
}

// ------------------------------------------------------------------------------ the run

test("author the end-to-end suite in the editor and save it", async ({ page }) => {
  test.setTimeout(300_000);
  rmSync(OUT, { recursive: true, force: true });
  mkdirSync(PROJECT, { recursive: true });
  await exposeDisk(page);
  await page.addInitScript(installDiskFolder);
  await page.addInitScript(() => localStorage.setItem("pathEditor.quickStartSeen", "1"));
  await page.goto("/");

  await page.getByTestId("open-folder").click();
  await expect(page.getByTestId("status")).toContainText("No path files");

  // the robot: spin limits a little under the follower's own (540°/s, 720°/s²), so the plan leaves
  // feedback room to catch up, as the plan's grip (μ 0.7) sits under the limiter's (1.0)
  await page.getByTestId("settings").click();
  await page.getByTestId("settings-robot").click();
  await setNumber(page.getByLabel("Max spin", { exact: true }), 7.5);
  await setNumber(page.getByLabel("Max spin accel", { exact: true }), 10);
  await page.getByTestId("page-editor").click();

  for (const [i, plan] of chosen.entries()) await author(page, plan, i === 0);

  await page.keyboard.press("Control+s");
  await expect(page.getByTestId("status")).toContainText("Saved to robot-project");

  // the editor's preview of every path, read back tab by tab once the worker has planned it
  const previews = [];
  for (const plan of chosen) {
    await page.getByRole("tab", { name: plan.name, exact: true }).click();
    previews.push(await page.evaluate(() => (window as unknown as { pathEditorTest: { preview(): Promise<unknown> } }).pathEditorTest.preview()));
  }
  writeFileSync(join(OUT, "preview.json"), JSON.stringify({ chains: CHAINS, paths: previews }, null, 2) + "\n");

  const pathsDir = join(PROJECT, "src", "main", "deploy", "paths");
  const files = readdirSync(pathsDir).filter((f) => f.endsWith(".path.json"));
  expect(files.sort()).toEqual(chosen.map((p) => `${p.name}.path.json`).sort());
  expect(readFileSync(join(pathsDir, "robot.json"), "utf8")).toContain('"maxAngularRadPerSecSquared": 10');
  for (const plan of chosen) {
    const text = readFileSync(join(pathsDir, `${plan.name}.path.json`), "utf8");
    for (const e of plan.events) expect(text, plan.name).toContain(`"name": "${e.name}"`);
  }
  // the generated names, which the Java suite compiles and uses (lib/src/e2eNames)
  const names = readFileSync(join(PROJECT, "src", "main", "java", "frc", "robot", "PathEditorNames.java"), "utf8");
  for (const plan of chosen) expect(names).toContain(`("${plan.name}")`);
  for (const e of chosen.flatMap((p) => p.events)) expect(names).toContain(`("${e.name}")`);
});
