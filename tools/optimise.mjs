#!/usr/bin/env node
// The editor's path optimiser, headless (the same code as the Optimise path button):
//
//   node tools/optimise.mjs <path.json> --robot <robot.json> --field <field.json> --budget-ms N --seed S
//        [--out <file>] [--corridor M] [--pin 2,5] [--segment 1,4] [--no-scales]
//
// Writes the optimised path file (schema 3, the core's writer) to --out, or to stdout; a one-line summary
// goes to stderr. The budget is turned into a number of plans to try (500 per second of budget), so a run
// is deterministic for a path, budget and seed on any machine; there is no wall-clock cut here.
//
// --robot and --field are the robot project's robot.json and field.json (both optional: defaults are the
// editor's). --pin keeps those waypoints where they are; --segment only moves waypoints between two.
// The start, the end, waypoints with a set heading and zone anchors always stay put.
//
// It loads the TypeScript sources directly: Node 22.6+ (type stripping; turned on here if needed).

import { spawnSync } from "node:child_process";
import { readFileSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

if (!process.features?.typescript) {
  const r = spawnSync(process.execPath, ["--experimental-strip-types", "--no-warnings=ExperimentalWarning", ...process.argv.slice(1)], {
    stdio: "inherit",
  });
  process.exit(r.status ?? 1);
}

const ROOT = join(dirname(fileURLToPath(import.meta.url)), "..");
const load = (path) => import(pathToFileURL(join(ROOT, path)).href);
const core = await load("editor/core/src/index.ts");
const { optimise, evaluationsFor, DEFAULT_OPTIONS } = await load("editor/app/src/compute/optimiser.ts");
const { parseField, parseRobot } = await load("editor/app/src/io/robotField.ts");
const { DEFAULT_FIELD, DEFAULT_ROBOT_SETTINGS } = await load("editor/app/src/model/project.ts");

function usage(message) {
  if (message) console.error(`optimise: ${message}`);
  console.error(
    "usage: node tools/optimise.mjs <path.json> --robot <robot.json> --field <field.json> --budget-ms N --seed S\n" +
      "       [--out <file>] [--corridor M] [--pin i,j] [--segment a,b] [--no-scales]",
  );
  process.exit(2);
}

const args = process.argv.slice(2);
const opts = {};
let pathFile = null;
for (let i = 0; i < args.length; i++) {
  const a = args[i];
  if (a === "--no-scales") opts.noScales = true;
  else if (a.startsWith("--")) {
    if (i + 1 >= args.length) usage(`${a} needs a value`);
    opts[a.slice(2)] = args[++i];
  } else if (pathFile === null) pathFile = a;
  else usage(`unexpected ${a}`);
}
if (!pathFile) usage("no path file");
const number = (key, fallback) => {
  if (opts[key] === undefined) return fallback;
  const v = Number(opts[key]);
  if (!Number.isFinite(v)) usage(`--${key} must be a number`);
  return v;
};
const list = (key) => (opts[key] === undefined ? [] : opts[key].split(",").map((s) => Number.parseInt(s, 10)));
const known = new Set(["robot", "field", "budget-ms", "seed", "out", "corridor", "pin", "segment", "noScales"]);
for (const k of Object.keys(opts)) if (!known.has(k)) usage(`unknown option --${k}`);

const spec = core.readPath(readFileSync(pathFile, "utf8"));
const robot = opts.robot ? parseRobot(readFileSync(opts.robot, "utf8")) : DEFAULT_ROBOT_SETTINGS;
const field = opts.field ? parseField(readFileSync(opts.field, "utf8")) : DEFAULT_FIELD;
const pins = list("pin");
const segment = list("segment");
if (opts.segment !== undefined && segment.length !== 2) usage("--segment takes two waypoint indices, a,b");

const budgetMs = number("budget-ms", 3000);
const result = optimise(
  {
    spec,
    robot: robot.config,
    bumperLengthMeters: robot.bumperLengthMeters,
    bumperWidthMeters: robot.bumperWidthMeters,
    fieldLengthMeters: field.lengthMeters,
    fieldWidthMeters: field.widthMeters,
    obstacles: field.obstacles,
  },
  {
    corridorMeters: number("corridor", DEFAULT_OPTIONS.corridorMeters),
    locked: spec.waypoints.map((_, i) => pins.includes(i)),
    segment: segment.length === 2 ? segment : null,
    optimiseScales: !opts.noScales,
    seed: number("seed", DEFAULT_OPTIONS.seed),
    maxEvaluations: evaluationsFor(budgetMs / 1000),
    timeBudgetMs: Infinity,
  },
);

const text = core.writePath(result.spec);
if (opts.out) writeFileSync(opts.out, text);
else process.stdout.write(text);
const saved = result.beforeSeconds - result.afterSeconds;
console.error(
  `optimise: ${result.beforeSeconds.toFixed(3)} s -> ${result.afterSeconds.toFixed(3)} s` +
    ` (${((100 * saved) / result.beforeSeconds).toFixed(1)} % faster), ${result.evaluations} plans, ${result.stoppedBy};` +
    ` moved waypoints ${result.free.join(", ") || "none"}`,
);
