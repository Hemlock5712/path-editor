#!/usr/bin/env node
// Phase 6, the whole chain in one command (from the repo root): `node tools/e2e.mjs`, or
// `npm --prefix editor/app run e2e:full`.
//
//   1. Playwright drives the editor to author every test path and save it (editor/app/e2e-author),
//      writing e2e/out/robot-project and the editor's preview numbers, e2e/out/preview.json.
//   2. Gradle runs the JUnit end-to-end suite (lib/src/e2eTest): PathLibrary.loadAll() on that folder,
//      every path through the Commands v3 and v2 adapters on blue and both red flips, in simulation.
//   3. That suite writes the table, e2e/RESULTS.md; its summary is printed here.
//
// Options: --skip-author reuses e2e/out from the last run (Java side only).

import { spawnSync } from "node:child_process";
import { existsSync, readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const ROOT = join(dirname(fileURLToPath(import.meta.url)), "..");
const APP = join(ROOT, "editor", "app");
const windows = process.platform === "win32";

function step(title, command, args, cwd) {
  console.log(`\n=== ${title}\n$ ${command} ${args.join(" ")}`);
  const r = spawnSync(command, args, { cwd, stdio: "inherit", shell: windows });
  return r.status === 0;
}

let ok = true;
if (!process.argv.includes("--skip-author")) {
  if (!existsSync(join(APP, "node_modules"))) ok = step("Installing the editor's packages", "npm", ["ci"], APP);
  ok = ok && step("1. Authoring every path in the editor (Playwright)", "npx", ["playwright", "test", "--config", "playwright.author.config.ts"], APP);
}
if (ok) {
  const gradlew = join(ROOT, "lib", windows ? "gradlew.bat" : "gradlew");
  ok = step("2. Running every path on the simulated robot (JUnit, WPILib adapters)", gradlew, ["-p", "lib", "e2eTest"], ROOT);
}

const report = join(ROOT, "e2e", "RESULTS.md");
if (existsSync(report)) {
  const text = readFileSync(report, "utf8");
  const summary = text.slice(text.indexOf("## Summary"), text.indexOf("## Tolerances")).trim();
  console.log(`\n=== 3. ${report}\n\n${summary}\n`);
}
console.log(ok ? "End-to-end: all passed." : "End-to-end: FAILED (see above and e2e/RESULTS.md).");
process.exit(ok ? 0 : 1);
