import { beforeEach, describe, expect, it } from "vitest";
import { appendWaypoint, moveWaypoint, withSpec } from "../src/model/edits.ts";
import { emptyProject, makeEditorPath, starterSpec } from "../src/model/project.ts";
import { deletePath, diff, insertPath, replacePath } from "../src/state/commands.ts";
import {
  activePath,
  beginGesture,
  editor,
  editPath,
  endGesture,
  execute,
  isDirty,
  loadProject,
  markSaved,
  previewPath,
  redo,
  select,
  undo,
} from "../src/state/editor.ts";
import { composite, History, type Command } from "../src/state/history.ts";

/** A new project with a short drawn path (a new one in the app starts empty). */
const starterProject = () => ({ ...emptyProject(), paths: [makeEditorPath("Path1", starterSpec(), 0)] });

describe("History", () => {
  const add = (n: number): Command<number> => ({ label: `+${n}`, apply: (d) => d + n, revert: (d) => d - n });

  it("undoes and redoes commands in order", () => {
    const h = new History<number>();
    let doc = 0;
    doc = h.execute(doc, add(1));
    doc = h.execute(doc, add(10));
    expect(doc).toBe(11);
    expect(h.undoLabel).toBe("+10");
    doc = h.undo(doc);
    expect(doc).toBe(1);
    doc = h.redo(doc);
    expect(doc).toBe(11);
    doc = h.undo(h.undo(doc));
    expect(doc).toBe(0);
    expect(h.canUndo).toBe(false);
    expect(h.undo(doc)).toBe(0);
  });

  it("drops the redo stack when a new command runs", () => {
    const h = new History<number>();
    let doc = h.execute(0, add(1));
    doc = h.undo(doc);
    doc = h.execute(doc, add(5));
    expect(h.canRedo).toBe(false);
    expect(doc).toBe(5);
  });

  it("runs a composite as one step, reverting in reverse", () => {
    const h = new History<string>();
    const push = (c: string): Command<string> => ({ label: c, apply: (d) => d + c, revert: (d) => d.slice(0, -1) });
    let doc = h.execute("", composite("ab", [push("a"), push("b")]));
    expect(doc).toBe("ab");
    doc = h.undo(doc);
    expect(doc).toBe("");
  });

  it("keeps at most its limit", () => {
    const h = new History<number>(3);
    let doc = 0;
    for (let i = 0; i < 5; i++) doc = h.execute(doc, add(1));
    for (let i = 0; i < 5; i++) doc = h.undo(doc);
    expect(doc).toBe(2);
  });
});

describe("project commands", () => {
  it("replace, insert and delete paths independently", () => {
    const p0 = starterProject();
    const a = p0.paths[0]!;
    const b = makeEditorPath("B", starterSpec(), 1);
    const p1 = insertPath("add", b, 1).apply(p0);
    expect(p1.paths.map((p) => p.name)).toEqual(["Path1", "B"]);
    const a2 = appendWaypoint(a, 0, 0);
    const p2 = replacePath("edit", a, a2).apply(p1);
    expect(p2.paths[0]!.spec.waypoints).toHaveLength(3);
    const p3 = deletePath("del", b, 1).apply(p2);
    expect(p3.paths).toHaveLength(1);
    // undoing the delete brings B back in its place, without touching A's edit
    const back = deletePath("del", b, 1).revert(p3);
    expect(back.paths.map((p) => p.name)).toEqual(["Path1", "B"]);
    expect(back.paths[0]).toBe(a2);
  });

  it("diff turns two project states into one command per changed part", () => {
    const before = starterProject();
    const after = { ...before, paths: [appendWaypoint(before.paths[0]!, 1, 1)], field: { ...before.field, lengthMeters: 17 } };
    const cmd = diff("drag", before, after)!;
    expect(cmd.apply(before)).toEqual(after);
    expect(cmd.revert(after)).toEqual(before);
    expect(diff("nothing", before, before)).toBeNull();
  });
});

describe("editor store", () => {
  beforeEach(() => loadProject(starterProject()));

  it("records each edit as an undo step and keeps the selection valid", () => {
    editPath("Add waypoint", (p) => appendWaypoint(p, 0, 0), { kind: "waypoint", index: 2 });
    expect(activePath().spec.waypoints).toHaveLength(3);
    expect(editor.get().selection).toEqual({ kind: "waypoint", index: 2 });
    expect(editor.get().undoLabel).toBe("Add waypoint");
    undo();
    expect(activePath().spec.waypoints).toHaveLength(2);
    expect(editor.get().selection).toBeNull(); // waypoint 2 is gone
    redo();
    expect(activePath().spec.waypoints).toHaveLength(3);
  });

  it("turns a drag into a single undo step", () => {
    const start = activePath().spec.waypoints[1]!;
    beginGesture();
    for (let i = 1; i <= 10; i++) previewPath((p) => moveWaypoint(p, 1, start.xMeters + i * 0.1, start.yMeters));
    endGesture("Move waypoint");
    expect(activePath().spec.waypoints[1]!.xMeters).toBeCloseTo(start.xMeters + 1, 12);
    undo();
    expect(activePath().spec.waypoints[1]).toEqual(start);
    expect(editor.get().canUndo).toBe(false);
  });

  it("refuses an edit the core rejects, with a message, and changes nothing", () => {
    const before = editor.get().project;
    const ok = editPath("Bad speed", (p) => withSpec(p, { maxMetersPerSecond: -1 }));
    expect(ok).toBe(false);
    expect(editor.get().project).toBe(before);
    expect(editor.get().status?.kind).toBe("error");
    expect(editor.get().canUndo).toBe(false);
  });

  it("tracks unsaved changes against the last save", () => {
    expect(isDirty(editor.get())).toBe(false);
    editPath("Move", (p) => moveWaypoint(p, 0, 0, 0));
    expect(isDirty(editor.get())).toBe(true);
    markSaved();
    expect(isDirty(editor.get())).toBe(false);
  });

  it("clears a selection the project no longer has", () => {
    select({ kind: "event", index: 0 });
    expect(editor.get().selection).toBeNull();
    const path = makeEditorPath("Two", starterSpec(), 1);
    execute(insertPath("add", path, 1));
    expect(editor.get().project.paths).toHaveLength(2);
    expect(editor.get().activePathId).toBe(editor.get().project.paths[0]!.id);
  });
});
