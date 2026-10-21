// The left rail: the project's paths as cards with a small preview of each on the field. Click to open
// one; double-click (or F2, or Rename) to rename it; right-click for more.

import { Copy, CornerDownRight, Pencil, Plus, Trash2 } from "lucide-react";
import { useEffect, useRef, useState } from "react";
import { isValidPathName, type EditorPath, type Project } from "../../model/project.ts";
import { duplicatePath, newPath, removeActivePath } from "../../state/actions.ts";
import { geometryOf } from "../../state/analysis.ts";
import { editPath, notify, setActivePath } from "../../state/editor.ts";
import { openContextMenu } from "../../state/ui.ts";
import { formatNumber, IconButton } from "../controls.tsx";
import { useEditor } from "../panels/common.tsx";
import { Pane } from "../shell.tsx";

/** A path drawn small: the field's outline and the path, in its colour. */
function Preview({ path, field }: { path: EditorPath; field: Project["field"] }) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    const c = ref.current;
    if (!c) return;
    const dpr = window.devicePixelRatio || 1;
    const W = 72;
    const H = 36;
    c.width = W * dpr;
    c.height = H * dpr;
    const ctx = c.getContext("2d")!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = "#0d1a14";
    ctx.fillRect(0, 0, W, H);
    const k = Math.min((W - 4) / field.lengthMeters, (H - 4) / field.widthMeters);
    const px = (x: number) => W / 2 + x * k;
    const py = (y: number) => H / 2 - y * k;
    ctx.strokeStyle = "rgba(255,255,255,0.18)";
    ctx.lineWidth = 1;
    ctx.strokeRect(px(-field.lengthMeters / 2), py(field.widthMeters / 2), field.lengthMeters * k, field.widthMeters * k);
    ctx.beginPath();
    ctx.moveTo(px(0), py(field.widthMeters / 2));
    ctx.lineTo(px(0), py(-field.widthMeters / 2));
    ctx.stroke();
    const g = geometryOf(path.spec);
    ctx.strokeStyle = path.colour;
    ctx.lineWidth = 1.6;
    ctx.lineJoin = "round";
    ctx.beginPath();
    if (g) {
      const n = 48;
      for (let i = 0; i <= n; i++) {
        const p = g.sample((g.lengthMeters * i) / n);
        if (i === 0) ctx.moveTo(px(p.xMeters), py(p.yMeters));
        else ctx.lineTo(px(p.xMeters), py(p.yMeters));
      }
    } else {
      path.spec.waypoints.forEach((w, i) => (i === 0 ? ctx.moveTo(px(w.xMeters), py(w.yMeters)) : ctx.lineTo(px(w.xMeters), py(w.yMeters))));
    }
    ctx.stroke();
    const start = path.spec.waypoints[0]!;
    ctx.fillStyle = "#fbbf24";
    ctx.fillRect(px(start.xMeters) - 1.5, py(start.yMeters) - 1.5, 3, 3);
  }, [path.spec, path.colour, field]);
  return <canvas ref={ref} aria-hidden="true" />;
}

function validName(paths: readonly EditorPath[], id: string, name: string): string | null {
  if (!isValidPathName(name)) return "A path name is letters, digits, _ and -, starting with a letter";
  if (paths.some((p) => p.id !== id && p.name === name)) return "Another path has that name";
  return null;
}

function RenameInput({ path, paths, done }: { path: EditorPath; paths: readonly EditorPath[]; done: () => void }) {
  const [text, setText] = useState(path.name);
  const finish = (keep: boolean) => {
    const t = text.trim();
    if (keep && t !== path.name) {
      const problem = validName(paths, path.id, t);
      if (problem) notify(problem, "error");
      else editPath("Rename path", (p) => ({ ...p, name: t }));
    }
    done();
  };
  return (
    <input
      className="rename-input"
      aria-label="Path name"
      data-testid="path-name"
      autoFocus
      value={text}
      onFocus={(e) => e.target.select()}
      onChange={(e) => setText(e.target.value)}
      onBlur={() => finish(true)}
      onKeyDown={(e) => {
        e.stopPropagation();
        if (e.key === "Enter") finish(true);
        if (e.key === "Escape") finish(false);
      }}
      onClick={(e) => e.stopPropagation()}
    />
  );
}

export function ProjectRail() {
  const paths = useEditor((s) => s.project.paths);
  const field = useEditor((s) => s.project.field);
  const activeId = useEditor((s) => s.activePathId);
  const [renaming, setRenaming] = useState<string | null>(null);

  const menuFor = (p: EditorPath, x: number, y: number) => {
    setActivePath(p.id);
    openContextMenu(x, y, `Path ${p.name}`, [
      { label: "Rename", icon: <Pencil size={14} />, shortcut: "F2", run: () => setRenaming(p.id) },
      { label: "Duplicate", icon: <Copy size={14} />, run: duplicatePath },
      { label: "New path from its end", icon: <CornerDownRight size={14} />, run: () => newPath() },
      "separator",
      { label: "Delete", icon: <Trash2 size={14} />, danger: true, disabled: paths.length <= 1, run: removeActivePath },
    ]);
  };

  return (
    <Pane
      title="Paths"
      actions={
        <>
          <IconButton label="New path, starting where this one ends" onClick={() => newPath()} testId="new-path">
            <Plus size={16} />
          </IconButton>
          <IconButton label="Duplicate this path" onClick={duplicatePath} testId="duplicate-path">
            <Copy size={15} />
          </IconButton>
          <IconButton label="Rename this path" shortcut="F2" onClick={() => setRenaming(activeId)} testId="rename-path">
            <Pencil size={15} />
          </IconButton>
          <IconButton label="Delete this path" onClick={removeActivePath} disabled={paths.length <= 1} testId="delete-path">
            <Trash2 size={15} />
          </IconButton>
        </>
      }
    >
      <ul className="path-cards" role="tablist" aria-label="Paths" aria-orientation="vertical">
        {paths.map((p) => {
          const g = geometryOf(p.spec);
          return (
            <li key={p.id} role="presentation">
              <div
                role="tab"
                tabIndex={p.id === activeId ? 0 : -1}
                aria-selected={p.id === activeId}
                aria-label={p.name}
                className="path-card"
                data-testid={`tab-${p.name}`}
                onClick={() => setActivePath(p.id)}
                onDoubleClick={() => setRenaming(p.id)}
                onContextMenu={(e) => {
                  e.preventDefault();
                  menuFor(p, e.clientX, e.clientY);
                }}
                onKeyDown={(e) => {
                  if (e.key === "F2") setRenaming(p.id);
                  if (e.key === "ArrowDown" || e.key === "ArrowUp") {
                    e.preventDefault();
                    e.stopPropagation();
                    const i = paths.indexOf(p) + (e.key === "ArrowDown" ? 1 : -1);
                    const next = paths[(i + paths.length) % paths.length]!;
                    setActivePath(next.id);
                    (e.currentTarget.parentElement?.parentElement?.children[paths.indexOf(next)]?.firstElementChild as HTMLElement | null)?.focus();
                  }
                }}
              >
                <Preview path={p} field={field} />
                <span>
                  {renaming === p.id ? (
                    <RenameInput path={p} paths={paths} done={() => setRenaming(null)} />
                  ) : (
                    <span className="name">
                      <span className="colour" style={{ background: p.colour }} />
                      <span>{p.name}</span>
                    </span>
                  )}
                  <span className="meta">
                    {p.spec.waypoints.length} points{g ? `, ${formatNumber(g.lengthMeters, 1)} m` : ""}
                    {p.spec.events.length > 0 ? `, ${p.spec.events.length} events` : ""}
                  </span>
                </span>
              </div>
            </li>
          );
        })}
      </ul>
    </Pane>
  );
}
