// Flip or mirror the active path: which way, and whether in place, as a copy, or as a linked copy that
// follows it. Named points the path uses get twins, kept in step.

import { useEffect, useRef, useState } from "react";
import { TRANSFORM_LABELS, TRANSFORMS, type Transform, type TransformMode } from "../../model/mirror.ts";
import { transformActivePath } from "../../state/actions.ts";
import { activePath } from "../../state/editor.ts";
import { useStore } from "../../state/store.ts";
import { ui } from "../../state/ui.ts";

const HOW: Record<Transform, string> = {
  flip: "The other alliance's version: everything turned half-way round the field's centre.",
  mirrorTopBottom: "The other side of the field (a left auto from a right one): y becomes −y, headings mirror.",
  mirrorLeftRight: "The other end of the field: x becomes −x, headings mirror.",
};

const MODES: { mode: TransformMode; label: string; hint: string }[] = [
  { mode: "linked", label: "Linked copy", hint: "A new path that follows this one: edit this one and the copy updates. Unlink it to edit it on its own." },
  { mode: "duplicate", label: "Copy", hint: "A new path, on its own from then on." },
  { mode: "inPlace", label: "This path", hint: "Turn this path itself." },
];

export function TransformDialog() {
  const open = useStore(ui, (s) => s.transformOpen);
  const ref = useRef<HTMLDialogElement>(null);
  const [transform, setTransform] = useState<Transform>("mirrorTopBottom");
  const [mode, setMode] = useState<TransformMode>("linked");
  useEffect(() => {
    const d = ref.current;
    if (!d) return;
    if (open && !d.open) d.showModal();
    if (!open && d.open) d.close();
  }, [open]);
  const name = open ? activePath().name : "";
  return (
    <dialog ref={ref} className="help transform" aria-labelledby="transform-title" onClose={() => ui.set({ transformOpen: false })} data-testid="transform-dialog">
      <h2 id="transform-title">Flip or mirror {name}</h2>
      <fieldset>
        <legend>Which way</legend>
        {TRANSFORMS.map((t) => (
          <label key={t} className="choice">
            <input type="radio" name="transform" checked={transform === t} onChange={() => setTransform(t)} data-testid={`transform-${t}`} />
            <span>
              <strong>{TRANSFORM_LABELS[t]}</strong>
              <small>{HOW[t]}</small>
            </span>
          </label>
        ))}
      </fieldset>
      <fieldset>
        <legend>Make</legend>
        {MODES.map((m) => (
          <label key={m.mode} className="choice">
            <input type="radio" name="mode" checked={mode === m.mode} onChange={() => setMode(m.mode)} data-testid={`transform-mode-${m.mode}`} />
            <span>
              <strong>{m.label}</strong>
              <small>{m.hint}</small>
            </span>
          </label>
        ))}
      </fieldset>
      <p className="hint">Waypoints, headings, tangents, rotation targets and face points all turn. Named points it uses get twins (“Score L” ↔ “Score L (mirror)”) that stay in step: move one and the other follows.</p>
      <div className="row" style={{ justifyContent: "flex-end", gap: 8 }}>
        <button type="button" className="btn ghost" onClick={() => ui.set({ transformOpen: false })}>
          Cancel
        </button>
        <button
          type="button"
          className="btn primary"
          data-testid="transform-apply"
          onClick={() => {
            ui.set({ transformOpen: false });
            transformActivePath(transform, mode);
          }}
        >
          {mode === "inPlace" ? "Turn this path" : "Make the copy"}
        </button>
      </div>
    </dialog>
  );
}
