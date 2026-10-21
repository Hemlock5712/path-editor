// The guided flows' step bar (over the field: which flow, its steps with the current one lit, what to do
// now, Cancel) and the small form a flow ends with (beside the spot it was placed at).

import { X } from "lucide-react";
import { useEffect, useId, useMemo, useRef, useState } from "react";
import { uniquePointName } from "../../model/namedPoints.ts";
import { eventNames } from "../../model/project.ts";
import { editor } from "../../state/editor.ts";
import {
  cancelFlow,
  finishConstraint,
  finishEvent,
  finishPoint,
  finishRotation,
  flow,
  FLOW_STEPS,
  FLOW_TITLES,
  setFlowHeading,
} from "../../state/flow.ts";
import { useStore } from "../../state/store.ts";
import { formatNumber } from "../controls.tsx";

const DEG = Math.PI / 180;

export function FlowBar() {
  const f = useStore(flow, (s) => s);
  if (!f.kind) return null;
  const steps = FLOW_STEPS[f.kind];
  return (
    <div className="float-bar flow-bar" role="status" aria-label={`${FLOW_TITLES[f.kind]}: step ${f.step + 1} of ${steps.length}`} data-testid="flow-bar">
      <strong className="flow-title">{FLOW_TITLES[f.kind]}</strong>
      <ol className="flow-steps">
        {steps.map((text, i) => (
          <li key={i} className={i === f.step ? "active" : i < f.step ? "done" : ""} aria-current={i === f.step ? "step" : undefined} title={text}>
            {i + 1}
          </li>
        ))}
      </ol>
      <span className="flow-text" data-testid="flow-step">
        Step {f.step + 1} of {steps.length}: {steps[f.step]!.charAt(0).toLowerCase() + steps[f.step]!.slice(1)}
      </span>
      <button type="button" className="btn ghost small" onClick={cancelFlow} data-tip="Stop without adding anything|Esc" data-testid="flow-cancel">
        <X size={14} />
        Cancel
      </button>
    </div>
  );
}

/** Esc in a flow form cancels the flow (the shortcuts ignore keys typed into fields). */
const onEsc = (e: React.KeyboardEvent) => {
  if (e.key === "Escape") {
    e.stopPropagation();
    cancelFlow();
  }
};

/** The last step's form, placed at (left, top) in the field's box. */
export function FlowForm({ left, top }: { left: number; top: number }) {
  const f = useStore(flow, (s) => s);
  if (!f.kind) return null;
  const last = FLOW_STEPS[f.kind].length - 1;
  if (f.step !== last || f.kind === "face") return null;
  return (
    <div className="flow-form" style={{ left, top }} role="dialog" aria-label={`${FLOW_TITLES[f.kind]} details`} data-testid="flow-form" onKeyDown={onEsc}>
      {f.kind === "constraint" && <ConstraintForm />}
      {f.kind === "rotation" && <HeadingForm />}
      {f.kind === "event" && <EventForm />}
      {f.kind === "point" && <PointForm />}
    </div>
  );
}

/** Focuses (and selects) an input once the click that opened the form is over (the field takes focus on press). */
function useFocus() {
  const ref = useRef<HTMLInputElement>(null);
  useEffect(() => {
    const t = setTimeout(() => {
      ref.current?.focus();
      ref.current?.select();
    }, 30);
    return () => clearTimeout(t);
  }, []);
  return ref;
}

function Field({ label, children, id }: { label: string; children: React.ReactNode; id: string }) {
  return (
    <div className="field">
      <label className="field-label" htmlFor={id}>
        {label}
      </label>
      <span className="field-input">{children}</span>
    </div>
  );
}

function ConstraintForm() {
  const id = useId();
  const [speed, setSpeed] = useState("1.5");
  const focus = useFocus();
  const [accel, setAccel] = useState("");
  const v = Number(speed);
  const a = accel.trim() === "" ? Infinity : Number(accel);
  const valid = speed.trim() !== "" && v > 0 && a > 0;
  const submit = () => valid && finishConstraint(v, a);
  return (
    <form
      onSubmit={(e) => {
        e.preventDefault();
        submit();
      }}
    >
      <Field label="Max speed (m/s)" id={`${id}-v`}>
        <input ref={focus} id={`${id}-v`} inputMode="decimal" value={speed} onChange={(e) => setSpeed(e.target.value)} onFocus={(e) => e.target.select()} data-testid="flow-speed" />
      </Field>
      <Field label="Max accel (m/s², blank: none)" id={`${id}-a`}>
        <input id={`${id}-a`} inputMode="decimal" value={accel} placeholder="none" onChange={(e) => setAccel(e.target.value)} data-testid="flow-accel" />
      </Field>
      <button type="submit" className="btn primary block" disabled={!valid} data-testid="flow-done">
        Add zone
      </button>
    </form>
  );
}

function HeadingForm() {
  const id = useId();
  const heading = useStore(flow, (s) => s.heading);
  const [text, setText] = useState(formatNumber(heading / DEG, 1));
  const editing = useRef(false);
  const focus = useFocus();
  // the knob on the field turns it too
  useEffect(() => {
    if (!editing.current) setText(formatNumber(heading / DEG, 1));
  }, [heading]);
  return (
    <form
      onSubmit={(e) => {
        e.preventDefault();
        finishRotation();
      }}
    >
      <Field label="Heading (°)" id={id}>
        <input
          ref={focus}
          id={id}
          inputMode="decimal"
          value={text}
          onFocus={(e) => {
            editing.current = true;
            e.target.select();
          }}
          onBlur={() => (editing.current = false)}
          onChange={(e) => {
            setText(e.target.value);
            const v = Number(e.target.value);
            if (e.target.value.trim() !== "" && Number.isFinite(v)) setFlowHeading(v * DEG);
          }}
          data-testid="flow-heading"
        />
      </Field>
      <p className="hint">Or drag the knob on the robot's ring and let go (15° steps; Alt turns freely).</p>
      <button type="submit" className="btn primary block" data-testid="flow-done">
        Add heading
      </button>
    </form>
  );
}

function EventForm() {
  const id = useId();
  const project = useStore(editor, (s) => s.project);
  const names = useMemo(() => eventNames(project), [project]);
  const [name, setName] = useState("");
  const focus = useFocus();
  return (
    <form
      onSubmit={(e) => {
        e.preventDefault();
        finishEvent(name);
      }}
    >
      <Field label="Event name" id={id}>
        <input ref={focus} id={id} list={`${id}-names`} value={name} autoComplete="off" spellCheck={false} onChange={(e) => setName(e.target.value)} data-testid="flow-name" />
        <datalist id={`${id}-names`}>
          {names.map((n) => (
            <option key={n} value={n} />
          ))}
        </datalist>
      </Field>
      {names.length > 0 && (
        <div className="chips" aria-label="Names in this project">
          {names.slice(0, 6).map((n) => (
            <button key={n} type="button" className="chip" onClick={() => finishEvent(n)}>
              {n}
            </button>
          ))}
        </div>
      )}
      <button type="submit" className="btn primary block" disabled={name.trim() === ""} data-testid="flow-done">
        Add event
      </button>
    </form>
  );
}

function PointForm() {
  const id = useId();
  const f = useStore(flow, (s) => s);
  const fallback = useStore(editor, (s) => uniquePointName(s.project));
  const [name, setName] = useState(f.point?.snappedTo ? `Near ${f.point.snappedTo}` : fallback);
  const [heading, setHeading] = useState("");
  const focus = useFocus();
  const h = heading.trim() === "" ? Number.NaN : Number(heading) * DEG;
  return (
    <form
      onSubmit={(e) => {
        e.preventDefault();
        finishPoint(name, h);
      }}
    >
      <Field label="Name" id={`${id}-n`}>
        <input ref={focus} id={`${id}-n`} value={name} onChange={(e) => setName(e.target.value)} data-testid="flow-name" />
      </Field>
      <Field label="Heading (°, blank: none)" id={`${id}-h`}>
        <input id={`${id}-h`} inputMode="decimal" value={heading} placeholder="none" onChange={(e) => setHeading(e.target.value)} data-testid="flow-heading" />
      </Field>
      <button type="submit" className="btn primary block" disabled={name.trim() === "" || (heading.trim() !== "" && !Number.isFinite(h))} data-testid="flow-done">
        Add named point
      </button>
    </form>
  );
}
