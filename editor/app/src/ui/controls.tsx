// The editor's controls, drawn by the app rather than the browser: number fields that scrub (drag the
// label) and step (arrows, or the steppers), selects with a custom list, switches, a slider, segmented
// choices, collapsible cards and icon buttons with tooltips. Edits commit on Enter, blur, a step or the
// end of a scrub (one undo step each), and Escape reverts. Every control has an accessible name.

import { Check, ChevronDown, ChevronRight, ChevronUp } from "lucide-react";
import { useEffect, useId, useLayoutEffect, useRef, useState, type CSSProperties, type ReactNode, type RefObject } from "react";
import { createPortal } from "react-dom";
import { createStore, useStore } from "../state/store.ts";

/** A number as the file would print it: up to 5 decimals, no trailing zeros. */
export function formatNumber(x: number, decimals = 5): string {
  if (!Number.isFinite(x)) return "";
  const s = x.toFixed(decimals);
  return s.includes(".") ? s.replace(/\.?0+$/, "") : s;
}

const DEFAULT_STEPS: Record<string, number> = { "°": 5, m: 0.05, "m/s": 0.1, "m/s²": 0.1, wp: 0.1, s: 0.1, kg: 0.5, A: 5, ms: 5 };

interface NumberFieldProps {
  readonly label: string;
  readonly value: number;
  readonly onCommit: (value: number) => void;
  readonly unit?: string;
  /** Shown when the value is blank (NaN or infinite), e.g. "auto" or "none". */
  readonly blank?: string;
  /** What a blank entry commits; leave out to refuse blanks. */
  readonly blankValue?: number;
  readonly decimals?: number;
  readonly min?: number;
  readonly max?: number;
  /** The arrow keys' and steppers' step (Shift: ten of them); a scrub moves one step per 4 px. */
  readonly step?: number;
  readonly title?: string;
  readonly testId?: string;
  readonly className?: string;
}

export function NumberField({ label, value, onCommit, unit, blank, blankValue, decimals = 5, min, max, step, title, testId, className }: NumberFieldProps) {
  const id = useId();
  const shown = formatNumber(value, decimals);
  const [text, setText] = useState(shown);
  const [scrubbing, setScrubbing] = useState(false);
  const input = useRef<HTMLInputElement>(null);
  useEffect(() => setText(shown), [shown]);
  const stepSize = step ?? (decimals === 0 ? 1 : (DEFAULT_STEPS[unit ?? ""] ?? 0.1));

  const clamp = (v: number) => Math.min(max ?? Infinity, Math.max(min ?? -Infinity, v));
  const round = (v: number) => Number(v.toFixed(Math.min(decimals, 5)));
  const commitValue = (v: number) => {
    const r = round(clamp(v));
    if (Number.isFinite(r) && r !== value) onCommit(r);
  };

  const commit = () => {
    const t = text.trim();
    if (t === "") {
      if (blankValue !== undefined) {
        if (!(Number.isNaN(blankValue) && Number.isNaN(value)) && blankValue !== value) onCommit(blankValue);
      } else setText(shown);
      return;
    }
    const v = Number(t);
    if (!Number.isFinite(v) || (min !== undefined && v < min) || (max !== undefined && v > max)) {
      setText(shown);
      return;
    }
    if (v !== value) onCommit(v);
  };

  const base = () => {
    const v = Number(text.trim());
    return text.trim() !== "" && Number.isFinite(v) ? v : Number.isFinite(value) ? value : 0;
  };
  const stepBy = (n: number) => commitValue(base() + n * stepSize);

  // drag the label to scrub; a click without a drag focuses the input
  const onLabelDown = (e: React.PointerEvent<HTMLLabelElement>) => {
    if (e.button !== 0) return;
    e.preventDefault();
    const start = e.clientX;
    const from = base();
    let moved = false;
    let latest = from;
    const target = e.currentTarget;
    target.setPointerCapture(e.pointerId);
    const move = (ev: PointerEvent) => {
      const dx = ev.clientX - start;
      if (!moved && Math.abs(dx) < 3) return;
      if (!moved) {
        moved = true;
        setScrubbing(true);
      }
      const per = ev.shiftKey ? stepSize / 10 : stepSize;
      latest = round(clamp(from + Math.round(dx / 4) * per));
      setText(formatNumber(latest, decimals));
    };
    const up = () => {
      target.removeEventListener("pointermove", move);
      target.removeEventListener("pointerup", up);
      target.removeEventListener("pointercancel", up);
      setScrubbing(false);
      if (moved) commitValue(latest);
      else input.current?.focus();
    };
    target.addEventListener("pointermove", move);
    target.addEventListener("pointerup", up);
    target.addEventListener("pointercancel", up);
  };

  return (
    <div className={`field nf${scrubbing ? " scrubbing" : ""}${className ? ` ${className}` : ""}`} data-tip={title}>
      <label className="field-label" id={`${id}-label`} htmlFor={id} onPointerDown={onLabelDown}>
        {label}
      </label>
      <span className="field-input">
        <input
          ref={input}
          id={id}
          data-testid={testId}
          // named by the label alone; the unit is its description
          aria-labelledby={`${id}-label`}
          aria-describedby={unit ? `${id}-unit` : undefined}
          inputMode="decimal"
          autoComplete="off"
          spellCheck={false}
          value={text}
          placeholder={blank}
          onChange={(e) => setText(e.target.value)}
          onBlur={commit}
          onKeyDown={(e) => {
            if (e.key === "Enter") (e.target as HTMLInputElement).blur();
            else if (e.key === "Escape") {
              setText(shown);
              (e.target as HTMLInputElement).blur();
            } else if (e.key === "ArrowUp" || e.key === "ArrowDown") {
              e.preventDefault();
              stepBy((e.key === "ArrowUp" ? 1 : -1) * (e.shiftKey ? 10 : 1));
            }
          }}
        />
        {unit && (
          <span className="unit" id={`${id}-unit`}>
            {unit}
          </span>
        )}
        {/* for the pointer; the keyboard's arrows step the field itself */}
        <span className="steppers" aria-hidden="true">
          <button type="button" tabIndex={-1} onPointerDown={(e) => e.preventDefault()} onClick={(e) => stepBy(e.shiftKey ? 10 : 1)}>
            <ChevronUp size={10} strokeWidth={2.5} />
          </button>
          <button type="button" tabIndex={-1} onPointerDown={(e) => e.preventDefault()} onClick={(e) => stepBy(e.shiftKey ? -10 : -1)}>
            <ChevronDown size={10} strokeWidth={2.5} />
          </button>
        </span>
      </span>
    </div>
  );
}

// ------------------------------------------------------------------------------ popovers

/** Renders into document.body, so fixed positioning isn't caught by a transformed ancestor. */
export function Portal({ children }: { children: ReactNode }) {
  return createPortal(children, document.body);
}

/** Where a popover under (or above, if there's no room) an anchor goes. */
export function usePopoverPosition(anchor: RefObject<HTMLElement | null>, open: boolean, width?: number, alignRight = false): CSSProperties {
  const [style, setStyle] = useState<CSSProperties>({ visibility: "hidden" });
  useLayoutEffect(() => {
    if (!open || !anchor.current) return;
    const place = () => {
      const r = anchor.current!.getBoundingClientRect();
      const below = window.innerHeight - r.bottom;
      const w = width ?? Math.max(r.width, 160);
      const left = Math.min(Math.max(8, alignRight ? r.right - w : r.left), window.innerWidth - w - 8);
      setStyle(
        below > 240 || below > r.top
          ? { left, top: r.bottom + 4, minWidth: w, maxHeight: Math.max(120, below - 16) }
          : { left, bottom: window.innerHeight - r.top + 4, minWidth: w, maxHeight: Math.max(120, r.top - 16) },
      );
    };
    place();
    window.addEventListener("resize", place);
    return () => window.removeEventListener("resize", place);
  }, [open, anchor, width, alignRight]);
  return style;
}

/** Closes on a press outside both the anchor and the popover, or on Escape. */
export function useDismiss(open: boolean, close: () => void, ...refs: RefObject<HTMLElement | null>[]): void {
  const closeRef = useRef(close);
  closeRef.current = close;
  useEffect(() => {
    if (!open) return;
    const down = (e: PointerEvent) => {
      if (refs.some((r) => r.current?.contains(e.target as Node))) return;
      closeRef.current();
    };
    const key = (e: KeyboardEvent) => {
      if (e.key === "Escape") {
        e.stopPropagation();
        closeRef.current();
      }
    };
    document.addEventListener("pointerdown", down, true);
    document.addEventListener("keydown", key, true);
    return () => {
      document.removeEventListener("pointerdown", down, true);
      document.removeEventListener("keydown", key, true);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [open]);
}

// ------------------------------------------------------------------------------ text

interface TextFieldProps {
  readonly label: string;
  readonly value: string;
  readonly onCommit: (value: string) => void;
  /** Suggestions, shown in a list under the field as you type. */
  readonly options?: readonly string[];
  readonly validate?: (value: string) => string | null;
  readonly testId?: string;
  readonly autoFocus?: boolean;
  readonly placeholder?: string;
}

export function TextField({ label, value, onCommit, options, validate, testId, autoFocus, placeholder }: TextFieldProps) {
  const id = useId();
  const [text, setText] = useState(value);
  const [error, setError] = useState<string | null>(null);
  const [open, setOpen] = useState(false);
  const [active, setActive] = useState(-1);
  const wrap = useRef<HTMLSpanElement>(null);
  const list = useRef<HTMLUListElement>(null);
  useEffect(() => setText(value), [value]);
  const matches = options ? options.filter((o) => o !== text && o.toLowerCase().includes(text.trim().toLowerCase())).slice(0, 8) : [];
  const showList = open && matches.length > 0;
  const style = usePopoverPosition(wrap, showList);
  useDismiss(showList, () => setOpen(false), wrap, list);

  const commit = (t0 = text) => {
    const t = t0.trim();
    const problem = t === "" ? "Can't be empty" : (validate?.(t) ?? null);
    if (problem) {
      setError(problem);
      setText(value);
      return;
    }
    setError(null);
    if (t !== value) onCommit(t);
  };

  return (
    <div className="field">
      <label className="field-label" htmlFor={id}>
        {label}
      </label>
      <span ref={wrap} className={error ? "field-input invalid" : "field-input"}>
        <input
          id={id}
          data-testid={testId}
          value={text}
          autoFocus={autoFocus}
          autoComplete="off"
          spellCheck={false}
          placeholder={placeholder}
          aria-invalid={error !== null}
          aria-autocomplete={options ? "list" : undefined}
          aria-expanded={options ? showList : undefined}
          aria-controls={options ? `${id}-list` : undefined}
          role={options ? "combobox" : undefined}
          onChange={(e) => {
            setText(e.target.value);
            setOpen(true);
            setActive(-1);
          }}
          onFocus={() => setOpen(true)}
          onBlur={() => {
            setOpen(false);
            commit();
          }}
          onKeyDown={(e) => {
            if (showList && (e.key === "ArrowDown" || e.key === "ArrowUp")) {
              e.preventDefault();
              setActive((a) => (a + (e.key === "ArrowDown" ? 1 : matches.length - 1) + (a < 0 ? 1 : 0)) % matches.length);
            } else if (e.key === "Enter") {
              const picked = showList && active >= 0 ? matches[active] : undefined;
              if (picked !== undefined) setText(picked);
              setOpen(false);
              if (picked !== undefined) {
                commit(picked);
                (e.target as HTMLInputElement).blur();
              } else (e.target as HTMLInputElement).blur();
            } else if (e.key === "Escape") {
              if (showList) setOpen(false);
              else {
                setText(value);
                (e.target as HTMLInputElement).blur();
              }
            }
          }}
        />
      </span>
      {showList && (
        <Portal>
          <ul ref={list} id={`${id}-list`} role="listbox" className="popover" style={style} aria-label={`${label} suggestions`}>
            {matches.map((m, i) => (
              <li
                key={m}
                role="option"
                aria-selected={i === active}
                className={i === active ? "option active" : "option"}
                onPointerDown={(e) => {
                  // keep the input focused, then take the suggestion
                  e.preventDefault();
                  setText(m);
                  setOpen(false);
                  commit(m);
                }}
              >
                {m}
              </li>
            ))}
          </ul>
        </Portal>
      )}
      {error && (
        <span className="field-error" role="alert">
          {error}
        </span>
      )}
    </div>
  );
}

// ------------------------------------------------------------------------------ select

interface SelectProps<T extends string> {
  readonly label: string;
  readonly value: T;
  readonly options: readonly { readonly value: T; readonly label: string }[];
  readonly onChange: (value: T) => void;
  /** Hide the label above (it still names the control). */
  readonly inline?: boolean;
  readonly testId?: string;
  readonly title?: string;
}

/** A select drawn by the app: a button that opens a list (arrows, Enter, Escape, first letters). */
export function SelectField<T extends string>({ label, value, options, onChange, inline, testId, title }: SelectProps<T>) {
  const id = useId();
  const [open, setOpen] = useState(false);
  const [active, setActive] = useState(0);
  const button = useRef<HTMLButtonElement>(null);
  const list = useRef<HTMLUListElement>(null);
  const style = usePopoverPosition(button, open);
  const current = options.find((o) => o.value === value);
  useDismiss(open, () => setOpen(false), button, list);

  const show = () => {
    setActive(Math.max(0, options.findIndex((o) => o.value === value)));
    setOpen(true);
  };
  const pick = (i: number) => {
    const o = options[i];
    setOpen(false);
    button.current?.focus();
    if (o && o.value !== value) onChange(o.value);
  };

  return (
    <div className="field" data-tip={title}>
      <span className={inline ? "sr-only" : "field-label"} id={`${id}-label`}>
        {label}
      </span>
      <button
        ref={button}
        type="button"
        className={inline ? "select-btn inline" : "select-btn"}
        role="combobox"
        aria-haspopup="listbox"
        aria-expanded={open}
        aria-controls={`${id}-list`}
        aria-labelledby={`${id}-label`}
        aria-activedescendant={open ? `${id}-o${active}` : undefined}
        data-value={value}
        data-testid={testId}
        onClick={() => (open ? setOpen(false) : show())}
        onKeyDown={(e) => {
          if (!open && (e.key === "ArrowDown" || e.key === "ArrowUp" || e.key === "Enter" || e.key === " ")) {
            e.preventDefault();
            show();
          } else if (open) {
            if (e.key === "ArrowDown" || e.key === "ArrowUp") {
              e.preventDefault();
              setActive((a) => (a + (e.key === "ArrowDown" ? 1 : options.length - 1)) % options.length);
            } else if (e.key === "Enter" || e.key === " ") {
              e.preventDefault();
              pick(active);
            } else if (e.key === "Tab") {
              setOpen(false);
            } else if (e.key.length === 1) {
              const i = options.findIndex((o) => o.label.toLowerCase().startsWith(e.key.toLowerCase()));
              if (i >= 0) setActive(i);
            }
          }
        }}
      >
        <span className="value">{current?.label ?? value}</span>
        <ChevronDown size={14} />
      </button>
      {open && (
        <Portal>
          <ul ref={list} id={`${id}-list`} role="listbox" aria-labelledby={`${id}-label`} className="popover" style={style}>
            {options.map((o, i) => (
              <li
                key={o.value}
                id={`${id}-o${i}`}
                role="option"
                aria-selected={o.value === value}
                className={i === active ? "option active" : "option"}
                onPointerEnter={() => setActive(i)}
                onPointerDown={(e) => e.preventDefault()}
                onClick={() => pick(i)}
              >
                <span className="tick">{o.value === value && <Check size={14} />}</span>
                {o.label}
              </li>
            ))}
          </ul>
        </Portal>
      )}
    </div>
  );
}

// ------------------------------------------------------------------------------ switch, slider, segmented

export function Switch({
  label,
  checked,
  onChange,
  disabled,
  title,
  testId,
}: {
  label: ReactNode;
  checked: boolean;
  onChange: (checked: boolean) => void;
  disabled?: boolean;
  title?: string;
  testId?: string;
}) {
  return (
    <button
      type="button"
      role="switch"
      className="switch"
      aria-checked={checked}
      disabled={disabled}
      data-tip={title}
      data-testid={testId}
      onClick={() => onChange(!checked)}
    >
      <span className="track" aria-hidden="true" />
      <span>{label}</span>
    </button>
  );
}

export function Slider({
  label,
  value,
  min,
  max,
  step,
  onChange,
  disabled,
  valueText,
  testId,
}: {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  onChange: (v: number) => void;
  disabled?: boolean;
  valueText?: string;
  testId?: string;
}) {
  const ref = useRef<HTMLDivElement>(null);
  const f = max > min ? Math.min(1, Math.max(0, (value - min) / (max - min))) : 0;
  const at = (clientX: number) => {
    const r = ref.current!.getBoundingClientRect();
    const v = min + Math.min(1, Math.max(0, (clientX - r.left) / r.width)) * (max - min);
    return Math.round(v / step) * step;
  };
  return (
    <div
      ref={ref}
      className="slider"
      role="slider"
      tabIndex={disabled ? -1 : 0}
      aria-label={label}
      aria-valuemin={min}
      aria-valuemax={max}
      aria-valuenow={value}
      aria-valuetext={valueText}
      aria-disabled={disabled}
      data-testid={testId}
      onPointerDown={(e) => {
        if (disabled || e.button !== 0) return;
        e.currentTarget.setPointerCapture(e.pointerId);
        onChange(at(e.clientX));
      }}
      onPointerMove={(e) => {
        if (!disabled && e.currentTarget.hasPointerCapture(e.pointerId)) onChange(at(e.clientX));
      }}
      onKeyDown={(e) => {
        if (disabled) return;
        const big = (max - min) / 10;
        const next =
          e.key === "ArrowRight" || e.key === "ArrowUp"
            ? value + step
            : e.key === "ArrowLeft" || e.key === "ArrowDown"
              ? value - step
              : e.key === "PageUp"
                ? value + big
                : e.key === "PageDown"
                  ? value - big
                  : e.key === "Home"
                    ? min
                    : e.key === "End"
                      ? max
                      : null;
        if (next === null) return;
        e.preventDefault();
        e.stopPropagation();
        onChange(Math.min(max, Math.max(min, next)));
      }}
    >
      <span className="rail" />
      <span className="fill" style={{ width: `${f * 100}%` }} />
      <span className="thumb" style={{ left: `${f * 100}%` }} />
    </div>
  );
}

export function Segmented<T extends string>({
  label,
  value,
  options,
  onChange,
}: {
  label: string;
  value: T;
  options: readonly { value: T; label: ReactNode; tip?: string }[];
  onChange: (v: T) => void;
}) {
  return (
    <div className="segmented" role="radiogroup" aria-label={label}>
      {options.map((o) => (
        <button key={o.value} type="button" role="radio" aria-checked={o.value === value} data-tip={o.tip} onClick={() => onChange(o.value)}>
          {o.label}
        </button>
      ))}
    </div>
  );
}

// ------------------------------------------------------------------------------ cards and buttons

/** Which cards are open, by key, so they stay as left when the inspector switches views. */
const cardsOpen = createStore<Record<string, boolean>>({});

export function Card({
  title,
  children,
  open = false,
  count,
  cardKey,
  testId,
}: {
  title: string;
  children: ReactNode;
  open?: boolean;
  count?: number;
  cardKey?: string;
  testId?: string;
}) {
  const key = cardKey ?? title;
  const isOpen = useStore(cardsOpen, (s) => s[key] ?? open);
  const id = useId();
  return (
    <section className="card" data-testid={testId} aria-labelledby={`${id}-head`}>
      <button
        type="button"
        id={`${id}-head`}
        className="card-head"
        aria-expanded={isOpen}
        aria-controls={`${id}-body`}
        onClick={() => cardsOpen.set((s) => ({ ...s, [key]: !isOpen }))}
      >
        <ChevronRight className="chev" size={14} />
        {title}
        {count !== undefined && <span className="count">{count}</span>}
      </button>
      <div className={isOpen ? "collapse open" : "collapse"} id={`${id}-body`}>
        <div className="collapse-inner">{isOpen && <div className="card-body">{children}</div>}</div>
      </div>
    </section>
  );
}

/** Opens a card (for actions that add something into it). */
export function openCard(key: string): void {
  cardsOpen.set((s) => ({ ...s, [key]: true }));
}

/** An icon button; its tooltip gives the label and the shortcut. */
export function IconButton({
  label,
  onClick,
  children,
  disabled,
  testId,
  shortcut,
  pressed,
  className,
}: {
  label: string;
  onClick: () => void;
  children: ReactNode;
  disabled?: boolean;
  testId?: string;
  shortcut?: string;
  pressed?: boolean;
  className?: string;
}) {
  return (
    <button
      type="button"
      className={className ? `icon-btn ${className}` : "icon-btn"}
      aria-label={label}
      aria-pressed={pressed}
      aria-keyshortcuts={shortcut}
      data-tip={shortcut ? `${label}|${shortcut}` : label}
      onClick={onClick}
      disabled={disabled}
      data-testid={testId}
    >
      {children}
    </button>
  );
}

/** A text button (with an optional icon). */
export function Button({
  children,
  onClick,
  variant,
  disabled,
  testId,
  title,
  label,
  className,
}: {
  children: ReactNode;
  onClick: () => void;
  variant?: "primary" | "ghost" | "danger";
  disabled?: boolean;
  testId?: string;
  title?: string;
  label?: string;
  className?: string;
}) {
  return (
    <button
      type="button"
      className={["btn", variant, className].filter(Boolean).join(" ")}
      onClick={onClick}
      disabled={disabled}
      data-testid={testId}
      data-tip={title}
      aria-label={label}
    >
      {children}
    </button>
  );
}
