// Form controls that commit on Enter or blur (one undo step per edit, not one per keystroke) and
// revert on Escape. Every control has a visible label.

import { useEffect, useId, useState, type ReactNode } from "react";

/** A number as the file would print it: up to 5 decimals, no trailing zeros. */
export function formatNumber(x: number, decimals = 5): string {
  if (!Number.isFinite(x)) return "";
  const s = x.toFixed(decimals);
  return s.includes(".") ? s.replace(/\.?0+$/, "") : s;
}

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
  readonly title?: string;
  readonly testId?: string;
}

export function NumberField({ label, value, onCommit, unit, blank, blankValue, decimals = 5, min, title, testId }: NumberFieldProps) {
  const id = useId();
  const shown = formatNumber(value, decimals);
  const [text, setText] = useState(shown);
  useEffect(() => setText(shown), [shown]);

  const commit = () => {
    const t = text.trim();
    if (t === "") {
      if (blankValue !== undefined) {
        if (!(Number.isNaN(blankValue) && Number.isNaN(value)) && blankValue !== value) onCommit(blankValue);
      } else setText(shown);
      return;
    }
    const v = Number(t);
    if (!Number.isFinite(v) || (min !== undefined && v < min)) {
      setText(shown);
      return;
    }
    if (v !== value) onCommit(v);
  };

  return (
    <label className="field" htmlFor={id} title={title}>
      <span className="field-label">{label}</span>
      <span className="field-input">
        <input
          id={id}
          data-testid={testId}
          inputMode="decimal"
          value={text}
          placeholder={blank}
          onChange={(e) => setText(e.target.value)}
          onBlur={commit}
          onKeyDown={(e) => {
            if (e.key === "Enter") (e.target as HTMLInputElement).blur();
            if (e.key === "Escape") {
              setText(shown);
              (e.target as HTMLInputElement).blur();
            }
          }}
        />
        {unit && <span className="unit">{unit}</span>}
      </span>
    </label>
  );
}

interface TextFieldProps {
  readonly label: string;
  readonly value: string;
  readonly onCommit: (value: string) => void;
  /** Suggestions (a datalist). */
  readonly options?: readonly string[];
  readonly validate?: (value: string) => string | null;
  readonly testId?: string;
  readonly autoFocus?: boolean;
}

export function TextField({ label, value, onCommit, options, validate, testId, autoFocus }: TextFieldProps) {
  const id = useId();
  const listId = `${id}-list`;
  const [text, setText] = useState(value);
  const [error, setError] = useState<string | null>(null);
  useEffect(() => setText(value), [value]);

  const commit = () => {
    const t = text.trim();
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
    <label className="field" htmlFor={id}>
      <span className="field-label">{label}</span>
      <span className="field-input">
        <input
          id={id}
          data-testid={testId}
          value={text}
          list={options ? listId : undefined}
          autoFocus={autoFocus}
          aria-invalid={error !== null}
          onChange={(e) => setText(e.target.value)}
          onBlur={commit}
          onKeyDown={(e) => {
            if (e.key === "Enter") (e.target as HTMLInputElement).blur();
            if (e.key === "Escape") {
              setText(value);
              (e.target as HTMLInputElement).blur();
            }
          }}
        />
        {options && (
          <datalist id={listId}>
            {options.map((o) => (
              <option key={o} value={o} />
            ))}
          </datalist>
        )}
      </span>
      {error && <span className="field-error" role="alert">{error}</span>}
    </label>
  );
}

interface SelectFieldProps<T extends string> {
  readonly label: string;
  readonly value: T;
  readonly options: readonly { readonly value: T; readonly label: string }[];
  readonly onChange: (value: T) => void;
}

export function SelectField<T extends string>({ label, value, options, onChange }: SelectFieldProps<T>) {
  const id = useId();
  return (
    <label className="field" htmlFor={id}>
      <span className="field-label">{label}</span>
      <span className="field-input">
        <select id={id} value={value} onChange={(e) => onChange(e.target.value as T)}>
          {options.map((o) => (
            <option key={o.value} value={o.value}>
              {o.label}
            </option>
          ))}
        </select>
      </span>
    </label>
  );
}

export function Section({ title, children, open = false, count }: { title: string; children: ReactNode; open?: boolean; count?: number }) {
  return (
    <details className="section" open={open}>
      <summary>
        {title}
        {count !== undefined && <span className="count">{count}</span>}
      </summary>
      <div className="section-body">{children}</div>
    </details>
  );
}

export function IconButton({ label, onClick, children, disabled, testId }: { label: string; onClick: () => void; children: ReactNode; disabled?: boolean; testId?: string }) {
  return (
    <button type="button" className="icon-button" aria-label={label} title={label} onClick={onClick} disabled={disabled} data-testid={testId}>
      {children}
    </button>
  );
}
