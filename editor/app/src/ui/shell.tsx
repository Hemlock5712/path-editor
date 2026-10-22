// The app's frame: resizable panes, tooltips, toasts, the context menu, the command palette, the help
// dialog, and the behaviour that makes the browser window act like an app (no page zoom, no text
// selection or drag ghosts on the chrome, no browser context menu).

import { AlertCircle, CheckCircle2, Clipboard, Copy, Info, Scissors, Search, TextSelect } from "lucide-react";
import { useEffect, useMemo, useRef, useState, type ReactNode } from "react";
import { fitView, zoomBy } from "../state/actions.ts";
import { question } from "../state/confirm.ts";
import { useStore } from "../state/store.ts";
import { closeContextMenu, contextMenu, openContextMenu, toasts, ui, type MenuEntry } from "../state/ui.ts";
import { COMMANDS } from "./commands.ts";
import { Portal } from "./controls.tsx";
import { QuickStartList } from "./QuickStart.tsx";
import { SHORTCUTS } from "./shortcuts.ts";

// ------------------------------------------------------------------------------ the app icon

/** The app's mark: a path with its start pose, on the accent. */
export function AppIcon({ size = 20 }: { size?: number }) {
  return (
    <svg width={size} height={size} viewBox="0 0 32 32" aria-hidden="true">
      <rect width="32" height="32" rx="8" fill="#2e8cf0" />
      <path d="M7 24 C13 24 12 9 25 9" stroke="#fff" strokeWidth="2.6" fill="none" strokeLinecap="round" />
      <rect x="4" y="21" width="6" height="6" rx="1.5" fill="#fbbf24" />
      <circle cx="25" cy="9" r="2.6" fill="#fff" />
    </svg>
  );
}

// ------------------------------------------------------------------------------ panes

/** A splitter between two panes: drag (or arrow keys) to resize; the width is the side pane's. */
export function Splitter({
  label,
  width,
  min,
  max,
  side,
  onResize,
}: {
  label: string;
  width: number;
  min: number;
  max: number;
  side: "left" | "right";
  onResize: (width: number) => void;
}) {
  const [dragging, setDragging] = useState(false);
  return (
    <div
      className={dragging ? "splitter dragging" : "splitter"}
      role="separator"
      aria-orientation="vertical"
      aria-label={label}
      aria-valuemin={min}
      aria-valuemax={max}
      aria-valuenow={Math.round(width)}
      tabIndex={0}
      onPointerDown={(e) => {
        if (e.button !== 0) return;
        e.preventDefault();
        e.currentTarget.setPointerCapture(e.pointerId);
        const x0 = e.clientX;
        const w0 = width;
        setDragging(true);
        const el = e.currentTarget;
        const move = (ev: PointerEvent) => {
          const dx = ev.clientX - x0;
          onResize(Math.min(max, Math.max(min, side === "left" ? w0 + dx : w0 - dx)));
        };
        const up = () => {
          setDragging(false);
          el.removeEventListener("pointermove", move);
          el.removeEventListener("pointerup", up);
          el.removeEventListener("pointercancel", up);
        };
        el.addEventListener("pointermove", move);
        el.addEventListener("pointerup", up);
        el.addEventListener("pointercancel", up);
      }}
      onDoubleClick={() => onResize(side === "left" ? min + (max - min) * 0.3 : min + (max - min) * 0.2)}
      onKeyDown={(e) => {
        if (e.key !== "ArrowLeft" && e.key !== "ArrowRight") return;
        e.preventDefault();
        const d = (e.key === "ArrowRight" ? 16 : -16) * (side === "left" ? 1 : -1);
        onResize(Math.min(max, Math.max(min, width + d)));
      }}
    />
  );
}

// ------------------------------------------------------------------------------ tooltips

/** One tooltip for the whole app: any element with data-tip="Label|Shortcut" gets it on hover or focus. */
export function TooltipHost() {
  const [tip, setTip] = useState<{ text: string; key?: string; x: number; y: number; below: boolean } | null>(null);
  useEffect(() => {
    let timer = 0;
    let watch: MutationObserver | null = null;
    let current: HTMLElement | null = null;
    const show = (el: HTMLElement) => {
      const raw = el.dataset.tip;
      if (!raw) return;
      const [text, key] = raw.split("|");
      const r = el.getBoundingClientRect();
      const below = r.bottom + 40 < window.innerHeight;
      setTip({ text: text!, key, x: Math.min(window.innerWidth - 16, Math.max(16, r.left + r.width / 2)), y: below ? r.bottom + 6 : r.top - 6, below });
      // a control swapped out from under the pointer (Stop becoming Optimise) takes its tip with it
      watch?.disconnect();
      watch = new MutationObserver(() => {
        if (!el.isConnected || el.dataset.tip !== raw) hide();
      });
      watch.observe(el, { attributes: true, attributeFilter: ["data-tip"] });
      if (el.parentElement) watch.observe(el.parentElement, { childList: true });
    };
    const over = (e: Event) => {
      const el = (e.target as HTMLElement).closest?.("[data-tip]") as HTMLElement | null;
      if (el === current) return;
      current = el;
      window.clearTimeout(timer);
      watch?.disconnect();
      setTip(null);
      if (el) timer = window.setTimeout(() => show(el), e.type === "focusin" ? 300 : 500);
    };
    const hide = () => {
      window.clearTimeout(timer);
      watch?.disconnect();
      current = null;
      setTip(null);
    };
    document.addEventListener("pointerover", over);
    document.addEventListener("focusin", over);
    document.addEventListener("pointerdown", hide, true);
    document.addEventListener("keydown", hide, true);
    window.addEventListener("blur", hide);
    return () => {
      window.clearTimeout(timer);
      watch?.disconnect();
      document.removeEventListener("pointerover", over);
      document.removeEventListener("focusin", over);
      document.removeEventListener("pointerdown", hide, true);
      document.removeEventListener("keydown", hide, true);
      window.removeEventListener("blur", hide);
    };
  }, []);
  if (!tip) return null;
  return (
    <Portal>
      <div
        className="tooltip"
        role="tooltip"
        style={{ left: tip.x, top: tip.below ? tip.y : undefined, bottom: tip.below ? undefined : window.innerHeight - tip.y }}
      >
        {tip.text}
        {tip.key && <kbd>{tip.key}</kbd>}
      </div>
    </Portal>
  );
}

// ------------------------------------------------------------------------------ toasts

export function Toasts() {
  const list = useStore(toasts, (s) => s.list);
  return (
    <div className="toasts" aria-live="polite">
      {list.map((t) => (
        <div key={t.id} className={`toast ${t.kind}`} role={t.kind === "error" ? "alert" : "status"}>
          {t.kind === "error" ? <AlertCircle size={16} /> : t.kind === "ok" ? <CheckCircle2 size={16} /> : <Info size={16} />}
          <span>{t.text}</span>
          {t.action && (
            <button
              type="button"
              className="btn small primary"
              data-testid="toast-action"
              onClick={() => {
                t.action!.run();
                toasts.set((s) => ({ list: s.list.filter((x) => x.id !== t.id) }));
              }}
            >
              {t.action.label}
            </button>
          )}
        </div>
      ))}
    </div>
  );
}

// ------------------------------------------------------------------------------ context menu

export function ContextMenuHost() {
  const at = useStore(contextMenu, (s) => s.at);
  const items = useStore(contextMenu, (s) => s.items);
  const label = useStore(contextMenu, (s) => s.label);
  const ref = useRef<HTMLDivElement>(null);
  const [pos, setPos] = useState({ left: 0, top: 0 });

  useEffect(() => {
    if (!at || !ref.current) return;
    const r = ref.current.getBoundingClientRect();
    setPos({ left: Math.min(at.x, window.innerWidth - r.width - 8), top: Math.min(at.y, window.innerHeight - r.height - 8) });
    ref.current.querySelector<HTMLButtonElement>("button:not(:disabled)")?.focus();
    const down = (e: PointerEvent) => !ref.current?.contains(e.target as Node) && closeContextMenu();
    const key = (e: KeyboardEvent) => {
      if (e.key === "Escape") {
        e.stopPropagation();
        closeContextMenu();
      }
    };
    document.addEventListener("pointerdown", down, true);
    document.addEventListener("keydown", key, true);
    window.addEventListener("blur", closeContextMenu);
    return () => {
      document.removeEventListener("pointerdown", down, true);
      document.removeEventListener("keydown", key, true);
      window.removeEventListener("blur", closeContextMenu);
    };
  }, [at]);

  if (!at) return null;
  return (
    <Portal>
      <div
        ref={ref}
        className="context-menu"
        role="menu"
        aria-label={label}
        style={{ left: pos.left || at.x, top: pos.top || at.y }}
        onKeyDown={(e) => {
          if (e.key !== "ArrowDown" && e.key !== "ArrowUp") return;
          e.preventDefault();
          const buttons = [...ref.current!.querySelectorAll<HTMLButtonElement>("button:not(:disabled)")];
          const i = buttons.indexOf(document.activeElement as HTMLButtonElement);
          buttons[(i + (e.key === "ArrowDown" ? 1 : buttons.length - 1)) % buttons.length]?.focus();
        }}
      >
        {items.map((item, i) =>
          item === "separator" ? (
            <div key={i} className="menu-sep" role="separator" />
          ) : (
            <button
              key={i}
              type="button"
              role="menuitem"
              className={item.danger ? "menu-item danger" : "menu-item"}
              disabled={item.disabled}
              onClick={() => {
                closeContextMenu();
                item.run();
              }}
            >
              {item.icon}
              {item.label}
              {item.shortcut && <kbd>{item.shortcut}</kbd>}
            </button>
          ),
        )}
      </div>
    </Portal>
  );
}

/** The menu for a text field (the browser's is never shown). */
function textMenu(input: HTMLInputElement | HTMLTextAreaElement): MenuEntry[] {
  const selected = (input.selectionEnd ?? 0) > (input.selectionStart ?? 0);
  const insert = (text: string) => {
    input.focus();
    input.setRangeText(text, input.selectionStart ?? 0, input.selectionEnd ?? 0, "end");
    input.dispatchEvent(new Event("input", { bubbles: true }));
  };
  return [
    {
      label: "Cut",
      icon: <Scissors size={14} />,
      shortcut: "Ctrl+X",
      disabled: !selected || input.readOnly,
      run: () => {
        input.focus();
        document.execCommand("cut");
      },
    },
    {
      label: "Copy",
      icon: <Copy size={14} />,
      shortcut: "Ctrl+C",
      disabled: !selected,
      run: () => {
        input.focus();
        document.execCommand("copy");
      },
    },
    {
      label: "Paste",
      icon: <Clipboard size={14} />,
      shortcut: "Ctrl+V",
      disabled: input.readOnly,
      run: () => void navigator.clipboard?.readText().then(insert, () => undefined),
    },
    "separator",
    {
      label: "Select all",
      icon: <TextSelect size={14} />,
      shortcut: "Ctrl+A",
      run: () => {
        input.focus();
        input.select();
      },
    },
  ];
}

// ------------------------------------------------------------------------------ the app's behaviour

/** Makes the window behave like an app; returns the undo. */
export function installAppBehaviour(): () => void {
  // Ctrl+wheel and pinch (a Ctrl+wheel) never zoom the page; over the field they zoom the field
  const wheel = (e: WheelEvent) => {
    if (e.ctrlKey && !(e.target as HTMLElement).closest?.(".field-canvas")) e.preventDefault();
  };
  // Ctrl + / - / 0 zoom the field, not the page
  const keys = (e: KeyboardEvent) => {
    if (!(e.ctrlKey || e.metaKey)) return;
    if (e.key === "=" || e.key === "+" || e.key === "-" || e.key === "0") {
      e.preventDefault();
      if (ui.get().page !== "editor") return;
      if (e.key === "0") fitView();
      else zoomBy(e.key === "-" ? 0.8 : 1.25);
    }
  };
  const drag = (e: DragEvent) => {
    if (!(e.target as HTMLElement).closest?.("[draggable=true]")) e.preventDefault();
  };
  const menu = (e: MouseEvent) => {
    e.preventDefault();
    const t = e.target as HTMLElement;
    if (t instanceof HTMLInputElement || t instanceof HTMLTextAreaElement) {
      if (t instanceof HTMLInputElement && t.type === "file") return;
      openContextMenu(e.clientX, e.clientY, "Text", textMenu(t));
    }
  };
  window.addEventListener("wheel", wheel, { passive: false });
  window.addEventListener("keydown", keys, true);
  document.addEventListener("dragstart", drag);
  document.addEventListener("contextmenu", menu);
  return () => {
    window.removeEventListener("wheel", wheel);
    window.removeEventListener("keydown", keys, true);
    document.removeEventListener("dragstart", drag);
    document.removeEventListener("contextmenu", menu);
  };
}

// ------------------------------------------------------------------------------ command palette

export function CommandPalette() {
  const open = useStore(ui, (s) => s.paletteOpen);
  const [query, setQuery] = useState("");
  const [active, setActive] = useState(0);
  const list = useRef<HTMLUListElement>(null);
  const restore = useRef<HTMLElement | null>(null);

  const results = useMemo(() => {
    const words = query.toLowerCase().split(/\s+/).filter(Boolean);
    return COMMANDS.filter((c) => (c.enabled?.() ?? true) && words.every((w) => `${c.label} ${c.group}`.toLowerCase().includes(w)));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [query, open]);

  useEffect(() => {
    if (open) {
      restore.current = document.activeElement as HTMLElement | null;
      setQuery("");
      setActive(0);
    }
  }, [open]);

  useEffect(() => {
    list.current?.querySelector(".active")?.scrollIntoView({ block: "nearest" });
  }, [active]);

  if (!open) return null;
  const close = () => {
    ui.set({ paletteOpen: false });
    restore.current?.focus?.();
  };
  const run = (i: number) => {
    const c = results[i];
    close();
    c?.run();
  };
  return (
    <Portal>
      <div className="overlay" onPointerDown={(e) => e.target === e.currentTarget && close()}>
        <div className="palette" role="dialog" aria-label="Command palette" data-testid="palette">
          <div className="palette-input">
            <Search size={16} />
            <input
              autoFocus
              placeholder="Type a command"
              value={query}
              role="combobox"
              aria-expanded="true"
              aria-controls="palette-list"
              aria-activedescendant={results[active] ? `palette-${results[active]!.id}` : undefined}
              onChange={(e) => {
                setQuery(e.target.value);
                setActive(0);
              }}
              onKeyDown={(e) => {
                if (e.key === "ArrowDown" || e.key === "ArrowUp") {
                  e.preventDefault();
                  if (results.length) setActive((a) => (a + (e.key === "ArrowDown" ? 1 : results.length - 1)) % results.length);
                } else if (e.key === "Enter") {
                  e.preventDefault();
                  run(active);
                } else if (e.key === "Escape") {
                  e.preventDefault();
                  e.stopPropagation();
                  close();
                }
              }}
            />
          </div>
          <ul ref={list} id="palette-list" role="listbox" aria-label="Commands">
            {results.map((c, i) => (
              <li
                key={c.id}
                id={`palette-${c.id}`}
                role="option"
                aria-selected={i === active}
                className={i === active ? "active" : ""}
                onPointerMove={() => setActive(i)}
                onClick={() => run(i)}
              >
                {c.label}
                <span className="group-name">{c.group}</span>
                {c.shortcut && <kbd>{c.shortcut}</kbd>}
              </li>
            ))}
            {results.length === 0 && <li className="empty">No command matches.</li>}
          </ul>
        </div>
      </div>
    </Portal>
  );
}

// ------------------------------------------------------------------------------ help

export function HelpDialog() {
  const open = useStore(ui, (s) => s.helpOpen);
  const ref = useRef<HTMLDialogElement>(null);
  useEffect(() => {
    const d = ref.current;
    if (!d) return;
    if (open && !d.open) d.showModal();
    if (!open && d.open) d.close();
  }, [open]);
  return (
    <dialog ref={ref} className="help" aria-labelledby="help-title" onClose={() => ui.set({ helpOpen: false })}>
      <h2 id="help-title">Quick start</h2>
      <QuickStartList />
      <h2>Keyboard shortcuts</h2>
      <table>
        <tbody>
          {SHORTCUTS.map(([keys, what]) => (
            <tr key={keys}>
              <th scope="row">
                <kbd>{keys}</kbd>
              </th>
              <td>{what}</td>
            </tr>
          ))}
        </tbody>
      </table>
      <p className="hint">
        The field origin is its centre (WPILib 2027). Paths are stored for blue; the red view shows them through the project's red flip, and
        edits made there are stored for blue.
      </p>
      <form method="dialog">
        <button type="submit" className="btn primary" autoFocus>
          Close
        </button>
      </form>
    </dialog>
  );
}

/** A labelled, scrollable pane with an optional header. */
export function Pane({ title, actions, children, className }: { title?: string; actions?: ReactNode; children: ReactNode; className?: string }) {
  return (
    <>
      {title !== undefined && (
        <div className="pane-head">
          <span className="title">{title}</span>
          {actions}
        </div>
      )}
      <div className={className ? `pane-body ${className}` : "pane-body"}>{children}</div>
    </>
  );
}

/** The app's yes/no dialog. */
export function ConfirmHost() {
  const q = useStore(question, (s) => s.current);
  if (!q) return null;
  return (
    <Portal>
      <div className="overlay" onPointerDown={(e) => e.target === e.currentTarget && q.answer(false)}>
        <div className="confirm" role="alertdialog" aria-modal="true" aria-label={q.text} onKeyDown={(e) => e.key === "Escape" && q.answer(false)}>
          <p>{q.text}</p>
          <div className="row" style={{ justifyContent: "flex-end" }}>
            <button type="button" className="btn ghost" onClick={() => q.answer(false)}>
              Cancel
            </button>
            <button type="button" className="btn primary" autoFocus onClick={() => q.answer(true)}>
              {q.ok}
            </button>
          </div>
        </div>
      </div>
    </Portal>
  );
}
