// The app's own state (not the document's): which page is showing, the pane sizes, the charts drawer,
// the settings tab, the command palette, toasts and the context menu. Layout choices are remembered in
// this browser.

import type { ReactNode } from "react";
import { bench, loadSnapshot } from "./bench.ts";
import { editor } from "./editor.ts";
import { createStore } from "./store.ts";

export type Page = "editor" | "telemetry" | "bench" | "settings";
export type SettingsTab = "robot" | "defaults" | "field" | "project" | "sim" | "editor";

export interface UiState {
  readonly page: Page;
  readonly settingsTab: SettingsTab;
  readonly railWidth: number;
  readonly inspectorWidth: number;
  readonly chartsOpen: boolean;
  readonly paletteOpen: boolean;
  readonly helpOpen: boolean;
  /** The Flip or mirror dialog, for the active path. */
  readonly transformOpen: boolean;
  /** Draw the live robot (NT4) on the editor's field. */
  readonly liveOverlay: boolean;
}

const LAYOUT_KEY = "pathEditor.layout";

function remembered(): Partial<UiState> {
  try {
    const saved = JSON.parse(localStorage.getItem(LAYOUT_KEY) ?? "{}") as Partial<UiState>;
    return {
      railWidth: typeof saved.railWidth === "number" ? saved.railWidth : undefined,
      inspectorWidth: typeof saved.inspectorWidth === "number" ? saved.inspectorWidth : undefined,
      chartsOpen: saved.chartsOpen === true,
      liveOverlay: saved.liveOverlay !== false,
    };
  } catch {
    return {};
  }
}

export const RAIL = { min: 180, max: 360, initial: 232 };
export const INSPECTOR = { min: 280, max: 480, initial: 320 };

export const ui = createStore<UiState>({
  page: "editor",
  settingsTab: "robot",
  railWidth: RAIL.initial,
  inspectorWidth: INSPECTOR.initial,
  chartsOpen: false,
  paletteOpen: false,
  helpOpen: false,
  transformOpen: false,
  liveOverlay: true,
  ...Object.fromEntries(Object.entries(remembered()).filter(([, v]) => v !== undefined)),
});

ui.subscribe(() => {
  const s = ui.get();
  try {
    localStorage.setItem(
      LAYOUT_KEY,
      JSON.stringify({ railWidth: s.railWidth, inspectorWidth: s.inspectorWidth, chartsOpen: s.chartsOpen, liveOverlay: s.liveOverlay }),
    );
  } catch {
    // not remembered
  }
});

export function setPage(page: Page): void {
  if (ui.get().page === page) return;
  ui.set({ page });
  bench.set({ open: page === "bench" });
  if (page === "bench" && !bench.get().data && !bench.get().loadingSnapshot) void loadSnapshot();
}

export function openSettings(tab?: SettingsTab): void {
  setPage("settings");
  if (tab) ui.set({ settingsTab: tab });
}

// ------------------------------------------------------------------------------ toasts

export interface Toast {
  readonly id: number;
  readonly text: string;
  readonly kind: "info" | "ok" | "error";
  /** A one-click follow-up, offered on the toast. */
  readonly action?: { readonly label: string; readonly run: () => void };
}

export const toasts = createStore<{ readonly list: readonly Toast[] }>({ list: [] });
let toastId = 0;

export function toast(text: string, kind: Toast["kind"] = "info", ms = kind === "error" ? 6000 : 3200, action?: Toast["action"]): void {
  const id = ++toastId;
  // one offer at a time: a new one replaces the last
  toasts.set((s) => ({ list: [...s.list.filter((t) => !(action && t.action)).slice(-3), { id, text, kind, action }] }));
  setTimeout(() => toasts.set((s) => ({ list: s.list.filter((t) => t.id !== id) })), ms);
}

// every status the editor reports (save, open, refusals) also shows as a toast
let lastStatus = 0;
editor.subscribe(() => {
  const st = editor.get().status;
  if (!st || st.at === lastStatus) return;
  lastStatus = st.at;
  toast(st.text, st.kind === "error" ? "error" : /^(Saved|Opened|Wrote)/.test(st.text) ? "ok" : "info");
});

// ------------------------------------------------------------------------------ context menu

export interface MenuItem {
  readonly label: string;
  readonly icon?: ReactNode;
  readonly shortcut?: string;
  readonly danger?: boolean;
  readonly disabled?: boolean;
  readonly run: () => void;
}

export type MenuEntry = MenuItem | "separator";

export const contextMenu = createStore<{ readonly at: { x: number; y: number } | null; readonly items: readonly MenuEntry[]; readonly label: string }>({
  at: null,
  items: [],
  label: "",
});

export function openContextMenu(x: number, y: number, label: string, items: readonly MenuEntry[]): void {
  contextMenu.set({ at: { x, y }, items, label });
}

export function closeContextMenu(): void {
  if (contextMenu.get().at) contextMenu.set({ at: null, items: [] });
}
