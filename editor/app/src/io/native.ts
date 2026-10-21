// The desktop app (Tauri, editor/desktop): the robot project folder through the app's own Rust
// commands, so it reads and writes src/main/deploy/paths directly, with no browser folder prompts
// after the first pick. In a browser none of this is used.

import { invoke } from "@tauri-apps/api/core";
import type { ProjectFiles } from "./files.ts";

/** True inside the desktop app. */
export function isTauri(): boolean {
  return typeof window !== "undefined" && "__TAURI_INTERNALS__" in window;
}

interface PickedProject {
  readonly root: string;
  readonly label: string;
}

/** A robot project folder on disk, by absolute path (the Rust side only serves folders the user picked). */
export class NativeFiles implements ProjectFiles {
  constructor(
    readonly root: string,
    readonly label: string,
  ) {}

  list(dir: string): Promise<string[]> {
    return invoke<string[]>("list_dir", { root: this.root, dir });
  }

  read(path: string): Promise<string | null> {
    return invoke<string | null>("read_text", { root: this.root, path });
  }

  write(path: string, text: string): Promise<void> {
    return invoke<void>("write_text", { root: this.root, path, text });
  }

  remove(path: string): Promise<void> {
    return invoke<void>("remove_file", { root: this.root, path });
  }
}

const LAST_KEY = "pathEditor.lastProject";

/** The native folder dialog. Null if the user cancels. */
export async function pickNativeFolder(): Promise<NativeFiles | null> {
  const picked = await invoke<PickedProject | null>("pick_project");
  if (!picked) return null;
  try {
    localStorage.setItem(LAST_KEY, picked.root);
  } catch {
    // not remembered
  }
  return new NativeFiles(picked.root, picked.label);
}

/** The folder opened last time, if it's still there (the app reopens it on start). */
export async function lastNativeFolder(): Promise<NativeFiles | null> {
  let root: string | null = null;
  try {
    root = localStorage.getItem(LAST_KEY);
  } catch {
    return null;
  }
  if (!root) return null;
  const picked = await invoke<PickedProject | null>("reopen_project", { root });
  return picked ? new NativeFiles(picked.root, picked.label) : null;
}
