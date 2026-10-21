// Where a project's files live. A robot project folder opened with the File System Access API, or an
// in-memory set (an uploaded folder, a test), behind one small interface. Paths are relative to the
// robot project's root and use '/'.

import { isTauri, pickNativeFolder } from "./native.ts";

export interface ProjectFiles {
  /** Shown in the title bar: the folder's name. */
  readonly label: string;
  /** File names (not paths) directly inside dir; empty if dir doesn't exist. */
  list(dir: string): Promise<string[]>;
  /** A file's text, or null if it doesn't exist. */
  read(path: string): Promise<string | null>;
  /** Writes a file, making its folders. */
  write(path: string, text: string): Promise<void>;
  /** Deletes a file if it exists. */
  remove(path: string): Promise<void>;
}

function split(path: string): [string[], string] {
  const parts = path.split("/").filter((p) => p !== "");
  const name = parts.pop();
  if (!name) throw new Error(`bad path: ${path}`);
  return [parts, name];
}

/** A robot project folder picked with showDirectoryPicker(). */
export class DirectoryFiles implements ProjectFiles {
  constructor(private readonly root: FileSystemDirectoryHandle) {}

  get label(): string {
    return this.root.name;
  }

  private async dir(parts: string[], create: boolean): Promise<FileSystemDirectoryHandle | null> {
    let d = this.root;
    for (const p of parts) {
      try {
        d = await d.getDirectoryHandle(p, { create });
      } catch (e) {
        if (!create && e instanceof DOMException && e.name === "NotFoundError") return null;
        throw e;
      }
    }
    return d;
  }

  async list(dir: string): Promise<string[]> {
    const d = await this.dir(dir.split("/").filter((p) => p !== ""), false);
    if (!d) return [];
    const names: string[] = [];
    for await (const [name, handle] of d.entries()) if (handle.kind === "file") names.push(name);
    return names.sort();
  }

  async read(path: string): Promise<string | null> {
    const [parts, name] = split(path);
    const d = await this.dir(parts, false);
    if (!d) return null;
    try {
      return await (await (await d.getFileHandle(name)).getFile()).text();
    } catch (e) {
      if (e instanceof DOMException && e.name === "NotFoundError") return null;
      throw e;
    }
  }

  async write(path: string, text: string): Promise<void> {
    const [parts, name] = split(path);
    const d = (await this.dir(parts, true))!;
    const writable = await (await d.getFileHandle(name, { create: true })).createWritable();
    await writable.write(text);
    await writable.close();
  }

  async remove(path: string): Promise<void> {
    const [parts, name] = split(path);
    const d = await this.dir(parts, false);
    if (!d) return;
    try {
      await d.removeEntry(name);
    } catch (e) {
      if (!(e instanceof DOMException && e.name === "NotFoundError")) throw e;
    }
  }
}

/** Files held in memory: an uploaded folder (saved back as a download) or a test. */
export class MemoryFiles implements ProjectFiles {
  readonly files: Map<string, string>;

  constructor(
    readonly label: string,
    files?: Iterable<[string, string]>,
  ) {
    this.files = new Map(files);
  }

  async list(dir: string): Promise<string[]> {
    const prefix = dir.replace(/\/?$/, "/");
    return [...this.files.keys()]
      .filter((p) => p.startsWith(prefix) && !p.slice(prefix.length).includes("/"))
      .map((p) => p.slice(prefix.length))
      .sort();
  }

  async read(path: string): Promise<string | null> {
    return this.files.get(path) ?? null;
  }

  async write(path: string, text: string): Promise<void> {
    this.files.set(path, text);
  }

  async remove(path: string): Promise<void> {
    this.files.delete(path);
  }
}

/** Whether this browser can open and write a folder in place (Chrome, Edge). */
export function canOpenFolders(): boolean {
  return isTauri() || (typeof window !== "undefined" && "showDirectoryPicker" in window);
}

declare global {
  interface Window {
    showDirectoryPicker?: (options?: { id?: string; mode?: "read" | "readwrite" }) => Promise<FileSystemDirectoryHandle>;
  }
}

/** Asks for a robot project folder (the native dialog in the desktop app). Null if the user cancels. */
export async function pickFolder(): Promise<ProjectFiles | null> {
  if (isTauri()) return pickNativeFolder();
  try {
    const handle = await window.showDirectoryPicker!({ id: "robot-project", mode: "readwrite" });
    return new DirectoryFiles(handle);
  } catch (e) {
    if (e instanceof DOMException && e.name === "AbortError") return null;
    throw e;
  }
}

/**
 * An uploaded folder (<input type="file" webkitdirectory>) as MemoryFiles. The paths are made
 * relative to the folder that holds src/main/deploy, whichever folder the user picked.
 */
export async function filesFromUpload(list: FileList): Promise<MemoryFiles> {
  const entries: [string, string][] = [];
  let label = "upload";
  for (const file of Array.from(list)) {
    const rel = (file as File & { webkitRelativePath?: string }).webkitRelativePath || file.name;
    entries.push([rel, await file.text()]);
    label = rel.split("/")[0] ?? label;
  }
  const marker = "src/main/deploy/paths/";
  const withMarker = entries.find(([p]) => p.includes(marker));
  if (withMarker) {
    const root = withMarker[0].slice(0, withMarker[0].indexOf(marker));
    return new MemoryFiles(label, entries.filter(([p]) => p.startsWith(root)).map(([p, t]) => [p.slice(root.length), t]));
  }
  // a folder of path files on its own: treat it as deploy/paths
  return new MemoryFiles(label, entries.map(([p, t]) => [marker + p.split("/").pop(), t]));
}
