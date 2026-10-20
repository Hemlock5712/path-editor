// Where the project came from and how it saves: a robot project folder (File System Access API),
// or, where the browser can't write folders, an uploaded folder saved back as a zip download.

import { canOpenFolders, DirectoryFiles, filesFromUpload, pickFolder, type ProjectFiles } from "../io/files.ts";
import { loadFrom, PATH_SUFFIX, PATHS_DIR, saveTo, toFiles } from "../io/projectIo.ts";
import { download, zip } from "../io/zip.ts";
import { emptyProject } from "../model/project.ts";
import { editor, isDirty, loadProject, markSaved, notify } from "./editor.ts";
import { createStore } from "./store.ts";

export interface SessionState {
  readonly files: ProjectFiles | null;
  /** Saving writes in place (a folder) or downloads a zip (an upload, or nothing opened). */
  readonly writable: boolean;
  readonly hadField: boolean;
  readonly hadRobot: boolean;
  /** Path files this session has loaded or saved: a tab deleted or renamed removes its file on save. */
  readonly knownPathFiles: ReadonlySet<string>;
  readonly busy: boolean;
}

export const session = createStore<SessionState>({
  files: null,
  writable: false,
  hadField: false,
  hadRobot: false,
  knownPathFiles: new Set(),
  busy: false,
});

function pathFilesOf(): Set<string> {
  return new Set(editor.get().project.paths.map((p) => `${PATHS_DIR}/${p.name}${PATH_SUFFIX}`));
}

function confirmDiscard(): boolean {
  return !isDirty(editor.get()) || window.confirm("Discard the unsaved changes?");
}

async function openFiles(files: ProjectFiles, writable: boolean): Promise<void> {
  session.set({ busy: true });
  try {
    const loaded = await loadFrom(files);
    if (loaded) {
      loadProject(loaded.project, loaded.view);
      session.set({ files, writable, hadField: loaded.hadField, hadRobot: loaded.hadRobot, knownPathFiles: pathFilesOf() });
      const n = loaded.project.paths.length;
      notify(
        loaded.problems.length > 0
          ? `Opened ${files.label} (${n} paths). Skipped: ${loaded.problems.join("; ")}`
          : `Opened ${files.label}: ${n} ${n === 1 ? "path" : "paths"}.`,
        loaded.problems.length > 0 ? "error" : "info",
      );
    } else {
      loadProject(emptyProject());
      session.set({ files, writable, hadField: false, hadRobot: false, knownPathFiles: new Set() });
      notify(`No path files in ${files.label}/${PATHS_DIR}. Saving will create them there.`);
    }
  } catch (e) {
    notify(`Couldn't open ${files.label}: ${(e as Error).message}`, "error");
  } finally {
    session.set({ busy: false });
  }
}

/** Opens a robot project folder (Chrome and Edge). */
export async function openFolder(): Promise<void> {
  if (!canOpenFolders()) {
    notify("This browser can't open folders; use Upload instead (and save as a zip).", "error");
    return;
  }
  if (!confirmDiscard()) return;
  const files = await pickFolder();
  if (files) await openFiles(files, true);
}

/** Loads an uploaded folder (any browser); saving it gives a zip to unpack over the project. */
export async function openUpload(list: FileList): Promise<void> {
  if (!confirmDiscard()) return;
  await openFiles(await filesFromUpload(list), false);
}

/** Opens a folder handle directly (for tests, and the recent folder if the browser hands one back). */
export async function openDirectory(handle: FileSystemDirectoryHandle): Promise<void> {
  await openFiles(new DirectoryFiles(handle), true);
}

export async function save(): Promise<void> {
  const s = session.get();
  if (s.busy) return;
  if (!s.files || !s.writable) {
    if (canOpenFolders() && !s.files) {
      const files = await pickFolder();
      if (!files) return;
      session.set({ files, writable: true });
    } else {
      downloadZip();
      return;
    }
  }
  const { files, hadField, hadRobot, knownPathFiles } = session.get();
  session.set({ busy: true });
  try {
    const { project, view } = editor.get();
    const result = await saveTo(files!, project, view, { hadField, hadRobot, knownPathFiles });
    markSaved();
    const fieldWritten = result.written.some((p) => p.endsWith("field.json"));
    const robotWritten = result.written.some((p) => p.endsWith("robot.json"));
    session.set({ knownPathFiles: pathFilesOf(), hadField: hadField || fieldWritten, hadRobot: hadRobot || robotWritten });
    const parts = [`${result.written.length} written`];
    if (result.removed.length > 0) parts.push(`${result.removed.length} removed`);
    notify(`Saved to ${files!.label}: ${parts.join(", ")}.`);
  } catch (e) {
    notify(`Save failed: ${(e as Error).message}`, "error");
  } finally {
    session.set({ busy: false });
  }
}

/** Every file the project saves, zipped with paths from the robot project's root. */
export function downloadZip(): void {
  const { project, view } = editor.get();
  const s = session.get();
  download("paths.zip", zip(toFiles(project, view, { hadField: s.hadField, hadRobot: s.hadRobot })), "application/zip");
  markSaved();
  notify("Downloaded paths.zip: unzip it over the robot project's folder.");
}

export function newProject(): void {
  if (!confirmDiscard()) return;
  loadProject(emptyProject());
  session.set({ files: null, writable: false, hadField: false, hadRobot: false, knownPathFiles: new Set() });
  notify("New project. Open a robot project folder or save to choose one.");
}
