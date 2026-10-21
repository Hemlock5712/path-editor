// The window's top bar (the app's own title bar: project, file actions, pages, the command palette,
// help and settings) and the status bar along the bottom.

import { Activity, BarChart3, CircleHelp, Download, FilePlus2, FolderOpen, PenTool, Save, Search, Settings, Upload } from "lucide-react";
import { useRef } from "react";
import { canOpenFolders } from "../io/files.ts";
import { isDirty } from "../state/editor.ts";
import { downloadZip, newProject, openFolder, openUpload, save, session } from "../state/session.ts";
import { useStore } from "../state/store.ts";
import { live } from "../state/live.ts";
import { openSettings, setPage, ui, type Page } from "../state/ui.ts";
import { IconButton } from "./controls.tsx";
import { useEditor } from "./panels/common.tsx";
import { AppIcon } from "./shell.tsx";

const PAGES: readonly { page: Page; label: string; icon: typeof PenTool }[] = [
  { page: "editor", label: "Editor", icon: PenTool },
  { page: "telemetry", label: "Telemetry", icon: Activity },
  { page: "bench", label: "Bench", icon: BarChart3 },
];

export function TopBar() {
  const dirty = useEditor(isDirty);
  const files = useStore(session, (s) => s.files);
  const writable = useStore(session, (s) => s.writable);
  const page = useStore(ui, (s) => s.page);
  const upload = useRef<HTMLInputElement>(null);

  return (
    <header className="topbar">
      <span className="brand">
        <AppIcon />
        Path Editor
      </span>
      <span className="project-name" data-testid="project-name" data-tip={files ? `Robot project: ${files.label}` : "Not saved to a robot project yet"}>
        {files ? files.label : "unsaved project"}
        {dirty && <span className="dirty-dot" aria-label="unsaved changes" />}
      </span>
      <div role="toolbar" aria-label="File" className="row" style={{ gap: 2 }}>
        <IconButton label="New project" onClick={newProject}>
          <FilePlus2 size={16} />
        </IconButton>
        <IconButton label="Open a robot project folder" shortcut="Ctrl+O" onClick={() => void openFolder()} disabled={!canOpenFolders()} testId="open-folder">
          <FolderOpen size={16} />
        </IconButton>
        <IconButton label="Load a project folder by upload (any browser)" onClick={() => upload.current?.click()}>
          <Upload size={16} />
        </IconButton>
        <input
          ref={upload}
          type="file"
          className="sr-only"
          aria-label="Upload a robot project folder"
          // @ts-expect-error webkitdirectory isn't in React's types, but every current browser supports it
          webkitdirectory=""
          multiple
          onChange={(e) => {
            if (e.target.files && e.target.files.length > 0) void openUpload(e.target.files);
            e.target.value = "";
          }}
        />
        <IconButton label={writable ? "Save to the folder" : "Save"} shortcut="Ctrl+S" onClick={() => void save()} testId="save" className={dirty ? "active" : undefined}>
          <Save size={16} />
        </IconButton>
        <IconButton label="Download every file as a zip" onClick={downloadZip}>
          <Download size={16} />
        </IconButton>
      </div>

      <nav className="page-tabs" role="tablist" aria-label="Pages">
        {PAGES.map(({ page: p, label, icon: Icon }) => (
          <button key={p} type="button" role="tab" className="page-tab" aria-selected={page === p} onClick={() => setPage(p)} data-testid={`page-${p}`}>
            <Icon size={14} />
            {label}
          </button>
        ))}
      </nav>

      <button type="button" className="palette-hint" onClick={() => ui.set({ paletteOpen: true })} aria-label="Command palette" data-testid="open-palette">
        <Search size={14} />
        Search commands
        <kbd>Ctrl+K</kbd>
      </button>
      <IconButton label="Quick start and shortcuts" shortcut="?" onClick={() => ui.set({ helpOpen: true })} testId="help">
        <CircleHelp size={16} />
      </IconButton>
      <IconButton label="Settings" shortcut="Ctrl+," onClick={() => openSettings()} pressed={page === "settings"} testId="settings">
        <Settings size={16} />
      </IconButton>
    </header>
  );
}

export function StatusBar() {
  const status = useEditor((s) => s.status);
  const draft = useEditor((s) => s.draftObstacle);
  const alliance = useEditor((s) => s.view.alliance);
  const zoom = useEditor((s) => s.view.zoom);
  const page = useStore(ui, (s) => s.page);
  const liveStatus = useStore(live, (s) => s.status);
  return (
    <footer className="statusbar">
      <span role="status" aria-live="polite" className={status?.kind === "error" ? "status error" : "status"} data-testid="status">
        {status?.text ?? "Ready"}
      </span>
      <span className="spacer" />
      {page === "editor" && draft && <span>{draft.length} corners: double-click or Enter to finish, Esc to cancel</span>}
      <span>{alliance === "red" ? "Red view" : "Blue view"}</span>
      <span>{Math.round(zoom * 100)} %</span>
      <span>NT4: {liveStatus}</span>
    </footer>
  );
}
