// The desktop shell around the web editor (editor/app). It adds what a browser can't do well:
//   - the robot project folder on disk, through the commands below: the editor reads and writes
//     src/main/deploy/paths directly, with no browser folder prompts after the first pick;
//   - a native menu bar, whose items run the editor's own commands.
// NT4 needs nothing here: the app's pages aren't served over https, so ws:// to the robot is plain.
//
// The file commands only serve folders the user picked in this session (or reopened from last time),
// and only paths inside them.

use std::collections::HashSet;
use std::fs;
use std::path::{Component, Path, PathBuf};
use std::sync::Mutex;

use serde::Serialize;
use tauri::menu::{Menu, MenuItem, PredefinedMenuItem, Submenu};
use tauri::{Emitter, State};

/** The project folders the editor may touch. */
#[derive(Default)]
struct Roots(Mutex<HashSet<PathBuf>>);

#[derive(Serialize)]
struct Project {
    root: String,
    label: String,
}

fn allow(roots: &Roots, root: PathBuf) -> Project {
    let label = root.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_else(|| root.to_string_lossy().into_owned());
    let out = Project { root: root.to_string_lossy().into_owned(), label };
    roots.0.lock().unwrap().insert(root);
    out
}

/** root/rel, if root was picked and rel stays inside it. */
fn resolve(roots: &Roots, root: &str, rel: &str) -> Result<PathBuf, String> {
    let root = PathBuf::from(root);
    if !roots.0.lock().unwrap().contains(&root) {
        return Err("that folder wasn't opened in this session".into());
    }
    let rel = Path::new(rel);
    if !rel.components().all(|c| matches!(c, Component::Normal(_) | Component::CurDir)) {
        return Err(format!("not a path inside the project: {}", rel.display()));
    }
    Ok(root.join(rel))
}

#[tauri::command]
async fn pick_project(roots: State<'_, Roots>) -> Result<Option<Project>, String> {
    let picked = rfd::AsyncFileDialog::new().set_title("Open the robot project folder (the one with build.gradle)").pick_folder().await;
    Ok(picked.map(|f| allow(&roots, f.path().to_path_buf())))
}

#[tauri::command]
fn reopen_project(roots: State<'_, Roots>, root: String) -> Option<Project> {
    let root = PathBuf::from(root);
    root.is_dir().then(|| allow(&roots, root))
}

/** File names (not folders) directly inside dir; empty if it doesn't exist. */
#[tauri::command]
fn list_dir(roots: State<'_, Roots>, root: String, dir: String) -> Result<Vec<String>, String> {
    let path = resolve(&roots, &root, &dir)?;
    let Ok(entries) = fs::read_dir(&path) else { return Ok(vec![]) };
    let mut names: Vec<String> = entries
        .filter_map(Result::ok)
        .filter(|e| e.file_type().map(|t| t.is_file()).unwrap_or(false))
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    Ok(names)
}

#[tauri::command]
fn read_text(roots: State<'_, Roots>, root: String, path: String) -> Result<Option<String>, String> {
    let path = resolve(&roots, &root, &path)?;
    match fs::read_to_string(&path) {
        Ok(text) => Ok(Some(text)),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(format!("{}: {e}", path.display())),
    }
}

#[tauri::command]
fn write_text(roots: State<'_, Roots>, root: String, path: String, text: String) -> Result<(), String> {
    let path = resolve(&roots, &root, &path)?;
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| format!("{}: {e}", parent.display()))?;
    }
    fs::write(&path, text).map_err(|e| format!("{}: {e}", path.display()))
}

#[tauri::command]
fn remove_file(roots: State<'_, Roots>, root: String, path: String) -> Result<(), String> {
    let path = resolve(&roots, &root, &path)?;
    match fs::remove_file(&path) {
        Ok(()) => Ok(()),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(e) => Err(format!("{}: {e}", path.display())),
    }
}

/** The menu bar; each item's id is one of the editor's command ids (ui/commands.ts). */
fn menu(app: &tauri::AppHandle) -> tauri::Result<Menu<tauri::Wry>> {
    let item = |id: &str, label: &str| MenuItem::with_id(app, id, label, true, None::<&str>);
    let file = Submenu::with_items(
        app,
        "File",
        true,
        &[
            &item("open", "Open robot project…")?,
            &item("save", "Save")?,
            &item("zip", "Export as zip…")?,
            &PredefinedMenuItem::separator(app)?,
            &item("new-project", "New project")?,
            &PredefinedMenuItem::separator(app)?,
            &PredefinedMenuItem::quit(app, Some("Quit"))?,
        ],
    )?;
    let edit = Submenu::with_items(
        app,
        "Edit",
        true,
        &[&item("undo", "Undo")?, &item("redo", "Redo")?, &PredefinedMenuItem::separator(app)?, &item("optimise", "Optimise path")?],
    )?;
    let view = Submenu::with_items(
        app,
        "View",
        true,
        &[
            &item("page-editor", "Editor")?,
            &item("page-telemetry", "Telemetry")?,
            &item("page-bench", "Bench")?,
            &item("settings", "Settings")?,
            &PredefinedMenuItem::separator(app)?,
            &item("alliance", "Blue or red view")?,
            &item("fit", "Fit the field")?,
            &item("charts", "Charts")?,
        ],
    )?;
    let help = Submenu::with_items(app, "Help", true, &[&item("help", "Quick start and shortcuts")?])?;
    Menu::with_items(app, &[&file, &edit, &view, &help])
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .manage(Roots::default())
        .menu(menu)
        .on_menu_event(|app, event| {
            let _ = app.emit("menu", event.id().0.as_str());
        })
        .invoke_handler(tauri::generate_handler![pick_project, reopen_project, list_dir, read_text, write_text, remove_file])
        .run(tauri::generate_context!())
        .expect("error while running the path editor");
}
