# Myelin: desktop app

The editor (`editor/app`) in its own window, built with Tauri 2. It is the same app as the website, plus
two things a browser can't do well:

- **The robot project on disk.** **Open** (Ctrl+O, or File > Open robot project…) asks for the folder once,
  with the system's folder dialog. From then on the app reads and writes `src/main/deploy/paths` (and
  `PathName.java`) directly, and it reopens the same project the next time it starts. The Rust side only
  serves folders picked this way, and only paths inside them (`src-tauri/src/lib.rs`).
- **NT4 without the browser's rules.** The app isn't an https page, so `ws://` to the robot (10.TE.AM.2)
  or a simulation (localhost) just connects. There is no mixed-content block and no Local Network Access
  prompt. This was checked with the Windows build against lib's real ntcore harness (`gradlew nt4Server`) on
  localhost: it connected and showed the follower's telemetry and the live robot on the field.

There is also a native menu bar (File, Edit, View, Help) that runs the editor's own commands.

## Building

The app needs Rust (stable, MSVC on Windows), Node 22, and on Windows the WebView2 runtime (part of
Windows 11). From this folder:

```
npm ci --prefix ../core && npm ci --prefix ../app && npm ci
npm run build        # the installers, in src-tauri/target/release/bundle/ (msi, nsis)
npm run dev          # the app around the Vite dev server, with hot reload
```

`npm run build` builds the web app first (`beforeBuildCommand`) and bundles `../app/dist`. Rust builds use a
lot of memory, so set `CARGO_BUILD_JOBS=4` on a small machine.

## Releases

`.github/workflows/desktop-app.yml` (**Desktop app**) builds the installers for Windows, macOS (universal)
and Linux:

- **Push a tag `v*`** (e.g. `v2.0.0`). It builds them all and attaches them to a draft release of that name.
- **Run it by hand** (Run workflow). It builds them all and keeps them as the run's artifacts, with no release.

The app's version is in `src-tauri/tauri.conf.json` and `src-tauri/Cargo.toml`.

## Icons

`src-tauri/icons/` is made from `app-icon.png` (the editor's mark, 1024 px) with `npx tauri icon app-icon.png
-o src-tauri/icons`.

## Checked on Windows

The release build was driven through WebView2's DevTools port
(`WEBVIEW2_ADDITIONAL_BROWSER_ARGUMENTS=--remote-debugging-port=9333`). These all worked:

- reopening a robot project folder;
- saving into its `src/main/deploy/paths` (the path, the sidecar and `PathName.java`);
- restarting and finding the project reopened;
- NT4 to the ntcore harness.

The screenshot is `app/screenshots/desktop-app-windows.png`. The native folder dialog itself was not
scripted.
