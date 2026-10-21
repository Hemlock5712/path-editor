import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { App } from "./App.tsx";
import "./theme.css";
import { isTauri } from "./io/native.ts";
import { installTestHook } from "./testHook.ts";

// Read-only, for the browser tests: the active path's preview numbers and poses, unrounded
installTestHook();

// The installable app (PWA): the service worker caches the app so it opens offline. Not in development,
// not under test automation (each test starts clean).
if (import.meta.env.PROD && "serviceWorker" in navigator && !navigator.webdriver && !isTauri()) {
  window.addEventListener("load", () => void navigator.serviceWorker.register("/sw.js").catch(() => undefined));
}

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
