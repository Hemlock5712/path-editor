import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { App } from "./App.tsx";
import "./styles.css";
import { installTestHook } from "./testHook.ts";

// Read-only, for the browser tests: the active path's preview numbers and poses, unrounded
installTestHook();

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
