import { StrictMode } from "react";
import { createRoot } from "react-dom/client";
import { App } from "./App.tsx";
import { currentPreview } from "./state/analysis.ts";
import "./styles.css";

// Read-only, for the end-to-end suite (e2e-author): the active path's preview numbers, unrounded
(window as unknown as { pathEditorTest: object }).pathEditorTest = { preview: currentPreview };

createRoot(document.getElementById("root")!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
