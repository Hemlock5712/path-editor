import react from "@vitejs/plugin-react";
import { defineConfig } from "vitest/config";

// The path maths lives in ../core and is imported by relative path, so Vite may serve files from it.
export default defineConfig({
  plugins: [react()],
  server: { fs: { allow: [".", "../core"] }, port: 5712, strictPort: false },
  preview: { port: 5713 },
  worker: { format: "es" },
  test: { include: ["test/**/*.test.ts"], environment: "node" },
});
