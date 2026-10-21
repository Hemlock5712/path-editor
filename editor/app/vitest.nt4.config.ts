import { defineConfig } from "vitest/config";

// The NT4 client against WPILib's real ntcore server (test/nt4/*.live.ts). Starts the Java harness
// through gradle, so it is slow and needs the WPILib 2027 install: npm run test:nt4.
export default defineConfig({
  test: { include: ["test/nt4/**/*.live.ts"], environment: "node", testTimeout: 300_000, hookTimeout: 300_000 },
});
