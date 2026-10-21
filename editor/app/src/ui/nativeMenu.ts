// The desktop app's native menu bar: its items arrive as "menu" events carrying a command id, and run
// the same commands as the palette.

import { listen } from "@tauri-apps/api/event";
import { commandById } from "./commands.ts";

export async function listenToNativeMenu(): Promise<void> {
  await listen<string>("menu", (event) => {
    const command = commandById(event.payload);
    if (command && (command.enabled?.() ?? true)) command.run();
  });
}
