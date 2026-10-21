// Reaching the robot (ws://) from the deployed editor (https). Measured in Chrome 155 and Edge 155 from
// https://paths.frc5712.com: a ws:// connection to a private IP (the robot's 10.TE.AM.2) or to localhost
// (a simulation) is not blocked as mixed content, but it is gated by Local Network Access. The browser asks
// once per site: "local-network" for private addresses and "loopback-network" for this computer. Until
// that is allowed the connection fails with net::ERR_BLOCKED_BY_LOCAL_NETWORK_ACCESS_CHECKS. A browser
// without those permissions (navigator.permissions.query throws) blocks ws:// from https outright, as
// does every browser for a public address. Over http (npm run dev, or vite preview) none of this applies.

export type AddressSpace = "loopback" | "local" | "public";

/** Where a host is, as Local Network Access sees it (by name or IP literal, not by DNS). */
export function addressSpace(host: string): AddressSpace {
  const h = host.trim().toLowerCase().replace(/^\[|\]$/g, "");
  if (h === "localhost" || h.endsWith(".localhost") || h === "::1" || /^127\./.test(h)) return "loopback";
  if (h.endsWith(".local")) return "local";
  const m = /^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$/.exec(h);
  if (m) {
    const [a, b] = [Number(m[1]), Number(m[2])];
    if (a === 10 || (a === 172 && b >= 16 && b <= 31) || (a === 192 && b === 168) || (a === 169 && b === 254)) return "local";
    return "public";
  }
  if (/^f[cd][0-9a-f]{2}:/.test(h) || /^fe[89ab][0-9a-f]:/.test(h)) return "local";
  // a bare name (roborio-5712-frc) may well be on the local network, but the browser can't know that
  // before it connects, so it is treated as public: ws:// to it from https is mixed content
  return "public";
}

export interface LocalAccess {
  /** fine: nothing in the way (http, or allowed); ask: the browser will ask; denied / blocked: it won't connect. */
  readonly state: "fine" | "ask" | "denied" | "blocked";
  readonly message: string;
}

const RUN_LOCALLY = "Or run the editor locally over http: npm --prefix editor/app run dev.";

/** What stands between this page and ws://host, and what to tell the user. */
export async function localAccess(
  host: string,
  protocol: string = location.protocol,
  permissions: Pick<Permissions, "query"> | undefined = navigator.permissions,
): Promise<LocalAccess> {
  if (protocol !== "https:") return { state: "fine", message: "" };
  const space = addressSpace(host);
  if (space === "public") {
    return {
      state: "blocked",
      message: `From this https page the browser blocks ws:// to ${host}. Use the robot's IP (10.TE.AM.2), a .local name or localhost. ${RUN_LOCALLY}`,
    };
  }
  const name = space === "loopback" ? "loopback-network" : "local-network";
  const what = space === "loopback" ? "apps on this computer" : "devices on your local network";
  let state: PermissionState;
  try {
    state = (await permissions!.query({ name: name as PermissionName })).state;
  } catch {
    return {
      state: "blocked",
      message: `This browser blocks ws:// connections from https pages. Use Chrome or Edge (they ask to allow local network access), or run the editor locally. ${RUN_LOCALLY}`,
    };
  }
  if (state === "granted") return { state: "fine", message: "" };
  if (state === "denied") {
    return {
      state: "denied",
      message: `This site isn't allowed to reach ${what}. Allow it in the site settings (the icon left of the address bar), then connect again. ${RUN_LOCALLY}`,
    };
  }
  return {
    state: "ask",
    message: `When you connect, the browser asks to let this site reach ${what}: allow it. ${RUN_LOCALLY}`,
  };
}
