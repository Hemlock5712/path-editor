import { describe, expect, it } from "vitest";
import { addressSpace, localAccess } from "../src/nt4/localAccess.ts";

const permissions = (states: Record<string, PermissionState>) => ({
  query: async ({ name }: { name: string }) => {
    if (!(name in states)) throw new TypeError(`unknown permission ${name}`);
    return { state: states[name]! } as PermissionStatus;
  },
});

describe("reaching the robot from the https site", () => {
  it("sorts hosts as Local Network Access does", () => {
    expect(addressSpace("localhost")).toBe("loopback");
    expect(addressSpace("127.0.0.1")).toBe("loopback");
    expect(addressSpace("[::1]")).toBe("loopback");
    expect(addressSpace("10.57.12.2")).toBe("local");
    expect(addressSpace("172.22.11.2")).toBe("local");
    expect(addressSpace("192.168.1.119")).toBe("local");
    expect(addressSpace("roborio-5712-frc.local")).toBe("local");
    expect(addressSpace("172.32.0.1")).toBe("public");
    expect(addressSpace("8.8.8.8")).toBe("public");
    expect(addressSpace("roborio-5712-frc")).toBe("public");
  });

  it("says nothing over http", async () => {
    expect(await localAccess("10.57.12.2", "http:", permissions({}))).toEqual({ state: "fine", message: "" });
  });

  it("asks for the right permission over https, and says what to do", async () => {
    const chrome = permissions({ "local-network": "prompt", "loopback-network": "granted" });
    const robot = await localAccess("10.57.12.2", "https:", chrome);
    expect(robot.state).toBe("ask");
    expect(robot.message).toContain("devices on your local network");
    expect(robot.message).toContain("npm --prefix editor/app run dev");
    expect((await localAccess("localhost", "https:", chrome)).state).toBe("fine");
    const denied = await localAccess("10.57.12.2", "https:", permissions({ "local-network": "denied" }));
    expect(denied.state).toBe("denied");
    expect(denied.message).toContain("site settings");
  });

  it("says to switch browser or run locally when the browser has no such permission", async () => {
    const other = await localAccess("10.57.12.2", "https:", permissions({}));
    expect(other.state).toBe("blocked");
    expect(other.message).toContain("Chrome or Edge");
    expect((await localAccess("roborio-5712-frc", "https:", permissions({ "local-network": "granted" }))).state).toBe("blocked");
  });
});
