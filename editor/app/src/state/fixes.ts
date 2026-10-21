// Fix-it buttons for the checks: the one or two edits that usually answer each one, as one undo step
// (or, for a collision, the optimiser or Route around, shown dashed until applied).

import { makeConstraintZone, makeFacePointZone, makePathEvent, makeRotationTarget } from "../../../core/src/index.ts";
import type { Check } from "../compute/analysis.ts";
import { addConstraintZone, addFacePointZone, updateConstraintZone, updateEvent, updateFacePointZone, updateRotationTarget, withSpec } from "../model/edits.ts";
import { round5, waypointAtS } from "../model/positions.ts";
import { geometryOf } from "./analysis.ts";
import { activePath, editPath, select } from "./editor.ts";
import { startOptimise, startRouteAround } from "./optimiser.ts";

export interface Fix {
  readonly label: string;
  readonly tip: string;
  readonly run: () => void;
}

const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));

/** The fixes for one check, best first (none when there's nothing sensible to offer). */
export function fixesFor(check: Check): Fix[] {
  const path = activePath();
  const spec = path.spec;
  const last = spec.waypoints.length - 1;
  const g = geometryOf(spec);
  switch (check.kind) {
    case "bumper":
      return [
        { label: "Route around", tip: "Plan a way round with the robot's pathfinder (shown dashed until you Apply)", run: startRouteAround },
        { label: "Optimise", tip: "Reshape the curve clear of it, if it can (shown dashed until you Apply)", run: () => startOptimise() },
      ];
    case "curvature": {
      const cap = check.capMetersPerSecond;
      if (!g || cap === undefined) return [];
      const v = Math.max(0.1, Math.floor(cap * 20) / 20);
      const from = round5(waypointAtS(g, Math.max(0, check.sFrom - 0.3)));
      const to = round5(waypointAtS(g, Math.min(g.lengthMeters, check.sTo + 0.3)));
      return [
        {
          label: `Slow to ${v} m/s here`,
          tip: "Add a constraint zone over the bend at its cornering speed, so the robot doesn't slide wide",
          run: () =>
            editPath("Slow for the bend", (p) => addConstraintZone(p, makeConstraintZone(from, Math.max(to, from + 0.01), v, Infinity)), {
              kind: "constraint",
              index: spec.constraintZones.length,
            }),
        },
        { label: "Optimise", tip: "Reshape the curve: a wider bend is faster", run: () => startOptimise() },
      ];
    }
    case "heading": {
      if (!g) return [];
      // the face-point zone whose blend makes the turn: double its blend
      const zone = spec.facePointZones.findIndex((z) => {
        const a0 = g.sAtWaypoint(z.fromWaypoint) - z.blendMeters;
        const b0 = g.sAtWaypoint(z.toWaypoint) + z.blendMeters;
        return a0 <= check.sTo && b0 >= check.sFrom;
      });
      if (zone >= 0) {
        const z = spec.facePointZones[zone]!;
        const length = g.sAtWaypoint(z.toWaypoint) - g.sAtWaypoint(z.fromWaypoint);
        const blend = round5(Math.min(Math.max(z.blendMeters * 2, z.blendMeters + 0.5), Math.max(z.blendMeters, length / 2)));
        if (blend > z.blendMeters) {
          return [
            {
              label: `Lengthen the blend to ${blend} m`,
              tip: "Ease into and out of facing the point over more distance, so the turn is gentler",
              run: () => editPath("Lengthen the blend", (p) => updateFacePointZone(p, zone, makeFacePointZone(z.fromWaypoint, z.toWaypoint, z.point, blend))),
            },
          ];
        }
      }
      // a heading along the path in the stretch: open it to move or soften
      const target = spec.rotationTargets.findIndex((t) => {
        const s = Number.isNaN(t.waypoint) ? t.meters : g.sAtWaypoint(t.waypoint);
        return s >= check.sFrom - 0.5 && s <= check.sTo + 0.5;
      });
      return target >= 0
        ? [{ label: "Edit the heading", tip: "Move it further along, or turn it less, to spread the turn", run: () => select({ kind: "rotation", index: target }) }]
        : [];
    }
    case "event": {
      const r = check.ref;
      const e = r?.kind === "event" ? spec.events[r.index] : undefined;
      if (!r || !e || !g) return [];
      return [
        {
          label: "Snap to the path",
          tip: "Move it to the nearest end of the path",
          run: () =>
            editPath("Snap event to the path", (p) =>
              updateEvent(
                p,
                r.index,
                Number.isNaN(e.waypoint) ? makePathEvent(e.name, Number.NaN, round5(clamp(e.meters, 0, g.lengthMeters))) : makePathEvent(e.name, clamp(e.waypoint, 0, last), Number.NaN),
              ),
            ),
        },
      ];
    }
    case "position": {
      const r = check.ref;
      if (!r || !g) return [];
      if (r.kind === "rotation") {
        const t = spec.rotationTargets[r.index];
        if (!t) return [];
        return [
          {
            label: "Snap to the path",
            tip: "Move it to the nearest end of the path",
            run: () =>
              editPath("Snap heading to the path", (p) =>
                updateRotationTarget(
                  p,
                  r.index,
                  Number.isNaN(t.waypoint)
                    ? makeRotationTarget(Number.NaN, round5(clamp(t.meters, 0, g.lengthMeters)), t.headingRadians)
                    : makeRotationTarget(clamp(t.waypoint, 0, last), Number.NaN, t.headingRadians),
                ),
              ),
          },
        ];
      }
      const clip = (from: number, to: number): [number, number] => {
        const a0 = clamp(from, 0, last);
        const b0 = clamp(to, 0, last);
        return b0 > a0 ? [a0, b0] : [Math.max(0, b0 - 0.5), b0];
      };
      if (r.kind === "constraint") {
        const z = spec.constraintZones[r.index];
        if (!z) return [];
        const [from, to] = clip(z.fromWaypoint, z.toWaypoint);
        return [
          {
            label: "Clip to the path",
            tip: `Make it run from waypoint ${from} to ${to}`,
            run: () => editPath("Clip zone to the path", (p) => updateConstraintZone(p, r.index, makeConstraintZone(from, to, z.maxMetersPerSecond, z.maxMetersPerSecondSquared))),
          },
        ];
      }
      if (r.kind === "face") {
        const z = spec.facePointZones[r.index];
        if (!z) return [];
        const [from, to] = clip(z.fromWaypoint, z.toWaypoint);
        return [
          {
            label: "Clip to the path",
            tip: `Make it run from waypoint ${from} to ${to}`,
            run: () => editPath("Clip zone to the path", (p) => updateFacePointZone(p, r.index, makeFacePointZone(from, to, z.point, z.blendMeters))),
          },
        ];
      }
      return [];
    }
    case "speed": {
      const v = check.capMetersPerSecond;
      if (v === undefined || !check.end) return [];
      const speed = Math.max(0, Math.floor(v * 100) / 100);
      return check.end === "start"
        ? [{ label: `Start at ${speed} m/s`, tip: "The speed the plan can start at here", run: () => editPath("Start speed", (p) => withSpec(p, { startMetersPerSecond: speed })) }]
        : [{ label: `End at ${speed} m/s`, tip: "The speed the plan can end at here", run: () => editPath("End speed", (p) => withSpec(p, { endMetersPerSecond: speed })) }];
    }
  }
}

/** Splits a zone at its middle into two that together cover the same stretch (one undo step). */
export function splitZone(kind: "constraint" | "face", index: number): void {
  const spec = activePath().spec;
  if (kind === "constraint") {
    const z = spec.constraintZones[index];
    if (!z) return;
    const mid = round5((z.fromWaypoint + z.toWaypoint) / 2);
    editPath(
      "Split zone",
      (p) => {
        const halved = updateConstraintZone(p, index, makeConstraintZone(z.fromWaypoint, mid, z.maxMetersPerSecond, z.maxMetersPerSecondSquared));
        return addConstraintZone(halved, makeConstraintZone(mid, z.toWaypoint, z.maxMetersPerSecond, z.maxMetersPerSecondSquared));
      },
      { kind: "constraint", index },
    );
  } else {
    const z = spec.facePointZones[index];
    if (!z) return;
    const mid = round5((z.fromWaypoint + z.toWaypoint) / 2);
    editPath(
      "Split zone",
      (p) => {
        const halved = updateFacePointZone(p, index, makeFacePointZone(z.fromWaypoint, mid, z.point, z.blendMeters));
        return addFacePointZone(halved, makeFacePointZone(mid, z.toWaypoint, z.point, z.blendMeters));
      },
      { kind: "face", index },
    );
  }
}
