// Read-only hooks for the browser tests (e2e-author and the Playwright specs): the active path's
// preview numbers, unrounded, and where its robots and their handles are on the field.

import { analysis, currentPreview, geometryOf } from "./state/analysis.ts";
import { activePath, editor } from "./state/editor.ts";
import { collisions, rotationHandle, rotationTargetS, waypointHandle, waypointPose } from "./ui/draw.ts";

export interface Poses {
  /** Each waypoint's pose (its set heading, or the plan's there) and its rotate knob, blue frame. */
  readonly waypoints: readonly { x: number; y: number; heading: number; set: boolean; knobX: number; knobY: number }[];
  /** Each heading along the path: where it sits, its heading and its knob. */
  readonly targets: readonly { s: number; x: number; y: number; heading: number; knobX: number; knobY: number }[];
  /** The plan every ~10 cm: arc length, position, heading and direction of travel (radians). */
  readonly plan: readonly { s: number; x: number; y: number; heading: number; travel: number }[];
  /** Where the swept bumpers hit an obstacle or the wall (drawn red on the path), and the least clearance, m. */
  readonly collisions: readonly { sFrom: number; sTo: number; message: string }[];
  readonly clearanceMeters: number;
}

/** The active path's poses, once the plan matches the document (a path still being drawn: just its robots). */
export async function currentPoses(): Promise<Poses> {
  const draft = activePath().spec.waypoints.length < 2;
  if (!draft) await currentPreview();
  const s = editor.get();
  const path = activePath(s);
  if (draft) {
    const model = { active: path, analysis: null, project: s.project };
    const waypoints = path.spec.waypoints.map((_, i) => {
      const h = waypointHandle(model, i);
      return { ...waypointPose(model, i), knobX: h[2], knobY: h[3] };
    });
    return { waypoints, targets: [], plan: [], collisions: [], clearanceMeters: Number.NaN };
  }
  const a = analysis.get().result;
  if (!a || !a.ok) throw new Error("no plan");
  const g = geometryOf(path.spec);
  if (!g) throw new Error("no path yet");
  const model = { active: path, analysis: a, project: s.project, geometry: g };
  const waypoints = path.spec.waypoints.map((_, i) => {
    const p = waypointPose(model, i);
    const h = waypointHandle(model, i);
    return { ...p, knobX: h[2], knobY: h[3] };
  });
  const targets = path.spec.rotationTargets.map((t) => {
    const h = rotationHandle(model, t)!;
    return { s: rotationTargetS(t, g), x: h[0], y: h[1], heading: t.headingRadians, knobX: h[2], knobY: h[3] };
  });
  const st = a.stations;
  const plan = [];
  let next = 0;
  for (let i = 0; i < st.s.length; i++) {
    if (st.s[i]! < next && i !== st.s.length - 1) continue;
    next = st.s[i]! + 0.1;
    const j = Math.min(i + 1, st.s.length - 1);
    const k = j === i ? i - 1 : i;
    const travel = Math.atan2(st.y[j]! - st.y[k]!, st.x[j]! - st.x[k]!);
    plan.push({ s: st.s[i]!, x: st.x[i]!, y: st.y[i]!, heading: st.heading[i]!, travel });
  }
  const hits = collisions(a).map((c) => ({ sFrom: c.sFrom, sTo: c.sTo, message: c.message }));
  return { waypoints, targets, plan, collisions: hits, clearanceMeters: a.minClearanceMeters };
}

export function installTestHook(): void {
  (window as unknown as { pathEditorTest: object }).pathEditorTest = { preview: currentPreview, poses: currentPoses };
}
