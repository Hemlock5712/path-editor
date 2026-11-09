// Which outline the robot has where along a path: the path's start footprint, switched by events that name
// one (deployIntake → "intakeOut", stowIntake → "bumpers"). A bigger outline counts from the moment its event
// fires; one being left keeps counting for its retractSeconds after (the mechanism is still coming in), as the
// smallest rectangle around both. Pure, for the analysis, the optimiser and Route around alike.

import {
  BUMPERS,
  eventChangesFootprint,
  eventMeters,
  unionFootprint,
  type Footprint,
  type Path,
  type PathSpec,
} from "../../../core/src/index.ts";

/** The outline in force at a place on the path. */
export interface Outline {
  /** The footprint (with any still retracting merged in, under the active one's name). */
  readonly footprint: Footprint;
  /** The footprints the path switched away from that are still coming in (empty: none). */
  readonly retracting: readonly string[];
}

/** A name the path uses that the robot has no footprint for: an event's (by index) or the start's (-1). */
export interface UnknownFootprint {
  readonly event: number;
  readonly name: string;
}

interface Change {
  readonly s: number;
  readonly t: number;
  readonly from: Footprint;
  readonly to: Footprint;
}

export class FootprintTimeline {
  readonly start: Footprint;
  readonly unknown: readonly UnknownFootprint[];
  private readonly changes: readonly Change[];

  /** footprints: the robot's, bumpers first (robotFootprints). Events' places and times come from path. */
  constructor(spec: PathSpec, footprints: readonly Footprint[], path: Path) {
    const byName = new Map(footprints.map((f) => [f.name, f]));
    const bumpers = byName.get(BUMPERS) ?? footprints[0]!;
    const unknown: UnknownFootprint[] = [];
    let current = byName.get(spec.startFootprint);
    if (current === undefined) {
      unknown.push({ event: -1, name: spec.startFootprint });
      current = bumpers;
    }
    this.start = current;
    const placed = spec.events
      .map((e, index) => ({ e, index, s: Math.min(path.lengthMeters, Math.max(0, eventMeters(e, path.geometry))) }))
      .filter(({ e }) => eventChangesFootprint(e))
      .sort((a, b) => a.s - b.s || a.index - b.index); // the follower fires them in path order
    const changes: Change[] = [];
    for (const { e, index, s } of placed) {
      const to = byName.get(e.footprint!);
      if (to === undefined) {
        unknown.push({ event: index, name: e.footprint! });
        continue;
      }
      if (to !== current) changes.push({ s, t: path.sample(s).seconds, from: current, to });
      current = to;
    }
    this.changes = changes;
    this.unknown = unknown;
  }

  /** True when the path never leaves the bumpers (every check is as it was before footprints). */
  get bumpersOnly(): boolean {
    return this.start.name === BUMPERS && this.changes.length === 0;
  }

  /** The outline at s metres along the path, reached t seconds in. */
  at(s: number, t: number): Outline {
    let active = this.start;
    let k = 0;
    while (k < this.changes.length && this.changes[k]!.s <= s) active = this.changes[k++]!.to;
    let footprint = active;
    const retracting: string[] = [];
    for (let j = 0; j < k; j++) {
      const c = this.changes[j]!;
      if (c.from !== active && t < c.t + c.from.retractSeconds && !retracting.includes(c.from.name)) {
        footprint = unionFootprint(footprint, c.from);
        retracting.push(c.from.name);
      }
    }
    return { footprint, retracting };
  }
}

/**
 * A check's subject and verb for the outline: "Bumpers hit", "With intakeOut, the robot hits", "With intakeOut
 * still retracting, the robot hits". plural and singular are the verb's two forms ("hit", "hits").
 */
export function outlineSays(o: Outline, plural: string, singular: string): string {
  const out = o.footprint.name === BUMPERS ? [] : [o.footprint.name];
  if (out.length === 0 && o.retracting.length === 0) return `Bumpers ${plural}`;
  const still = o.retracting.length > 0 ? `${o.retracting.join(" and ")} still retracting` : "";
  const what = out.length > 0 && still ? `${out[0]} out and ${still}` : out.length > 0 ? out[0] : still;
  return `With ${what}, the robot ${singular}`;
}

/** Stretches of the path by outline: each where the same footprint (and the same ones retracting) is in force. */
export interface OutlineStretch {
  readonly sFrom: number;
  readonly sTo: number;
  readonly outline: Outline;
}

/** The outline in force at s (the stretch containing it; the last one past the end). */
export function outlineAtS(stretches: readonly OutlineStretch[], s: number): Outline {
  return (stretches.find((r) => s <= r.sTo + 1e-9) ?? stretches[stretches.length - 1]!).outline;
}

/** The smallest rectangle around every outline in force between sFrom and sTo, named after the largest. */
export function outlineOver(stretches: readonly OutlineStretch[], sFrom: number, sTo: number): Footprint {
  const over = stretches.filter((r) => r.sTo >= sFrom - 1e-9 && r.sFrom <= sTo + 1e-9).map((r) => r.outline.footprint);
  if (over.length === 0) return outlineAtS(stretches, sFrom).footprint;
  const area = (f: Footprint) => (f.frontMeters + f.backMeters) * (f.leftMeters + f.rightMeters);
  const largest = over.reduce((a, b) => (area(b) > area(a) ? b : a));
  return over.reduce((u, f) => unionFootprint(u, f), largest);
}
