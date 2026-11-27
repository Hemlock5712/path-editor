// The animations' data, worked out when the site is built.
//
// Nothing here is drawn by hand. Each scene builds a real path with the editor's TypeScript port of the
// library's maths (editor/core, which reproduces the Java library's fixtures exactly), plans it, and drives
// it with the real follower on the simulated drivetrain the editor's sim preview uses. The page then only
// plays back the numbers. The driver-assist scenes (rails, the intake assist, the tag goal) have no
// TypeScript port, so they come from the Java library itself: see ../data/README.md.

import {
  DEFAULT_DRIVE,
  DEFAULT_PLANNER,
  DEFAULT_ROBOT,
  Path,
  Pathfinder,
  eventMeters,
  fieldFromJson,
  flipPoint,
  followerConfig,
  readPath,
  runFollowerSim,
  tractionMetersPerSecondSquared,
  type FieldFrame,
  type FollowTrace,
  type RobotConfig,
  type Shove,
} from '../../../editor/core/src/index.ts';
import assists from '../data/assists.json' with { type: 'json' };
import field2026 from '../../../fixtures/plan/field_2026.json' with { type: 'json' };

// ------------------------------------------------------------------------------------------ the shapes

export type Xy = [number, number];

/** One thing that moves: a robot outline (or a dot) played back from samples every `dt` seconds. */
export interface Track {
  /** How it is drawn: the robot, a faint "what would have happened" robot, or a small dot. */
  kind: 'robot' | 'ghost' | 'dot';
  label?: string;
  dt: number;
  x: number[];
  y: number[];
  h: number[];
}

/** A line on the field. `v` (one per point) colours it by speed; otherwise `style` picks the pen. */
export interface Line {
  pts: Xy[];
  v?: number[];
  style?: 'path' | 'route' | 'rail' | 'faint' | 'driver';
  /** Seconds into the scene when it appears (0 = from the start). */
  at?: number;
  label?: string;
}

export interface Shape {
  pts: Xy[];
  style: 'obstacle' | 'grown' | 'zone';
  at?: number;
  label?: string;
}

export interface Mark {
  x: number;
  y: number;
  style: 'waypoint' | 'event' | 'goal' | 'piece' | 'tag' | 'start' | 'point';
  label?: string;
  /** A tag's facing, radians: the way its face points. */
  h?: number;
  /** Drawn faint: where something was expected, not where it is. */
  faint?: boolean;
  /** Where the label sits (above and to the right unless set). */
  side?: 'above' | 'below' | 'left';
  /** For an event: the second it fires, when its label lights up. */
  firesAt?: number;
  at?: number;
  /** A mark that moves: its position at each `dt` (overrides x, y). */
  dt?: number;
  xs?: number[];
  ys?: number[];
}

/** A strip chart under the field, against the scene's time. */
export interface Chart {
  yLabel: string;
  yMax: number;
  dt: number;
  series: { label: string; style: 'plan' | 'actual' | 'limit'; y: number[] }[];
}

/** A caption that changes as the scene plays. */
export interface Step {
  at: number;
  text: string;
}

export interface Variant {
  label: string;
  /** Seconds of playback (the tracks may start late: see `driveAt`). */
  seconds: number;
  /** When the tracks start moving, after the lines and shapes have been revealed. */
  driveAt: number;
  lines: Line[];
  shapes: Shape[];
  marks: Mark[];
  tracks: Track[];
  chart?: Chart;
  steps?: Step[];
  /** Numbers shown beside the controls, such as the time taken. */
  facts?: { label: string; value: string }[];
}

export interface Scene {
  /** The part of the field shown, metres: [xMin, yMin, xMax, yMax]. */
  view: [number, number, number, number];
  /** The robot's bumpers, metres: [length, width]. */
  robot: [number, number];
  /** The field's outline, if any of it is in view: [length, width] about the origin. */
  field?: [number, number];
  speedMax: number;
  variants: Variant[];
}

// --------------------------------------------------------------------------------------------- helpers

const ROBOT: RobotConfig = DEFAULT_ROBOT;
const BUMPERS: [number, number] = [DEFAULT_PLANNER.bumperLengthMeters, DEFAULT_PLANNER.bumperWidthMeters];
const FIELD: [number, number] = [field2026.field.length_m, field2026.field.width_m];
const r3 = (n: number) => Math.round(n * 1000) / 1000;
const deg = (d: number) => (d * Math.PI) / 180;

function build(pathJson: object): Path {
  return Path.build(readPath(JSON.stringify({ schema: 4, ...pathJson })), ROBOT);
}

/** The plan as a line: a point every 5 cm with the planned speed there, in `frame`. */
function planLine(path: Path, frame: FieldFrame = 'BLUE', at = 0): Line {
  const pts: Xy[] = [];
  const v: number[] = [];
  const n = Math.max(2, Math.ceil(path.lengthMeters / 0.05));
  for (let i = 0; i <= n; i++) {
    const s = path.sample((path.lengthMeters * i) / n);
    const p = flipPoint(frame, { x: s.xMeters, y: s.yMeters });
    pts.push([r3(p.x), r3(p.y)]);
    v.push(r3(s.metersPerSecond));
  }
  return { pts, v, style: 'path', at };
}

function waypointMarks(path: Path, frame: FieldFrame = 'BLUE', at = 0): Mark[] {
  return path.spec.waypoints.map((w) => {
    const p = flipPoint(frame, { x: w.xMeters, y: w.yMeters });
    return { x: r3(p.x), y: r3(p.y), style: 'waypoint' as const, at };
  });
}

/** Drives `path` with the real follower on the simulated drivetrain. */
function drive(path: Path, frame: FieldFrame = 'BLUE', shove: Shove | null = null): FollowTrace {
  return runFollowerSim(path, followerConfig(ROBOT), DEFAULT_DRIVE, frame, shove, 0.5, path.totalTimeSeconds + 6);
}

/** A trace as a track, at 25 samples a second. */
function track(trace: FollowTrace, kind: Track['kind'] = 'robot', label?: string): Track {
  const pick = (a: number[]) => a.filter((_, i) => i % 2 === 0).map(r3);
  return { kind, label, dt: 0.04, x: pick(trace.x), y: pick(trace.y), h: pick(trace.heading) };
}

function traceSeconds(trace: FollowTrace): number {
  return trace.t.length > 0 ? trace.t[trace.t.length - 1]! - trace.t[0]! : 0;
}

function finishSeconds(trace: FollowTrace): number {
  return trace.finishLoop >= 0 ? trace.t[trace.finishLoop]! - trace.t[0]! : Number.NaN;
}

/** Planned and driven speed against time, for the strip chart. */
function speedChart(path: Path, trace: FollowTrace): Chart {
  const plan: number[] = [];
  const actual: number[] = [];
  for (let i = 0; i < trace.t.length; i += 2) {
    const t = trace.t[i]! - trace.t[0]!;
    plan.push(r3(t <= path.totalTimeSeconds ? path.sampleAtTime(t).metersPerSecond : path.spec.endMetersPerSecond));
    actual.push(r3(Math.hypot(trace.vx[i]!, trace.vy[i]!)));
  }
  return {
    yLabel: 'm/s',
    yMax: Math.ceil(Math.max(...plan, ...actual) + 0.2),
    dt: 0.04,
    series: [
      { label: 'Planned speed', style: 'plan', y: plan },
      { label: 'Driven speed', style: 'actual', y: actual },
    ],
  };
}

function eventMarks(path: Path, trace: FollowTrace): Mark[] {
  return path.spec.events.map((e) => {
    const s = path.sample(eventMeters(e, path.geometry));
    const fired = trace.events.find((f) => f.name === e.name);
    return {
      x: r3(s.xMeters),
      y: r3(s.yMeters),
      style: 'event' as const,
      label: e.name,
      firesAt: fired ? r3(trace.t[fired.loop]! - trace.t[0]!) : undefined,
    };
  });
}

const obstacleShapes = (at = 0): Shape[] =>
  field2026.obstacles.map((o) => ({ pts: o.polygon.map((p) => [p[0]!, p[1]!] as Xy), style: 'obstacle' as const, at }));

const seconds = (t: number) => `${t.toFixed(2)} s`;

// ------------------------------------------------------------------------------------------ the scenes

/** The path most pages drive: out from the blue wall, between the hub and a trench support, and round. */
const TOUR = {
  headingMode: 'hold',
  waypoints: [
    { x: 6.9, y: -3.0, tangentDegrees: 180 },
    { x: 5.3, y: -1.6 },
    { x: 3.65, y: -1.52 },
    { x: 2.3, y: -2.3 },
    { x: 1.2, y: -0.9, tangentDegrees: 90 },
  ],
  rotationTargets: [
    { waypoint: 0, degrees: 180 },
    { waypoint: 2, degrees: 180 },
    { waypoint: 4, degrees: 90 },
  ],
  events: [
    { name: 'intakeDown', atWaypoint: 1.5 },
    { name: 'spinUp', atWaypoint: 3.2 },
    { name: 'shoot', atWaypoint: 4 },
  ],
  maxMetersPerSecond: 4,
  maxMetersPerSecondSquared: 6,
};
const TOUR_VIEW: Scene['view'] = [0.2, -3.8, 8.45, 0.95];

function follow(withEvents: boolean, withChart: boolean): Scene {
  const path = build(TOUR);
  const trace = drive(path);
  return {
    view: TOUR_VIEW,
    robot: BUMPERS,
    field: FIELD,
    speedMax: 4,
    variants: [
      {
        label: 'Run',
        seconds: traceSeconds(trace),
        driveAt: 0,
        lines: [planLine(path)],
        shapes: obstacleShapes(),
        marks: [...waypointMarks(path), ...(withEvents ? eventMarks(path, trace) : [])],
        tracks: [track(trace)],
        chart: withChart ? speedChart(path, trace) : undefined,
        facts: [
          { label: 'Length', value: `${path.lengthMeters.toFixed(2)} m` },
          { label: 'Planned', value: seconds(path.totalTimeSeconds) },
          { label: 'Driven', value: seconds(finishSeconds(trace)) },
        ],
      },
    ],
  };
}

/** The same path at three acceleration limits: what the path's own limit does to the plan. */
function limits(): Scene {
  const variants = [3, 6, 12].map((a): Variant => {
    const path = build({ ...TOUR, events: [], maxMetersPerSecondSquared: a });
    const trace = drive(path);
    return {
      label: `${a} m/s²`,
      seconds: traceSeconds(trace),
      driveAt: 0,
      lines: [planLine(path)],
      shapes: obstacleShapes(),
      marks: waypointMarks(path),
      tracks: [track(trace)],
      chart: speedChart(path, trace),
      facts: [
        { label: 'Planned', value: seconds(path.totalTimeSeconds) },
        { label: 'Driven', value: seconds(finishSeconds(trace)) },
      ],
    };
  });
  return { view: TOUR_VIEW, robot: BUMPERS, field: FIELD, speedMax: 4.6, variants };
}

/** One path, drawn for blue, on each alliance frame. */
function alliances(): Scene {
  const path = build({ ...TOUR, events: [] });
  const frames: [string, FieldFrame][] = [
    ['Blue', 'BLUE'],
    ['Red, half turn', 'RED_HALF_TURN'],
    ['Red, mirror', 'RED_MIRROR'],
  ];
  return {
    view: [-8.45, -4.2, 8.45, 4.2],
    robot: BUMPERS,
    field: FIELD,
    speedMax: 4,
    variants: frames.map(([label, frame]) => {
      const trace = drive(path, frame);
      return {
        label,
        seconds: traceSeconds(trace),
        driveAt: 0,
        lines: [...(frame === 'BLUE' ? [] : [{ ...planLine(path), v: undefined, style: 'faint' as const }]), planLine(path, frame)],
        shapes: obstacleShapes(),
        marks: waypointMarks(path, frame),
        tracks: [track(trace)],
      };
    }),
  };
}

/** Two paths in a row: stopping between them, and handing over at speed. */
function chain(): Scene {
  const first = {
    headingMode: 'hold',
    waypoints: [
      { x: 6.9, y: -3.0, tangentDegrees: 180 },
      { x: 5.3, y: -1.6 },
      { x: 3.65, y: -1.52, tangentDegrees: 180 },
    ],
    rotationTargets: [{ waypoint: 0, degrees: 180 }],
    maxMetersPerSecond: 4,
    maxMetersPerSecondSquared: 6,
  };
  const second = {
    headingMode: 'hold',
    waypoints: [
      { x: 3.65, y: -1.52, tangentDegrees: 180 },
      { x: 2.3, y: -2.3 },
      { x: 1.2, y: -0.9, tangentDegrees: 90 },
    ],
    rotationTargets: [
      { waypoint: 0, degrees: 180 },
      { waypoint: 2, degrees: 90 },
    ],
    maxMetersPerSecond: 4,
    maxMetersPerSecondSquared: 6,
  };
  const make = (label: string, handOver: number): Variant => {
    const a = build({ ...first, endMetersPerSecond: handOver });
    const b = build({ ...second, startMetersPerSecond: handOver });
    const ta = drive(a);
    const tb = drive(b);
    // the first path's run up to its finish, then the second's from its start
    const cut = ta.finishLoop >= 0 ? ta.finishLoop + 1 : ta.t.length;
    const joined: FollowTrace = {
      ...tb,
      t: [...ta.t.slice(0, cut), ...tb.t.map((t) => t - tb.t[0]! + ta.t[cut - 1]! + 0.02)],
      x: [...ta.x.slice(0, cut), ...tb.x],
      y: [...ta.y.slice(0, cut), ...tb.y],
      heading: [...ta.heading.slice(0, cut), ...tb.heading],
      vx: [...ta.vx.slice(0, cut), ...tb.vx],
      vy: [...ta.vy.slice(0, cut), ...tb.vy],
    };
    const speed: number[] = [];
    for (let i = 0; i < joined.t.length; i += 2) speed.push(r3(Math.hypot(joined.vx[i]!, joined.vy[i]!)));
    const total = finishSeconds(ta) + 0.02 + finishSeconds(tb);
    return {
      label,
      seconds: joined.t[joined.t.length - 1]! - joined.t[0]!,
      driveAt: 0,
      lines: [planLine(a), planLine(b)],
      shapes: obstacleShapes(),
      marks: [...waypointMarks(a), ...waypointMarks(b).slice(1), { x: 3.65, y: -1.52, style: 'point', label: 'hand-over' }],
      tracks: [track(joined)],
      chart: { yLabel: 'm/s', yMax: 5, dt: 0.04, series: [{ label: 'Driven speed', style: 'actual', y: speed }] },
      facts: [
        { label: 'Hand-over speed', value: `${handOver} m/s` },
        { label: 'Both paths', value: seconds(total) },
      ],
    };
  };
  return { view: TOUR_VIEW, robot: BUMPERS, field: FIELD, speedMax: 4, variants: [make('Stop between', 0), make('Rolling end', 2)] };
}

/** A shove partway along: the follower steers back onto the path where it is. */
function shove(): Scene {
  const path = build({ ...TOUR, events: [] });
  const clean = drive(path);
  const hit = drive(path, 'BLUE', { atSeconds: 1.1, dvx: 0.4, dvy: -2.2 });
  return {
    view: TOUR_VIEW,
    robot: BUMPERS,
    field: FIELD,
    speedMax: 4,
    variants: [
      {
        label: 'Shoved at 1.1 s',
        seconds: traceSeconds(hit),
        driveAt: 0,
        lines: [planLine(path)],
        shapes: obstacleShapes(),
        marks: waypointMarks(path),
        tracks: [track(clean, 'ghost', 'no shove'), track(hit)],
        steps: [
          { at: 0, text: 'Following the plan.' },
          { at: 1.1, text: 'A hit knocks the robot sideways at 2.2 m/s.' },
          { at: 1.5, text: 'The follower pushes back toward the line, in proportion to how far off it is.' },
          { at: 2.6, text: 'Back on the path. The plan was never rebuilt.' },
        ],
        facts: [
          { label: 'Without the shove', value: seconds(finishSeconds(clean)) },
          { label: 'With it', value: seconds(finishSeconds(hit)) },
        ],
      },
    ],
  };
}

/** On the fly: grow the obstacles, find the route, smooth it, drive it. */
function pathfind(): Scene {
  const field = fieldFromJson(field2026);
  const finder = new Pathfinder(field, ROBOT, DEFAULT_PLANNER);
  const trips: [string, [number, number, number], [number, number, number]][] = [
    ['Round the hub', [6.4, 0.2, 180], [1.3, -0.3, 180]],
    ['Across the field', [6.6, -2.9, 180], [-5.6, 2.6, 180]],
    ['Through the trench gap', [5.6, 3.4, -90], [5.4, -3.3, -90]],
  ];
  const variants = trips.map(([label, from, to]): Variant => {
    const start = { xMeters: from[0], yMeters: from[1], headingRadians: deg(from[2]) };
    const goal = { xMeters: to[0], yMeters: to[1], headingRadians: deg(to[2]) };
    const plan = finder.plan(start, { vx: 0, vy: 0, omega: 0 }, goal);
    if (plan.path === null) throw new Error(`docs scene "${label}": ${plan.failure}`);
    const trace = drive(plan.path);
    return {
      label,
      seconds: 3.2 + traceSeconds(trace),
      driveAt: 3.2,
      lines: [
        { pts: [[from[0], from[1]], [to[0], to[1]]], style: 'faint', at: 0 },
        { pts: plan.waypoints.map((w) => [r3(w[0]), r3(w[1])] as Xy), style: 'route', at: 1.5 },
        planLine(plan.path, 'BLUE', 2.3),
      ],
      shapes: [
        ...obstacleShapes(),
        ...finder.grown.map((g) => ({ pts: Array.from(g.xs, (x, i) => [r3(x), r3(g.ys[i]!)] as Xy), style: 'grown' as const, at: 0.7 })),
      ],
      marks: [
        { x: from[0], y: from[1], style: 'start', label: 'robot' },
        { x: to[0], y: to[1], style: 'goal', label: 'goal' },
      ],
      tracks: [track(trace)],
      steps: [
        { at: 0, text: 'The straight line to the goal goes through an obstacle.' },
        { at: 0.7, text: 'Grow every obstacle by the robot’s reach. The robot’s center must stay outside the dashed shapes.' },
        { at: 1.5, text: 'The shortest route round them runs corner to corner.' },
        { at: 2.3, text: 'Smooth the corners into a path and plan its speed.' },
        { at: 3.2, text: 'Follow it, like any drawn path.' },
      ],
      facts: [
        { label: 'Straight line', value: `${plan.straightLineMeters.toFixed(2)} m` },
        { label: 'Route', value: `${plan.path.lengthMeters.toFixed(2)} m` },
        { label: 'Closest approach', value: `${(plan.minClearanceMeters * 100).toFixed(0)} cm` },
      ],
    };
  });
  return { view: [-8.45, -4.2, 8.45, 4.2], robot: BUMPERS, field: FIELD, speedMax: 4, variants };
}

/** Drive to a pose on an open floor, from three places. */
function driveToPose(): Scene {
  const open = fieldFromJson({ field: { length_m: 1000, width_m: 1000 }, obstacles: [] });
  const finder = new Pathfinder(open, ROBOT, DEFAULT_PLANNER);
  const goal = { xMeters: 0, yMeters: 0, headingRadians: deg(90) };
  const starts: [string, [number, number, number]][] = [
    ['From 3 m away', [-2.6, -1.5, 0]],
    ['From the side', [2.4, 1.2, 200]],
    ['From 15 cm away', [0.12, -0.09, 80]],
  ];
  const variants = starts.map(([label, from]): Variant => {
    const start = { xMeters: from[0], yMeters: from[1], headingRadians: deg(from[2]) };
    const plan = finder.plan(start, { vx: 0, vy: 0, omega: 0 }, goal);
    if (plan.path === null) throw new Error(`docs scene "${label}": ${plan.failure}`);
    const trace = drive(plan.path);
    return {
      label,
      seconds: traceSeconds(trace),
      driveAt: 0,
      lines: [planLine(plan.path)],
      shapes: [],
      marks: [{ x: 0, y: 0, style: 'goal', label: 'goal' }],
      tracks: [track(trace)],
      chart: speedChart(plan.path, trace),
      facts: [
        { label: 'Distance', value: `${plan.path.lengthMeters.toFixed(2)} m` },
        { label: 'Settled after', value: seconds(finishSeconds(trace)) },
      ],
    };
  });
  return { view: [-3.6, -2.3, 3.6, 2.1], robot: BUMPERS, speedMax: 4, variants };
}

const scenes = { follow, limits, alliances, chain, shove, pathfind, driveToPose } as const;
export type SceneName = 'follow' | 'events' | 'speed' | 'rail' | 'piece' | 'tag' | keyof typeof scenes;

const cache = new Map<string, Scene>();

/** The scene called `name`, built once per site build. */
export function scene(name: SceneName): Scene {
  let s = cache.get(name);
  if (!s) {
    if (name === 'follow') s = follow(false, false);
    else if (name === 'events') s = follow(true, false);
    else if (name === 'speed') s = follow(false, true);
    else if (name === 'rail' || name === 'piece' || name === 'tag') s = assists[name] as unknown as Scene;
    else s = scenes[name]();
    cache.set(name, s);
  }
  return s;
}

/** The default robot's traction limit, m/s², for the pages that quote it. */
export const TRACTION_LIMIT = tractionMetersPerSecondSquared(ROBOT);
