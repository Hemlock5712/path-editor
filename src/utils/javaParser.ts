import {
  NamedPath,
  Point,
  HeadingWaypoint,
  VelocityConstraints,
  ConstraintZone,
  RotationZone,
  WaypointFlag,
  DEFAULT_CONSTRAINTS,
} from '../types';

/** Regex fragment matching a Java numeric literal (integers, decimals, scientific notation). */
const NUM = '[-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?';

/**
 * Strips single-line (//) and multi-line block comments from Java source.
 */
function stripComments(source: string): string {
  return source.replace(/\/\/.*$/gm, '').replace(/\/\*[\s\S]*?\*\//g, '');
}

/**
 * Finds the substring from `start` to the matching closing parenthesis,
 * accounting for nested parens.  Returns the content *inside* the parens.
 */
function extractBalancedParens(source: string, start: number): string | null {
  if (source[start] !== '(') return null;
  let depth = 0;
  for (let i = start; i < source.length; i++) {
    if (source[i] === '(') depth++;
    else if (source[i] === ')') {
      depth--;
      if (depth === 0) return source.slice(start + 1, i);
    }
  }
  return null;
}

/**
 * Splits a string by commas at parenthesis-depth 0 only.
 */
function splitTopLevelArgs(content: string): string[] {
  const args: string[] = [];
  let depth = 0;
  let current = '';
  for (const ch of content) {
    if (ch === '(' || ch === '{' || ch === '[') depth++;
    else if (ch === ')' || ch === '}' || ch === ']') depth--;

    if (ch === ',' && depth === 0) {
      args.push(current.trim());
      current = '';
    } else {
      current += ch;
    }
  }
  if (current.trim()) args.push(current.trim());
  return args;
}

function parseControlPoints(arg: string): Point[] {
  const points: Point[] = [];
  const re = new RegExp(
    `new\\s+Translation2d\\(\\s*(${NUM})\\s*,\\s*(${NUM})\\s*\\)`,
    'g'
  );
  let m: RegExpExecArray | null;
  while ((m = re.exec(arg)) !== null) {
    points.push({ x: parseFloat(m[1]), y: parseFloat(m[2]) });
  }
  return points;
}

function parseHeadingWaypoints(arg: string): HeadingWaypoint[] {
  const waypoints: HeadingWaypoint[] = [];
  const re = new RegExp(
    `new\\s+PathData\\.HeadingWaypoint\\(\\s*(${NUM})\\s*,\\s*Rotation2d\\.fromDegrees\\(\\s*(${NUM})\\s*\\)\\s*\\)`,
    'g'
  );
  let m: RegExpExecArray | null;
  while ((m = re.exec(arg)) !== null) {
    waypoints.push({
      waypointIndex: parseFloat(m[1]),
      degrees: parseFloat(m[2]),
    });
  }
  return waypoints;
}

function parseConstraints(arg: string): VelocityConstraints {
  const c: VelocityConstraints = { ...DEFAULT_CONSTRAINTS };
  const extract = (method: string): number | null => {
    const re = new RegExp(`\\.${method}\\(\\s*(${NUM})\\s*\\)`);
    const m = re.exec(arg);
    return m ? parseFloat(m[1]) : null;
  };
  const v = extract('withMaxVelocity');
  if (v !== null) c.maxVelocity = v;
  const a = extract('withMaxAcceleration');
  if (a !== null) c.maxAcceleration = a;
  const sv = extract('withStartVelocity');
  if (sv !== null) c.startVelocity = sv;
  const ev = extract('withEndVelocity');
  if (ev !== null) c.endVelocity = ev;
  return c;
}

function parseConstraintZones(arg: string): ConstraintZone[] {
  const zones: ConstraintZone[] = [];
  const re = new RegExp(
    `new\\s+PathData\\.ConstraintZone\\(\\s*(${NUM})\\s*,\\s*(${NUM})\\s*,\\s*(${NUM})\\s*,\\s*(${NUM})\\s*\\)`,
    'g'
  );
  let m: RegExpExecArray | null;
  while ((m = re.exec(arg)) !== null) {
    zones.push({
      id: crypto.randomUUID(),
      startWaypointIndex: parseInt(m[1], 10),
      endWaypointIndex: parseInt(m[2], 10),
      maxVelocity: parseFloat(m[3]),
      maxAcceleration: parseFloat(m[4]),
    });
  }
  return zones;
}

function parseRotationZones(arg: string): RotationZone[] {
  const zones: RotationZone[] = [];
  const re = new RegExp(
    `new\\s+PathData\\.RotationZone\\(\\s*"([^"]*)"\\s*,\\s*(${NUM})\\s*,\\s*(${NUM})\\s*,\\s*new\\s+Translation2d\\(\\s*(${NUM})\\s*,\\s*(${NUM})\\s*\\)\\s*\\)`,
    'g'
  );
  let m: RegExpExecArray | null;
  while ((m = re.exec(arg)) !== null) {
    zones.push({
      id: m[1] || crypto.randomUUID(),
      startWaypointIndex: parseFloat(m[2]),
      endWaypointIndex: parseFloat(m[3]),
      targetPoint: { x: parseFloat(m[4]), y: parseFloat(m[5]) },
    });
  }
  return zones;
}

function parseWaypointFlags(arg: string): WaypointFlag[] {
  const flags: WaypointFlag[] = [];
  const re = new RegExp(
    `new\\s+PathData\\.WaypointFlag\\(\\s*"([^"]*)"\\s*,\\s*(\\d+)\\s*,\\s*"([^"]*)"\\s*\\)`,
    'g'
  );
  let m: RegExpExecArray | null;
  while ((m = re.exec(arg)) !== null) {
    flags.push({
      id: m[1] || crypto.randomUUID(),
      waypointIndex: parseInt(m[2], 10),
      label: m[3],
    });
  }
  return flags;
}

/**
 * Parses a Paths.java source file and returns all PathData constants.
 */
export function parsePathsJava(source: string): NamedPath[] {
  const cleaned = stripComments(source);
  const paths: NamedPath[] = [];

  // Match each: public static final PathData NAME = new PathData(
  const declRe =
    /public\s+static\s+final\s+PathData\s+(\w+)\s*=\s*new\s+PathData\s*\(/g;
  let declMatch: RegExpExecArray | null;

  while ((declMatch = declRe.exec(cleaned)) !== null) {
    const name = declMatch[1];
    const parenStart = declMatch.index + declMatch[0].length - 1; // index of '('
    const inner = extractBalancedParens(cleaned, parenStart);
    if (!inner) {
      console.warn(
        `javaParser: could not find matching paren for ${name}, skipping`
      );
      continue;
    }

    const args = splitTopLevelArgs(inner);
    if (args.length < 3) {
      console.warn(
        `javaParser: expected at least 3 args for ${name}, got ${args.length}, skipping`
      );
      continue;
    }

    const controlPoints = parseControlPoints(args[0]);
    if (controlPoints.length < 2) {
      console.warn(
        `javaParser: ${name} has fewer than 2 control points, skipping`
      );
      continue;
    }

    paths.push({
      name,
      controlPoints,
      controlPointRefs: Array(controlPoints.length).fill(null),
      headingWaypoints: args.length > 1 ? parseHeadingWaypoints(args[1]) : [],
      constraints:
        args.length > 2
          ? parseConstraints(args[2])
          : { ...DEFAULT_CONSTRAINTS },
      constraintZones: args.length > 3 ? parseConstraintZones(args[3]) : [],
      rotationZones: args.length > 4 ? parseRotationZones(args[4]) : [],
      waypointFlags: args.length > 5 ? parseWaypointFlags(args[5]) : [],
    });
  }

  if (paths.length === 0) {
    throw new Error('No valid PathData constants found in the Java file.');
  }

  return paths;
}
