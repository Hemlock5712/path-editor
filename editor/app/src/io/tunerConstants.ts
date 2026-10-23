// Reads the drivetrain numbers out of a CTRE Tuner X TunerConstants.java: the gear ratio, the wheel
// radius, where the modules are, and the speed at 12 V. Only what the plan needs; everything else in the
// file (CAN ids, gains, offsets) is ignored. Values may be plain numbers or WPILib units
// (Inches.of(2.167), MetersPerSecond.of(4.54)), with simple arithmetic inside.

export interface TunerImport {
  readonly driveGearRatio?: number;
  readonly wheelRadiusMeters?: number;
  /** Robot centre to the furthest module, metres. */
  readonly moduleRadiusMeters?: number;
  /** Centre to the nearest tipping edge, the smallest |x| or |y| of any module, metres (AccelerationLimiter's TIP_DISTANCE). */
  readonly tipDistanceMeters?: number;
  readonly speedAt12VoltsMetersPerSecond?: number;
  /** The names of the constants found, for saying what was filled in. */
  readonly found: readonly string[];
}

const LENGTH: Readonly<Record<string, number>> = { Meters: 1, Centimeters: 0.01, Millimeters: 0.001, Inches: 0.0254, Feet: 0.3048 };
const SPEED: Readonly<Record<string, number>> = { MetersPerSecond: 1, FeetPerSecond: 0.3048, InchesPerSecond: 0.0254 };
const MODULES = ["FrontLeft", "FrontRight", "BackLeft", "BackRight"] as const;

/** The constants in a TunerConstants.java; throws when it has none of them. */
export function parseTunerConstants(text: string): TunerImport {
  const code = text.replace(/\/\*[\s\S]*?\*\//g, "").replace(/\/\/[^\n]*/g, "");
  const found: string[] = [];
  const value = (name: string, units: Readonly<Record<string, number>> | null): number | undefined => {
    const m = new RegExp(`\\b${name}\\s*=\\s*([^;]+);`).exec(code);
    if (!m) return undefined;
    const v = units ? withUnit(m[1]!.trim(), units) : arithmetic(m[1]!.trim());
    if (v === undefined || !Number.isFinite(v)) return undefined;
    found.push(name);
    return v;
  };

  const driveGearRatio = value("kDriveGearRatio", null);
  const wheelRadiusMeters = value("kWheelRadius", LENGTH);
  const speedAt12VoltsMetersPerSecond = value("kSpeedAt12Volts", SPEED);
  const modules = MODULES.map((m) => ({ x: value(`k${m}XPos`, LENGTH), y: value(`k${m}YPos`, LENGTH) })).filter(
    (p): p is { x: number; y: number } => p.x !== undefined && p.y !== undefined,
  );
  const moduleRadiusMeters = modules.length > 0 ? Math.max(...modules.map((p) => Math.hypot(p.x, p.y))) : undefined;
  const tipDistanceMeters = modules.length > 0 ? Math.min(...modules.map((p) => Math.min(Math.abs(p.x), Math.abs(p.y)))) : undefined;
  if (found.length === 0) throw new RangeError("no TunerConstants found (kDriveGearRatio, kWheelRadius, the module positions, kSpeedAt12Volts)");
  return {
    ...(positive(driveGearRatio) && { driveGearRatio }),
    ...(positive(wheelRadiusMeters) && { wheelRadiusMeters }),
    ...(positive(moduleRadiusMeters) && { moduleRadiusMeters }),
    ...(positive(tipDistanceMeters) && { tipDistanceMeters }),
    ...(positive(speedAt12VoltsMetersPerSecond) && { speedAt12VoltsMetersPerSecond }),
    found,
  };
}

const positive = (v: number | undefined): v is number => v !== undefined && v > 0;

/** "Inches.of(2.167)" in metres (or metres per second); a bare number is taken as SI. */
function withUnit(expr: string, units: Readonly<Record<string, number>>): number | undefined {
  const m = /^(\w+)\.of\(([\s\S]*)\)$/.exec(expr);
  if (!m) return arithmetic(expr);
  const scale = units[m[1]!];
  const v = arithmetic(m[2]!.trim());
  return scale === undefined || v === undefined ? undefined : v * scale;
}

/** + - * / and brackets over plain numbers (Java's 7.36 or 50.0 / 7.0); anything else is undefined. */
function arithmetic(expr: string): number | undefined {
  const tokens = expr.match(/\d+\.?\d*(?:[eE][+-]?\d+)?[dDfF]?|\.\d+(?:[eE][+-]?\d+)?[dDfF]?|[-+*/()]|\S/g) ?? [];
  let i = 0;
  const peek = () => tokens[i];
  const sum = (): number => {
    let v = product();
    while (peek() === "+" || peek() === "-") v = tokens[i++] === "+" ? v + product() : v - product();
    return v;
  };
  const product = (): number => {
    let v = unary();
    while (peek() === "*" || peek() === "/") v = tokens[i++] === "*" ? v * unary() : v / unary();
    return v;
  };
  const unary = (): number => {
    const t = tokens[i++];
    if (t === "-") return -unary();
    if (t === "+") return unary();
    if (t === "(") {
      const v = sum();
      if (tokens[i++] !== ")") return Number.NaN;
      return v;
    }
    return t !== undefined && /^[\d.]/.test(t) ? Number(t.replace(/[dDfF]$/, "")) : Number.NaN;
  };
  const v = sum();
  return i === tokens.length && Number.isFinite(v) ? v : undefined;
}
