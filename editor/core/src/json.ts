// Deterministic JSON writing, the same characters as lib's Json.java: 2-space indents, an object or
// array holding only plain values on one line, anything else one entry per line, keys in insertion
// order. Reading is JSON.parse.

export type JsonValue = null | boolean | number | string | JsonValue[] | { [key: string]: JsonValue };

/**
 * Rounds to 5 decimals and drops trailing zeros: 1.5, -0.00001, 3. Integer arithmetic after one
 * rounding (Math.round rounds half up in both languages), so Java and TS print the same text.
 */
export function fixed5(x: number): string {
  if (!Number.isFinite(x)) throw new RangeError(`JSON can't hold ${x}`);
  const r = Math.round(x * 100000.0);
  if (r === 0) return "0";
  const sign = r < 0 ? "-" : "";
  const abs = Math.abs(r);
  const whole = Math.floor(abs / 100000);
  const frac = abs % 100000;
  if (frac === 0) return `${sign}${whole}`;
  const digits = String(frac).padStart(5, "0").replace(/0+$/, "");
  return `${sign}${whole}.${digits}`;
}

/** Shortest text that reads back to the same double. */
export function exact(x: number): string {
  if (!Number.isFinite(x)) throw new RangeError(`JSON can't hold ${x}`);
  return Object.is(x, -0) ? "0" : String(x);
}

function isPlain(v: JsonValue): boolean {
  return v === null || typeof v !== "object";
}

/** Writes a value with the given number format, ending in a newline. */
export function writeJson(value: JsonValue, numbers: (x: number) => string): string {
  return write(value, numbers, "") + "\n";
}

function write(v: JsonValue, numbers: (x: number) => string, indent: string): string {
  if (v === null) return "null";
  if (typeof v === "number") return numbers(v);
  if (typeof v === "boolean") return String(v);
  if (typeof v === "string") return quote(v);
  const inner = indent + "  ";
  if (Array.isArray(v)) {
    if (v.length === 0) return "[]";
    if (v.every(isPlain)) return "[" + v.map((e) => write(e, numbers, inner)).join(", ") + "]";
    return "[\n" + v.map((e) => inner + write(e, numbers, inner)).join(",\n") + "\n" + indent + "]";
  }
  const entries = Object.entries(v);
  if (entries.length === 0) return "{}";
  if (entries.every(([, e]) => isPlain(e))) {
    return "{ " + entries.map(([k, e]) => `${quote(k)}: ${write(e, numbers, inner)}`).join(", ") + " }";
  }
  return (
    "{\n" +
    entries.map(([k, e]) => `${inner}${quote(k)}: ${write(e, numbers, inner)}`).join(",\n") +
    "\n" +
    indent +
    "}"
  );
}

function quote(s: string): string {
  let out = '"';
  for (const ch of s) {
    const c = ch.charCodeAt(0);
    if (ch === '"') out += '\\"';
    else if (ch === "\\") out += "\\\\";
    else if (ch === "\n") out += "\\n";
    else if (ch === "\r") out += "\\r";
    else if (ch === "\t") out += "\\t";
    else if (c < 0x20) out += "\\u" + c.toString(16).padStart(4, "0");
    else out += ch;
  }
  return out + '"';
}
