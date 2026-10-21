// Java's Math.hypot, bit for bit (the JDK's FdLibm.Hypot), so the follower port rounds as Java does. V8's
// Math.hypot uses a different algorithm and can differ in the last bit, which a closed loop then carries.

const buf = new DataView(new ArrayBuffer(8));

function hi(x: number): number {
  buf.setFloat64(0, x);
  return buf.getInt32(0);
}

function withHi(x: number, high: number): number {
  buf.setFloat64(0, x);
  buf.setInt32(0, high);
  return buf.getFloat64(0);
}

const TWO_MINUS_600 = 2 ** -600;
const TWO_PLUS_600 = 2 ** 600;
const MIN_NORMAL = 2.2250738585072014e-308;
const ABOVE_2_500 = 3.2733937296446915e150; // 0x1.00000_ffff_ffffp500

/** sqrt(x² + y²) without undue overflow or underflow, as java.lang.Math.hypot computes it. */
export function javaHypot(x: number, y: number): number {
  let a = Math.abs(x);
  let b = Math.abs(y);
  if (!Number.isFinite(a) || !Number.isFinite(b)) {
    if (a === Infinity || b === Infinity) return Infinity;
    return a + b;
  }
  if (b > a) {
    const tmp = a;
    a = b;
    b = tmp;
  }
  let ha = hi(a);
  let hb = hi(b);
  if (ha - hb > 0x3c00000) return a + b;
  let k = 0;
  if (a > ABOVE_2_500) {
    ha -= 0x25800000;
    hb -= 0x25800000;
    a = a * TWO_MINUS_600;
    b = b * TWO_MINUS_600;
    k += 600;
  }
  let t1: number;
  let t2: number;
  if (b < 2 ** -500) {
    if (b < MIN_NORMAL) {
      if (b === 0.0) return a;
      t1 = 2 ** 1022;
      b *= t1;
      a *= t1;
      k -= 1022;
    } else {
      ha += 0x25800000;
      hb += 0x25800000;
      a = a * TWO_PLUS_600;
      b = b * TWO_PLUS_600;
      k -= 600;
    }
  }
  let w = a - b;
  if (w > b) {
    t1 = withHi(0, ha);
    t2 = a - t1;
    w = Math.sqrt(t1 * t1 - (b * -b - t2 * (a + t1)));
  } else {
    a = a + a;
    const y1 = withHi(0, hb);
    const y2 = b - y1;
    t1 = withHi(0, ha + 0x00100000);
    t2 = a - t1;
    w = Math.sqrt(t1 * y1 - (w * -w - (t1 * y2 + t2 * b)));
  }
  return k !== 0 ? 2 ** k * w : w;
}
