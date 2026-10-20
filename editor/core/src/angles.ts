// Angle helpers. Port of lib's Angles.java: Math.atan2/sin/cos in V8 are fdlibm ports, the same
// code as Java's StrictMath, so both languages get the same bits.

const TWO_PI = 2.0 * Math.PI;

/** Wraps an angle to (-π, π]. Plain floor arithmetic, so both languages round the same way. */
export function wrap(radians: number): number {
  const r = radians - TWO_PI * Math.floor((radians + Math.PI) / TWO_PI);
  return r <= -Math.PI ? r + TWO_PI : r;
}

/** The minimum-jerk blend 10τ³ − 15τ⁴ + 6τ⁵ on [0, 1]: zero slope and zero bend at both ends. */
export function smoothstep5(tau: number): number {
  return tau * tau * tau * (10.0 + tau * (-15.0 + tau * 6.0));
}

/** d/dτ of smoothstep5: 30τ²(1 − τ)². */
export function smoothstep5Slope(tau: number): number {
  const k = tau * (1.0 - tau);
  return 30.0 * k * k;
}

/** d²/dτ² of smoothstep5: 60τ(1 − τ)(1 − 2τ). */
export function smoothstep5Bend(tau: number): number {
  return 60.0 * tau * (1.0 - tau) * (1.0 - 2.0 * tau);
}
