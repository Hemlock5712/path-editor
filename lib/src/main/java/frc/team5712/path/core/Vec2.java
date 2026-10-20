package frc.team5712.path.core;

/** A point or vector on the field, in metres. */
public record Vec2(double x, double y) {
  public static final Vec2 ZERO = new Vec2(0.0, 0.0);

  public Vec2 plus(Vec2 o) {
    return new Vec2(x + o.x, y + o.y);
  }

  public Vec2 minus(Vec2 o) {
    return new Vec2(x - o.x, y - o.y);
  }

  public Vec2 times(double k) {
    return new Vec2(x * k, y * k);
  }

  public double dot(Vec2 o) {
    return x * o.x + y * o.y;
  }

  /** The z part of the 3D cross product: positive when {@code o} is to the left of this vector. */
  public double cross(Vec2 o) {
    return x * o.y - y * o.x;
  }

  /** Length. Written as sqrt(x² + y²), not hypot, so the TypeScript port rounds the same way. */
  public double norm() {
    return Math.sqrt(x * x + y * y);
  }

  /** This vector turned 90° to the left. */
  public Vec2 left() {
    return new Vec2(-y, x);
  }

  public static Vec2 fromAngle(double radians) {
    return new Vec2(Angles.cos(radians), Angles.sin(radians));
  }
}
