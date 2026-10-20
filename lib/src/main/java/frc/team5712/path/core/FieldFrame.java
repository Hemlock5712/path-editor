package frc.team5712.path.core;

/**
 * The frame a path is followed in. Paths are authored for blue. On red, nothing is rebuilt: each
 * loop converts the measured pose into the blue frame, follows the blue path, and converts the
 * output speeds back. With the field origin at the centre (WPILib 2027), both flips are linear maps
 * that fix the origin, and each is its own inverse:
 *
 * <ul>
 *   <li>{@link #RED_HALF_TURN}: rotationally symmetric fields. (x, y) → (−x, −y), θ → θ + π,
 *       (vx, vy) → (−vx, −vy), ω unchanged.
 *   <li>{@link #RED_MIRROR}: mirror-symmetric fields, mirrored across x = 0. x → −x, θ → π − θ,
 *       vx → −vx, ω → −ω.
 * </ul>
 *
 * Neither changes arc length, |κ|, the heading profile's |dθ/ds| or the velocity profile, so the
 * blue plan is the red plan.
 */
public enum FieldFrame {
  BLUE,
  RED_HALF_TURN,
  RED_MIRROR;

  /** A point from this frame into blue (also blue into this frame: the map is its own inverse). */
  public Vec2 toBlue(Vec2 p) {
    return switch (this) {
      case BLUE -> p;
      case RED_HALF_TURN -> new Vec2(-p.x(), -p.y());
      case RED_MIRROR -> new Vec2(-p.x(), p.y());
    };
  }

  public Vec2 fromBlue(Vec2 p) {
    return toBlue(p);
  }

  /** A heading from this frame into blue, wrapped to (-π, π]. */
  public double headingToBlue(double radians) {
    return switch (this) {
      case BLUE -> radians;
      case RED_HALF_TURN -> Angles.wrap(radians + Math.PI);
      case RED_MIRROR -> Angles.wrap(Math.PI - radians);
    };
  }

  public double headingFromBlue(double radians) {
    return headingToBlue(radians);
  }

  public Pose toBlue(Pose pose) {
    return switch (this) {
      case BLUE -> pose;
      case RED_HALF_TURN ->
          new Pose(-pose.xMeters(), -pose.yMeters(), headingToBlue(pose.headingRadians()));
      case RED_MIRROR ->
          new Pose(-pose.xMeters(), pose.yMeters(), headingToBlue(pose.headingRadians()));
    };
  }

  public Pose fromBlue(Pose pose) {
    return toBlue(pose);
  }

  public FieldSpeeds toBlue(FieldSpeeds v) {
    return switch (this) {
      case BLUE -> v;
      case RED_HALF_TURN ->
          new FieldSpeeds(-v.vxMetersPerSecond(), -v.vyMetersPerSecond(), v.omegaRadPerSec());
      case RED_MIRROR ->
          new FieldSpeeds(-v.vxMetersPerSecond(), v.vyMetersPerSecond(), -v.omegaRadPerSec());
    };
  }

  public FieldSpeeds fromBlue(FieldSpeeds v) {
    return toBlue(v);
  }

  /** True when the map reverses handedness (left turns become right turns, κ changes sign). */
  public boolean mirrors() {
    return this == RED_MIRROR;
  }
}
