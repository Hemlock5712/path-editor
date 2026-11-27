package frc.team5712.path.follow;

import frc.team5712.path.core.Angles;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Vec2;
import java.util.List;
import java.util.Optional;

/**
 * Helps the driver line the intake up on a game piece, without taking over. The driver still drives and turns;
 * the robot only adds a small push sideways. Every loop:
 *
 * <ol>
 *   <li>Which piece? Only one the driver is already heading for: ahead of the intake and within {@link
 *       #MAX_ANGLE_RADIANS} of where the stick points. It never picks a different piece for them.
 *   <li>Draw a {@link Rail} through that piece, pointing the way the driver is pushing.
 *   <li>Ride that rail from the <b>intake</b>, not the robot's centre: the rail's sideways pull lines the intake
 *       up, the driver's speed carries it along.
 *   <li>Keep it a nudge: the sideways pull is never more than {@link #ASSIST_RATIO} of the driver's speed. Let
 *       go of the stick and the help stops too.
 * </ol>
 *
 * <p>The ideas come from two teams' talks: Orbit (1690) pulls only toward the piece the driver is already
 * heading for, sideways to their direction; OP Robotics (2056) scales the push by how fast the robot is going.
 */
public final class PieceAssist {
  /** Pieces more than this far off the stick's direction (seen from the intake) are ignored. */
  public static final double MAX_ANGLE_RADIANS = Math.toRadians(30);
  /** Or farther than this from the intake. */
  public static final double MAX_DISTANCE_METERS = 3.0;
  /** Below this stick speed there is no help: the driver is lining up by hand. */
  public static final double MIN_DRIVER_METERS_PER_SECOND = 0.3;
  /** The sideways pull is at most this share of the driver's speed. */
  public static final double ASSIST_RATIO = 0.6;

  private PieceAssist() {}

  /**
   * The piece the driver is heading for, if any: ahead of the intake, within {@link #MAX_ANGLE_RADIANS} of the
   * stick, the most in line wins.
   *
   * @param intake where the intake is on the field
   * @param driver the driver's stick velocity (field directions)
   */
  public static Optional<Vec2> choose(List<Vec2> pieces, Vec2 intake, FieldSpeeds driver) {
    double speed = Math.hypot(driver.vxMetersPerSecond(), driver.vyMetersPerSecond());
    if (speed < MIN_DRIVER_METERS_PER_SECOND) {
      return Optional.empty();
    }
    double pushing = Math.atan2(driver.vyMetersPerSecond(), driver.vxMetersPerSecond());
    Vec2 best = null;
    double bestAngle = MAX_ANGLE_RADIANS;
    for (Vec2 piece : pieces) {
      double dx = piece.x() - intake.x();
      double dy = piece.y() - intake.y();
      double distance = Math.hypot(dx, dy);
      if (distance < 1e-6 || distance > MAX_DISTANCE_METERS) {
        continue;
      }
      double angle = Math.abs(Angles.wrap(Math.atan2(dy, dx) - pushing));
      if (angle < bestAngle) { // also means it's ahead: behind is 90° or more off
        bestAngle = angle;
        best = piece;
      }
    }
    return Optional.ofNullable(best);
  }

  /** A piece within this of the one chosen last loop is that piece again. */
  public static final double SAME_PIECE_METERS = 0.25;
  /** Another piece takes over only when it is this much more in line with the stick than the one being chased. */
  public static final double SWITCH_RADIANS = Math.toRadians(10);

  /**
   * {@link #choose(List, Vec2, FieldSpeeds)}, staying with the piece chosen last loop: while that piece is still
   * a fair choice (ahead, within range and {@link #MAX_ANGLE_RADIANS}), another one takes over only if it is
   * {@link #SWITCH_RADIANS} more in line with the stick. Two pieces about equally in line would otherwise swap
   * with every wobble of the camera, and the robot would weave between them and take neither.
   *
   * @param previous the piece chosen last loop, if any
   */
  public static Optional<Vec2> choose(List<Vec2> pieces, Vec2 intake, FieldSpeeds driver, Optional<Vec2> previous) {
    Optional<Vec2> best = choose(pieces, intake, driver);
    if (previous.isEmpty() || best.isEmpty()) {
      return best;
    }
    Vec2 kept = null;
    double nearest = SAME_PIECE_METERS;
    for (Vec2 piece : pieces) {
      double d = Math.hypot(piece.x() - previous.get().x(), piece.y() - previous.get().y());
      if (d < nearest) {
        nearest = d;
        kept = piece;
      }
    }
    if (kept == null || choose(List.of(kept), intake, driver).isEmpty()) {
      return best; // the old piece is gone, behind or too far off the stick
    }
    double pushing = Math.atan2(driver.vyMetersPerSecond(), driver.vxMetersPerSecond());
    return offStick(best.get(), intake, pushing) < offStick(kept, intake, pushing) - SWITCH_RADIANS
        ? best : Optional.of(kept);
  }

  private static double offStick(Vec2 piece, Vec2 intake, double pushing) {
    return Math.abs(Angles.wrap(Math.atan2(piece.y() - intake.y(), piece.x() - intake.x()) - pushing));
  }

  /** The rail to ride to {@code piece}: through it, pointing the way the driver is pushing, heading free. */
  public static Rail railThrough(Vec2 piece, FieldSpeeds driver) {
    return Rail.through(piece.x(), piece.y(), Math.atan2(driver.vyMetersPerSecond(), driver.vxMetersPerSecond()));
  }

  /** The most the assist may pull sideways for this stick velocity, m/s. */
  public static double maxCrossSpeed(FieldSpeeds driver) {
    return ASSIST_RATIO * Math.hypot(driver.vxMetersPerSecond(), driver.vyMetersPerSecond());
  }
}
