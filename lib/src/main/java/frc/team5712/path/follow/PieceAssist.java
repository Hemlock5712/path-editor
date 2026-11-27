package frc.team5712.path.follow;

import frc.team5712.path.core.Angles;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Vec2;
import java.util.List;
import java.util.Optional;

/**
 * Helps the driver line the intake up on a game piece, without taking over: it keeps the driver's speed and
 * turns their direction a little, to point at the piece. Done well the driver doesn't notice it; they just
 * always hit the intake. Every loop:
 *
 * <ol>
 *   <li>Which piece? Only one the driver is already heading for: ahead of the intake and within {@link
 *       #MAX_ANGLE_RADIANS} of where the stick points. It never picks a different piece for them.
 *   <li>How far is it <b>ahead</b> of the intake along the way the driver is pushing, and how far to the
 *       <b>side</b>?
 *   <li>Slide sideways at {@code side ÷ ahead} of the driver's speed. A piece 25 cm to the left and 2 m ahead:
 *       slide left at an eighth of the speed. That is exactly the slide that carries the intake onto the piece
 *       as it arrives, and it is the smallest change of direction that does. ({@link #EAGERNESS} makes it a
 *       little more, to be lined up slightly early.)
 *   <li>Keep it a nudge: never more than {@link #ASSIST_RATIO} of the driver's speed. Let go of the stick and
 *       the help stops too.
 * </ol>
 *
 * <p>It needs the piece's direction far more than its distance, and direction is what a camera is good at: if
 * the piece is really 20 % further off than the camera thinks, the slide starts 20 % too strong and corrects
 * itself as the robot closes in.
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
  /**
   * How much more than "just in time" the slide is: at 1 the intake comes onto the piece only as it arrives; at
   * 1.5 two thirds of the gap is gone by halfway.
   */
  public static final double EAGERNESS = 1.5;
  /** Closer than this the slide stops growing: side ÷ ahead would run away as ahead goes to nothing. */
  public static final double CLOSE_METERS = 0.20;

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

  /**
   * The sideways velocity to add to the driver's, to bring the intake onto {@code piece} as it arrives: {@link
   * #EAGERNESS} × side ÷ ahead of the driver's speed, at right angles to the way they are pushing, and at most
   * {@link #ASSIST_RATIO} of it.
   *
   * @param driver the driver's stick velocity (field directions)
   * @param intake where the intake is on the field
   * @param piece the piece chosen ({@link #choose})
   */
  public static FieldSpeeds nudge(FieldSpeeds driver, Vec2 intake, Vec2 piece) {
    return nudge(driver, intake, piece, EAGERNESS);
  }

  /** {@link #nudge(FieldSpeeds, Vec2, Vec2)} with another eagerness, for tuning. */
  public static FieldSpeeds nudge(FieldSpeeds driver, Vec2 intake, Vec2 piece, double eagerness) {
    double speed = Math.hypot(driver.vxMetersPerSecond(), driver.vyMetersPerSecond());
    if (speed < 1e-9) {
      return new FieldSpeeds(0.0, 0.0, 0.0);
    }
    // the way the driver is pushing, and the direction to its left
    double pushX = driver.vxMetersPerSecond() / speed;
    double pushY = driver.vyMetersPerSecond() / speed;
    double toPieceX = piece.x() - intake.x();
    double toPieceY = piece.y() - intake.y();
    double ahead = toPieceX * pushX + toPieceY * pushY;
    double side = -toPieceX * pushY + toPieceY * pushX; // + = the piece is to the left of the push
    double slide = eagerness * side / Math.max(ahead, CLOSE_METERS) * speed;
    slide = Math.max(-ASSIST_RATIO * speed, Math.min(ASSIST_RATIO * speed, slide));
    return new FieldSpeeds(-pushY * slide, pushX * slide, 0.0);
  }
}
