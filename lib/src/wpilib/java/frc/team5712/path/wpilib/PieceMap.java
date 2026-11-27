package frc.team5712.path.wpilib;

import frc.team5712.path.core.Vec2;
import frc.team5712.path.follow.PieceFilter;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Where the game pieces on the floor are, from a camera's sightings of them ({@link PieceFilter} with WPILib's
 * types). Robot code keeps the camera: it reads each photo and turns every detection into a spot on the floor in
 * front of the robot. This remembers which piece is which, weighs each sighting by how good it is, ignores a
 * detection seen only once, and gives the pieces back as field positions.
 *
 * <pre>
 * PieceMap pieces = new PieceMap();
 *
 * // for every new photo (an empty list too, when it shows nothing):
 * pieces.add(piecesInRobot, cameraOnRobot, lensHeight - pieceHeight, drivetrain.getOdometryPoseAt(captureTime), now);
 *
 * // for the intake assist:
 * pieces.fieldPieces(drivetrain.getOdometryPose(), drivetrain.getPose(), now);
 * </pre>
 */
public final class PieceMap {
  private final PieceFilter filter;

  /** For a camera whose angles are good to about 0.3°. */
  public PieceMap() {
    this.filter = new PieceFilter();
  }

  /** @param angleNoiseRadians how good the camera's angles are, 1 sigma (measure it: a still piece's spread) */
  public PieceMap(double angleNoiseRadians) {
    this.filter = new PieceFilter(angleNoiseRadians);
  }

  /** Forgets every piece. */
  public void reset() {
    filter.reset();
  }

  /**
   * Folds one photo into the map.
   *
   * @param piecesInRobot every piece the photo shows, on the floor in the robot's frame
   * @param cameraInRobot where the lens is on the robot
   * @param dropMeters how far the lens is above the pieces' centres
   * @param odometryAtCapture the robot's wheel-odometry pose when the photo was taken
   * @param timeSeconds now
   */
  public void add(List<Translation2d> piecesInRobot, Translation2d cameraInRobot, double dropMeters,
      Pose2d odometryAtCapture, double timeSeconds) {
    List<Vec2> seen = new ArrayList<>(piecesInRobot.size());
    for (Translation2d p : piecesInRobot) {
      seen.add(new Vec2(p.getX(), p.getY()));
    }
    filter.add(seen, new Vec2(cameraInRobot.getX(), cameraInRobot.getY()), dropMeters,
        Conversions.toPose(odometryAtCapture), timeSeconds);
  }

  /**
   * The pieces on the field: each measured from where the robot is now in wheel odometry, then placed on its
   * field pose, so a vision correction moves the robot and the pieces together.
   */
  public List<Translation2d> fieldPieces(Pose2d odometryNow, Pose2d fieldNow, double timeSeconds) {
    List<Translation2d> out = new ArrayList<>();
    for (Vec2 p : filter.fieldPieces(Conversions.toPose(odometryNow), Conversions.toPose(fieldNow), timeSeconds)) {
      out.add(new Translation2d(p.x(), p.y()));
    }
    return out;
  }
}
