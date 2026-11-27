package frc.team5712.path.follow;

import frc.team5712.path.core.Pose;
import frc.team5712.path.core.Vec2;
import java.util.ArrayList;
import java.util.List;

/**
 * Where the game pieces on the floor are, worked out from a camera's sightings of them. It knows nothing about
 * cameras: robot code reads the camera, turns each detection into a spot on the floor in front of the robot, and
 * hands over every photo; this keeps one steady estimate per piece and gives them back as field positions.
 *
 * <p>Like {@link TagFilter}, the estimates are kept in the wheel-odometry frame and each photo is placed using
 * where odometry said the robot was when it was taken, so a piece stays put while the robot drives.
 *
 * <p>What it adds to using the newest photo as it comes:
 *
 * <ul>
 *   <li><b>It knows which piece is which.</b> A detection near a piece it already has is that piece, seen again;
 *       so a piece keeps its place when another drops out of a photo, and whoever picks a piece to drive at can
 *       stay with it.
 *   <li><b>It weighs each sighting by how good it is.</b> A camera measures angles. Looking down at the floor
 *       from a low height, a small error in the angle is a large error in how far away the piece is (at 2 m, ten
 *       times larger than the error to the side), and both grow with distance. Each sighting moves the estimate
 *       a lot to the side and a little in range when it is far, and a lot both ways when it is near.
 *   <li><b>One stray detection is not a piece.</b> A piece counts once it has been seen twice.
 * </ul>
 */
public final class PieceFilter {
  /** A piece not seen for this long is forgotten. */
  public static final double MEMORY_SECONDS = 0.5;
  /** A piece counts once it has been seen this many times. */
  public static final int SIGHTINGS_TO_COUNT = 2;
  /** A detection further than this to the side of every piece (and further than the noise allows) is a new piece. */
  public static final double SAME_PIECE_METERS = 0.15;
  /** ... or further than this nearer or further along the line of sight, where the camera is far less sure. */
  public static final double SAME_RANGE_METERS = 0.5;
  // How far a piece may seem to move between photos (odometry drift, a nudged piece): keeps the filter listening
  private static final double DRIFT_METERS = 0.004;

  private final double angleNoiseRadians;
  private final List<Track> tracks = new ArrayList<>();

  /** One piece: where it is (odometry frame), how sure (a 2×2 covariance), and how often it has been seen. */
  private static final class Track {
    double x;
    double y;
    double pxx;
    double pxy;
    double pyy;
    int sightings;
    double lastSeen;
    boolean claimed; // already matched to a detection in the photo being added
  }

  /** A filter for a camera whose angles are good to about 0.3° (1 sigma), typical of a detector's box centre. */
  public PieceFilter() {
    this(Math.toRadians(0.3));
  }

  /** @param angleNoiseRadians how good the camera's angles are, 1 sigma */
  public PieceFilter(double angleNoiseRadians) {
    this.angleNoiseRadians = angleNoiseRadians;
  }

  /** Forgets every piece. */
  public void reset() {
    tracks.clear();
  }

  /**
   * Folds one photo into the estimates.
   *
   * @param piecesFromRobot every piece the photo shows, on the floor in the robot's frame (x forward, y left)
   * @param cameraFromRobot where the lens is on the robot (x forward, y left)
   * @param dropMeters how far the lens is above the pieces' centres
   * @param odometryAtCapture the robot's wheel-odometry pose when the photo was taken
   * @param timeSeconds now
   */
  public void add(List<Vec2> piecesFromRobot, Vec2 cameraFromRobot, double dropMeters, Pose odometryAtCapture,
      double timeSeconds) {
    forget(timeSeconds);
    for (Track t : tracks) {
      t.claimed = false;
      t.pxx += DRIFT_METERS * DRIFT_METERS;
      t.pyy += DRIFT_METERS * DRIFT_METERS;
    }
    double cos = Math.cos(odometryAtCapture.headingRadians());
    double sin = Math.sin(odometryAtCapture.headingRadians());
    for (Vec2 piece : piecesFromRobot) {
      // the line of sight along the floor, from the lens to the piece
      double sightX = piece.x() - cameraFromRobot.x();
      double sightY = piece.y() - cameraFromRobot.y();
      double range = Math.hypot(sightX, sightY);
      if (!(range > 1e-6) || !(dropMeters > 0.0)) {
        continue;
      }
      // An angle error e moves the spot e·range to the side, and e·(range² + drop²)/drop along the line of
      // sight (the ray meets the floor at a shallow angle, so a little tilt is a long way).
      double side = angleNoiseRadians * range;
      double along = angleNoiseRadians * (range * range + dropMeters * dropMeters) / dropMeters;
      // that noise as a covariance in the odometry frame: along the line of sight, and across it
      double ux = (cos * sightX - sin * sightY) / range;
      double uy = (sin * sightX + cos * sightY) / range;
      double rxx = along * along * ux * ux + side * side * uy * uy;
      double rxy = (along * along - side * side) * ux * uy;
      double ryy = along * along * uy * uy + side * side * ux * ux;
      double x = odometryAtCapture.xMeters() + cos * piece.x() - sin * piece.y();
      double y = odometryAtCapture.yMeters() + sin * piece.x() + cos * piece.y();

      Track match = null;
      double best = Double.POSITIVE_INFINITY;
      for (Track t : tracks) {
        if (t.claimed) {
          continue;
        }
        // how far off, counted in the noise this sighting and the estimate have together: range errors are
        // expected to be large, sideways ones are not
        double dx = x - t.x;
        double dy = y - t.y;
        double sxx = t.pxx + rxx;
        double sxy = t.pxy + rxy;
        double syy = t.pyy + ryy;
        double det = sxx * syy - sxy * sxy;
        double sigmas2 = (syy * dx * dx - 2.0 * sxy * dx * dy + sxx * dy * dy) / det;
        // The same piece if it is on the same line of sight and not wildly nearer or further. To the side the
        // camera is sharp, so the noise decides. In range it is not only noisy but often wrong the same way
        // photo after photo (a piece height or camera tilt slightly off), which no amount of averaging shows:
        // so a fixed, generous allowance, and the nearer sighting then corrects the estimate.
        double across = Math.abs(-dx * uy + dy * ux);
        double alongSight = Math.abs(dx * ux + dy * uy);
        double acrossNoise = Math.sqrt(Math.max(0.0, sxx * uy * uy - 2.0 * sxy * ux * uy + syy * ux * ux));
        boolean close = across < Math.max(SAME_PIECE_METERS, 3.0 * acrossNoise) && alongSight < SAME_RANGE_METERS;
        if (close && sigmas2 < best) {
          best = sigmas2;
          match = t;
        }
      }
      if (match == null) {
        Track t = new Track();
        t.x = x;
        t.y = y;
        t.pxx = rxx;
        t.pxy = rxy;
        t.pyy = ryy;
        t.sightings = 1;
        t.lastSeen = timeSeconds;
        t.claimed = true;
        tracks.add(t);
        continue;
      }
      // The Kalman update: gain = P·(P + R)⁻¹, each 2×2
      double sxx = match.pxx + rxx;
      double sxy = match.pxy + rxy;
      double syy = match.pyy + ryy;
      double det = sxx * syy - sxy * sxy;
      double kxx = (match.pxx * syy - match.pxy * sxy) / det;
      double kxy = (match.pxy * sxx - match.pxx * sxy) / det;
      double kyx = (match.pxy * syy - match.pyy * sxy) / det;
      double kyy = (match.pyy * sxx - match.pxy * sxy) / det;
      double dx = x - match.x;
      double dy = y - match.y;
      match.x += kxx * dx + kxy * dy;
      match.y += kyx * dx + kyy * dy;
      double pxx = (1.0 - kxx) * match.pxx - kxy * match.pxy;
      double pxy = (1.0 - kxx) * match.pxy - kxy * match.pyy;
      double pyy = -kyx * match.pxy + (1.0 - kyy) * match.pyy;
      match.pxx = pxx;
      match.pxy = pxy;
      match.pyy = pyy;
      match.sightings++;
      match.lastSeen = timeSeconds;
      match.claimed = true;
    }
  }

  private void forget(double timeSeconds) {
    tracks.removeIf(t -> timeSeconds - t.lastSeen > MEMORY_SECONDS);
  }

  /** The pieces that count, in the wheel-odometry frame. */
  public List<Vec2> pieces(double timeSeconds) {
    forget(timeSeconds);
    List<Vec2> out = new ArrayList<>();
    for (Track t : tracks) {
      if (t.sightings >= SIGHTINGS_TO_COUNT) {
        out.add(new Vec2(t.x, t.y));
      }
    }
    return out;
  }

  /**
   * The pieces that count, on the field: each measured from where the robot is now in wheel odometry, then
   * placed on the robot's field pose, so a vision correction moves the robot and the pieces together.
   */
  public List<Vec2> fieldPieces(Pose odometryNow, Pose fieldNow, double timeSeconds) {
    List<Vec2> out = new ArrayList<>();
    for (Vec2 p : pieces(timeSeconds)) {
      Pose fromRobot = TagFilter.minus(new Pose(p.x(), p.y(), 0.0), odometryNow);
      Pose onField = TagFilter.plus(fieldNow, new Pose(fromRobot.xMeters(), fromRobot.yMeters(), 0.0));
      out.add(new Vec2(onField.xMeters(), onField.yMeters()));
    }
    return out;
  }
}
