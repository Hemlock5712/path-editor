package frc.team5712.path.follow;

import frc.team5712.path.core.Angles;
import frc.team5712.path.core.Pose;
import java.util.Optional;

/**
 * Where one AprilTag really is, worked out from the camera's sightings of it: the goal for aligning on a field
 * element that isn't quite where the field map says. It knows nothing about cameras. Robot code reads the camera
 * and hands over each sighting; this filters them into one steady estimate and turns it into a pose to drive to.
 *
 * <p>The estimate is kept in the wheel-odometry frame (the pose the wheels and gyro alone give, never corrected
 * by vision). Each sighting is placed using where odometry said the robot was when the picture was taken, so the
 * robot's own motion since doesn't smear it, and when the tag leaves view odometry carries the estimate on.
 *
 * <p>The filter is a small Kalman filter on each of x, y and yaw: a far sighting is noisy and moves the estimate
 * a little, a near one is sharp and moves it more. A sighting a long way from the estimate is a wrong tag or a
 * flipped solve and is ignored, unless three in a row agree; then the old estimate was the wrong one.
 *
 * <pre>
 * tag.reset();                                             // when an approach starts
 * tag.add(tagFromRobot, odometryPoseAtCapture, now);       // for every new camera frame that sees the tag
 * tag.fieldGoal(tagToGoal, odometryPoseNow, fieldPoseNow); // the pose to drive to (empty until locked)
 * </pre>
 */
public final class TagFilter {
  /** Sightings from further than this are skipped. */
  public static final double MAX_DISTANCE_METERS = 3.5;
  /** A sighting this far from the estimate is a wrong tag or a flipped solve ... */
  public static final double JUMP_METERS = 0.25;
  /** ... unless this many in a row agree. */
  public static final int JUMPS_TO_RESTART = 3;
  /** Locked once this many sightings agree. */
  public static final int SIGHTINGS_TO_LOCK = 3;

  // One sighting's noise (1 sigma), growing with distance d: xy = floor + k·d², yaw = floor + k·d
  private static final double XY_NOISE_FLOOR = 0.005; // m
  private static final double XY_NOISE_PER_M2 = 0.01; // m per m²
  private static final double YAW_NOISE_FLOOR = Math.toRadians(0.5);
  private static final double YAW_NOISE_PER_M = Math.toRadians(1.5);
  // How far the tag may seem to move between sightings (odometry drift): keeps the filter listening
  private static final double XY_DRIFT = 0.002; // m
  private static final double YAW_DRIFT = Math.toRadians(0.1);

  private Pose tag; // odometry frame; null until the first sighting
  private final double[] variance = new double[3]; // of the estimate's x, y and yaw
  private int sightings;
  private int jumpsInARow;
  private double lastSeenSeconds = Double.NEGATIVE_INFINITY;

  /** Forgets the estimate. Call when a new approach starts. */
  public void reset() {
    tag = null;
    sightings = 0;
    jumpsInARow = 0;
    lastSeenSeconds = Double.NEGATIVE_INFINITY;
  }

  /**
   * Folds one sighting into the estimate.
   *
   * @param tagFromRobot the tag as the camera saw it, in the robot's frame (x forward, y left; the heading is the
   *     way the tag's face points), with the camera's own correction already applied
   * @param odometryAtCapture the robot's wheel-odometry pose when the picture was taken
   * @param timeSeconds now, on the clock {@link #secondsSinceSeen} is asked with
   * @return false if the sighting was skipped: too far away, or a jump from the estimate
   */
  public boolean add(Pose tagFromRobot, Pose odometryAtCapture, double timeSeconds) {
    double distance = Math.hypot(tagFromRobot.xMeters(), tagFromRobot.yMeters());
    if (!(distance <= MAX_DISTANCE_METERS)) {
      return false;
    }
    Pose seen = plus(odometryAtCapture, tagFromRobot);
    double xyNoise = XY_NOISE_FLOOR + XY_NOISE_PER_M2 * distance * distance;
    double yawNoise = YAW_NOISE_FLOOR + YAW_NOISE_PER_M * distance;
    double[] noise = {xyNoise * xyNoise, xyNoise * xyNoise, yawNoise * yawNoise};
    if (tag == null) {
      restart(seen, noise, timeSeconds);
      return true;
    }
    double dx = seen.xMeters() - tag.xMeters();
    double dy = seen.yMeters() - tag.yMeters();
    double dYaw = Angles.wrap(seen.headingRadians() - tag.headingRadians());
    if (Math.hypot(dx, dy) > JUMP_METERS) {
      if (++jumpsInARow < JUMPS_TO_RESTART) {
        return false;
      }
      restart(seen, noise, timeSeconds); // the old estimate was the wrong one
      return true;
    }
    jumpsInARow = 0;
    double[] drift = {XY_DRIFT * XY_DRIFT, XY_DRIFT * XY_DRIFT, YAW_DRIFT * YAW_DRIFT};
    double[] gain = new double[3];
    for (int axis = 0; axis < 3; axis++) {
      variance[axis] += drift[axis];
      gain[axis] = variance[axis] / (variance[axis] + noise[axis]); // how much to trust this sighting
      variance[axis] *= 1.0 - gain[axis];
    }
    tag = new Pose(tag.xMeters() + gain[0] * dx, tag.yMeters() + gain[1] * dy,
        Angles.wrap(tag.headingRadians() + gain[2] * dYaw));
    sightings++;
    lastSeenSeconds = timeSeconds;
    return true;
  }

  private void restart(Pose seen, double[] noise, double timeSeconds) {
    tag = seen;
    System.arraycopy(noise, 0, variance, 0, 3);
    sightings = 1;
    jumpsInARow = 0;
    lastSeenSeconds = timeSeconds;
  }

  /** True once a few sightings agree on where the tag is. Stays true while odometry carries the estimate. */
  public boolean hasLock() {
    return tag != null && sightings >= SIGHTINGS_TO_LOCK;
  }

  /** The tag's pose in the wheel-odometry frame (empty before the first sighting). */
  public Optional<Pose> tagPose() {
    return Optional.ofNullable(tag);
  }

  /** Seconds since a sighting last moved the estimate (infinite if never). */
  public double secondsSinceSeen(double timeSeconds) {
    return tag == null ? Double.POSITIVE_INFINITY : timeSeconds - lastSeenSeconds;
  }

  /**
   * The spot {@code tagToGoal} away from the tag, as a field pose to drive to. It is measured from where the
   * robot is now in wheel odometry, then placed on the robot's field pose, so when vision corrects the field
   * pose the robot and the goal move together and the correction cancels.
   *
   * @param tagToGoal where to stop, in the tag's frame: x out of the tag's face, y to the tag's left, and a
   *     heading of 180° facing the tag
   * @param odometryNow the robot's wheel-odometry pose now
   * @param fieldNow the robot's field pose now (the one the follower is given)
   * @return empty until {@link #hasLock()}
   */
  public Optional<Pose> fieldGoal(Pose tagToGoal, Pose odometryNow, Pose fieldNow) {
    if (!hasLock()) {
      return Optional.empty();
    }
    Pose goalInOdometry = plus(tag, tagToGoal);
    return Optional.of(plus(fieldNow, minus(goalInOdometry, odometryNow)));
  }

  /** The pose {@code step} away from {@code from}, {@code step} being in {@code from}'s own frame. */
  static Pose plus(Pose from, Pose step) {
    double cos = Math.cos(from.headingRadians());
    double sin = Math.sin(from.headingRadians());
    return new Pose(from.xMeters() + cos * step.xMeters() - sin * step.yMeters(),
        from.yMeters() + sin * step.xMeters() + cos * step.yMeters(),
        Angles.wrap(from.headingRadians() + step.headingRadians()));
  }

  /** {@code pose} as seen from {@code from}: the step that {@link #plus} would take from one to the other. */
  static Pose minus(Pose pose, Pose from) {
    double cos = Math.cos(from.headingRadians());
    double sin = Math.sin(from.headingRadians());
    double dx = pose.xMeters() - from.xMeters();
    double dy = pose.yMeters() - from.yMeters();
    return new Pose(cos * dx + sin * dy, -sin * dx + cos * dy,
        Angles.wrap(pose.headingRadians() - from.headingRadians()));
  }
}
