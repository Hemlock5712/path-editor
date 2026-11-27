package frc.team5712.path.wpilib;

import frc.team5712.path.follow.TagFilter;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform2d;

/**
 * A goal measured from an AprilTag itself, for aligning on a field element that isn't quite where the field map
 * says ({@link TagFilter} with WPILib's types). Robot code keeps the camera: it reads the frames and hands each
 * sighting of the tag over. This filters them and gives the pose to drive to.
 *
 * <pre>
 * TagGoal reefTag = new TagGoal(new Transform2d(0.45, 0.0, Rotation2d.k180deg));   // 45 cm out, facing the tag
 *
 * // every loop, for each new camera frame that sees the tag:
 * reefTag.add(tagInRobotSpace, drivetrain.getOdometryPoseAt(captureTime), now);
 *
 * Command align = otf.toTarget(drivetrain, () -> mapGoal)                 // the field map's spot, to set off to
 *     .withMovingGoal(() -> reefTag.fieldGoal(drivetrain.getOdometryPose(), drivetrain.getPose()))
 *     .finishWhen(reefTag::hasLock);                                      // not done until the tag was really seen
 * </pre>
 *
 * Call {@link #reset} when an approach starts (e.g. from the button that starts the command).
 */
public final class TagGoal {
  private final TagFilter filter = new TagFilter();
  private final Transform2d tagToGoal;

  /**
   * @param tagToGoal where to stop, in the tag's frame: +X out of the tag's face, +Y to the tag's left, and a
   *     rotation of 180° facing the tag
   */
  public TagGoal(Transform2d tagToGoal) {
    this.tagToGoal = tagToGoal;
  }

  /** Forgets the estimate. Call when a new approach starts. */
  public void reset() {
    filter.reset();
  }

  /**
   * Folds one sighting of the tag into the estimate.
   *
   * @param tagInRobot the tag as the camera saw it, in the robot's frame, with the camera's own correction
   *     already applied
   * @param odometryAtCapture the robot's wheel-odometry pose when the picture was taken
   * @param timeSeconds now
   * @return false if the sighting was skipped (too far away, or a jump from the estimate)
   */
  public boolean add(Pose2d tagInRobot, Pose2d odometryAtCapture, double timeSeconds) {
    return filter.add(Conversions.toPose(tagInRobot), Conversions.toPose(odometryAtCapture), timeSeconds);
  }

  /** True once a few sightings agree on where the tag is. */
  public boolean hasLock() {
    return filter.hasLock();
  }

  /** Seconds since a sighting last moved the estimate (infinite if never). */
  public double secondsSinceSeen(double timeSeconds) {
    return filter.secondsSinceSeen(timeSeconds);
  }

  /** The tag's pose in the wheel-odometry frame (empty before the first sighting), e.g. to log. */
  public Optional<Pose2d> tagPose() {
    return filter.tagPose().map(Conversions::toPose2d);
  }

  /**
   * The pose to drive to, in the field frame: the goal measured from where the robot is now in wheel odometry,
   * placed on its field pose, so a vision correction moves the robot and the goal together and cancels. Empty
   * until {@link #hasLock()}, so a command bound to it keeps its first goal until then.
   *
   * @param odometryNow the robot's wheel-odometry pose now
   * @param fieldNow the robot's field pose now
   */
  public Optional<Pose2d> fieldGoal(Pose2d odometryNow, Pose2d fieldNow) {
    return filter.fieldGoal(
        Conversions.toPose(new Pose2d(tagToGoal.getTranslation(), tagToGoal.getRotation())),
        Conversions.toPose(odometryNow), Conversions.toPose(fieldNow)).map(Conversions::toPose2d);
  }
}
