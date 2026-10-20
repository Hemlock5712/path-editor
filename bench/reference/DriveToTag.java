package frc.robot.commands;

import frc.robot.subsystems.DriveMechanism;
import frc.robot.subsystems.vision.ScoringSpot;
import frc.robot.subsystems.vision.TagTracker;
import frc.robot.subsystems.vision.Vision;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.telemetry.Telemetry;

/**
 * Drive To Pose with a goal that moves. Far away: drive to where the field map says the spot is. Once the cameras
 * lock on: drive to the spot measured from the tag itself. The wheel-only pose carries the last bit if the tag
 * drops out, so it doesn't need to see the tag to finish. It finishes just like Drive To Pose; if the tag is never
 * seen, that means stopped on the map's spot without a lock, so it never waits forever (check hasLock() after).
 */
public class DriveToTag extends DriveToPose {
  private static final Field FIELD = Field.loadField(Fields.DEFAULT_FIELD);

  private final DriveMechanism drivetrain;
  private final ScoringSpot spot;
  private final TagTracker tracker;
  private Pose2d mapGoal;

  public DriveToTag(DriveMechanism drivetrain, Vision vision, ScoringSpot spot) {
    super(drivetrain, Pose2d.ZERO); // the goal comes from goal() below
    this.drivetrain = drivetrain;
    this.spot = spot;
    this.tracker = new TagTracker(drivetrain, vision, spot);
  }

  @Override
  protected void initialize() {
    tracker.reset();
    // Far away: the field map's tag, plus the recorded spot. No map entry: hold still until we lock on.
    mapGoal = FIELD.getTagPose(spot.tagId())
        .map(tag -> tag.toPose2d().transformBy(spot.tagToSpot()))
        .orElse(drivetrain.getPose());
    super.initialize();
  }

  @Override
  protected void execute() {
    tracker.update();
    super.execute();
    Telemetry.getTable("DriveToTag").log("Goal", goal()); // watch it hop in AdvantageScope
    Telemetry.getTable("DriveToTag").log("Locked", tracker.hasLock());
  }

  /** Question one, every loop: the tag's spot once we're locked on, the map's spot until then. */
  @Override
  protected Pose2d goal() {
    return tracker.getFieldGoal().orElse(mapGoal);
  }

  /** True if we locked on to the tag, false if we stopped on the map's spot because the tag was never seen. */
  public boolean hasLock() {
    return tracker.hasLock();
  }
}
