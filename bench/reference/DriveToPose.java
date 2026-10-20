package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.DriveMechanism;
import frc.robot.utils.AccelerationLimiter;
import frc.robot.utils.AllianceFlip;
import java.util.Set;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.telemetry.Telemetry;

/** Drive to a field pose. Every loop: where is the goal, how far, how fast, how much change. */
public class DriveToPose implements Command {
  // How fast (ch08, ch09)
  private static final double MAX_SPEED = 4.0; // m/s, a little under our 4.5 top speed
  private static final double DECEL = 6.0; // m/s^2, below AccelerationLimiter's grip limit
  private static final double REACTION_TIME = 0.050; // s (50 ms), brake this much early
  private static final double EASE_IN_RATE = 6.0; // (m/s) per m of distance left: the ease-in line
  private static final double STOP_DEADBAND = 0.003; // m, ask for zero inside it

  // How fast to turn (ch11): the same ideas, in rotations (WPILib works in radians underneath)
  private static final double MAX_OMEGA = Units.rotationsToRadians(1.0); // 1 rotation per second
  private static final double ANGULAR_DECEL = Units.rotationsToRadians(1.0); // 1 rotation/s^2
  private static final double EASE_IN_RATE_HEADING = 5.0; // (rotations/s) per rotation left
  private static final double HEADING_REACTION_TIME = 0.050; // s (50 ms)
  private static final double HEADING_DEADBAND = Math.toRadians(0.3);

  // When is it done (ch13)
  private static final double SETTLED_SPEED = 0.05; // m/s, measured
  private double positionTolerance = 0.02; // m
  private double headingTolerance = Math.toRadians(2.0); // rad

  private final DriveMechanism drivetrain;
  private final Pose2d blueGoal; // written for the blue alliance, flipped on red
  private Pose2d activeGoal;

  private final SwerveRequest.ApplyFieldVelocity driveRequest =
      new SwerveRequest.ApplyFieldVelocity()
          .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.Velocity);

  private ChassisVelocities commanded = new ChassisVelocities(); // last loop's command
  private double lastTime;
  private double distance = Double.POSITIVE_INFINITY;
  private double headingError = Double.POSITIVE_INFINITY;

  public DriveToPose(DriveMechanism drivetrain, Pose2d blueGoal) {
    this.drivetrain = drivetrain;
    this.blueGoal = blueGoal;
  }

  /** Finish tolerances (defaults: 2 cm, 2 degrees). */
  public DriveToPose withTolerance(double meters, double degrees) {
    positionTolerance = meters;
    headingTolerance = Math.toRadians(degrees);
    return this;
  }

  /** Question 1: where is the goal? We pass it in; asking every loop lets lesson 3 make it move. */
  protected Pose2d goal() {
    return activeGoal;
  }

  /** Question 3: how fast? The smallest of three speeds (ch08, ch09). */
  public static double wantedSpeed(double distance, double speedNow) {
    if (distance < STOP_DEADBAND) {
      return 0.0;
    }
    double brakingDistance = Math.max(0.0, distance - speedNow * REACTION_TIME);
    double stopSpeed = Math.sqrt(2 * DECEL * brakingDistance); // v = sqrt(2 a d)
    double easeInSpeed = EASE_IN_RATE * distance;
    return Math.min(MAX_SPEED, Math.min(stopSpeed, easeInSpeed));
  }

  /** The same three speeds for turning (ch11). Returns a signed spin rate. */
  public static double wantedSpin(double headingError, double spinNow) {
    double angle = Math.abs(headingError);
    if (angle < HEADING_DEADBAND) {
      return 0.0;
    }
    double brakingAngle = Math.max(0.0, angle - Math.abs(spinNow) * HEADING_REACTION_TIME);
    double stopSpin = Math.sqrt(2 * ANGULAR_DECEL * brakingAngle);
    double easeInSpin = EASE_IN_RATE_HEADING * angle;
    return Math.copySign(Math.min(MAX_OMEGA, Math.min(stopSpin, easeInSpin)), headingError);
  }

  @Override
  public String name() {
    return "DriveToPose";
  }

  @Override
  public Set<Mechanism> requirements() {
    return Set.of(drivetrain);
  }

  @Override
  public void run(Coroutine coroutine) {
    initialize();
    execute();
    while (!isFinished()) {
      coroutine.yield();
      execute();
    }
    stop();
  }

  @Override
  public void onCancel() {
    stop();
  }

  /** Start from the robot's measured velocity, once. After this, each step starts from our own last command. */
  protected void initialize() {
    activeGoal = AllianceFlip.apply(blueGoal); // the alliance is known now, not when the auto is built
    commanded = drivetrain.getFieldVelocity();
    lastTime = Utils.getCurrentTimeSeconds();
  }

  /** One loop: the four questions. */
  protected void execute() {
    double now = Utils.getCurrentTimeSeconds();
    double dt = Math.clamp(now - lastTime, 0.001, 0.1); // guards a skipped loop
    lastTime = now;

    // 1 and 2: where is the goal, and how far?
    Pose2d pose = drivetrain.getPose();
    Pose2d goal = goal();
    Translation2d toGoal = goal.getTranslation().minus(pose.getTranslation());
    distance = toGoal.getNorm();
    headingError = goal.getRotation().minus(pose.getRotation()).getRadians(); // -pi..pi

    // 3: how fast? A speed, pointed straight at the goal
    double speed = wantedSpeed(distance, Math.hypot(commanded.vx, commanded.vy));
    double vx = distance > 1e-6 ? toGoal.getX() / distance * speed : 0.0;
    double vy = distance > 1e-6 ? toGoal.getY() / distance * speed : 0.0;
    double omega = wantedSpin(headingError, commanded.omega);

    // 4: how much may the velocity change? Lesson 1's limiter, from last loop's command
    commanded = AccelerationLimiter.limit(commanded, new ChassisVelocities(vx, vy, omega), dt);
    drivetrain.setControl(driveRequest.withVelocity(commanded));

    Telemetry.getTable("DriveToPose").log("Distance", distance);
    Telemetry.getTable("DriveToPose").log("Commanded", commanded);
  }

  /** Done when close enough, facing right, and really stopped (measured), all at once. */
  protected boolean isFinished() {
    ChassisVelocities measured = drivetrain.getFieldVelocity();
    return distance < positionTolerance
        && Math.abs(headingError) < headingTolerance
        && Math.hypot(measured.vx, measured.vy) < SETTLED_SPEED;
  }

  /** Every leg ends by sending zero, so the robot stays put. */
  private void stop() {
    drivetrain.setControl(driveRequest.withVelocity(new ChassisVelocities()));
  }
}
