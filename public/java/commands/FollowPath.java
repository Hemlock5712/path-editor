package frc.robot.commands;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.path.PathData;
import frc.robot.utils.path.ProjectionResult;
import frc.robot.utils.path.RotationSupplier;
import frc.robot.utils.path.SplinePath;
import frc.robot.utils.path.VelocityConstraints;
import frc.robot.utils.path.VelocityProfile;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Distance-based path following command for swerve drive.
 *
 * <p>Follows a spline path using arc-length parameterization with adaptive lookahead and
 * cross-track PD correction. The path parameter tracks the robot's actual position, not a clock —
 * if the robot gets hit or stalls, the path "waits" for the robot.
 *
 * <p>All output is fed through {@link AccelerationLimiter#integrateVelocity} to enforce friction
 * circle, motor torque, and jerk limits.
 */
public class FollowPath extends Command {

  private final CommandSwerveDrivetrain swerve;
  private final SplinePath path;
  private final VelocityProfile velocityProfile;

  // Rotation supplier: returns desired omega (rad/s). Null = no rotation.
  private RotationSupplier rotationSupplier;

  // Maximum fraction of friction budget that rotation can consume (0 to 1)
  private double maxRotationBudgetFraction = 0.30;

  // Lookahead parameters: lookahead = k * speed + min, clamped to [min, max]
  private double lookaheadK = 0.15; // seconds
  private double lookaheadMin = 0.15; // meters
  private double lookaheadMax = 1.0; // meters
  private double lookaheadMaxArcAngle = Math.PI / 6; // ~30 degrees

  // Cross-track PD gains + curvature feedforward
  private double crossTrackKp = 3.0; // m/s per meter error
  private double crossTrackKd = 0.5; // m/s per m/s error rate
  private double curvatureFfGain = 0.1; // seconds — converts v²κ (m/s²) to velocity (m/s)

  // Completion criteria
  private double completionTolerance = 0.05; // meters from path end
  private final double endVelocity;

  /**
   * Maximum arc-length the projection can move per cycle (meters). Derived from physics: at max FRC
   * speed (5 m/s) with a 20ms loop, the robot moves 0.1m per cycle. 0.5m gives 5x safety margin,
   * handling loop overruns up to 100ms at max speed. This window (1.0m total) is too small to span
   * both segments at a path crossing (minimum separation ~1.6m for a 0.5m-radius turn).
   */
  private static final double PROJECTION_MAX_DELTA = 0.5;

  /** Number of poses to sample for the logged path trajectory. */
  private static final int PATH_LOG_SAMPLES = 50;

  // Per-axis speed overrides (null = use path-computed value)
  private DoubleSupplier overrideVx = null;
  private DoubleSupplier overrideVy = null;
  private DoubleSupplier overrideOmega = null;
  private boolean limitOverrideVx = true;
  private boolean limitOverrideVy = true;
  private boolean limitOverrideOmega = true;

  // State tracking between execute cycles (same pattern as DriveToPoint/OrbitDrive)
  private ChassisSpeeds lastCommandedVelocity = new ChassisSpeeds();
  private double lastTime;
  private double lastCrossTrackError;
  private double lastProjectedS;

  private final SwerveRequest.ApplyFieldSpeeds request =
      new SwerveRequest.ApplyFieldSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /**
   * Creates a FollowPath command with default constraints.
   *
   * @param swerve The swerve drivetrain
   * @param path The spline path to follow
   */
  public FollowPath(CommandSwerveDrivetrain swerve, SplinePath path) {
    this(swerve, path, VelocityConstraints.defaults());
  }

  /**
   * Creates a FollowPath command with specified constraints.
   *
   * @param swerve The swerve drivetrain
   * @param path The spline path to follow
   * @param constraints Velocity and acceleration limits
   */
  public FollowPath(
      CommandSwerveDrivetrain swerve, SplinePath path, VelocityConstraints constraints) {
    this(swerve, path, constraints, List.of());
  }

  /**
   * Creates a FollowPath command with specified constraints and constraint zones.
   *
   * @param swerve The swerve drivetrain
   * @param path The spline path to follow
   * @param constraints Velocity and acceleration limits
   * @param constraintZones Per-zone overrides for velocity and acceleration
   */
  public FollowPath(
      CommandSwerveDrivetrain swerve,
      SplinePath path,
      VelocityConstraints constraints,
      List<PathData.ConstraintZone> constraintZones) {
    this.swerve = swerve;
    this.path = path;
    this.velocityProfile = new VelocityProfile(path, constraints, constraintZones);
    this.endVelocity = constraints.getEndVelocity();
    addRequirements(swerve);
  }

  // ---- Builder methods (same pattern as DriveToPoint) ----

  /**
   * Sets adaptive lookahead parameters.
   *
   * @param k Lookahead time gain (seconds): lookahead = k * speed + min
   * @param min Minimum lookahead distance (meters)
   * @param max Maximum lookahead distance (meters)
   * @return This command for chaining
   */
  public FollowPath withLookahead(double k, double min, double max) {
    this.lookaheadK = k;
    this.lookaheadMin = min;
    this.lookaheadMax = max;
    return this;
  }

  /**
   * Sets the maximum arc angle the lookahead can subtend on a curve.
   *
   * <p>On sharp curves, the speed-based lookahead is capped so that {@code lookahead * curvature <=
   * maxArcAngle}. This prevents the lookahead chord from cutting across the arc.
   *
   * @param radians Maximum arc angle in radians (default π/6 ≈ 30°)
   * @return This command for chaining
   */
  public FollowPath withLookaheadMaxArcAngle(double radians) {
    this.lookaheadMaxArcAngle = radians;
    return this;
  }

  /**
   * Sets cross-track error PD gains.
   *
   * @param kp Proportional gain (m/s per meter of cross-track error)
   * @param kd Derivative gain (m/s per m/s of cross-track error rate)
   * @return This command for chaining
   */
  public FollowPath withCrossTrackGains(double kp, double kd) {
    this.crossTrackKp = kp;
    this.crossTrackKd = kd;
    return this;
  }

  /**
   * Sets curvature feedforward gain.
   *
   * <p>Proactively pushes toward the center of curvature with velocity {@code gain * v² * κ},
   * preventing cross-track error from building up on curves.
   *
   * @param gain Feedforward gain in seconds (default 0.1)
   * @return This command for chaining
   */
  public FollowPath withCurvatureFeedforward(double gain) {
    this.curvatureFfGain = gain;
    return this;
  }

  /**
   * Sets completion tolerance (distance from path end to finish).
   *
   * @param meters Tolerance in meters
   * @return This command for chaining
   */
  public FollowPath withCompletionTolerance(double meters) {
    this.completionTolerance = meters;
    return this;
  }

  /**
   * Sets a path-aware rotation supplier.
   *
   * @param supplier Supplies desired angular velocity given robot pose and path context
   * @return This command for chaining
   */
  public FollowPath withRotationSupplier(RotationSupplier supplier) {
    this.rotationSupplier = supplier;
    return this;
  }

  /**
   * Sets an external rotation supplier (backward-compatible with simple DoubleSupplier).
   *
   * @param supplier Supplies desired angular velocity in rad/s
   * @return This command for chaining
   */
  public FollowPath withRotationSupplier(DoubleSupplier supplier) {
    this.rotationSupplier = (pose, s, tangent) -> supplier.getAsDouble();
    return this;
  }

  /**
   * Sets the maximum fraction of friction budget that rotation can consume.
   *
   * <p>When rotation demands exceed this fraction, omega is capped and translation gets the
   * remaining budget. AccelerationLimiter enforces the hard friction circle as a final safety net.
   *
   * @param fraction Fraction of MAX_FRICTION_ACCEL reserved for rotation (0.0 to 1.0, default 0.3)
   * @return This command for chaining
   */
  public FollowPath withMaxRotationBudget(double fraction) {
    this.maxRotationBudgetFraction = fraction;
    return this;
  }

  /**
   * Overrides the field-relative X velocity with a custom supplier. The output still passes
   * through AccelerationLimiter (friction, motor, jerk limits apply).
   *
   * @param supplier Supplies desired field-relative X velocity in m/s
   * @return This command for chaining
   */
  public FollowPath overrideXSpeedWithLimits(DoubleSupplier supplier) {
    this.overrideVx = supplier;
    this.limitOverrideVx = true;
    return this;
  }

  /**
   * Overrides the field-relative Y velocity with a custom supplier. The output still passes
   * through AccelerationLimiter (friction, motor, jerk limits apply).
   *
   * @param supplier Supplies desired field-relative Y velocity in m/s
   * @return This command for chaining
   */
  public FollowPath overrideYSpeedWithLimits(DoubleSupplier supplier) {
    this.overrideVy = supplier;
    this.limitOverrideVy = true;
    return this;
  }

  /**
   * Overrides angular velocity with a custom supplier. The output still passes through
   * AccelerationLimiter (friction, motor, jerk limits apply). If a RotationSupplier is also set via
   * {@link #withRotationSupplier}, this override takes precedence.
   *
   * @param supplier Supplies desired angular velocity in rad/s
   * @return This command for chaining
   */
  public FollowPath overrideRotSpeedWithLimits(DoubleSupplier supplier) {
    this.overrideOmega = supplier;
    this.limitOverrideOmega = true;
    return this;
  }

  /**
   * Overrides the field-relative X velocity with a custom supplier. Bypasses all acceleration
   * limits on this axis — the caller is responsible for not exceeding hardware limits. The
   * unlimited axis does not consume friction budget from the remaining limited axes.
   *
   * @param supplier Supplies desired field-relative X velocity in m/s
   * @return This command for chaining
   */
  public FollowPath overrideXSpeed(DoubleSupplier supplier) {
    this.overrideVx = supplier;
    this.limitOverrideVx = false;
    return this;
  }

  /**
   * Overrides the field-relative Y velocity with a custom supplier. Bypasses all acceleration
   * limits on this axis — the caller is responsible for not exceeding hardware limits. The
   * unlimited axis does not consume friction budget from the remaining limited axes.
   *
   * @param supplier Supplies desired field-relative Y velocity in m/s
   * @return This command for chaining
   */
  public FollowPath overrideYSpeed(DoubleSupplier supplier) {
    this.overrideVy = supplier;
    this.limitOverrideVy = false;
    return this;
  }

  /**
   * Overrides angular velocity with a custom supplier. Bypasses all acceleration limits on this
   * axis — the caller is responsible for not exceeding hardware limits. Rotation budget allocation
   * is skipped entirely (rotation does not reduce translation budget).
   *
   * @param supplier Supplies desired angular velocity in rad/s
   * @return This command for chaining
   */
  public FollowPath overrideRotSpeed(DoubleSupplier supplier) {
    this.overrideOmega = supplier;
    this.limitOverrideOmega = false;
    return this;
  }

  // ---- Command lifecycle ----

  @Override
  public void initialize() {
    // Start from current velocity for smooth transitions (same as OrbitDrive/DriveToPoint)
    lastCommandedVelocity = swerve.getFieldSpeeds();
    lastTime = Utils.getCurrentTimeSeconds();
    lastCrossTrackError = 0;

    // Default: hold the robot's current heading (swerve should not rotate unless told to)
    if (rotationSupplier == null) {
      Rotation2d currentHeading = swerve.getPose().getRotation();
      rotationSupplier = frc.robot.utils.path.RotationSuppliers.holdHeading(currentHeading);
    }

    // Initial projection: find where the robot is on the path
    Pose2d pose = swerve.getPose();
    ProjectionResult proj = path.getClosestPoint(pose.getTranslation());
    lastProjectedS = proj.s();

    // Log the reference path as a Pose2d array (shows as trajectory in AdvantageScope)
    logReferencePath();
  }

  @Override
  public void execute() {
    double currentTime = Utils.getCurrentTimeSeconds();
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    Pose2d robotPose = swerve.getPose();
    Translation2d robotPos = robotPose.getTranslation();

    // Step 1: Project robot onto path — bounded search around last known position.
    // The bounded window prevents jumping to distant segments when hit or at crossings.
    ProjectionResult proj =
        path.getClosestPointInRange(
            robotPos,
            lastProjectedS - PROJECTION_MAX_DELTA,
            lastProjectedS + PROJECTION_MAX_DELTA);
    double sRobot = proj.s();
    double crossTrackError = proj.crossTrackError();
    Translation2d tangent = proj.tangent();

    // Step 2: Adaptive lookahead — further ahead when moving faster
    double currentSpeed =
        Math.hypot(
            lastCommandedVelocity.vxMetersPerSecond, lastCommandedVelocity.vyMetersPerSecond);
    double lookaheadDist =
        MathUtil.clamp(lookaheadK * currentSpeed + lookaheadMin, lookaheadMin, lookaheadMax);

    // Curvature cap: prevent chord from deviating too far from the arc
    double kappa = Math.abs(path.getCurvature(sRobot));
    if (kappa > 1e-6) {
      lookaheadDist = Math.min(lookaheadDist, lookaheadMaxArcAngle / kappa);
    }
    lookaheadDist = Math.max(lookaheadDist, lookaheadMin);

    double sTarget = Math.min(sRobot + lookaheadDist, path.getTotalLength());

    // Step 3: Get target point and profiled velocity
    Translation2d targetPoint = path.getPoint(sTarget);
    double profiledSpeed = velocityProfile.getVelocity(sRobot);

    // Step 4: Velocity direction — toward lookahead point
    Translation2d toTarget = targetPoint.minus(robotPos);
    double distToTarget = toTarget.getNorm();
    Translation2d direction;
    if (distToTarget > 1e-6) {
      direction = toTarget.div(distToTarget);
    } else {
      direction = tangent;
    }

    // Step 5: Cross-track PD correction + curvature feedforward
    double crossTrackRate = (dt > 1e-6) ? (crossTrackError - lastCrossTrackError) / dt : 0;
    double correction = crossTrackKp * crossTrackError + crossTrackKd * crossTrackRate;
    // Normal vector: 90 degrees CCW from tangent (points left of path direction)
    Translation2d normal = new Translation2d(-tangent.getY(), tangent.getX());
    // Curvature feedforward: proactively push toward center of curvature before error builds.
    // Signed curvature: positive = turning left = center is in +normal direction.
    double signedKappa = path.getCurvature(sRobot);
    double curvatureFf = curvatureFfGain * profiledSpeed * profiledSpeed * signedKappa;
    // -correction pushes toward path, +curvatureFf pushes toward center of curvature
    Translation2d correctionVec = normal.times(-correction + curvatureFf);

    // Step 6: Combine path velocity + correction
    Translation2d desiredVel = direction.times(profiledSpeed).plus(correctionVec);

    // Step 6.5: Apply per-axis overrides
    double vx = (overrideVx != null) ? overrideVx.getAsDouble() : desiredVel.getX();
    double vy = (overrideVy != null) ? overrideVy.getAsDouble() : desiredVel.getY();
    boolean vxUnlimited = (overrideVx != null && !limitOverrideVx);
    boolean vyUnlimited = (overrideVy != null && !limitOverrideVy);
    boolean omegaUnlimited = (overrideOmega != null && !limitOverrideOmega);

    // Step 7: Determine omega (override > rotationSupplier > 0) and rotation budget allocation
    double omega;
    if (overrideOmega != null) {
      omega = overrideOmega.getAsDouble();
    } else if (rotationSupplier != null) {
      omega = rotationSupplier.getOmega(robotPose, sRobot, tangent);
    } else {
      omega = 0.0;
    }

    // Rotation budget allocation: only when omega goes through limits
    if (!omegaUnlimited && omega != 0.0) {
      // Cap angular contribution to configured fraction of friction budget
      double maxAngularContrib = maxRotationBudgetFraction * AccelerationLimiter.MAX_FRICTION_ACCEL;
      double angularContrib = Math.abs(omega) * AccelerationLimiter.DRIVE_BASE_RADIUS;
      if (angularContrib > maxAngularContrib) {
        omega = Math.copySign(maxAngularContrib / AccelerationLimiter.DRIVE_BASE_RADIUS, omega);
        angularContrib = maxAngularContrib;
      }

      // Scale limited translation axes to leave room for rotation in friction budget
      double maxFriction = AccelerationLimiter.MAX_FRICTION_ACCEL;
      double availableFraction =
          Math.sqrt(
              Math.max(0, 1.0 - (angularContrib * angularContrib) / (maxFriction * maxFriction)));
      if (!vxUnlimited) vx *= availableFraction;
      if (!vyUnlimited) vy *= availableFraction;
    }

    // Step 8: Build limiter inputs — zero out unlimited axes so they don't steal friction budget
    double limitedVx = vxUnlimited ? 0 : vx;
    double limitedVy = vyUnlimited ? 0 : vy;
    double limitedOmega = omegaUnlimited ? 0 : omega;
    double currentVxForLimiter = vxUnlimited ? 0 : lastCommandedVelocity.vxMetersPerSecond;
    double currentVyForLimiter = vyUnlimited ? 0 : lastCommandedVelocity.vyMetersPerSecond;
    double currentOmegaForLimiter =
        omegaUnlimited ? 0 : lastCommandedVelocity.omegaRadiansPerSecond;

    ChassisSpeeds targetSpeeds =
        AccelerationLimiter.normalizeSpeeds(
            new ChassisSpeeds(limitedVx, limitedVy, limitedOmega));
    ChassisSpeeds limitedOutput =
        AccelerationLimiter.integrateVelocity(
            new ChassisSpeeds(currentVxForLimiter, currentVyForLimiter, currentOmegaForLimiter),
            targetSpeeds,
            dt);

    // Inject raw unlimited values back into the output
    ChassisSpeeds output =
        new ChassisSpeeds(
            vxUnlimited ? vx : limitedOutput.vxMetersPerSecond,
            vyUnlimited ? vy : limitedOutput.vyMetersPerSecond,
            omegaUnlimited ? omega : limitedOutput.omegaRadiansPerSecond);
    swerve.setControl(request.withSpeeds(output));

    // Update state for next cycle
    lastCommandedVelocity = output;
    lastCrossTrackError = crossTrackError;
    lastProjectedS = sRobot;

    // Log tracking data
    double progress = (path.getTotalLength() > 0) ? sRobot / path.getTotalLength() : 0;
    Logger.recordOutput("FollowPath/CrossTrackError", crossTrackError);
    Logger.recordOutput("FollowPath/CrossTrackRate", crossTrackRate);
    Logger.recordOutput("FollowPath/ArcLengthS", sRobot);
    Logger.recordOutput("FollowPath/Progress", progress);
    Logger.recordOutput("FollowPath/ProfiledSpeed", profiledSpeed);
    Logger.recordOutput("FollowPath/ActualSpeed", currentSpeed);
    Logger.recordOutput("FollowPath/LookaheadDist", lookaheadDist);
    Logger.recordOutput("FollowPath/Curvature", kappa);
    Logger.recordOutput("FollowPath/CurvatureFeedforward", curvatureFf);
    Logger.recordOutput("FollowPath/SpeedError", currentSpeed - profiledSpeed);
    Logger.recordOutput("FollowPath/Omega", omega);
    Logger.recordOutput("FollowPath/TargetPoint", new Pose2d(targetPoint, Rotation2d.kZero));
    Logger.recordOutput("FollowPath/ClosestPoint", new Pose2d(proj.point(), Rotation2d.kZero));
    Logger.recordOutput("FollowPath/RobotPose", new Pose2d(robotPos, robotPose.getRotation()));

    // Publish as double[] for path editor (NT4-friendly format)
    Logger.recordOutput(
        "PathEditor/TargetPoint", new double[] {targetPoint.getX(), targetPoint.getY()});
    Logger.recordOutput(
        "PathEditor/ClosestPoint", new double[] {proj.point().getX(), proj.point().getY()});
    Logger.recordOutput("PathEditor/CrossTrackError", crossTrackError);
    Logger.recordOutput("PathEditor/Progress", progress);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setControl(new SwerveRequest.Idle());

    // Clear logged path on end so it doesn't persist in AdvantageScope
    Logger.recordOutput("FollowPath/ReferencePath", new Pose2d[0]);
    Logger.recordOutput("FollowPath/TargetPoint", new Pose2d[0]);
    Logger.recordOutput("FollowPath/ClosestPoint", new Pose2d[0]);
    Logger.recordOutput("FollowPath/RobotPose", new Pose2d[0]);
  }

  /**
   * Logs the full reference path as a Pose2d array for visualization in AdvantageScope.
   *
   * <p>Sampled at PATH_LOG_SAMPLES points. The rotation of each pose is set to the tangent
   * direction so the trajectory arrow shows the path direction.
   */
  private void logReferencePath() {
    Pose2d[] pathPoses = new Pose2d[PATH_LOG_SAMPLES + 1];
    double ds = path.getTotalLength() / PATH_LOG_SAMPLES;

    for (int i = 0; i <= PATH_LOG_SAMPLES; i++) {
      double s = i * ds;
      Translation2d point = path.getPoint(s);
      Translation2d tangent = path.getTangent(s);
      Rotation2d heading = new Rotation2d(tangent.getX(), tangent.getY());
      pathPoses[i] = new Pose2d(point, heading);
    }

    Logger.recordOutput("FollowPath/ReferencePath", pathPoses);
    Logger.recordOutput("FollowPath/TotalLength", path.getTotalLength());
  }

  @Override
  public boolean isFinished() {
    // Finished when projected near path end and speed is low (if stopping)
    boolean nearEnd = lastProjectedS >= path.getTotalLength() - completionTolerance;
    if (endVelocity > 0) {
      // Pass-through: finish when near end regardless of speed
      return nearEnd;
    }
    double speed =
        Math.hypot(
            lastCommandedVelocity.vxMetersPerSecond, lastCommandedVelocity.vyMetersPerSecond);
    return nearEnd && speed < 0.1;
  }
}
