package frc.team5712.path.wpilib;

import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.follow.OutputLimiter;
import frc.team5712.path.follow.WheelForces;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * The limiter for driving by hand: the driver asks for a velocity, and this gives the closest one the robot can
 * reach this loop without spinning its wheels or tipping. It is the limiter the paths go through
 * ({@link OutputLimiter}), reading the same robot file, so teleop and autonomous share one set of numbers.
 *
 * <pre>
 * DriveLimiter limiter = new DriveLimiter(paths.robot());        // robot init
 *
 * limiter.reset(drivetrain.getFieldVelocity());                  // when the drive command starts
 * ChassisVelocities command = limiter.limit(fromTheSticks, dt);  // every loop, then send it
 * </pre>
 *
 * Use one frame throughout: field-relative, or the driver's, but the same for {@link #reset} and {@link #limit}.
 */
public final class DriveLimiter {
  private final RobotConfig robot;
  private final OutputLimiter limiter;
  private WheelForces forces;

  public DriveLimiter(RobotConfig robot) {
    this.robot = robot;
    this.limiter = new OutputLimiter(robot);
  }

  /** Starts from the measured velocity, so a robot already moving isn't asked to stop in one loop. */
  public void reset(ChassisVelocities measured) {
    limiter.reset(Conversions.toFieldSpeeds(measured));
  }

  /** The velocity to command this loop: as close to {@code wanted} as the robot can get in {@code dtSeconds}. */
  public ChassisVelocities limit(ChassisVelocities wanted, double dtSeconds) {
    return Conversions.toChassisVelocities(limiter.limit(Conversions.toFieldSpeeds(wanted), dtSeconds));
  }

  /**
   * Each wheel's push for the step {@link #limit} just took, for a drivetrain that takes the feed-forward
   * ({@link WheelForceDrive}): {@code [0]} the forces along x and {@code [1]} along y, in newtons, in the frame
   * the velocities are in. {@code heading} is the robot's heading in that frame.
   *
   * @param moduleLocations the modules' positions, in the drivetrain's order
   */
  public double[][] wheelForces(Translation2d[] moduleLocations, Rotation2d heading) {
    if (forces == null || forces.moduleCount() != moduleLocations.length) {
      double[] x = new double[moduleLocations.length];
      double[] y = new double[moduleLocations.length];
      for (int i = 0; i < x.length; i++) {
        x[i] = moduleLocations[i].getX();
        y[i] = moduleLocations[i].getY();
      }
      forces = new WheelForces(robot, x, y);
    }
    return forces.forAcceleration(limiter.acceleration(), heading.getRadians());
  }
}
