package frc.team5712.path.follow;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.RobotConfig;

/**
 * The push each wheel needs to give the robot an acceleration: the feed-forward that goes to the drivetrain next
 * to the velocity, so the wheels' own velocity loops only have the error left to correct.
 *
 * <p>Newton's second law, shared between the wheels. Driving: every wheel pushes with an equal share of {@code
 * mass · acceleration}. Spinning: the turning effort {@code inertia · spin acceleration} is shared too, each
 * wheel pushing at right angles to its arm from the robot's centre, the far wheels harder.
 *
 * <p>It leaves friction out: the drive motors' own static gain (kS) covers it.
 */
public final class WheelForces {
  private final double massKg;
  private final double inertiaKgM2;
  private final double[] moduleX;
  private final double[] moduleY;
  private final double sumArmSquared;

  /**
   * @param robot the robot file's mass and moment of inertia
   * @param moduleXMeters each module's position forward of the robot's centre, in the drivetrain's module order
   * @param moduleYMeters each module's position left of the robot's centre, in the same order
   */
  public WheelForces(RobotConfig robot, double[] moduleXMeters, double[] moduleYMeters) {
    if (moduleXMeters.length == 0 || moduleXMeters.length != moduleYMeters.length) {
      throw new IllegalArgumentException("one x and one y for every module");
    }
    this.massKg = robot.massKg();
    this.inertiaKgM2 = robot.momentOfInertiaKgM2();
    this.moduleX = moduleXMeters.clone();
    this.moduleY = moduleYMeters.clone();
    double sum = 0.0;
    for (int i = 0; i < moduleX.length; i++) {
      sum += moduleX[i] * moduleX[i] + moduleY[i] * moduleY[i];
    }
    if (!(sum > 0.0)) {
      throw new IllegalArgumentException("the modules can't all be at the robot's centre");
    }
    this.sumArmSquared = sum;
  }

  public int moduleCount() {
    return moduleX.length;
  }

  /**
   * The forces for {@code acceleration} ({@link PathFollower#commandedAcceleration()}: field frame, m/s² and
   * rad/s²) with the robot facing {@code headingRadians}.
   *
   * @return two new arrays, {@code [0]} the forces along the field's x and {@code [1]} along its y, in newtons,
   *     one per module in the order given to the constructor
   */
  public double[][] forAcceleration(FieldSpeeds acceleration, double headingRadians) {
    int n = moduleX.length;
    double[] fx = new double[n];
    double[] fy = new double[n];
    double driveX = massKg * acceleration.vxMetersPerSecond() / n;
    double driveY = massKg * acceleration.vyMetersPerSecond() / n;
    double spin = inertiaKgM2 * acceleration.omegaRadPerSec() / sumArmSquared;
    double cos = Math.cos(headingRadians);
    double sin = Math.sin(headingRadians);
    for (int i = 0; i < n; i++) {
      // the module's arm from the robot's centre, turned into the field frame
      double armX = cos * moduleX[i] - sin * moduleY[i];
      double armY = sin * moduleX[i] + cos * moduleY[i];
      fx[i] = driveX - spin * armY;
      fy[i] = driveY + spin * armX;
    }
    return new double[][] {fx, fy};
  }

  /** No push from any wheel (a stopped robot). */
  public double[][] none() {
    return new double[][] {new double[moduleX.length], new double[moduleX.length]};
  }
}
