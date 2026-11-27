package frc.team5712.path.follow;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.RobotConfig;

/**
 * The push each wheel needs to give the robot an acceleration: the feed-forward that goes to the drivetrain next
 * to the velocity, so the wheels' own velocity loops only have the error left to correct.
 *
 * <p>Newton's second law, shared between the wheels:
 *
 * <ul>
 *   <li><b>Driving.</b> Together the wheels push with {@code mass · acceleration}. They don't each take a
 *       quarter: a robot speeding up leans back, so its back wheels carry more of its weight and its front wheels
 *       less, and a wheel can only push as hard as it is pressed down. Each wheel takes the share of the push that
 *       it carries of the weight. An equal share asks the light wheels for more than their grip, and they spin.
 *   <li><b>Spinning.</b> The turning effort {@code inertia · spin acceleration} is shared too, each wheel pushing
 *       at right angles to its arm from the robot's centre, the far wheels harder.
 * </ul>
 *
 * <p>The weight on each wheel comes from the centre of mass's height ({@link
 * RobotConfig#centerOfMassHeightMeters}): the higher it is, the more the robot leans. At 0 the shares are equal.
 *
 * <p>It leaves friction out: the drive motors' own static gain (kS) covers it.
 */
public final class WheelForces {
  private final double massKg;
  private final double inertiaKgM2;
  private final double[] moduleX;
  private final double[] moduleY;
  private final double sumArmSquared;
  // How much weight moves off a wheel per m/s² of acceleration, for each metre it sits forward (or left) of the
  // centre: the floor's push has to balance mass · acceleration · height trying to tip the robot
  private final double leanPerMeterX;
  private final double leanPerMeterY;

  /**
   * @param robot the robot file's mass, moment of inertia and centre of mass height
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
    double sumX2 = 0.0;
    double sumY2 = 0.0;
    for (int i = 0; i < moduleX.length; i++) {
      sumX2 += moduleX[i] * moduleX[i];
      sumY2 += moduleY[i] * moduleY[i];
    }
    if (!(sumX2 + sumY2 > 0.0)) {
      throw new IllegalArgumentException("the modules can't all be at the robot's centre");
    }
    this.sumArmSquared = sumX2 + sumY2;
    double tipping = massKg * Math.max(0.0, robot.centerOfMassHeightMeters());
    this.leanPerMeterX = sumX2 > 0.0 ? tipping / sumX2 : 0.0;
    this.leanPerMeterY = sumY2 > 0.0 ? tipping / sumY2 : 0.0;
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
    double cos = Math.cos(headingRadians);
    double sin = Math.sin(headingRadians);
    double[] share = weightShares(
        cos * acceleration.vxMetersPerSecond() + sin * acceleration.vyMetersPerSecond(),
        -sin * acceleration.vxMetersPerSecond() + cos * acceleration.vyMetersPerSecond());
    double driveX = massKg * acceleration.vxMetersPerSecond();
    double driveY = massKg * acceleration.vyMetersPerSecond();
    double spin = inertiaKgM2 * acceleration.omegaRadPerSec() / sumArmSquared;
    for (int i = 0; i < n; i++) {
      // the module's arm from the robot's centre, turned into the field frame
      double armX = cos * moduleX[i] - sin * moduleY[i];
      double armY = sin * moduleX[i] + cos * moduleY[i];
      fx[i] = driveX * share[i] - spin * armY;
      fy[i] = driveY * share[i] + spin * armX;
    }
    return new double[][] {fx, fy};
  }

  /**
   * The share of the robot's weight on each wheel while it accelerates at ({@code forward}, {@code left}) m/s² in
   * its own frame: equal at rest, more on the wheels it is leaning onto. They add up to 1.
   */
  double[] weightShares(double forwardMetersPerSecondSquared, double leftMetersPerSecondSquared) {
    int n = moduleX.length;
    double[] share = new double[n];
    double atRest = massKg * RobotConfig.GRAVITY / n;
    double total = 0.0;
    for (int i = 0; i < n; i++) {
      double load = atRest - leanPerMeterX * forwardMetersPerSecondSquared * moduleX[i]
          - leanPerMeterY * leftMetersPerSecondSquared * moduleY[i];
      share[i] = Math.max(0.0, load); // a wheel can be lifted, not pulled down
      total += share[i];
    }
    for (int i = 0; i < n; i++) {
      share[i] = total > 0.0 ? share[i] / total : 1.0 / n;
    }
    return share;
  }

  /** No push from any wheel (a stopped robot). */
  public double[][] none() {
    return new double[][] {new double[moduleX.length], new double[moduleX.length]};
  }
}
