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
 *       at right angles to its arm, the far wheels harder.
 * </ul>
 *
 * <p>Two sums always hold, whatever the shares (the tests check them on random robots): the forces add up to
 * {@code mass · acceleration}, and their turning effort about the robot's centre is {@code inertia · spin
 * acceleration}. Unequal shares on their own would turn the robot a little; the spin term takes that out.
 *
 * <p>What it assumes, so the numbers are only as good as these:
 *
 * <ul>
 *   <li>The module positions are measured from the centre of mass. A robot whose weight sits off-centre should
 *       give them from where the weight is, not from the middle of the frame.
 *   <li>The weight on each wheel is the stiff-frame answer: the loads lie on a plane across the wheels, tilted
 *       just enough to balance {@code mass · acceleration · height}, the height being {@link
 *       RobotConfig#centerOfMassHeightMeters}. At 0 the robot doesn't lean and the shares are equal. A wheel can
 *       be lifted, not pulled down: a load below zero counts as zero.
 *   <li>The lean follows the acceleration asked for, at once.
 *   <li>The push is what the floor must give the robot. Spinning up the motors' own rotors takes a little more
 *       (a few per cent), and friction more again; the drive motors' velocity loops and static gain (kS) cover
 *       both.
 * </ul>
 */
public final class WheelForces {
  private final double massKg;
  private final double inertiaKgM2;
  private final double[] moduleX;
  private final double[] moduleY;
  // the modules' middle, and the sum of their squared distances from it: the spin term pushes about the middle,
  // so its forces cancel whatever the layout
  private final double middleX;
  private final double middleY;
  private final double sumArmSquared;
  // The plane of loads: load_i = p[0] + p[1]·x_i + p[2]·y_i, where p = plane · (weight, pitching moment, rolling
  // moment). Null when the modules are in a line (no plane fits): the shares are then equal.
  private final double[][] plane;
  private final double tipping; // mass · centre of mass height

  /**
   * @param robot the robot file's mass, moment of inertia and centre of mass height
   * @param moduleXMeters each module's position forward of the centre of mass, in the drivetrain's module order
   * @param moduleYMeters each module's position left of the centre of mass, in the same order
   */
  public WheelForces(RobotConfig robot, double[] moduleXMeters, double[] moduleYMeters) {
    int n = moduleXMeters.length;
    if (n == 0 || n != moduleYMeters.length) {
      throw new IllegalArgumentException("one x and one y for every module");
    }
    this.massKg = robot.massKg();
    this.inertiaKgM2 = robot.momentOfInertiaKgM2();
    this.moduleX = moduleXMeters.clone();
    this.moduleY = moduleYMeters.clone();
    double sx = 0.0;
    double sy = 0.0;
    double sxx = 0.0;
    double sxy = 0.0;
    double syy = 0.0;
    for (int i = 0; i < n; i++) {
      sx += moduleX[i];
      sy += moduleY[i];
      sxx += moduleX[i] * moduleX[i];
      sxy += moduleX[i] * moduleY[i];
      syy += moduleY[i] * moduleY[i];
    }
    this.middleX = sx / n;
    this.middleY = sy / n;
    this.sumArmSquared = sxx + syy - n * (middleX * middleX + middleY * middleY);
    if (!(sumArmSquared > 0.0)) {
      throw new IllegalArgumentException("the modules can't all be in one place");
    }
    this.tipping = massKg * Math.max(0.0, robot.centerOfMassHeightMeters());
    // The loads balance three things: they add up to the weight; their moment about the centre of mass's
    // sideways line balances the pitch (Σ load·x = -mass·height·a forward); and likewise the roll (Σ load·y).
    // With load_i = p0 + p1·x_i + p2·y_i those are three equations in p; this is their matrix, inverted once.
    this.plane = inverse(new double[][] {{n, sx, sy}, {sx, sxx, sxy}, {sy, sxy, syy}});
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
    double cos = Math.cos(headingRadians);
    double sin = Math.sin(headingRadians);
    // in the robot's own frame: the lean and the arms are the robot's
    double forward = cos * acceleration.vxMetersPerSecond() + sin * acceleration.vyMetersPerSecond();
    double left = -sin * acceleration.vxMetersPerSecond() + cos * acceleration.vyMetersPerSecond();
    double[] share = weightShares(forward, left);
    double driveForward = massKg * forward;
    double driveLeft = massKg * left;
    // unequal shares push off-centre, which would turn the robot: the spin term gives the turning effort asked
    // for less what the driving forces already give
    double turnFromDriving = 0.0;
    for (int i = 0; i < n; i++) {
      turnFromDriving += share[i] * (moduleX[i] * driveLeft - moduleY[i] * driveForward);
    }
    double spin = (inertiaKgM2 * acceleration.omegaRadPerSec() - turnFromDriving) / sumArmSquared;
    double[] fx = new double[n];
    double[] fy = new double[n];
    for (int i = 0; i < n; i++) {
      double pushForward = driveForward * share[i] - spin * (moduleY[i] - middleY);
      double pushLeft = driveLeft * share[i] + spin * (moduleX[i] - middleX);
      fx[i] = cos * pushForward - sin * pushLeft; // back into the field frame
      fy[i] = sin * pushForward + cos * pushLeft;
    }
    return new double[][] {fx, fy};
  }

  /**
   * The share of the robot's weight on each wheel while it accelerates at ({@code forward}, {@code left}) m/s² in
   * its own frame: more on the wheels it is leaning onto. They add up to 1.
   */
  double[] weightShares(double forwardMetersPerSecondSquared, double leftMetersPerSecondSquared) {
    int n = moduleX.length;
    double[] share = new double[n];
    if (plane == null) {
      java.util.Arrays.fill(share, 1.0 / n);
      return share;
    }
    double weight = massKg * RobotConfig.GRAVITY;
    double pitch = -tipping * forwardMetersPerSecondSquared;
    double roll = -tipping * leftMetersPerSecondSquared;
    double p0 = plane[0][0] * weight + plane[0][1] * pitch + plane[0][2] * roll;
    double p1 = plane[1][0] * weight + plane[1][1] * pitch + plane[1][2] * roll;
    double p2 = plane[2][0] * weight + plane[2][1] * pitch + plane[2][2] * roll;
    double total = 0.0;
    for (int i = 0; i < n; i++) {
      share[i] = Math.max(0.0, p0 + p1 * moduleX[i] + p2 * moduleY[i]); // a wheel can be lifted, not pulled down
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

  /** The inverse of a 3×3 matrix, or null if it has none. */
  private static double[][] inverse(double[][] m) {
    double a = m[0][0], b = m[0][1], c = m[0][2];
    double d = m[1][0], e = m[1][1], f = m[1][2];
    double g = m[2][0], h = m[2][1], k = m[2][2];
    double det = a * (e * k - f * h) - b * (d * k - f * g) + c * (d * h - e * g);
    double scale = Math.abs(a * e * k);
    if (!(Math.abs(det) > 1e-9 * Math.max(scale, 1e-12))) {
      return null;
    }
    return new double[][] {
      {(e * k - f * h) / det, (c * h - b * k) / det, (b * f - c * e) / det},
      {(f * g - d * k) / det, (a * k - c * g) / det, (c * d - a * f) / det},
      {(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det}
    };
  }
}
