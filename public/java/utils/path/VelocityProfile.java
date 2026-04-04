package frc.robot.utils.path;

import frc.robot.commands.AccelerationLimiter;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.Motor;
import java.util.List;

/**
 * Distance-based velocity profile along a spline path.
 *
 * <p>Uses a forward-backward pass to compute the maximum achievable speed at every point along the
 * path, respecting curvature limits, motor torque curves, and friction constraints.
 *
 * <p>This is the same v = sqrt(v_end^2 + 2*a*d) math used in DriveToPointUtils, applied as a sweep
 * over the entire path.
 */
public final class VelocityProfile {

  /** Minimum velocity floor to prevent stalling at tight curves (m/s). */
  private static final double MIN_VELOCITY = 0.1;

  // Motor parameters for speed-dependent acceleration (same as AccelerationLimiter)
  private static final Motor MOTOR = Motor.KRAKEN_X60_FOC;
  private static final double GEAR_RATIO = TunerConstants.FrontLeft.DriveMotorGearRatio;
  private static final double WHEEL_RADIUS = TunerConstants.FrontLeft.WheelRadius;
  private static final double ROBOT_MASS = 60;
  private static final int NUM_DRIVE_MOTORS = 4;
  private static final double STATOR_CURRENT_LIMIT = 150.0;

  private final double[] sSamples;
  private final double[] velocities;
  private final double totalLength;

  /**
   * Builds a velocity profile for the given path and constraints (no constraint zones).
   *
   * @param path The spline path
   * @param constraints Velocity and acceleration limits
   */
  public VelocityProfile(SplinePath path, VelocityConstraints constraints) {
    this(path, constraints, List.of());
  }

  /**
   * Builds a velocity profile for the given path, constraints, and constraint zones.
   *
   * @param path The spline path
   * @param constraints Velocity and acceleration limits
   * @param constraintZones Per-zone overrides for velocity and acceleration
   */
  public VelocityProfile(
      SplinePath path,
      VelocityConstraints constraints,
      List<PathData.ConstraintZone> constraintZones) {
    totalLength = path.getTotalLength();

    // Determine number of samples (~1000 per meter, minimum 100)
    int numSamples = Math.max(100, (int) (totalLength * 1000));
    sSamples = new double[numSamples + 1];
    velocities = new double[numSamples + 1];

    double ds = totalLength / numSamples;

    // Precompute effective per-sample velocity and acceleration limits from constraint zones
    double[] effectiveMaxVel = new double[numSamples + 1];
    double[] effectiveMaxAccel = new double[numSamples + 1];
    for (int i = 0; i <= numSamples; i++) {
      effectiveMaxVel[i] = constraints.getMaxVelocity();
      effectiveMaxAccel[i] = constraints.getMaxAcceleration();
    }
    for (PathData.ConstraintZone zone : constraintZones) {
      double zoneStartS = path.getArcLengthAtWaypointIndex(zone.startWaypointIndex());
      double zoneEndS = path.getArcLengthAtWaypointIndex(zone.endWaypointIndex());
      for (int i = 0; i <= numSamples; i++) {
        double s = i * ds;
        if (s >= zoneStartS && s <= zoneEndS) {
          effectiveMaxVel[i] = Math.min(effectiveMaxVel[i], zone.maxVelocity());
          effectiveMaxAccel[i] = Math.min(effectiveMaxAccel[i], zone.maxAcceleration());
        }
      }
    }

    // Step 1: Curvature limit at each point (also store raw curvature for friction circle)
    double[] curvatureLimit = new double[numSamples + 1];
    double[] kappas = new double[numSamples + 1];
    double maxFriction = constraints.getCurvatureFrictionFraction() * AccelerationLimiter.GRAVITY;
    for (int i = 0; i <= numSamples; i++) {
      double s = i * ds;
      sSamples[i] = s;

      double kappa = Math.abs(path.getCurvature(s));
      kappas[i] = kappa;
      if (kappa > 1e-6) {
        // v_max = sqrt(friction * g / kappa) — centripetal acceleration constraint
        curvatureLimit[i] = Math.sqrt(maxFriction / kappa);
      } else {
        curvatureLimit[i] = effectiveMaxVel[i];
      }
      curvatureLimit[i] = Math.min(curvatureLimit[i], effectiveMaxVel[i]);
      curvatureLimit[i] = Math.max(curvatureLimit[i], MIN_VELOCITY);
    }

    // Step 1b: Smooth kappas with a lookahead window so that the friction circle
    // in the forward/backward passes accounts for curvature changes within one control cycle.
    // At Catmull-Rom knots, curvature can be discontinuous (C1 only). Without smoothing,
    // the passes immediately exploit freed-up friction budget when curvature drops,
    // but the robot's 50Hz control loop can't react that fast.
    // Window: maxVelocity * reactionTime in each direction.
    // Matches BRAKING_REACTION_TIME from DriveToPoint — accounts for CAN bus latency,
    // motor response, and control loop delay, not just one 50Hz step.
    double reactionTime = 0.1; // seconds (same as DriveToPoint.BRAKING_REACTION_TIME)
    double windowMeters = constraints.getMaxVelocity() * reactionTime;
    int windowSamples = Math.max(1, (int) (windowMeters / ds));
    double[] smoothedKappas = new double[numSamples + 1];
    for (int i = 0; i <= numSamples; i++) {
      double maxKappa = kappas[i];
      for (int j = Math.max(0, i - windowSamples);
          j <= Math.min(numSamples, i + windowSamples);
          j++) {
        maxKappa = Math.max(maxKappa, kappas[j]);
      }
      smoothedKappas[i] = maxKappa;
    }

    // Step 2: Forward pass — acceleration-limited from start
    double[] forward = new double[numSamples + 1];
    forward[0] = Math.min(constraints.getStartVelocity(), curvatureLimit[0]);
    for (int i = 1; i <= numSamples; i++) {
      // 1. Tentative speed using full budget (upper bound for centripetal estimate)
      double motorAccelTentative = getMotorMaxAcceleration(forward[i - 1]);
      double fullAccel = Math.min(motorAccelTentative, effectiveMaxAccel[i]);
      double vTentative = Math.sqrt(forward[i - 1] * forward[i - 1] + 2 * fullAccel * ds);
      vTentative = Math.min(vTentative, curvatureLimit[i]);
      vTentative = Math.min(vTentative, effectiveMaxVel[i]);

      // 2. Use smoothed curvature for friction circle (accounts for C1 discontinuities)
      double segmentKappa = Math.max(smoothedKappas[i - 1], smoothedKappas[i]);
      double centripetal = vTentative * vTentative * segmentKappa;

      // 3. Friction circle: recompute available tangential after centripetal
      double frictionBudget = effectiveMaxAccel[i];
      double frictionAvail;
      if (centripetal < frictionBudget) {
        frictionAvail = Math.sqrt(frictionBudget * frictionBudget - centripetal * centripetal);
      } else {
        frictionAvail = MIN_VELOCITY;
      }

      // 4. Motor torque at tentative speed (more conservative than at forward[i-1])
      double motorAccel = getMotorMaxAcceleration(vTentative);
      double maxAccel = Math.min(motorAccel, frictionAvail);

      // v = sqrt(v_prev^2 + 2 * a * ds) — kinematic equation, distance-based
      double vFromAccel = Math.sqrt(forward[i - 1] * forward[i - 1] + 2 * maxAccel * ds);
      forward[i] = Math.min(vFromAccel, curvatureLimit[i]);
      forward[i] = Math.min(forward[i], effectiveMaxVel[i]);
    }

    // Step 3: Backward pass — deceleration-limited from end
    double[] backward = new double[numSamples + 1];
    backward[numSamples] = Math.min(constraints.getEndVelocity(), curvatureLimit[numSamples]);
    for (int i = numSamples - 1; i >= 0; i--) {
      double fullDecel = Math.min(constraints.getMaxDeceleration(), effectiveMaxAccel[i]);

      // 1. Tentative speed using full decel budget (upper bound for centripetal estimate)
      double vTentative = Math.sqrt(backward[i + 1] * backward[i + 1] + 2 * fullDecel * ds);
      vTentative = Math.min(vTentative, curvatureLimit[i]);
      vTentative = Math.min(vTentative, effectiveMaxVel[i]);

      // 2. Use smoothed curvature for friction circle (accounts for C1 discontinuities)
      double segmentKappa = Math.max(smoothedKappas[i], smoothedKappas[i + 1]);
      double centripetal = vTentative * vTentative * segmentKappa;

      // 3. Friction circle: recompute available tangential deceleration after centripetal
      double availDecel;
      if (centripetal < fullDecel) {
        availDecel = Math.sqrt(fullDecel * fullDecel - centripetal * centripetal);
      } else {
        availDecel = MIN_VELOCITY;
      }

      // v = sqrt(v_next^2 + 2 * a * ds) — same equation, backward
      double vFromDecel = Math.sqrt(backward[i + 1] * backward[i + 1] + 2 * availDecel * ds);
      backward[i] = Math.min(vFromDecel, curvatureLimit[i]);
      backward[i] = Math.min(backward[i], effectiveMaxVel[i]);
    }

    // Step 4: Final profile is the minimum of forward and backward at each point
    for (int i = 0; i <= numSamples; i++) {
      velocities[i] = Math.max(MIN_VELOCITY, Math.min(forward[i], backward[i]));
    }
  }

  /**
   * Returns the profiled velocity at arc length s.
   *
   * @param s Arc length in meters
   * @return Velocity in m/s
   */
  public double getVelocity(double s) {
    if (s <= 0) {
      return velocities[0];
    }
    if (s >= totalLength) {
      return velocities[velocities.length - 1];
    }

    // Linear interpolation between samples
    double ds = totalLength / (velocities.length - 1);
    double indexF = s / ds;
    int lo = (int) indexF;
    lo = Math.min(lo, velocities.length - 2);
    double frac = indexF - lo;

    return velocities[lo] + frac * (velocities[lo + 1] - velocities[lo]);
  }

  /** Returns the total path length in meters. */
  public double getTotalLength() {
    return totalLength;
  }

  /** Returns the arc-length sample points (for visualization). */
  public double[] getSamples() {
    return sSamples.clone();
  }

  /** Returns the velocity at each sample point (for visualization). */
  public double[] getVelocities() {
    return velocities.clone();
  }

  /**
   * Gets the motor-limited max acceleration at a given speed.
   *
   * <p>Same motor model used by AccelerationLimiter — torque decreases with speed.
   */
  private static double getMotorMaxAcceleration(double speed) {
    return MOTOR.getMaxAcceleration(
        speed, GEAR_RATIO, WHEEL_RADIUS, ROBOT_MASS, NUM_DRIVE_MOTORS, STATOR_CURRENT_LIMIT);
  }
}
