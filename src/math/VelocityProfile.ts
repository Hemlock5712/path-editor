import { VelocityConstraints, ConstraintZone, HeadingWaypoint } from '../types';
import { SplinePath } from './SplinePath';
import { getRobotConfig, type RobotSettings } from '../stores/settingsStore';
import { computeHeadingRate } from './ProfileAnalytics';

const GRAVITY = 9.81;

/** Fraction of friction budget reserved for centripetal acceleration in curvature limit. */
const CURVATURE_FRICTION_FRACTION = 0.9;

/**
 * Editor samples per meter of path length. Lower than robot-side (1000/m) for
 * rendering performance — the reduced density is acceptable for visualization but
 * means timing estimates may differ slightly from on-robot execution.
 */
const SAMPLES_PER_METER = 100;

/** Maximum editor samples to prevent huge allocations on very long paths. */
const MAX_EDITOR_SAMPLES = 200;

/**
 * Lookahead reaction time (seconds) for curvature smoothing window.
 * Accounts for CAN bus latency, motor response, and control loop delay.
 * Matches BRAKING_REACTION_TIME from the robot-side DriveToPoint.
 */
const REACTION_TIME = 0.1;

/**
 * Distance-based velocity profile using forward-backward pass.
 * Port of VelocityProfile.java with robot-accurate motor model
 * and friction-circle coupling between centripetal and tangential acceleration.
 */
export class VelocityProfile {
  readonly samples: number[];
  readonly velocities: number[];
  readonly totalLength: number;

  constructor(
    path: SplinePath,
    constraints: VelocityConstraints,
    headingWaypoints: HeadingWaypoint[],
    numControlPoints: number,
    config?: RobotSettings,
    constraintZones?: ConstraintZone[]
  ) {
    const c = config ?? getRobotConfig();
    const kt = c.stallTorque / c.stallCurrent;
    const maxFrictionAccel = c.frictionCoefficient * GRAVITY;

    this.totalLength = path.getTotalLength();

    const numSamples = Math.max(
      50,
      Math.min(
        MAX_EDITOR_SAMPLES,
        Math.floor(this.totalLength * SAMPLES_PER_METER)
      )
    );
    this.samples = [];
    this.velocities = [];

    const ds = this.totalLength / numSamples;

    // Precompute effective constraints per sample (zones are restrictive-only)
    const effectiveMaxVel: number[] = new Array(numSamples + 1).fill(
      constraints.maxVelocity
    );
    const effectiveMaxAccel: number[] = new Array(numSamples + 1).fill(
      constraints.maxAcceleration
    );

    if (constraintZones && constraintZones.length > 0) {
      const zoneBounds = constraintZones.map((z) => ({
        startS: path.getArcLengthAtWaypointIndex(z.startWaypointIndex),
        endS: path.getArcLengthAtWaypointIndex(z.endWaypointIndex),
        maxVelocity: z.maxVelocity,
        maxAcceleration: z.maxAcceleration,
      }));

      for (let i = 0; i <= numSamples; i++) {
        const s = i * ds;
        for (const zb of zoneBounds) {
          if (s >= zb.startS && s <= zb.endS) {
            effectiveMaxVel[i] = Math.min(effectiveMaxVel[i], zb.maxVelocity);
            effectiveMaxAccel[i] = Math.min(
              effectiveMaxAccel[i],
              zb.maxAcceleration
            );
          }
        }
      }
    }

    const motorMaxAccel = (speed: number): number => {
      const wheelAngVel = speed / c.wheelRadius;
      const motorAngVel = wheelAngVel * c.gearRatio;
      const motorRpm = (motorAngVel * 60) / (2 * Math.PI);

      let torque = c.stallTorque * (1.0 - motorRpm / c.freeSpeedRpm);
      torque = Math.min(torque, kt * c.statorCurrentLimit);
      torque = Math.max(torque, 0);

      const wheelTorque = torque * c.gearRatio;
      const force = (wheelTorque / c.wheelRadius) * c.numDriveMotors;
      return force / c.robotMass;
    };

    // Step 1: Curvature limit at each point (also store raw curvature for friction circle)
    const maxFriction = CURVATURE_FRICTION_FRACTION * maxFrictionAccel;
    const kappas: number[] = new Array(numSamples + 1);
    const curvatureLimit: number[] = new Array(numSamples + 1);
    for (let i = 0; i <= numSamples; i++) {
      const s = i * ds;
      this.samples.push(s);

      const kappa = Math.abs(path.getCurvature(s));
      kappas[i] = kappa;
      let limit: number;
      if (kappa > 1e-6) {
        limit = Math.sqrt(maxFriction / kappa);
      } else {
        limit = effectiveMaxVel[i];
      }
      limit = Math.min(limit, effectiveMaxVel[i]);
      limit = Math.max(limit, c.minVelocity);
      curvatureLimit[i] = limit;
    }

    // Step 1b: Smooth kappas with a lookahead window so that the friction circle
    // in the forward/backward passes accounts for curvature changes within one control cycle.
    // At Catmull-Rom knots, curvature can be discontinuous (C1 only). Without smoothing,
    // the passes immediately exploit freed-up friction budget when curvature drops,
    // but the robot's control loop can't react that fast.
    const windowMeters = constraints.maxVelocity * REACTION_TIME;
    const windowSamples = Math.max(1, Math.floor(windowMeters / ds));
    const smoothedKappas: number[] = new Array(numSamples + 1);
    for (let i = 0; i <= numSamples; i++) {
      let maxKappa = kappas[i];
      const jStart = Math.max(0, i - windowSamples);
      const jEnd = Math.min(numSamples, i + windowSamples);
      for (let j = jStart; j <= jEnd; j++) {
        if (kappas[j] > maxKappa) maxKappa = kappas[j];
      }
      smoothedKappas[i] = maxKappa;
    }

    // Step 1c: Angular velocity limit — slow translation where rotation demand is high
    const dthetaDs = computeHeadingRate(
      headingWaypoints,
      path,
      this.samples,
      this.totalLength
    );
    for (let i = 0; i <= numSamples; i++) {
      const absDthetaDs = Math.abs(dthetaDs[i]);
      if (absDthetaDs > 1e-9) {
        const angularLimit = constraints.maxAngularVelocity / absDthetaDs;
        curvatureLimit[i] = Math.min(
          curvatureLimit[i],
          Math.max(angularLimit, c.minVelocity)
        );
      }
    }

    // Step 2: Forward pass — acceleration-limited from start (with friction circle)
    const forward: number[] = [
      Math.min(constraints.startVelocity, curvatureLimit[0]),
    ];
    for (let i = 1; i <= numSamples; i++) {
      // 1. Tentative speed using full budget (upper bound for centripetal estimate)
      const motorAccelTentative = motorMaxAccel(forward[i - 1]);
      const fullAccel = Math.min(motorAccelTentative, effectiveMaxAccel[i]);
      let vTentative = Math.sqrt(
        forward[i - 1] * forward[i - 1] + 2 * fullAccel * ds
      );
      vTentative = Math.min(vTentative, curvatureLimit[i], effectiveMaxVel[i]);

      // 2. Use smoothed curvature for friction circle (accounts for C1 discontinuities)
      const segmentKappa = Math.max(smoothedKappas[i - 1], smoothedKappas[i]);
      const centripetal = vTentative * vTentative * segmentKappa;

      // 3. Friction circle: recompute available tangential after centripetal
      const frictionBudget = effectiveMaxAccel[i];
      let frictionAvail: number;
      if (centripetal < frictionBudget) {
        frictionAvail = Math.sqrt(
          frictionBudget * frictionBudget - centripetal * centripetal
        );
      } else {
        frictionAvail = c.minVelocity;
      }

      // 4. Motor torque at tentative speed
      const motorAccel = motorMaxAccel(vTentative);
      let maxAccel = Math.min(motorAccel, frictionAvail);

      // 5. Angular acceleration limit
      const absDthetaDsFwd = Math.abs(dthetaDs[i]);
      if (absDthetaDsFwd > 1e-9) {
        maxAccel = Math.min(
          maxAccel,
          constraints.maxAngularAcceleration / absDthetaDsFwd
        );
      }

      const vFromAccel = Math.sqrt(
        forward[i - 1] * forward[i - 1] + 2 * maxAccel * ds
      );
      let v = Math.min(vFromAccel, curvatureLimit[i]);
      v = Math.min(v, effectiveMaxVel[i]);
      forward.push(v);
    }

    // Step 3: Backward pass — deceleration-limited from end (with friction circle)
    const backward: number[] = new Array(numSamples + 1);
    backward[numSamples] = Math.min(
      constraints.endVelocity,
      curvatureLimit[numSamples]
    );
    for (let i = numSamples - 1; i >= 0; i--) {
      const fullDecel = effectiveMaxAccel[i];

      // 1. Tentative speed using full decel budget
      let vTentative = Math.sqrt(
        backward[i + 1] * backward[i + 1] + 2 * fullDecel * ds
      );
      vTentative = Math.min(vTentative, curvatureLimit[i], effectiveMaxVel[i]);

      // 2. Use smoothed curvature for friction circle
      const segmentKappa = Math.max(smoothedKappas[i], smoothedKappas[i + 1]);
      const centripetal = vTentative * vTentative * segmentKappa;

      // 3. Friction circle: recompute available tangential deceleration after centripetal
      let availDecel: number;
      if (centripetal < fullDecel) {
        availDecel = Math.sqrt(
          fullDecel * fullDecel - centripetal * centripetal
        );
      } else {
        availDecel = c.minVelocity;
      }

      // 4. Angular deceleration limit
      const absDthetaDsBack = Math.abs(dthetaDs[i]);
      if (absDthetaDsBack > 1e-9) {
        availDecel = Math.min(
          availDecel,
          constraints.maxAngularAcceleration / absDthetaDsBack
        );
      }

      const vFromDecel = Math.sqrt(
        backward[i + 1] * backward[i + 1] + 2 * availDecel * ds
      );
      let v = Math.min(vFromDecel, curvatureLimit[i]);
      v = Math.min(v, effectiveMaxVel[i]);
      backward[i] = v;
    }

    // Step 4: Final profile = min of forward and backward
    for (let i = 0; i <= numSamples; i++) {
      this.velocities.push(
        Math.max(c.minVelocity, Math.min(forward[i], backward[i]))
      );
    }
  }

  getVelocity(s: number): number {
    if (s <= 0) return this.velocities[0];
    if (s >= this.totalLength)
      return this.velocities[this.velocities.length - 1];

    const ds = this.totalLength / (this.velocities.length - 1);
    const indexF = s / ds;
    const lo = Math.min(Math.floor(indexF), this.velocities.length - 2);
    const frac = indexF - lo;

    return (
      this.velocities[lo] +
      frac * (this.velocities[lo + 1] - this.velocities[lo])
    );
  }
}
