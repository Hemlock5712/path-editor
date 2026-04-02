import { VelocityConstraints } from '../types';
import { SplinePath } from './SplinePath';
import { getRobotConfig, type RobotSettings } from '../stores/settingsStore';

const GRAVITY = 9.81;

/** Number of samples for the editor velocity profile (lower than robot-side for perf). */
const EDITOR_SAMPLES = 200;

/**
 * Distance-based velocity profile using forward-backward pass.
 * Port of VelocityProfile.java with robot-accurate motor model.
 */
export class VelocityProfile {
  readonly samples: number[];
  readonly velocities: number[];
  readonly totalLength: number;

  constructor(path: SplinePath, constraints: VelocityConstraints, config?: RobotSettings) {
    const c = config ?? getRobotConfig();
    const kt = c.stallTorque / c.stallCurrent;
    const maxFrictionAccel = c.frictionCoefficient * GRAVITY;

    this.totalLength = path.getTotalLength();

    const numSamples = Math.max(50, Math.min(EDITOR_SAMPLES, Math.floor(this.totalLength * 100)));
    this.samples = [];
    this.velocities = [];

    const ds = this.totalLength / numSamples;

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

    // Step 1: Curvature limit at each point
    const maxFriction = 0.9 * maxFrictionAccel; // 90% for centripetal, 10% for tangential
    const curvatureLimit: number[] = [];
    for (let i = 0; i <= numSamples; i++) {
      const s = i * ds;
      this.samples.push(s);

      const kappa = Math.abs(path.getCurvature(s));
      let limit: number;
      if (kappa > 1e-6) {
        limit = Math.sqrt(maxFriction / kappa);
      } else {
        limit = constraints.maxVelocity;
      }
      limit = Math.min(limit, constraints.maxVelocity);
      limit = Math.max(limit, c.minVelocity);
      curvatureLimit.push(limit);
    }

    // Step 2: Forward pass — acceleration-limited from start
    const forward: number[] = [Math.min(constraints.startVelocity, curvatureLimit[0])];
    for (let i = 1; i <= numSamples; i++) {
      const maxAccel = Math.min(constraints.maxAcceleration, motorMaxAccel(forward[i - 1]));
      const vFromAccel = Math.sqrt(forward[i - 1] * forward[i - 1] + 2 * maxAccel * ds);
      let v = Math.min(vFromAccel, curvatureLimit[i]);
      v = Math.min(v, constraints.maxVelocity);
      forward.push(v);
    }

    // Step 3: Backward pass — deceleration-limited from end
    const backward: number[] = new Array(numSamples + 1);
    backward[numSamples] = Math.min(constraints.endVelocity, curvatureLimit[numSamples]);
    for (let i = numSamples - 1; i >= 0; i--) {
      const maxDecel = constraints.maxAcceleration;
      const vFromDecel = Math.sqrt(backward[i + 1] * backward[i + 1] + 2 * maxDecel * ds);
      let v = Math.min(vFromDecel, curvatureLimit[i]);
      v = Math.min(v, constraints.maxVelocity);
      backward[i] = v;
    }

    // Step 4: Final profile = min of forward and backward
    for (let i = 0; i <= numSamples; i++) {
      this.velocities.push(Math.max(c.minVelocity, Math.min(forward[i], backward[i])));
    }
  }

  getVelocity(s: number): number {
    if (s <= 0) return this.velocities[0];
    if (s >= this.totalLength) return this.velocities[this.velocities.length - 1];

    const ds = this.totalLength / (this.velocities.length - 1);
    const indexF = s / ds;
    const lo = Math.min(Math.floor(indexF), this.velocities.length - 2);
    const frac = indexF - lo;

    return this.velocities[lo] + frac * (this.velocities[lo + 1] - this.velocities[lo]);
  }
}
