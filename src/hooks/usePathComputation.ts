import { useMemo } from 'react';
import { usePathStore } from '../stores/pathStore';
import { useSettingsStore, type RobotSettings } from '../stores/settingsStore';
import { SplinePath } from '../math/SplinePath';
import { VelocityProfile } from '../math/VelocityProfile';
import { TimeEstimator } from '../math/TimeEstimator';
import {
  computeAnalytics,
  computeStats,
  type AnalyticsArrays,
  type PathStats,
} from '../math/ProfileAnalytics';

export function usePathComputation() {
  const controlPoints = usePathStore((s) => s.controlPoints);
  const headingWaypoints = usePathStore((s) => s.headingWaypoints);
  const constraints = usePathStore((s) => s.constraints);
  const constraintZones = usePathStore((s) => s.constraintZones);

  // Robot settings that affect velocity profiling
  const stallTorque = useSettingsStore((s) => s.stallTorque);
  const freeSpeedRpm = useSettingsStore((s) => s.freeSpeedRpm);
  const stallCurrent = useSettingsStore((s) => s.stallCurrent);
  const statorCurrentLimit = useSettingsStore((s) => s.statorCurrentLimit);
  const gearRatio = useSettingsStore((s) => s.gearRatio);
  const wheelRadius = useSettingsStore((s) => s.wheelRadius);
  const robotMass = useSettingsStore((s) => s.robotMass);
  const numDriveMotors = useSettingsStore((s) => s.numDriveMotors);
  const frictionCoefficient = useSettingsStore((s) => s.frictionCoefficient);
  const minVelocity = useSettingsStore((s) => s.minVelocity);
  const splinePath = useMemo(() => {
    if (controlPoints.length < 2) return null;
    try {
      return new SplinePath(controlPoints);
    } catch {
      return null;
    }
  }, [controlPoints]);

  const velocityProfile = useMemo(() => {
    if (!splinePath) return null;
    try {
      return new VelocityProfile(splinePath, constraints, headingWaypoints, controlPoints.length, undefined, constraintZones);
    } catch {
      return null;
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [splinePath, constraints, headingWaypoints, controlPoints.length, constraintZones, stallTorque,
    freeSpeedRpm, stallCurrent, statorCurrentLimit, gearRatio, wheelRadius, robotMass,
    numDriveMotors, frictionCoefficient, minVelocity]);

  const timeEstimator = useMemo(() => {
    if (!velocityProfile) return null;
    return new TimeEstimator(velocityProfile);
  }, [velocityProfile]);

  const analytics = useMemo(() => {
    if (!splinePath || !velocityProfile || !timeEstimator) return null;
    return computeAnalytics(
      splinePath,
      velocityProfile,
      timeEstimator,
      headingWaypoints,
      controlPoints.length,
    );
  }, [splinePath, velocityProfile, timeEstimator, headingWaypoints, controlPoints.length]);

  const stats = useMemo(() => {
    if (!splinePath || !velocityProfile || !timeEstimator) return null;
    return computeStats(
      splinePath,
      velocityProfile,
      timeEstimator,
      headingWaypoints,
      controlPoints.length,
      headingWaypoints.length,
    );
  }, [
    splinePath,
    velocityProfile,
    timeEstimator,
    headingWaypoints,
    controlPoints.length,
    headingWaypoints.length,
  ]);

  return { splinePath, velocityProfile, timeEstimator, analytics, stats };
}
