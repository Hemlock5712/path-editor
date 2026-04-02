import {
  PathJson,
  Point,
  HeadingWaypoint,
  VelocityConstraints,
  DEFAULT_CONSTRAINTS,
} from '../types';

export function serialize(
  controlPoints: Point[],
  headingWaypoints: HeadingWaypoint[],
  constraints: VelocityConstraints,
): string {
  const data: PathJson = {
    version: '1.0',
    controlPoints: controlPoints.map((p) => ({ x: p.x, y: p.y })),
    constraints: {
      maxVelocity: constraints.maxVelocity,
      maxAcceleration: constraints.maxAcceleration,
      startVelocity: constraints.startVelocity,
      endVelocity: constraints.endVelocity,
    },
  };

  if (headingWaypoints.length > 0) {
    data.headingWaypoints = headingWaypoints.map((hw) => ({
      waypointIndex: hw.waypointIndex,
      degrees: hw.degrees,
    }));
  }

  return JSON.stringify(data, null, 2);
}

export function deserialize(json: string): {
  controlPoints: Point[];
  headingWaypoints: HeadingWaypoint[];
  constraints: VelocityConstraints;
} {
  const data: PathJson = JSON.parse(json);

  return {
    controlPoints: data.controlPoints.map((p) => ({ x: p.x, y: p.y })),
    headingWaypoints: (data.headingWaypoints ?? []).map((hw) => ({
      waypointIndex: hw.waypointIndex,
      degrees: hw.degrees,
    })),
    constraints: data.constraints
      ? {
          maxVelocity: data.constraints.maxVelocity ?? DEFAULT_CONSTRAINTS.maxVelocity,
          maxAcceleration: data.constraints.maxAcceleration ?? DEFAULT_CONSTRAINTS.maxAcceleration,
          startVelocity: data.constraints.startVelocity ?? DEFAULT_CONSTRAINTS.startVelocity,
          endVelocity: data.constraints.endVelocity ?? DEFAULT_CONSTRAINTS.endVelocity,
        }
      : { ...DEFAULT_CONSTRAINTS },
  };
}
