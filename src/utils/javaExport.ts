import {
  NamedPath,
  NamedPoint,
  Point,
  HeadingWaypoint,
  VelocityConstraints,
  ConstraintZone,
  RotationZone,
  WaypointFlag,
} from '../types';

const INDENT = '    '; // 4 spaces per level

/**
 * Sanitizes a user-provided name into a valid Java UPPER_SNAKE_CASE constant name.
 */
function toJavaConstantName(name: string): string {
  let result = name
    .replace(/[^a-zA-Z0-9_\s-]/g, '')
    .replace(/[\s-]+/g, '_')
    .replace(/_+/g, '_')
    .replace(/^_|_$/g, '')
    .toUpperCase();

  if (!result) return 'PATH';
  if (/^[0-9]/.test(result)) result = '_' + result;
  return result;
}

function generateSinglePath(
  constantName: string,
  controlPoints: Point[],
  headingWaypoints: HeadingWaypoint[],
  constraints: VelocityConstraints,
  constraintZones: ConstraintZone[],
  rotationZones: RotationZone[],
  waypointFlags: WaypointFlag[]
): string {
  const name = toJavaConstantName(constantName);
  const indent = INDENT.repeat(3); // 12 spaces for list entries

  const points = controlPoints
    .map((p) => `${indent}new Translation2d(${p.x}, ${p.y})`)
    .join(',\n');

  let headings: string;
  if (headingWaypoints.length === 0) {
    headings = 'List.of()';
  } else {
    const entries = headingWaypoints
      .map(
        (hw) =>
          `${indent}new PathData.HeadingWaypoint(${hw.waypointIndex}, Rotation2d.fromDegrees(${hw.degrees}))`
      )
      .join(',\n');
    headings = `List.of(\n${entries}\n${INDENT.repeat(2)})`;
  }

  const chainIndent = INDENT.repeat(3);
  let constraintChain = `VelocityConstraints.defaults()\n${chainIndent}.withMaxVelocity(${constraints.maxVelocity})\n${chainIndent}.withMaxAcceleration(${constraints.maxAcceleration})`;
  if (constraints.startVelocity !== 0) {
    constraintChain += `\n${chainIndent}.withStartVelocity(${constraints.startVelocity})`;
  }
  if (constraints.endVelocity !== 0) {
    constraintChain += `\n${chainIndent}.withEndVelocity(${constraints.endVelocity})`;
  }

  let zones: string;
  if (constraintZones.length === 0) {
    zones = 'List.of()';
  } else {
    const entries = constraintZones
      .map(
        (z) =>
          `${indent}new PathData.ConstraintZone(${z.startWaypointIndex}, ${z.endWaypointIndex}, ${z.maxVelocity}, ${z.maxAcceleration})`
      )
      .join(',\n');
    zones = `List.of(\n${entries}\n${INDENT.repeat(2)})`;
  }

  let rotZones: string;
  if (rotationZones.length === 0) {
    rotZones = 'List.of()';
  } else {
    const entries = rotationZones
      .map(
        (rz) =>
          `${indent}new PathData.RotationZone("${rz.id}", ${rz.startWaypointIndex}, ${rz.endWaypointIndex}, new Translation2d(${rz.targetPoint.x}, ${rz.targetPoint.y}))`
      )
      .join(',\n');
    rotZones = `List.of(\n${entries}\n${INDENT.repeat(2)})`;
  }

  let flags: string;
  if (waypointFlags.length === 0) {
    flags = 'List.of()';
  } else {
    const entries = waypointFlags
      .map(
        (flag) =>
          `${indent}new PathData.WaypointFlag("${flag.id}", ${flag.waypointIndex}, "${flag.label.replace(/\\/g, '\\\\').replace(/"/g, '\\"')}")`
      )
      .join(',\n');
    flags = `List.of(\n${entries}\n${INDENT.repeat(2)})`;
  }

  return `    public static final PathData ${name} = new PathData(
        List.of(
${points}
        ),
        ${headings},
        ${constraintChain},
        ${zones},
        ${rotZones},
        ${flags}
    );`;
}

function generateNamedPointConstants(
  namedPoints: Record<string, NamedPoint>
): string {
  const points = Object.values(namedPoints);
  if (points.length === 0) return '';

  const lines = points
    .map((np) => {
      const name = toJavaConstantName(np.name);
      if (np.headingDegrees !== null) {
        return `    public static final Pose2d ${name} = new Pose2d(new Translation2d(${np.x}, ${np.y}), Rotation2d.fromDegrees(${np.headingDegrees}));`;
      }
      return `    public static final Translation2d ${name} = new Translation2d(${np.x}, ${np.y});`;
    })
    .join('\n');

  return `    // Named field positions\n${lines}\n`;
}

/**
 * Generates a compilable Paths.java file with all path constants.
 */
export function generatePathsJava(
  paths: NamedPath[],
  namedPoints?: Record<string, NamedPoint>
): string {
  const constants = paths
    .map((p) =>
      generateSinglePath(
        p.name,
        p.controlPoints,
        p.headingWaypoints,
        p.constraints,
        p.constraintZones,
        p.rotationZones,
        p.waypointFlags
      )
    )
    .join('\n\n');

  const namedPointConstants = namedPoints
    ? generateNamedPointConstants(namedPoints)
    : '';
  const hasPosePoints = namedPoints
    ? Object.values(namedPoints).some((np) => np.headingDegrees !== null)
    : false;

  return `package frc.robot.utils.path;

${hasPosePoints ? 'import edu.wpi.first.math.geometry.Pose2d;\n' : ''}import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;

/** Path constants generated by FRC Path Editor. */
public final class Paths {
    private Paths() {}

${namedPointConstants}${constants}

    /**
     * Returns the path mirrored for the red alliance if needed.
     * Call during autonomousInit() or later when the alliance is known.
     */
    public static PathData forAlliance(PathData path) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return path.mirrorForRedAlliance();
        }
        return path;
    }
}
`;
}
