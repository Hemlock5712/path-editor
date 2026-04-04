import { describe, it, expect } from 'vitest';
import { parsePathsJava } from '../javaParser';

const WELL_FORMED_JAVA = `
package frc.robot.utils.path;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public final class Paths {
    private Paths() {}

    public static final PathData TEST_PATH = new PathData(
        List.of(
            new Translation2d(1.5, 2.0),
            new Translation2d(3.0, 4.5),
            new Translation2d(6.0, 2.0)
        ),
        List.of(
            new PathData.HeadingWaypoint(0, Rotation2d.fromDegrees(45)),
            new PathData.HeadingWaypoint(2, Rotation2d.fromDegrees(90))
        ),
        VelocityConstraints.defaults()
            .withMaxVelocity(4.0)
            .withMaxAcceleration(8.5)
            .withStartVelocity(1.0)
            .withEndVelocity(0.5),
        List.of(
            new PathData.ConstraintZone(0, 1, 2.0, 5.0)
        ),
        List.of(
            new PathData.RotationZone("zone1", 0, 2, new Translation2d(4.0, 3.0))
        )
    );
}`;

describe('parsePathsJava', () => {
  describe('well-formed input', () => {
    const paths = parsePathsJava(WELL_FORMED_JAVA);

    it('finds one path', () => {
      expect(paths).toHaveLength(1);
    });

    it('extracts the path name', () => {
      expect(paths[0].name).toBe('TEST_PATH');
    });

    it('parses control points', () => {
      expect(paths[0].controlPoints).toEqual([
        { x: 1.5, y: 2.0 },
        { x: 3.0, y: 4.5 },
        { x: 6.0, y: 2.0 },
      ]);
    });

    it('parses heading waypoints', () => {
      expect(paths[0].headingWaypoints).toEqual([
        { waypointIndex: 0, degrees: 45 },
        { waypointIndex: 2, degrees: 90 },
      ]);
    });

    it('parses velocity constraints', () => {
      expect(paths[0].constraints.maxVelocity).toBe(4.0);
      expect(paths[0].constraints.maxAcceleration).toBe(8.5);
      expect(paths[0].constraints.startVelocity).toBe(1.0);
      expect(paths[0].constraints.endVelocity).toBe(0.5);
    });

    it('parses constraint zones', () => {
      expect(paths[0].constraintZones).toHaveLength(1);
      expect(paths[0].constraintZones[0].startWaypointIndex).toBe(0);
      expect(paths[0].constraintZones[0].endWaypointIndex).toBe(1);
      expect(paths[0].constraintZones[0].maxVelocity).toBe(2.0);
      expect(paths[0].constraintZones[0].maxAcceleration).toBe(5.0);
    });

    it('parses rotation zones', () => {
      expect(paths[0].rotationZones).toHaveLength(1);
      expect(paths[0].rotationZones[0].startWaypointIndex).toBe(0);
      expect(paths[0].rotationZones[0].endWaypointIndex).toBe(2);
      expect(paths[0].rotationZones[0].targetPoint).toEqual({ x: 4.0, y: 3.0 });
    });
  });

  describe('scientific notation', () => {
    it('parses coordinates with scientific notation', () => {
      const java = `
public final class Paths {
    public static final PathData SCI = new PathData(
        List.of(
            new Translation2d(1.5e1, 2.0E-1),
            new Translation2d(3.0e+2, 4.5e0)
        ),
        List.of(),
        VelocityConstraints.defaults().withMaxVelocity(5.0).withMaxAcceleration(10.0),
        List.of(),
        List.of()
    );
}`;
      const paths = parsePathsJava(java);
      expect(paths[0].controlPoints[0].x).toBeCloseTo(15);
      expect(paths[0].controlPoints[0].y).toBeCloseTo(0.2);
      expect(paths[0].controlPoints[1].x).toBeCloseTo(300);
      expect(paths[0].controlPoints[1].y).toBeCloseTo(4.5);
    });
  });

  describe('comments are stripped', () => {
    it('handles single-line comments', () => {
      const java = `
public final class Paths {
    // This is a path
    public static final PathData P = new PathData(
        List.of(
            new Translation2d(1, 2), // inline comment
            new Translation2d(3, 4)
        ),
        List.of(),
        VelocityConstraints.defaults().withMaxVelocity(5.0).withMaxAcceleration(10.0),
        List.of(),
        List.of()
    );
}`;
      const paths = parsePathsJava(java);
      expect(paths[0].controlPoints).toHaveLength(2);
    });

    it('handles block comments', () => {
      const java = `
public final class Paths {
    /* multi
       line */
    public static final PathData P = new PathData(
        List.of(
            new Translation2d(1, 2),
            new Translation2d(3, 4)
        ),
        List.of(),
        VelocityConstraints.defaults().withMaxVelocity(5.0).withMaxAcceleration(10.0),
        List.of(),
        List.of()
    );
}`;
      const paths = parsePathsJava(java);
      expect(paths[0].controlPoints).toHaveLength(2);
    });
  });

  describe('multiple paths', () => {
    it('parses multiple PathData constants', () => {
      const java = `
public final class Paths {
    public static final PathData A = new PathData(
        List.of(new Translation2d(0, 0), new Translation2d(1, 1)),
        List.of(),
        VelocityConstraints.defaults().withMaxVelocity(5.0).withMaxAcceleration(10.0),
        List.of(),
        List.of()
    );
    public static final PathData B = new PathData(
        List.of(new Translation2d(2, 2), new Translation2d(3, 3)),
        List.of(),
        VelocityConstraints.defaults().withMaxVelocity(5.0).withMaxAcceleration(10.0),
        List.of(),
        List.of()
    );
}`;
      const paths = parsePathsJava(java);
      expect(paths).toHaveLength(2);
      expect(paths[0].name).toBe('A');
      expect(paths[1].name).toBe('B');
    });
  });

  describe('malformed input', () => {
    it('throws when no PathData found', () => {
      expect(() => parsePathsJava('public class Empty {}')).toThrow();
    });

    it('skips paths with fewer than 2 control points', () => {
      const java = `
public final class Paths {
    public static final PathData BAD = new PathData(
        List.of(new Translation2d(0, 0)),
        List.of(),
        VelocityConstraints.defaults().withMaxVelocity(5.0).withMaxAcceleration(10.0),
        List.of(),
        List.of()
    );
    public static final PathData GOOD = new PathData(
        List.of(new Translation2d(0, 0), new Translation2d(1, 1)),
        List.of(),
        VelocityConstraints.defaults().withMaxVelocity(5.0).withMaxAcceleration(10.0),
        List.of(),
        List.of()
    );
}`;
      const paths = parsePathsJava(java);
      expect(paths).toHaveLength(1);
      expect(paths[0].name).toBe('GOOD');
    });
  });
});
