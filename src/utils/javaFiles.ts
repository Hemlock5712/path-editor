export interface JavaFileInfo {
  filename: string;
  relativePath: string;
  packagePath: string;
  description: string;
  category: 'core' | 'command' | 'dependency';
  lineCount: number;
  private?: boolean;
}

export const JAVA_FILES: JavaFileInfo[] = [
  // Core path utilities
  {
    filename: 'SplinePath.java',
    relativePath: 'utils/path/SplinePath.java',
    packagePath: 'frc/robot/utils/path/',
    description:
      'Arc-length parameterized Catmull-Rom spline with closest-point projection',
    category: 'core',
    lineCount: 437,
  },
  {
    filename: 'CubicSegment.java',
    relativePath: 'utils/path/CubicSegment.java',
    packagePath: 'frc/robot/utils/path/',
    description:
      'Individual cubic Hermite spline segment with curvature computation',
    category: 'core',
    lineCount: 122,
  },
  {
    filename: 'VelocityProfile.java',
    relativePath: 'utils/path/VelocityProfile.java',
    packagePath: 'frc/robot/utils/path/',
    description:
      'Forward-backward pass velocity profiling with motor torque and friction limits',
    category: 'core',
    lineCount: 208,
  },
  {
    filename: 'VelocityConstraints.java',
    relativePath: 'utils/path/VelocityConstraints.java',
    packagePath: 'frc/robot/utils/path/',
    description:
      'Builder-pattern configuration for path velocity and acceleration limits',
    category: 'core',
    lineCount: 129,
  },
  {
    filename: 'RotationSupplier.java',
    relativePath: 'utils/path/RotationSupplier.java',
    packagePath: 'frc/robot/utils/path/',
    description: 'Functional interface for path-aware rotation strategies',
    category: 'core',
    lineCount: 24,
  },
  {
    filename: 'RotationSuppliers.java',
    relativePath: 'utils/path/RotationSuppliers.java',
    packagePath: 'frc/robot/utils/path/',
    description:
      'Factory methods for heading strategies: face forward, face point, hold, interpolate, zone-based',
    category: 'core',
    lineCount: 260,
  },
  {
    filename: 'PathData.java',
    relativePath: 'utils/path/PathData.java',
    packagePath: 'frc/robot/utils/path/',
    description:
      'Data model record with red alliance mirroring for control points, headings, constraints, and rotation zones',
    category: 'core',
    lineCount: 104,
  },
  {
    filename: 'ProjectionResult.java',
    relativePath: 'utils/path/ProjectionResult.java',
    packagePath: 'frc/robot/utils/path/',
    description: 'Result record for projecting robot position onto the path',
    category: 'core',
    lineCount: 15,
  },

  // Command
  {
    filename: 'FollowPath.java',
    relativePath: 'commands/FollowPath.java',
    packagePath: 'frc/robot/commands/',
    description:
      'Main path-following command with adaptive lookahead and cross-track correction',
    category: 'command',
    lineCount: 548,
  },

  // Dependencies
  {
    filename: 'AccelerationLimiter.java',
    relativePath: 'commands/AccelerationLimiter.java',
    packagePath: 'frc/robot/commands/',
    description:
      'Physics-based friction circle, motor torque, and jerk limiting',
    category: 'dependency',
    lineCount: 250,
    private: true,
  },
  {
    filename: 'Motor.java',
    relativePath: 'utils/Motor.java',
    packagePath: 'frc/robot/utils/',
    description:
      'Motor torque-speed curves from dyno data (Kraken X60, Falcon 500, etc.)',
    category: 'dependency',
    lineCount: 100,
    private: true,
  },
];
