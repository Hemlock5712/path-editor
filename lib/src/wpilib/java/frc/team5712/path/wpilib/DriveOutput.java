package frc.team5712.path.wpilib;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.follow.WheelForces;
import java.util.function.Consumer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Where the follow commands send each loop's command: a velocity alone, or, once {@link #withWheelForces} has
 * been called, the velocity and the wheel forces that go with it.
 */
final class DriveOutput {
  private final Consumer<ChassisVelocities> velocityOnly;
  private WheelForces forces;
  private WheelForceDrive forceDrive;

  DriveOutput(Consumer<ChassisVelocities> velocityOnly) {
    this.velocityOnly = velocityOnly;
  }

  void withWheelForces(RobotConfig robot, Translation2d[] moduleLocations, WheelForceDrive drive) {
    double[] x = new double[moduleLocations.length];
    double[] y = new double[moduleLocations.length];
    for (int i = 0; i < moduleLocations.length; i++) {
      x[i] = moduleLocations[i].getX();
      y[i] = moduleLocations[i].getY();
    }
    this.forces = new WheelForces(robot, x, y);
    this.forceDrive = drive;
  }

  /** Sends {@code velocity}, with the wheel forces for {@code acceleration} if the drivetrain takes them. */
  void send(FieldSpeeds velocity, FieldSpeeds acceleration, Rotation2d heading) {
    ChassisVelocities v = Conversions.toChassisVelocities(velocity);
    if (forceDrive == null) {
      velocityOnly.accept(v);
      return;
    }
    double[][] f = forces.forAcceleration(acceleration, heading.getRadians());
    forceDrive.drive(v, f[0], f[1]);
  }

  /** Sends zero velocity (and no push). */
  void stop() {
    if (forceDrive == null) {
      velocityOnly.accept(new ChassisVelocities());
      return;
    }
    double[][] f = forces.none();
    forceDrive.drive(new ChassisVelocities(), f[0], f[1]);
  }
}
