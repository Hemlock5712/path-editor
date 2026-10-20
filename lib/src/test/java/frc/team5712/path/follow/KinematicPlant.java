package frc.team5712.path.follow;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Pose;

/**
 * The simplest closed loop for tests: field velocity follows the command one loop late (the
 * latency the follower allows for), and the pose integrates it. Deterministic.
 */
final class KinematicPlant {
  double x;
  double y;
  double heading;
  double vx;
  double vy;
  double omega;
  private FieldSpeeds pending = new FieldSpeeds(0, 0, 0);

  KinematicPlant(Pose start, FieldSpeeds velocity) {
    x = start.xMeters();
    y = start.yMeters();
    heading = start.headingRadians();
    vx = velocity.vxMetersPerSecond();
    vy = velocity.vyMetersPerSecond();
    omega = velocity.omegaRadPerSec();
    pending = velocity;
  }

  Pose pose() {
    return new Pose(x, y, heading);
  }

  FieldSpeeds velocity() {
    return new FieldSpeeds(vx, vy, omega);
  }

  /** Applies the command sent last loop, integrates dt, and queues this loop's command. */
  void step(FieldSpeeds command, double dt) {
    vx = pending.vxMetersPerSecond();
    vy = pending.vyMetersPerSecond();
    omega = pending.omegaRadPerSec();
    x += vx * dt;
    y += vy * dt;
    heading += omega * dt;
    pending = command;
  }

  /** Shoves the robot sideways: moves it without touching its velocity. */
  void displace(double dx, double dy) {
    x += dx;
    y += dy;
  }
}
