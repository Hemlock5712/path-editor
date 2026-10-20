package frc.team5712.path.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.tools.FixturePaths;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

class FieldFrameTest {
  private static final RobotConfig ROBOT = RobotConfig.DEFAULT;

  @ParameterizedTest
  @EnumSource(FieldFrame.class)
  void everyFlipIsItsOwnInverse(FieldFrame frame) {
    Pose pose = new Pose(3.2, -1.7, 2.5);
    Pose back = frame.fromBlue(frame.toBlue(pose));
    assertEquals(pose.xMeters(), back.xMeters(), 0.0);
    assertEquals(pose.yMeters(), back.yMeters(), 0.0);
    assertEquals(0.0, Angles.wrap(pose.headingRadians() - back.headingRadians()), 1e-15);
    FieldSpeeds v = new FieldSpeeds(1.5, -0.4, 0.9);
    assertEquals(v, frame.fromBlue(frame.toBlue(v)));
    Vec2 p = new Vec2(-4, 2);
    assertEquals(p, frame.fromBlue(frame.toBlue(p)));
  }

  @ParameterizedTest
  @EnumSource(FieldFrame.class)
  void flipsMatchTheFieldSymmetry(FieldFrame frame) {
    Pose blue = new Pose(5.0, 2.0, Math.PI / 6);
    Pose red = frame.fromBlue(blue);
    switch (frame) {
      case BLUE -> assertEquals(blue, red);
      case RED_HALF_TURN -> {
        assertEquals(new Vec2(-5.0, -2.0), new Vec2(red.xMeters(), red.yMeters()));
        assertEquals(Math.PI / 6 - Math.PI, red.headingRadians(), 1e-15);
        assertEquals(new FieldSpeeds(-1, -2, 3), frame.fromBlue(new FieldSpeeds(1, 2, 3)));
      }
      case RED_MIRROR -> {
        assertEquals(new Vec2(-5.0, 2.0), new Vec2(red.xMeters(), red.yMeters()));
        assertEquals(5 * Math.PI / 6, red.headingRadians(), 1e-15);
        assertEquals(new FieldSpeeds(-1, 2, -3), frame.fromBlue(new FieldSpeeds(1, 2, 3)));
      }
    }
    assertEquals(frame == FieldFrame.RED_MIRROR, frame.mirrors());
  }

  /** The path as it would be authored on red: every point and angle flipped. Test-only. */
  static PathSpec flip(PathSpec spec, FieldFrame frame) {
    List<Waypoint> w = new ArrayList<>();
    for (Waypoint p : spec.waypoints()) {
      Vec2 q = frame.fromBlue(p.position());
      double t = p.hasTangent() ? frame.headingFromBlue(p.tangentRadians()) : Double.NaN;
      w.add(new Waypoint(q.x(), q.y(), t, p.tangentScale()));
    }
    List<RotationTarget> targets = new ArrayList<>();
    for (RotationTarget t : spec.rotationTargets()) {
      targets.add(new RotationTarget(t.waypoint(), t.meters(), frame.headingFromBlue(t.headingRadians())));
    }
    List<FacePointZone> zones = new ArrayList<>();
    for (FacePointZone z : spec.facePointZones()) {
      zones.add(new FacePointZone(z.fromWaypoint(), z.toWaypoint(), frame.fromBlue(z.point()), z.blendMeters()));
    }
    return new PathSpec(w, targets, zones, spec.constraintZones(), spec.maxMetersPerSecond(),
        spec.maxMetersPerSecondSquared(), spec.startMetersPerSecond(), spec.endMetersPerSecond());
  }

  @ParameterizedTest
  @EnumSource(value = FieldFrame.class, names = {"RED_HALF_TURN", "RED_MIRROR"})
  void theBluePlanIsTheRedPlan(FieldFrame frame) {
    // Building the red path from scratch gives the flipped blue path: same length, same |κ|,
    // same speeds and times. So converting frames each loop is exact, and nothing needs rebuilding.
    double sign = frame.mirrors() ? -1 : 1;
    for (PathSpec spec : FixturePaths.all().values()) {
      Path blue = Path.build(spec, ROBOT);
      Path red = Path.build(flip(spec, frame), ROBOT);
      assertEquals(blue.lengthMeters(), red.lengthMeters(), 1e-9);
      assertEquals(blue.totalTimeSeconds(), red.totalTimeSeconds(), 1e-9);
      for (double s = 0; s <= blue.lengthMeters(); s += 0.05) {
        PathSample b = blue.sample(s);
        PathSample r = red.sample(s);
        Pose expected = frame.fromBlue(b.pose());
        FieldSpeeds bs = frame.fromBlue(b.fieldSpeeds());
        FieldSpeeds rs = r.fieldSpeeds();
        close(expected.xMeters(), r.xMeters(), "x", s);
        close(expected.yMeters(), r.yMeters(), "y", s);
        close(0.0, Angles.wrap(expected.headingRadians() - r.headingRadians()), "heading", s);
        close(sign * b.curvature(), r.curvature(), "curvature", s);
        close(sign * b.headingRadiansPerMeter(), r.headingRadiansPerMeter(), "dθ/ds", s);
        close(bs.vxMetersPerSecond(), rs.vxMetersPerSecond(), "vx", s);
        close(bs.vyMetersPerSecond(), rs.vyMetersPerSecond(), "vy", s);
        close(bs.omegaRadPerSec(), rs.omegaRadPerSec(), "ω", s);
        // a is a difference of speeds over a 3 cm cell, which magnifies rounding about 100 times
        assertEquals(b.metersPerSecondSquared(), r.metersPerSecondSquared(), 1e-6, "a at s = " + s);
        close(b.seconds(), r.seconds(), "t", s);
      }
    }
  }

  /** Equal to a few parts in 10⁹: the two builds round differently, nothing more. */
  private static void close(double expected, double actual, String what, double s) {
    assertEquals(expected, actual, 1e-8 * Math.max(1.0, Math.abs(expected)), what + " at s = " + s);
  }

  /** A minimal kinematic follower: track the projected point with P on position and heading. */
  private static List<Pose> follow(Path path, FieldFrame frame, Pose start) {
    List<Pose> trace = new ArrayList<>();
    Pose pose = start;
    double s = path.geometry().projectGlobal(frame.toBlue(pose).xMeters(), frame.toBlue(pose).yMeters()).s();
    double dt = 0.02;
    for (int k = 0; k < 600; k++) {
      Pose blue = frame.toBlue(pose); // measured pose, into the path's frame
      s = path.geometry().project(blue.xMeters(), blue.yMeters(), s, 0.5).s();
      PathSample ref = path.sample(s);
      double v = Math.max(path.profile().velocityAt(s + 0.05), 0.2);
      double vx = v * ref.tangentX() + 3.0 * (ref.xMeters() - blue.xMeters());
      double vy = v * ref.tangentY() + 3.0 * (ref.yMeters() - blue.yMeters());
      double omega =
          v * ref.headingRadiansPerMeter() + 4.0 * Angles.wrap(ref.headingRadians() - blue.headingRadians());
      if (s >= path.lengthMeters() - 1e-6) {
        vx = vy = omega = 0;
      }
      FieldSpeeds out = frame.fromBlue(new FieldSpeeds(vx, vy, omega)); // back to the robot's frame
      pose =
          new Pose(
              pose.xMeters() + out.vxMetersPerSecond() * dt,
              pose.yMeters() + out.vyMetersPerSecond() * dt,
              Angles.wrap(pose.headingRadians() + out.omegaRadPerSec() * dt));
      trace.add(pose);
    }
    return trace;
  }

  @ParameterizedTest
  @EnumSource(value = FieldFrame.class, names = {"RED_HALF_TURN", "RED_MIRROR"})
  void followingTheBluePathOnRedDrivesTheRedPath(FieldFrame frame) {
    PathSpec spec = FixturePaths.autoRoute();
    Path blue = Path.build(spec, ROBOT);
    Path red = Path.build(flip(spec, frame), ROBOT);
    // start 10 cm off the path and 5° off heading, on the red side
    Pose redStart = frame.fromBlue(new Pose(-5.2, -2.4, 0.09));

    List<Pose> viaBlue = follow(blue, frame, redStart); // the robot code: convert, follow blue
    List<Pose> native_ = follow(red, FieldFrame.BLUE, redStart); // a red-built path, no conversion
    double worst = 0;
    for (int k = 0; k < viaBlue.size(); k++) {
      Pose a = viaBlue.get(k);
      Pose b = native_.get(k);
      worst = Math.max(worst, Math.hypot(a.xMeters() - b.xMeters(), a.yMeters() - b.yMeters()));
      assertEquals(0.0, Angles.wrap(a.headingRadians() - b.headingRadians()), 1e-6);
    }
    assertTrue(worst < 1e-6, "worst position difference " + worst);
    Pose end = viaBlue.get(viaBlue.size() - 1);
    Pose redEnd = frame.fromBlue(new Pose(-5.44, 3.0, Math.PI));
    assertEquals(redEnd.xMeters(), end.xMeters(), 0.05);
    assertEquals(redEnd.yMeters(), end.yMeters(), 0.05);
  }
}
