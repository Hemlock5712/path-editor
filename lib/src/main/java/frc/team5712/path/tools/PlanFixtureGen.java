package frc.team5712.path.tools;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Json;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.plan.ConvexPolygon;
import frc.team5712.path.plan.FieldMap;
import frc.team5712.path.plan.Pathfinder;
import frc.team5712.path.plan.PlanResult;
import frc.team5712.path.plan.PlannerConfig;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

/**
 * Golden plans for the TypeScript pathfinder ({@code editor/core/src/plan}): {@code fixtures/plan/plans.json},
 * the 2026 field ({@code fixtures/plan/field_2026.json}) with the default robot and planner settings, the grown
 * obstacles and graph size, and for each start and goal: the waypoints, the planner's figures and the path
 * sampled every 0.25 m. Written by {@code lib/gradlew -p lib fixtures} (through {@link FixtureGen}).
 */
public final class PlanFixtureGen {
  public static final String FIELD = "plan/field_2026.json";
  public static final String OUT = "plan/plans.json";

  private PlanFixtureGen() {}

  /** One query: start pose, its field velocity, goal pose, and the end speed and direction (NaN = free). */
  public record Case(String name, Pose start, FieldSpeeds velocity, Pose goal, double endSpeed, double endTangent) {}

  public static List<Case> cases(Pathfinder pf) {
    List<Case> out = new ArrayList<>();
    Random r = new Random(2027);
    FieldSpeeds still = new FieldSpeeds(0, 0, 0);
    while (out.size() < 8) {
      Pose s = random(pf, r);
      Pose g = random(pf, r);
      if (Math.hypot(g.xMeters() - s.xMeters(), g.yMeters() - s.yMeters()) > 1.0) {
        out.add(new Case("random " + out.size(), s, still, g, 0.0, Double.NaN));
      }
    }
    out.add(new Case("moving start", new Pose(-6.0, -1.5, 0.3), new FieldSpeeds(1.2, -1.5, 0.5),
        new Pose(1.0, -1.5, Math.PI / 2), 0.0, Double.NaN));
    out.add(new Case("start on the hub", new Pose(-3.4, 0.3, 0.0), still, new Pose(-6.0, 2.0, 0.0), 0.0, Double.NaN));
    out.add(new Case("goal on a trench support", new Pose(-6.5, -2.0, 0.0), still, new Pose(-3.6, 2.6, Math.PI), 0.0,
        Double.NaN));
    out.add(new Case("hand-over", new Pose(-6.0, 2.5, 0.0), still, new Pose(-1.4, -2.2, Math.PI / 2), 1.5, Math.PI / 2));
    out.add(new Case("across the field", new Pose(-7.0, -3.0, 0.0), still, new Pose(7.0, 3.0, Math.PI), 0.0, Double.NaN));
    return out;
  }

  private static Pose random(Pathfinder pf, Random r) {
    while (true) {
      double x = (r.nextDouble() - 0.5) * 15.6;
      double y = (r.nextDouble() - 0.5) * 7.2;
      double h = (r.nextDouble() * 2 - 1) * Math.PI;
      if (pf.isFree(x, y) && pf.clearance(x, y, h) > 0.15) {
        return new Pose(x, y, h);
      }
    }
  }

  public static String fixture(java.nio.file.Path fixtures) throws IOException {
    FieldMap field = FieldMap.read(fixtures.resolve(FIELD));
    Pathfinder pf = new Pathfinder(field, RobotConfig.DEFAULT, PlannerConfig.DEFAULT);
    Map<String, Object> root = new LinkedHashMap<>();
    root.put("field", FIELD);
    List<Object> grown = new ArrayList<>();
    for (ConvexPolygon p : pf.grownObstacles()) {
      List<Object> pts = new ArrayList<>();
      for (int i = 0; i < p.size(); i++) {
        pts.add(List.of(p.x(i), p.y(i)));
      }
      grown.add(pts);
    }
    root.put("grown", grown);
    root.put("nodeCount", pf.nodeCount());
    List<Object> plans = new ArrayList<>();
    for (Case c : cases(pf)) {
      PlanResult r = pf.plan(c.start(), c.velocity(), c.goal(), c.endSpeed(), c.endTangent());
      Map<String, Object> m = new LinkedHashMap<>();
      m.put("name", c.name());
      m.put("start", List.of(c.start().xMeters(), c.start().yMeters(), c.start().headingRadians()));
      m.put("velocity", List.of(c.velocity().vxMetersPerSecond(), c.velocity().vyMetersPerSecond(),
          c.velocity().omegaRadPerSec()));
      m.put("goal", List.of(c.goal().xMeters(), c.goal().yMeters(), c.goal().headingRadians()));
      m.put("endSpeed", c.endSpeed());
      if (Double.isFinite(c.endTangent())) {
        m.put("endTangent", c.endTangent());
      }
      m.put("ok", r.ok());
      List<Object> wps = new ArrayList<>();
      for (double[] w : r.waypoints()) {
        wps.add(List.of(w[0], w[1]));
      }
      m.put("waypoints", wps);
      m.put("routeLengthMeters", r.routeLengthMeters());
      m.put("straightLineMeters", r.straightLineMeters());
      m.put("startSnapMeters", r.startSnapMeters());
      m.put("goalSnapMeters", r.goalSnapMeters());
      m.put("refits", r.refits());
      m.put("clear", r.clear());
      m.put("minClearanceMeters", r.minClearanceMeters());
      Path path = r.path();
      m.put("lengthMeters", path.lengthMeters());
      m.put("totalTimeSeconds", path.totalTimeSeconds());
      List<Object> samples = new ArrayList<>();
      int n = (int) Math.ceil(path.lengthMeters() / 0.25);
      for (int i = 0; i <= n; i++) {
        PathSample p = path.sample(Math.min(path.lengthMeters(), i * 0.25));
        samples.add(List.of(p.s(), p.xMeters(), p.yMeters(), p.headingRadians(), p.metersPerSecond(), p.seconds()));
      }
      m.put("samples", samples);
      plans.add(m);
    }
    root.put("plans", plans);
    return Json.write(root, Json::exact);
  }

  public static void write(java.nio.file.Path fixtures) throws IOException {
    java.nio.file.Path file = fixtures.resolve(OUT);
    Files.writeString(file, fixture(fixtures), StandardCharsets.UTF_8);
    System.out.println("wrote " + file);
  }
}
