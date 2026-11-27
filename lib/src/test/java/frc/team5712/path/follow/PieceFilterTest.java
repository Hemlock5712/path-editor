package frc.team5712.path.follow;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.Vec2;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import org.junit.jupiter.api.Test;

/**
 * The game-piece filter, and the experiment behind it: the intake assist driven at pieces seen by a simulated
 * camera (angles good to 0.3°, a photo every 33 ms that is 40 ms old, a fifth of detections missing, an
 * occasional false one), once with the newest photo used as it comes and once through the filter.
 */
class PieceFilterTest {
  private static final FollowerConfig CFG = FollowerConfig.defaults(RobotConfig.DEFAULT);
  private static final double DT = 0.02;
  private static final double MAX = 4.5;
  // Precision-Drive's piece camera: 0.30 m ahead of centre, lens 0.45 m up, pieces' centres 0.12 m up
  private static final Vec2 CAMERA = new Vec2(0.30, 0.0);
  private static final double DROP = 0.45 - 0.12;
  private static final double INTAKE_AHEAD = 0.40;
  private static final double ANGLE_NOISE = Math.toRadians(0.3);

  /** What the camera reports for the pieces, from {@code robot}: noisy, sometimes missing, sometimes false. */
  private static List<Vec2> photo(Pose robot, List<Vec2> pieces, Random r, double dropout, double falseRate) {
    List<Vec2> seen = new ArrayList<>();
    for (Vec2 piece : pieces) {
      Pose rel = TagFilter.minus(new Pose(piece.x(), piece.y(), 0.0), robot);
      double sx = rel.xMeters() - CAMERA.x();
      double sy = rel.yMeters() - CAMERA.y();
      double range = Math.hypot(sx, sy);
      double bearing = Math.atan2(sy, sx);
      if (sx < 0.29 || range > 3.5 || Math.abs(bearing) > Math.toRadians(30) || r.nextDouble() < dropout) {
        continue; // under the bumper, too far, out of view, or missed this time
      }
      double down = Math.atan2(DROP, range) + ANGLE_NOISE * r.nextGaussian();
      double seenRange = DROP / Math.tan(Math.max(down, Math.toRadians(2.0)));
      double seenBearing = bearing + ANGLE_NOISE * r.nextGaussian();
      seen.add(new Vec2(CAMERA.x() + seenRange * Math.cos(seenBearing), CAMERA.y() + seenRange * Math.sin(seenBearing)));
    }
    if (r.nextDouble() < falseRate) {
      double range = 0.5 + 2.5 * r.nextDouble();
      double bearing = Math.toRadians(25) * (2 * r.nextDouble() - 1);
      seen.add(new Vec2(CAMERA.x() + range * Math.cos(bearing), CAMERA.y() + range * Math.sin(bearing)));
    }
    return seen;
  }

  private record Outcome(double missMeters, double jitter, int switches) {}

  /**
   * Full stick straight down the field at {@code speed}; the assist nudges the intake toward whichever piece
   * it picks. {@code filtered}: through {@link PieceFilter} and the sticky choice; otherwise the newest photo
   * as it comes and the plain choice, as Precision-Drive's PieceTracker did.
   */
  private static Outcome drive(List<Vec2> pieces, double speed, boolean filtered, long seed, double dropout,
      double falseRate) {
    Random r = new Random(seed);
    RailFollower f = new RailFollower(CFG);
    KinematicPlant plant = new KinematicPlant(new Pose(0, 0, 0), new FieldSpeeds(speed, 0, 0));
    f.reset(plant.velocity());
    FieldSpeeds stick = new FieldSpeeds(speed, 0, 0);
    PieceFilter filter = new PieceFilter(ANGLE_NOISE);
    List<Vec2> newest = List.of();
    double newestAt = Double.NEGATIVE_INFINITY;
    List<Pose> history = new ArrayList<>();
    Optional<Vec2> chosen = Optional.empty();
    double nextPhoto = 0.0;
    double lastVy = 0.0;
    double jitter = 0.0;
    int steps = 0;
    int switches = 0;
    double miss = Double.NaN;
    Vec2 target = pieces.get(0);
    for (int i = 0; i < 400; i++) {
      double t = i * DT;
      history.add(plant.pose());
      if (t >= nextPhoto) {
        nextPhoto += 1.0 / 30.0;
        Pose captured = history.get(Math.max(0, history.size() - 3)); // 40 ms ago
        List<Vec2> seen = photo(captured, pieces, r, dropout, falseRate);
        filter.add(seen, CAMERA, DROP, captured, t);
        if (!seen.isEmpty()) {
          List<Vec2> placed = new ArrayList<>();
          for (Vec2 p : seen) {
            Pose on = TagFilter.plus(captured, new Pose(p.x(), p.y(), 0.0));
            placed.add(new Vec2(on.xMeters(), on.yMeters()));
          }
          newest = placed;
          newestAt = t;
        }
      }
      List<Vec2> known = filtered ? filter.pieces(t) : (t - newestAt > 0.5 ? List.<Vec2>of() : newest);
      Pose robot = plant.pose();
      Pose intake = TagFilter.plus(robot, new Pose(INTAKE_AHEAD, 0.0, 0.0));
      Vec2 at = new Vec2(intake.xMeters(), intake.yMeters());
      Optional<Vec2> pick = filtered ? PieceAssist.choose(known, at, stick, chosen) : PieceAssist.choose(known, at, stick);
      if (pick.isPresent() && chosen.isPresent()
          && Math.hypot(pick.get().x() - chosen.get().x(), pick.get().y() - chosen.get().y()) > 0.25) {
        switches++;
      }
      chosen = pick;
      FieldSpeeds cmd = pick.isEmpty() ? f.step(stick, DT)
          : f.step(PieceAssist.railThrough(pick.get(), stick), intake, stick, MAX, DT, PieceAssist.maxCrossSpeed(stick));
      jitter += Math.pow((cmd.vyMetersPerSecond() - lastVy) / DT, 2);
      steps++;
      lastVy = cmd.vyMetersPerSecond();
      double before = intake.xMeters();
      plant.step(cmd, DT);
      double after = plant.x + INTAKE_AHEAD;
      if (Double.isNaN(miss) && before < target.x() && after >= target.x()) {
        // the nearest piece to the intake as it passes: the one it took, or the gap it went through
        miss = Double.POSITIVE_INFINITY;
        for (Vec2 p : pieces) {
          miss = Math.min(miss, Math.abs(plant.y - p.y()));
        }
        break;
      }
    }
    return new Outcome(miss, Math.sqrt(jitter / Math.max(1, steps)), switches);
  }

  private static double[] average(List<Vec2> pieces, double speed, boolean filtered, double dropout, double falseRate) {
    int n = 300;
    double miss = 0;
    double worst = 0;
    double jitter = 0;
    double switches = 0;
    int taken = 0;
    for (int seed = 0; seed < n; seed++) {
      Outcome o = drive(pieces, speed, filtered, seed, dropout, falseRate);
      miss += o.missMeters();
      worst = Math.max(worst, o.missMeters());
      jitter += o.jitter();
      switches += o.switches();
      if (o.missMeters() < 0.05) {
        taken++;
      }
    }
    return new double[] {100 * miss / n, 100 * worst, jitter / n, switches / n, 100.0 * taken / n};
  }

  /** The experiment: prints the table in bench/RESULTS.md, and holds the filter to what it showed. */
  @Test
  void theFilterAgainstTheNewestPhoto() {
    List<Vec2> one = List.of(new Vec2(3.0, 0.25));
    List<Vec2> oblique = List.of(new Vec2(2.6, 0.9));
    List<Vec2> pair = List.of(new Vec2(3.0, 0.28), new Vec2(3.15, -0.30));
    Object[][] cases = {
      {"one piece, 25 cm to the side", one, 3.0, 0.2, 0.05},
      {"one piece, well off to the side", oblique, 3.0, 0.2, 0.05},
      {"one piece, clean camera", one, 3.0, 0.0, 0.0},
      {"one piece, half the detections missing", one, 3.0, 0.5, 0.05},
      {"one piece, a false detection in 1 photo of 4", one, 3.0, 0.2, 0.25},
      {"two pieces about equally in line", pair, 3.0, 0.2, 0.05},
      {"two pieces, slower (1.5 m/s)", pair, 1.5, 0.2, 0.05},
    };
    System.out.println("    intake assist, 300 runs each: mean miss cm / worst miss cm / sideways jitter m/s² rms / "
        + "piece switches per run / taken within 5 cm %");
    double rawTaken = 0;
    double filteredTaken = 0;
    for (Object[] c : cases) {
      @SuppressWarnings("unchecked")
      List<Vec2> pieces = (List<Vec2>) c[1];
      double[] raw = average(pieces, (double) c[2], false, (double) c[3], (double) c[4]);
      double[] fil = average(pieces, (double) c[2], true, (double) c[3], (double) c[4]);
      System.out.printf("    %-46s newest photo %5.1f / %5.1f / %5.2f / %4.1f / %3.0f%%   filtered %5.1f / %5.1f / %5.2f / %4.1f / %3.0f%%%n",
          c[0], raw[0], raw[1], raw[2], raw[3], raw[4], fil[0], fil[1], fil[2], fil[3], fil[4]);
      rawTaken += raw[4];
      filteredTaken += fil[4];
      assertTrue(fil[4] >= raw[4] - 2.0, c[0] + ": the filter takes the piece at least as often");
    }
    assertTrue(filteredTaken > rawTaken, "and more often over all the cases");
  }

  @Test
  void aPieceCountsOnceSeenTwiceAndIsForgottenAfterHalfASecond() {
    PieceFilter f = new PieceFilter();
    Pose robot = new Pose(1.0, 2.0, 0.5);
    f.add(List.of(new Vec2(1.5, 0.2)), CAMERA, DROP, robot, 0.0);
    assertTrue(f.pieces(0.0).isEmpty(), "one sighting could be a stray detection");
    f.add(List.of(new Vec2(1.5, 0.2)), CAMERA, DROP, robot, 0.03);
    assertEquals(1, f.pieces(0.03).size());
    Pose placed = TagFilter.plus(robot, new Pose(1.5, 0.2, 0.0));
    assertEquals(placed.xMeters(), f.pieces(0.03).get(0).x(), 1e-9);
    assertEquals(placed.yMeters(), f.pieces(0.03).get(0).y(), 1e-9);
    assertEquals(1, f.pieces(0.5).size(), "still remembered");
    assertTrue(f.pieces(0.6).isEmpty(), "forgotten 0.5 s after it was last seen");
  }

  @Test
  void twoPiecesStayTwoAndAVisionCorrectionMovesThemWithTheRobot() {
    PieceFilter f = new PieceFilter();
    Pose robot = new Pose(0.0, 0.0, 0.0);
    for (int i = 0; i < 3; i++) {
      // the second photo misses one of them: it keeps its place
      f.add(i == 1 ? List.of(new Vec2(2.0, 0.4)) : List.of(new Vec2(2.0, 0.4), new Vec2(1.8, -0.5)), CAMERA, DROP,
          robot, 0.03 * i);
    }
    assertEquals(2, f.pieces(0.1).size());
    // the field pose says the robot is elsewhere: the pieces are as far from it as they were
    Pose field = new Pose(5.0, 3.0, Math.PI / 2);
    List<Vec2> onField = f.fieldPieces(robot, field, 0.1);
    assertEquals(2, onField.size());
    assertEquals(5.0 - 0.4, onField.get(0).x(), 1e-9);
    assertEquals(3.0 + 2.0, onField.get(0).y(), 1e-9);
  }

  /** Far away the range is a guess and the side is sharp: a later, nearer sighting corrects the range. */
  @Test
  void aNearSightingOutweighsFarOnesInRange() {
    PieceFilter f = new PieceFilter();
    for (int i = 0; i < 5; i++) {
      f.add(List.of(new Vec2(3.2, 0.3)), CAMERA, DROP, new Pose(0, 0, 0), 0.03 * i); // from 3 m: 20 cm long
    }
    f.add(List.of(new Vec2(1.0, 0.3)), CAMERA, DROP, new Pose(2.0, 0, 0), 0.2); // from 1 m: it is at x = 3.0
    Vec2 est = f.pieces(0.2).get(0);
    assertEquals(3.0, est.x(), 0.02, "one near sighting pulled the range almost all the way");
    assertEquals(0.3, est.y(), 0.005);
  }

  @Test
  void theChoiceStaysWithThePieceBeingChased() {
    Vec2 intake = new Vec2(0, 0);
    FieldSpeeds ahead = new FieldSpeeds(3.0, 0, 0);
    Vec2 left = new Vec2(2.0, 0.21);
    Vec2 right = new Vec2(2.0, -0.20); // a hair more in line
    assertEquals(Optional.of(right), PieceAssist.choose(List.of(left, right), intake, ahead));
    assertEquals(Optional.of(left), PieceAssist.choose(List.of(left, right), intake, ahead, Optional.of(left)),
        "already chasing the left one: a hair isn't enough to switch");
    Vec2 farRight = new Vec2(2.0, -0.02);
    assertEquals(Optional.of(farRight),
        PieceAssist.choose(List.of(new Vec2(2.0, 0.6), farRight), intake, ahead, Optional.of(new Vec2(2.0, 0.6))),
        "one much more in line does take over");
    assertEquals(Optional.of(right), PieceAssist.choose(List.of(right), intake, ahead, Optional.of(left)),
        "and so does any piece once the old one is gone");
  }
}
