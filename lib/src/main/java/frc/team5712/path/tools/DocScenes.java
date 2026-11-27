package frc.team5712.path.tools;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathFile;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.RobotConfig;
import frc.team5712.path.core.Vec2;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.follow.PathFollower;
import frc.team5712.path.follow.PieceAssist;
import frc.team5712.path.follow.Rail;
import frc.team5712.path.follow.RailFollower;
import frc.team5712.path.sim.SimDrivetrain;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

/**
 * Writes the documentation site's driver-assist animations (docs/src/data/assists.json): the rail, the intake
 * assist and a goal that moves, each run with the library's own classes on the simulated drivetrain. These have
 * no TypeScript port, so the site cannot work them out when it is built as it does its other animations
 * (docs/src/lib/scenes.ts, whose shapes this file writes).
 *
 * <pre>
 * lib/gradlew -p lib jar
 * java -cp lib/build/libs/pathcore.jar frc.team5712.path.tools.DocScenes docs/src/data/assists.json
 * </pre>
 */
public final class DocScenes {
  private static final RobotConfig ROBOT = RobotConfig.DEFAULT;
  private static final FollowerConfig CFG = FollowerConfig.defaults(ROBOT);
  private static final double DT = 0.02;
  private static final double BUMPER = 0.85;

  private DocScenes() {}

  /** A run: the robot's pose every loop. */
  private static final class Run {
    final List<double[]> poses = new ArrayList<>();
    double peakAcross;

    void add(Pose p) {
      poses.add(new double[] {p.xMeters(), p.yMeters(), p.headingRadians()});
    }

    double seconds() {
      return (poses.size() - 1) * DT;
    }

    /** As a track for the site's player, at 25 samples a second. */
    String track(String kind, String label) {
      StringBuilder x = new StringBuilder();
      StringBuilder y = new StringBuilder();
      StringBuilder h = new StringBuilder();
      for (int i = 0; i < poses.size(); i += 2) {
        String sep = i == 0 ? "" : ",";
        x.append(sep).append(num(poses.get(i)[0]));
        y.append(sep).append(num(poses.get(i)[1]));
        h.append(sep).append(num(poses.get(i)[2]));
      }
      return "{\"kind\":\"" + kind + "\"" + (label == null ? "" : ",\"label\":\"" + label + "\"") + ",\"dt\":0.04,\"x\":[" + x
          + "],\"y\":[" + y + "],\"h\":[" + h + "]}";
    }
  }

  private static String num(double v) {
    return String.format(Locale.ROOT, "%.3f", v).replaceAll("0+$", "").replaceAll("\\.$", "");
  }

  private static String two(double v) {
    return String.format(Locale.ROOT, "%.2f", v);
  }

  private static String cm(double meters) {
    return String.format(Locale.ROOT, "%.1f", meters * 100);
  }

  private static String fact(String label, String value) {
    return "{\"label\":\"" + label + "\",\"value\":\"" + value + "\"}";
  }

  private static String step(double at, String text) {
    return "{\"at\":" + num(at) + ",\"text\":\"" + text + "\"}";
  }

  private static String variant(String label, double seconds, String lines, String marks, String tracks, String steps,
      String facts) {
    return "{\"label\":\"" + label + "\",\"seconds\":" + num(seconds) + ",\"driveAt\":0,\"lines\":[" + lines
        + "],\"shapes\":[],\"marks\":[" + marks + "],\"tracks\":[" + tracks + "]"
        + (steps == null ? "" : ",\"steps\":[" + steps + "]") + ",\"facts\":[" + facts + "]}";
  }

  private static String scene(String view, double speedMax, List<String> variants) {
    return "{\"view\":" + view + ",\"robot\":[" + BUMPER + "," + BUMPER + "],\"speedMax\":" + num(speedMax)
        + ",\"variants\":[" + String.join(",", variants) + "]}";
  }

  // ------------------------------------------------------------------------------------------------ rail

  /** The driver holds {@code stick} for {@code seconds}, on {@code rail} (null: plain teleop). */
  private static Run onRail(Rail rail, Pose start, FieldSpeeds stick, double maxSpeed, double seconds) {
    RailFollower f = new RailFollower(CFG);
    SimDrivetrain plant = new SimDrivetrain(start, new FieldSpeeds(0, 0, 0), SimDrivetrain.DEFAULT_LATENCY_SECONDS,
        SimDrivetrain.DEFAULT_LAG_SECONDS, SimDrivetrain.DEFAULT_SUBSTEP_SECONDS);
    f.reset(plant.velocity());
    Run run = new Run();
    run.add(plant.pose());
    for (int i = 0; i * DT < seconds; i++) {
      FieldSpeeds cmd = rail == null ? f.step(stick, DT) : f.step(rail, plant.pose(), stick, maxSpeed, DT);
      plant.command(cmd);
      plant.advanceTo((i + 1) * DT);
      run.add(plant.pose());
      // the rail in these scenes runs along x, so "across" is y
      run.peakAcross = Math.max(run.peakAcross, Math.abs(plant.velocity().vyMetersPerSecond()));
    }
    return run;
  }

  private static String railScene() {
    double maxSpeed = 4.5;
    Pose start = new Pose(-3.6, 1.2, 0.0);
    FieldSpeeds stick = new FieldSpeeds(3.0, 0.0, 0.0);
    String railLine = "{\"pts\":[[-4.4,0],[1.2,0],[4.4,0]],\"style\":\"rail\",\"label\":\"rail\"}";
    Run alone = onRail(null, start, stick, maxSpeed, 2.3);
    List<String> variants = new ArrayList<>();
    double[] shares = {0.2, 0.5, 1.0, Double.POSITIVE_INFINITY};
    for (double share : shares) {
      Rail rail = Rail.through(0.0, 0.0, 0.0).withHeading(0.0);
      if (Double.isFinite(share)) {
        rail = rail.withPullIn(share);
      }
      Run run = onRail(rail, start, stick, maxSpeed, 2.3);
      double joined = Double.NaN;
      for (double[] p : run.poses) {
        if (Math.abs(p[1]) < 0.02) {
          joined = p[0] - start.xMeters();
          break;
        }
      }
      String label = Double.isFinite(share) ? "Pull-in " + num(share) : "No pull-in set";
      variants.add(variant(label, run.seconds(), railLine, "",
          alone.track("ghost", "driver alone") + "," + run.track("robot", null),
          step(0.0, "The driver pushes straight along the field at 3 m/s, starting 1.2 m off the rail.") + ","
              + step(0.25, "The rail pulls the robot sideways onto the line and squares it up. The driver still sets the speed along it."),
          fact("Peak speed across", two(run.peakAcross) + " m/s") + ","
              + fact("On the line after", Double.isNaN(joined) ? "not yet" : two(joined) + " m")));
    }
    // on the line already: a push across it goes nowhere
    Rail rail = Rail.through(0.0, 0.0, 0.0).withHeading(0.0);
    Pose onIt = new Pose(-3.6, 0.0, 0.0);
    FieldSpeeds diagonal = new FieldSpeeds(2.2, 2.2, 0.0);
    Run held = onRail(rail, onIt, diagonal, maxSpeed, 2.3);
    Run free = onRail(null, onIt, diagonal, maxSpeed, 2.3);
    variants.add(variant("A push across", held.seconds(), railLine, "",
        free.track("ghost", "driver alone") + "," + held.track("robot", null),
        step(0.0, "On the rail, the driver pushes the stick diagonally: 2.2 m/s along and 2.2 m/s across.") + ","
            + step(0.3, "The part along the rail drives the robot. The part across it does nothing."),
        fact("Stick across", "2.2 m/s") + "," + fact("Robot across", two(held.peakAcross) + " m/s")));
    return scene("[-4.4,-1.0,4.4,2.1]", 4.0, variants);
  }

  // ----------------------------------------------------------------------------------------------- piece

  private static Vec2 intakeOf(Pose p, double ahead) {
    return new Vec2(p.xMeters() + ahead * Math.cos(p.headingRadians()), p.yMeters() + ahead * Math.sin(p.headingRadians()));
  }

  private static Run towardPiece(List<Vec2> pieces, Pose start, FieldSpeeds stick, boolean assist, double eagerness,
      double untilX) {
    RailFollower f = new RailFollower(CFG);
    SimDrivetrain plant = new SimDrivetrain(start, new FieldSpeeds(0, 0, 0), SimDrivetrain.DEFAULT_LATENCY_SECONDS,
        SimDrivetrain.DEFAULT_LAG_SECONDS, SimDrivetrain.DEFAULT_SUBSTEP_SECONDS);
    f.reset(plant.velocity());
    Run run = new Run();
    run.add(plant.pose());
    Optional<Vec2> chosen = Optional.empty();
    for (int i = 0; i < 400; i++) {
      Vec2 intake = intakeOf(plant.pose(), BUMPER / 2);
      FieldSpeeds cmd;
      chosen = assist ? PieceAssist.choose(pieces, intake, stick, chosen) : Optional.empty();
      if (chosen.isPresent()) {
        cmd = f.step(stick, PieceAssist.nudge(stick, intake, chosen.get(), eagerness), DT);
      } else {
        cmd = f.step(stick, DT);
      }
      plant.command(cmd);
      plant.advanceTo((i + 1) * DT);
      run.add(plant.pose());
      if (intakeOf(plant.pose(), BUMPER / 2).x() >= untilX) {
        break;
      }
    }
    return run;
  }

  private static String pieceMark(Vec2 p, String label) {
    return "{\"x\":" + num(p.x()) + ",\"y\":" + num(p.y()) + ",\"style\":\"piece\""
        + (label == null ? "" : ",\"label\":\"" + label + "\"") + "}";
  }

  private static String pieceScene() {
    Pose start = new Pose(-3.4, 0.0, 0.0);
    FieldSpeeds stick = new FieldSpeeds(3.0, 0.0, 0.0);
    List<String> variants = new ArrayList<>();
    Vec2 piece = new Vec2(1.8, 0.3);
    Run without = towardPiece(List.of(piece), start, stick, false, PieceAssist.EAGERNESS, piece.x());
    double missWithout = Math.abs(without.poses.get(without.poses.size() - 1)[1] - piece.y());
    for (double eagerness : new double[] {PieceAssist.EAGERNESS, 1.0}) {
      Run with = towardPiece(List.of(piece), start, stick, true, eagerness, piece.x());
      double miss = Math.abs(with.poses.get(with.poses.size() - 1)[1] - piece.y());
      variants.add(variant("Eagerness " + num(eagerness), with.seconds(), "", pieceMark(piece, "piece"),
          without.track("ghost", "no assist") + "," + with.track("robot", null),
          step(0.0, "The driver pushes straight ahead at 3 m/s. The piece is 30 cm to the left.") + ","
              + step(0.4, "The assist adds a sideways slide and keeps the driver’s speed. The intake arrives on the piece."),
          fact("Miss without", cm(missWithout) + " cm") + "," + fact("Miss with", cm(miss) + " cm")));
    }
    // two pieces: only the one the driver is heading for
    Vec2 near = new Vec2(1.8, -0.35);
    Vec2 off = new Vec2(0.6, 1.5);
    Run two = towardPiece(List.of(near, off), start, stick, true, PieceAssist.EAGERNESS, near.x());
    Run twoWithout = towardPiece(List.of(near, off), start, stick, false, PieceAssist.EAGERNESS, near.x());
    variants.add(variant("Two pieces", two.seconds(), "",
        pieceMark(near, "in line") + "," + pieceMark(off, "off to the side"),
        twoWithout.track("ghost", "no assist") + "," + two.track("robot", null),
        step(0.0, "Two pieces are in view. Only one is within 30° of where the stick points.") + ","
            + step(0.4, "The assist helps with that one and ignores the other."),
        fact("Pieces in view", "2") + "," + fact("Helped toward", "1")));
    return scene("[-4.2,-1.3,3.0,2.1]", 4.0, variants);
  }

  // ------------------------------------------------------------------------------------------------- tag

  private static Run followWithGoal(Path path, Pose seenGoal, double seenFromX) {
    PathFollower follower = new PathFollower(path, CFG);
    Pose start = new Pose(-2.6, -0.5, 0.0);
    SimDrivetrain plant = new SimDrivetrain(start, new FieldSpeeds(0, 0, 0), SimDrivetrain.DEFAULT_LATENCY_SECONDS,
        SimDrivetrain.DEFAULT_LAG_SECONDS, SimDrivetrain.DEFAULT_SUBSTEP_SECONDS);
    if (seenGoal != null) {
      // nothing until the robot is close enough to see the tag, then the spot measured from it
      follower.bindGoal(() -> plant.pose().xMeters() > seenFromX ? Optional.of(seenGoal) : Optional.empty());
    }
    follower.start(0.0, plant.pose(), plant.velocity(), FieldFrame.BLUE);
    Run run = new Run();
    run.add(plant.pose());
    int after = 0;
    for (int i = 0; i < 500 && after < 25; i++) {
      plant.command(follower.step(i * DT, plant.pose(), plant.velocity()));
      plant.advanceTo((i + 1) * DT);
      run.add(plant.pose());
      if (follower.isFinished()) {
        after++;
      }
    }
    return run;
  }

  private static String tagScene() {
    Path path = Path.build(PathFile.read("{\"schema\":4,\"headingMode\":\"hold\",\"waypoints\":[{\"x\":-2.6,\"y\":-0.5,"
        + "\"tangentDegrees\":0},{\"x\":-1.0,\"y\":0.05},{\"x\":0.6,\"y\":0,\"tangentDegrees\":0}],\"rotationTargets\":["
        + "{\"waypoint\":0,\"degrees\":0},{\"waypoint\":2,\"degrees\":0}],\"maxMetersPerSecond\":4,"
        + "\"maxMetersPerSecondSquared\":6}"), ROBOT);
    Pose seen = new Pose(0.84, 0.27, Math.toRadians(10));
    Run toMap = followWithGoal(path, null, 0);
    Run toTag = followWithGoal(path, seen, -1.2);
    double[] end = toTag.poses.get(toTag.poses.size() - 1);
    double miss = Math.hypot(end[0] - seen.xMeters(), end[1] - seen.yMeters());
    double[] mapEnd = toMap.poses.get(toMap.poses.size() - 1);
    double mapMiss = Math.hypot(mapEnd[0] - seen.xMeters(), mapEnd[1] - seen.yMeters());
    String marks = "{\"x\":0.6,\"y\":0,\"style\":\"point\",\"side\":\"below\",\"label\":\"where the map says\"},"
        + "{\"x\":" + num(seen.xMeters()) + ",\"y\":" + num(seen.yMeters()) + ",\"style\":\"goal\",\"side\":\"above\",\"label\":\"measured from the tag\"},"
        + "{\"x\":1.75,\"y\":0.35,\"style\":\"tag\",\"label\":\"tag\"}";
    List<String> variants = new ArrayList<>();
    variants.add(variant("Goal follows the tag", toTag.seconds(), "", marks,
        toMap.track("ghost", null) + "," + toTag.track("robot", null),
        step(0.0, "The robot sets off for the spot the field map gives.") + ","
            + step(0.7, "The camera now sees the tag. The real spot is " + String.format(Locale.ROOT, "%.0f", mapMiss * 100) + " cm away and turned 10°.") + ","
            + step(1.0, "The rest of the path is planned again to the measured spot, and the robot stops on it."),
        fact("Map-only stop is off by", cm(mapMiss) + " cm") + ","
            + fact("Stops within", cm(miss) + " cm")));
    return scene("[-3.3,-1.35,2.4,1.45]", 4.0, variants);
  }

  public static void main(String[] args) throws IOException {
    if (args.length != 1) {
      System.err.println("usage: DocScenes <output file, e.g. docs/src/data/assists.json>");
      System.exit(2);
    }
    String json = "{\n\"rail\":" + railScene() + ",\n\"piece\":" + pieceScene() + ",\n\"tag\":" + tagScene() + "\n}\n";
    java.nio.file.Path out = java.nio.file.Path.of(args[0]);
    Files.createDirectories(out.toAbsolutePath().getParent());
    Files.writeString(out, json, StandardCharsets.UTF_8);
    System.out.println("wrote " + out + " (" + json.length() + " characters)");
  }
}
