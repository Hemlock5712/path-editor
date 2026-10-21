package frc.team5712.path.e2e;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathGeometry;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.Pose;
import frc.team5712.path.core.VelocityProfile;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.follow.PathFollower;
import frc.team5712.path.wpilib.FollowPathCommand;
import frc.team5712.path.wpilib.FollowPathCommandV2;
import frc.team5712.path.wpilib.PathLibrary;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Subsystem;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

/**
 * One end-to-end run: a path (or a rolling chain of paths) from the library, followed by the real
 * {@link PathFollower} inside a WPILib follow command, on {@link SimDrive} at 50 Hz, on one alliance.
 * Events go to the commands bound in the library, which record the true pose when they run.
 */
final class EndToEndRun {
  enum Framework {
    V3,
    V2
  }

  /** One fired event: which segment of the chain, its name, the sim time and the true pose then. */
  record Fired(int segment, String name, double t, Pose truePose) {}

  /** One path of the run, with what the editor previewed for it. */
  static final class Segment {
    final String name;
    final Path path;
    final Preview.PathPreview preview;
    PathFollower follower;
    double startT = Double.NaN;
    double finishT = Double.NaN;
    /** First loop with the true pose inside the finish band (or the finish, for a rolling end). */
    double arriveT = Double.NaN;
    /** When the plan says the robot arrives: entering the finish band, or the end for a rolling end. */
    double arrivalDue = Double.NaN;
    Pose finishPose;
    double peakSpeed;

    Segment(String name, Path path, Preview.PathPreview preview) {
      this.name = name;
      this.path = path;
      this.preview = preview;
    }
  }

  /** What one event did, for the report. */
  record EventResult(String path, String name, double authoredS, double positionError, double timeError, double positionTolerance) {}

  /** -De2e.trace=PathName prints that path's run loop by loop (blue, v3 only matters little). */
  private static final String TRACE = System.getProperty("e2e.trace");

  final List<Segment> segments = new ArrayList<>();
  final List<Fired> fired = new ArrayList<>();
  final List<EventResult> events = new ArrayList<>();
  final List<String> failures = new ArrayList<>();
  double cancelledAt = Double.NaN;
  double minHandoffSpeed = Double.NaN;
  long geometryBuilds;
  long profileBuilds;

  private final Framework framework;
  private final FieldFrame frame;
  private final double interruptFraction;
  private final PathLibrary library;
  private SimDrive drive;
  /** The robot loop's clock: what the follow commands read, and when recorded events ran. */
  private final double[] clock = {0.0};

  /**
   * @param interruptFraction NaN to run to the end, else cancel the command this far through the
   *     first path's previewed time
   */
  EndToEndRun(PathLibrary library, Preview preview, List<String> chain, Framework framework, FieldFrame frame,
      double interruptFraction) {
    this.library = library;
    this.framework = framework;
    this.frame = frame;
    this.interruptFraction = interruptFraction;
    for (String name : chain) {
      segments.add(new Segment(name, library.get(name), preview.path(name)));
    }
  }

  /** Events the followers have fired whose commands haven't run yet: {segment, name}. */
  private final java.util.ArrayDeque<Object[]> firedByFollower = new java.util.ArrayDeque<>();

  /**
   * Called by the recording event commands, when the command runs: that moment and the true pose then
   * are what gets checked. Which path it belongs to comes from the followers' own listeners (in v2 the
   * commands run at the end of the scheduler's loop, after a chain's next path may have started).
   */
  void record(String name) {
    int seg = -1;
    for (var it = firedByFollower.iterator(); it.hasNext(); ) {
      Object[] f = it.next();
      if (f[1].equals(name)) {
        seg = (Integer) f[0];
        it.remove();
        break;
      }
    }
    if (seg < 0) {
      failures.add("the command for " + name + " ran with no follower event behind it");
      return;
    }
    fired.add(new Fired(seg, name, clock[0], drive.truePose()));
  }

  EndToEndRun run(long seed) {
    Segment first = segments.get(0);
    PathSample start = first.path.sample(0.0);
    Pose bluePose = new Pose(start.xMeters(), start.yMeters(), start.headingRadians());
    double v0 = first.path.spec().startMetersPerSecond();
    FieldSpeeds blueVelocity = new FieldSpeeds(v0 * start.tangentX(), v0 * start.tangentY(), 0.0);
    drive = new SimDrive(frame.fromBlue(bluePose), frame.fromBlue(blueVelocity), seed);

    for (int i = 0; i < segments.size(); i++) {
      Segment s = segments.get(i);
      int index = i;
      s.follower = new PathFollower(s.path, followerConfig(library))
          .onEvent(name -> firedByFollower.add(new Object[] {index, name}));
    }

    Driver driver = framework == Framework.V3 ? new V3Driver(clock) : new V2Driver(clock);
    long g0 = PathGeometry.buildCount();
    long p0 = VelocityProfile.buildCount();
    driver.schedule();

    double previewTotal = 0.0;
    for (Segment s : segments) {
      previewTotal += s.preview.totalTimeSeconds();
    }
    double interruptAt = interruptFraction * first.preview.totalTimeSeconds();
    double limit = previewTotal + 5.0;
    double endT = Double.NaN;
    for (int k = 0; ; k++) {
      double t = k * Tolerances.LOOP_SECONDS;
      clock[0] = t;
      drive.advanceTo(t);
      if (!Double.isNaN(interruptAt) && Double.isNaN(cancelledAt) && t >= interruptAt) {
        driver.cancel();
        cancelledAt = t;
      }
      driver.run();
      if (TRACE != null && segments.get(0).name.equals(TRACE)) {
        Segment s0 = segments.get(0);
        Pose p = drive.truePose();
        Pose goalP = frame.fromBlue(new Pose(s0.path.sample(s0.path.lengthMeters()).xMeters(),
            s0.path.sample(s0.path.lengthMeters()).yMeters(), s0.path.sample(s0.path.lengthMeters()).headingRadians()));
        System.out.printf("TRACE t=%.2f phase=%s s=%.4f speed=%.3f d=%.4f yawErr=%.2fdeg%n", t, s0.follower.phase(),
            s0.follower.progressMeters(), drive.trueSpeed(), Math.hypot(p.xMeters() - goalP.xMeters(), p.yMeters() - goalP.yMeters()),
            Math.toDegrees(Math.IEEEremainder(p.headingRadians() - goalP.headingRadians(), 2 * Math.PI)));
      }
      for (Segment s : segments) {
        if (Double.isNaN(s.startT) && s.follower.phase() != PathFollower.Phase.IDLE) {
          s.startT = t;
        }
        boolean active = !Double.isNaN(s.startT) && Double.isNaN(s.finishT) && Double.isNaN(cancelledAt);
        if (active) {
          s.peakSpeed = Math.max(s.peakSpeed, drive.trueSpeed());
        }
        if (Double.isNaN(s.arriveT) && !Double.isNaN(s.startT) && Double.isNaN(cancelledAt) && inFinishBand(s)) {
          s.arriveT = t;
        }
        if (Double.isNaN(s.finishT) && s.follower.isFinished()) {
          s.finishT = t;
          s.finishPose = drive.truePose();
          if (Double.isNaN(s.arriveT)) {
            s.arriveT = t;
          }
        }
      }
      if (segments.size() > 1 && !Double.isNaN(segments.get(0).finishT) && t <= segments.get(0).finishT + 0.3) {
        minHandoffSpeed = Double.isNaN(minHandoffSpeed) ? drive.trueSpeed() : Math.min(minHandoffSpeed, drive.trueSpeed());
      }
      if (Double.isNaN(endT) && !driver.running() && k > 0) {
        endT = t;
      }
      // keep simulating a while after the end (or the cancel): nothing may fire then
      if (!Double.isNaN(endT) && t >= endT + 1.0) {
        break;
      }
      if (t > limit) {
        failures.add("still running " + limit + " s after starting");
        break;
      }
    }
    if (!firedByFollower.isEmpty()) {
      failures.add(firedByFollower.size() + " events fired by the follower never ran their command");
    }
    geometryBuilds = PathGeometry.buildCount() - g0;
    profileBuilds = VelocityProfile.buildCount() - p0;
    driver.close();
    if (TRACE != null && segments.get(0).name.equals(TRACE)) {
      for (Segment sg : segments) {
        System.out.printf("TRACE segment %s start %.2f finish %.2f%n", sg.name, sg.startT, sg.finishT);
      }
      for (Fired f : fired) {
        System.out.printf("TRACE fired seg=%d %s t=%.2f s=%.3f%n", f.segment(), f.name(), f.t(), sAlong(segments.get(f.segment()), f.truePose()));
      }
    }
    check();
    return this;
  }

  /** The robot code's follower settings: the defaults, with the latency allowance set for this drivetrain. */
  static FollowerConfig followerConfig(PathLibrary library) {
    return FollowerConfig.defaults(library.robot()).toBuilder()
        .latencySeconds(SimDrive.RESPONSE_DELAY_SECONDS)
        .build();
  }

  /**
   * When the plan itself enters the finish band (within 2 cm and 1° of the end): the time an event at the
   * end of a path that stops is due, since the follower fires those when the robot gets inside the band.
   */
  static double planTimeIntoFinishBand(Path path, FollowerConfig cfg) {
    PathSample end = path.sample(path.lengthMeters());
    double s = path.lengthMeters();
    double step = Math.min(5e-4, path.lengthMeters() / 2000);
    for (double back = 0.0; back <= path.lengthMeters(); back += step) {
      PathSample p = path.sample(path.lengthMeters() - back);
      boolean inside = Math.hypot(p.xMeters() - end.xMeters(), p.yMeters() - end.yMeters()) < cfg.positionToleranceMeters()
          && Math.abs(Math.IEEEremainder(p.headingRadians() - end.headingRadians(), 2 * Math.PI)) < cfg.headingToleranceRadians();
      if (!inside) {
        break;
      }
      s = p.s();
    }
    return path.sample(s).seconds();
  }

  /** True pose within the follower's finish tolerances of the goal (2 cm, 1°): a stop, not a rolling end. */
  private boolean inFinishBand(Segment s) {
    if (s.path.spec().endMetersPerSecond() > 0) {
      return false;
    }
    FollowerConfig cfg = followerConfig(library);
    PathSample end = s.path.sample(s.path.lengthMeters());
    Pose goal = frame.fromBlue(new Pose(end.xMeters(), end.yMeters(), end.headingRadians()));
    Pose p = drive.truePose();
    return Math.hypot(p.xMeters() - goal.xMeters(), p.yMeters() - goal.yMeters()) < cfg.positionToleranceMeters()
        && Math.abs(Math.IEEEremainder(p.headingRadians() - goal.headingRadians(), 2 * Math.PI)) < cfg.headingToleranceRadians();
  }

  // ------------------------------------------------------------------------------ checks

  private void check() {
    if (geometryBuilds != 0 || profileBuilds != 0) {
      failures.add("path work after pressing go: " + geometryBuilds + " geometries, " + profileBuilds + " profiles built");
    }
    if (!Double.isNaN(cancelledAt)) {
      checkInterrupted();
      return;
    }
    for (int i = 0; i < segments.size(); i++) {
      checkSegment(i);
    }
    if (segments.size() > 1) {
      double needed = Tolerances.HANDOFF_SPEED_FRACTION * segments.get(0).path.spec().endMetersPerSecond();
      if (!(minHandoffSpeed >= needed)) {
        failures.add(String.format("rolling hand-off slowed to %.2f m/s (needs %.2f)", minHandoffSpeed, needed));
      }
    }
  }

  /** Events of a segment in the order the follower fires them: by position, ties in file order. */
  private static List<Preview.EventPreview> expectedOrder(Segment s) {
    List<Preview.EventPreview> order = new ArrayList<>(s.preview.events());
    order.sort(Comparator.comparingDouble(Preview.EventPreview::s));
    return order;
  }

  /** Metres along the segment's path at a true pose. */
  private double sAlong(Segment s, Pose truePose) {
    Pose blue = frame.toBlue(truePose);
    return s.path.geometry().projectGlobal(blue.xMeters(), blue.yMeters()).s();
  }

  private void checkSegment(int i) {
    Segment s = segments.get(i);
    String where = s.name + ": ";
    if (Double.isNaN(s.finishT)) {
      failures.add(where + "never finished");
      return;
    }
    // the end: a stop on the goal, or (rolling) passing the end at speed
    PathSample end = s.path.sample(s.path.lengthMeters());
    Pose goal = frame.fromBlue(new Pose(end.xMeters(), end.yMeters(), end.headingRadians()));
    double posErr = Math.hypot(s.finishPose.xMeters() - goal.xMeters(), s.finishPose.yMeters() - goal.yMeters());
    double headErr = Math.toDegrees(Math.abs(Math.IEEEremainder(s.finishPose.headingRadians() - goal.headingRadians(), 2 * Math.PI)));
    boolean rolling = s.path.spec().endMetersPerSecond() > 0;
    double endTol = rolling ? s.path.spec().endMetersPerSecond() * Tolerances.LOOP_SECONDS + Tolerances.END_POSITION_METERS
        : Tolerances.END_POSITION_METERS;
    if (posErr > endTol || headErr > Tolerances.END_HEADING_DEGREES) {
      failures.add(String.format("%sended %.1f cm and %.2f° from the goal", where, posErr * 100, headErr));
    }

    // events: each once, in order, where and when the editor said
    List<Preview.EventPreview> expected = expectedOrder(s);
    List<Fired> mine = fired.stream().filter(f -> f.segment() == i).toList();
    List<String> names = mine.stream().map(Fired::name).toList();
    List<String> wanted = expected.stream().map(Preview.EventPreview::name).toList();
    if (!names.equals(wanted)) {
      failures.add(where + "events fired " + names + ", expected " + wanted);
      return;
    }
    double[] followerS = s.follower.eventMeters();
    for (int j = 0; j < mine.size(); j++) {
      Fired f = mine.get(j);
      Preview.EventPreview e = expected.get(j);
      if (Math.abs(followerS[j] - e.s()) > 1e-9) {
        failures.add(String.format("%s%s: the robot puts it at %.9f m, the editor at %.9f m", where, e.name(), followerS[j], e.s()));
      }
      double posError = sAlong(s, f.truePose()) - e.s();
      // an event at the end of a path that stops is due when the plan enters the finish band
      boolean atStop = !rolling && e.s() >= s.path.lengthMeters() - 1e-9;
      double due = atStop ? planTimeIntoFinishBand(s.path, followerConfig(library)) : e.t();
      double timeError = (f.t() - s.startT) - due;
      double posTol = Tolerances.eventPositionTolerance(s.path.sample(e.s()).metersPerSecond());
      events.add(new EventResult(s.name, e.name(), e.s(), posError, timeError, posTol));
      if (Math.abs(posError) > posTol) {
        failures.add(String.format("%s%s fired %.1f cm from where it was authored (%.3f m; allowed %.1f cm)", where, e.name(),
            posError * 100, e.s(), posTol * 100));
      }
      if (Math.abs(timeError) > Tolerances.EVENT_TIME_SECONDS) {
        failures.add(String.format("%s%s fired at %.3f s, due at %.3f s (preview t(s) %.3f s)", where, e.name(), f.t() - s.startT, due, e.t()));
      }
      if (e.s() == 0.0 && f.t() != s.startT) {
        failures.add(where + e.name() + " is at 0 m but didn't fire when the path started");
      }
      if (f.t() > s.finishT) {
        failures.add(where + e.name() + " fired after the path finished");
      }
    }

    // the preview against the robot: arrival time, the finish's settling, and peak speed
    double arrival = s.arriveT - s.startT;
    double due = rolling ? s.preview.totalTimeSeconds() : planTimeIntoFinishBand(s.path, followerConfig(library));
    s.arrivalDue = due;
    if (Math.abs(arrival - due) > Tolerances.EVENT_TIME_SECONDS) {
      failures.add(String.format("%sarrived at %.3f s, the plan enters the finish band at %.3f s (ends at %.3f s)", where,
          arrival, due, s.preview.totalTimeSeconds()));
    }
    if (s.finishT - s.arriveT > Tolerances.SETTLE_SECONDS) {
      failures.add(String.format("%stook %.3f s to settle after arriving", where, s.finishT - s.arriveT));
    }
    double peak = s.preview.peakMetersPerSecond();
    if (Math.abs(s.peakSpeed - peak) > Math.max(Tolerances.PEAK_SPEED_ABSOLUTE, Tolerances.PEAK_SPEED_FRACTION * peak)) {
      failures.add(String.format("%speak speed %.3f m/s, the preview says %.3f m/s", where, s.peakSpeed, peak));
    }
  }

  private void checkInterrupted() {
    Segment s = segments.get(0);
    if (s.follower.phase() != PathFollower.Phase.CANCELLED) {
      failures.add("after the interrupt the follower is " + s.follower.phase() + ", not CANCELLED");
    }
    List<Fired> late = fired.stream().filter(f -> f.t() > cancelledAt).toList();
    if (!late.isEmpty()) {
      failures.add("events fired after the interrupt: " + late.stream().map(Fired::name).toList());
    }
    // what fired is the start of the expected order, and nothing past where the robot was
    List<String> wanted = expectedOrder(s).stream().map(Preview.EventPreview::name).toList();
    List<String> names = fired.stream().map(Fired::name).toList();
    if (names.size() >= wanted.size() || !wanted.subList(0, names.size()).equals(names)) {
      failures.add("interrupted at " + cancelledAt + " s, yet these fired: " + names + " (all: " + wanted + ")");
    }
    if (drive.trueSpeed() > 0.05) {
      failures.add(String.format("still moving at %.2f m/s a second after the interrupt", drive.trueSpeed()));
    }
  }

  // ------------------------------------------------------------------------------ the two frameworks

  private interface Driver {
    void schedule();

    void run();

    void cancel();

    boolean running();

    void close();
  }

  /** Commands v3 under an independent scheduler; event commands are forked children. */
  private final class V3Driver implements Driver {
    private final Scheduler scheduler = Scheduler.createIndependentScheduler();
    private final Command command;

    V3Driver(double[] clock) {
      Mechanism mechanism = new Mechanism() {
        @Override
        public Scheduler getRegisteredScheduler() {
          return scheduler;
        }
      };
      Command chained = null;
      FollowPathCommand previous = null;
      for (Segment s : segments) {
        FollowPathCommand c = new FollowPathCommand(s.name, mechanism, s.follower,
            drive::measuredPose, drive::measuredVelocity, drive::command, FieldFrame.RED_HALF_TURN)
            .withClock(() -> clock[0])
            .withFrame(() -> frame)
            .withEvents(library);
        if (previous != null) {
          c.continuingFrom(previous);
        }
        chained = chained == null ? c : chained.andThen(c).named(chained.name() + " then " + s.name);
        previous = c;
      }
      command = chained;
    }

    @Override
    public void schedule() {
      scheduler.schedule(command);
    }

    @Override
    public void run() {
      scheduler.run();
    }

    @Override
    public void cancel() {
      scheduler.cancel(command);
    }

    @Override
    public boolean running() {
      return scheduler.isScheduledOrRunning(command);
    }

    @Override
    public void close() {
      scheduler.cancelAll();
    }
  }

  /** Commands v2 on the global CommandScheduler; event commands are scheduled independently. */
  private final class V2Driver implements Driver {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final org.wpilib.command2.Command command;

    V2Driver(double[] clock) {
      scheduler.cancelAll();
      Subsystem subsystem = new Subsystem() {};
      org.wpilib.command2.Command chained = null;
      FollowPathCommandV2 previous = null;
      for (Segment s : segments) {
        FollowPathCommandV2 c = new FollowPathCommandV2(s.name, subsystem, s.follower,
            drive::measuredPose, drive::measuredVelocity, drive::command, FieldFrame.RED_HALF_TURN)
            .withClock(() -> clock[0])
            .withFrame(() -> frame)
            .withEvents(library);
        if (previous != null) {
          c.continuingFrom(previous);
        }
        chained = chained == null ? c : chained.andThen(c);
        previous = c;
      }
      command = chained;
    }

    @Override
    public void schedule() {
      scheduler.schedule(command);
    }

    @Override
    public void run() {
      scheduler.run();
    }

    @Override
    public void cancel() {
      scheduler.cancel(command);
    }

    @Override
    public boolean running() {
      return scheduler.isScheduled(command);
    }

    @Override
    public void close() {
      scheduler.cancelAll();
    }
  }
}
