package frc.team5712.path.sim;

import frc.team5712.path.core.FieldFrame;
import frc.team5712.path.core.FieldSpeeds;
import frc.team5712.path.core.Path;
import frc.team5712.path.core.PathSample;
import frc.team5712.path.core.Pose;
import frc.team5712.path.follow.FollowerConfig;
import frc.team5712.path.follow.PathFollower;
import java.util.ArrayList;
import java.util.List;

/**
 * Runs a {@link PathFollower} closed-loop on a {@link SimDrivetrain} at 50 Hz, as the end-to-end suite
 * does (start and the first step in the same loop, the command sent each loop, zero once a stopping path
 * finishes), and records every loop. The editor's TypeScript port runs the same loop; this one writes
 * its golden traces (fixtures/follow).
 */
public final class FollowerSim {
  public static final double LOOP_SECONDS = 0.020;

  private FollowerSim() {}

  /** A hit during the run: at {@code atSeconds}, add (dvx, dvy) m/s, field frame. */
  public record Shove(double atSeconds, double dvxMetersPerSecond, double dvyMetersPerSecond) {}

  /** The drivetrain's numbers. */
  public record Drive(double latencySeconds, double lagSeconds, double substepSeconds) {
    public static final Drive DEFAULT = new Drive(SimDrivetrain.DEFAULT_LATENCY_SECONDS,
        SimDrivetrain.DEFAULT_LAG_SECONDS, SimDrivetrain.DEFAULT_SUBSTEP_SECONDS);
  }

  /** One event as it fired: its name and the loop it fired in. */
  public record Fired(String name, int loop) {}

  /** Every loop of a run, column by column; the command is the one sent that loop. */
  public static final class Trace {
    public final List<Double> t = new ArrayList<>();
    public final List<Double> x = new ArrayList<>();
    public final List<Double> y = new ArrayList<>();
    public final List<Double> heading = new ArrayList<>();
    public final List<Double> vx = new ArrayList<>();
    public final List<Double> vy = new ArrayList<>();
    public final List<Double> omega = new ArrayList<>();
    public final List<Double> s = new ArrayList<>();
    public final List<Double> crossTrack = new ArrayList<>();
    public final List<Integer> phase = new ArrayList<>();
    public final List<Fired> events = new ArrayList<>();
    /** The loop the follower finished in (-1 if it never did). */
    public int finishLoop = -1;
  }

  /**
   * Runs the path from its start pose (with the path's start speed along the start tangent), in
   * {@code frame}, until the follower finishes and {@code afterSeconds} more have passed, or
   * {@code maxSeconds}.
   */
  public static Trace run(
      Path path, FollowerConfig config, Drive drive, FieldFrame frame, Shove shove, double afterSeconds,
      double maxSeconds) {
    PathSample start = path.sample(0.0);
    double v0 = path.spec().startMetersPerSecond();
    Pose startPose = frame.fromBlue(new Pose(start.xMeters(), start.yMeters(), start.headingRadians()));
    FieldSpeeds startVel = frame.fromBlue(new FieldSpeeds(v0 * start.tangentX(), v0 * start.tangentY(), 0.0));
    SimDrivetrain sim = new SimDrivetrain(startPose, startVel, drive.latencySeconds(), drive.lagSeconds(),
        drive.substepSeconds());
    PathFollower follower = new PathFollower(path, config);
    Trace trace = new Trace();
    int[] loop = {0};
    follower.onEvent(name -> trace.events.add(new Fired(name, loop[0])));
    boolean shoved = false;
    boolean done = false;
    FieldSpeeds sent = new FieldSpeeds(0.0, 0.0, 0.0);
    for (int k = 0; ; k++) {
      loop[0] = k;
      double t = k * LOOP_SECONDS;
      sim.advanceTo(t);
      if (shove != null && !shoved && t >= shove.atSeconds() - 1e-12) {
        sim.shove(shove.dvxMetersPerSecond(), shove.dvyMetersPerSecond());
        shoved = true;
      }
      Pose p = sim.pose();
      FieldSpeeds v = sim.velocity();
      if (k == 0) {
        follower.start(t, p, v, frame);
      }
      if (!done) {
        sent = follower.step(t, p, v);
        sim.command(sent);
        if (follower.isFinished()) {
          done = true;
          trace.finishLoop = k;
          if (!follower.rollingEnd()) {
            sent = new FieldSpeeds(0.0, 0.0, 0.0);
            sim.command(sent);
          }
        }
      }
      trace.t.add(t);
      trace.x.add(p.xMeters());
      trace.y.add(p.yMeters());
      trace.heading.add(p.headingRadians());
      trace.vx.add(sent.vxMetersPerSecond());
      trace.vy.add(sent.vyMetersPerSecond());
      trace.omega.add(sent.omegaRadPerSec());
      trace.s.add(follower.progressMeters());
      trace.crossTrack.add(follower.crossTrackMeters());
      trace.phase.add(follower.phase().ordinal());
      if (done && t >= trace.finishLoop * LOOP_SECONDS + afterSeconds - 1e-9 || t >= maxSeconds - 1e-9) {
        return trace;
      }
    }
  }
}
