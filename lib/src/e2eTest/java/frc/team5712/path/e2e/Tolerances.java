package frc.team5712.path.e2e;

/**
 * Every pass mark of the end-to-end suite, in one place. The reasons are in e2e/RESULTS.md's
 * "Tolerances" section; the figures there are measured by this suite.
 */
final class Tolerances {
  private Tolerances() {}

  /** Robot loop period, seconds (50 Hz). */
  static final double LOOP_SECONDS = 0.020;

  /**
   * Where the robot really is (true pose, along the path) when an event's command runs, versus where the
   * event was authored: this much, or half a loop of travel at the planned speed there plus
   * {@link #EVENT_POSITION_MARGIN_METERS}, whichever is more. Commands only start on a loop, and the robot
   * covers v·20 ms between loops (8 cm at 4 m/s), so the nearest loop can be v·10 ms away; the margin
   * covers the pose noise and the robot trailing its command.
   */
  static final double EVENT_POSITION_METERS = 0.03;
  static final double EVENT_POSITION_MARGIN_METERS = 0.01;

  static double eventPositionTolerance(double plannedMetersPerSecond) {
    return Math.max(EVENT_POSITION_METERS, 0.5 * plannedMetersPerSecond * LOOP_SECONDS + EVENT_POSITION_MARGIN_METERS);
  }

  /**
   * When an event's command runs (from the path's start) versus the plan's time there (editor preview,
   * t(s)). Also when the robot arrives inside the finish band (2 cm and 1°) versus the plan's total.
   */
  static final double EVENT_TIME_SECONDS = 0.10;

  /** Final true position and heading against the goal (the follower finishes inside 2 cm and 1°, measured). */
  static final double END_POSITION_METERS = 0.03;
  static final double END_HEADING_DEGREES = 1.5;

  /**
   * From arriving in the finish band to the follower calling the path finished: the DriveToPose finish
   * settles (slower than 5 cm/s, inside 2 cm and 1°) and then holds for 0.1 s. The preview's total stops at
   * the plan's end, so this is the time the robot legitimately takes beyond it.
   */
  static final double SETTLE_SECONDS = 0.50;

  /** Peak true speed versus the preview's peak: within this fraction, or this absolute amount for slow paths. */
  static final double PEAK_SPEED_FRACTION = 0.05;
  static final double PEAK_SPEED_ABSOLUTE = 0.05;

  /** A rolling end hands over without stopping: the true speed stays above this share of the end speed. */
  static final double HANDOFF_SPEED_FRACTION = 0.5;
}
