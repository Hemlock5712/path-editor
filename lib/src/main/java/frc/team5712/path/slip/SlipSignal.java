package frc.team5712.path.slip;

/** What a follower needs to know about wheel slip, each loop. */
public interface SlipSignal {
  /** 0 (gripping) to 1 (sure the wheels are slipping). */
  double confidence();

  /**
   * The chassis acceleration the robot is actually achieving, m/s² (from the IMU when there is one).
   * When slip starts, this is about what the floor can give.
   */
  double achievedAccelMetersPerSecondSquared();

  /**
   * How far the wheels' acceleration is from the IMU's, m/s² (NaN without an IMU). Wheels that disagree
   * with each other (some slipping, the rest gripping) keep it small; wheels spinning up together while
   * the chassis doesn't (the whole robot sliding on a slick floor) make it large.
   */
  default double imuGapMetersPerSecondSquared() {
    return Double.NaN;
  }
}
