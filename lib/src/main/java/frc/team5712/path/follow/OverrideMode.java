package frc.team5712.path.follow;

/** How a raw field-axis override meets the output limiter. */
public enum OverrideMode {
  /** The override replaces the follower's wish before the limiter: it ramps within the grip budget. */
  RESPECT_LIMITS,
  /** The override replaces the output after the limiter: it takes effect at once. */
  BYPASS
}
