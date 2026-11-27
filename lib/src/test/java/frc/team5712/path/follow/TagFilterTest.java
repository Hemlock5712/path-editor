package frc.team5712.path.follow;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.team5712.path.core.Pose;
import java.util.Random;
import org.junit.jupiter.api.Test;

class TagFilterTest {
  private static final Pose TAG = new Pose(3.0, 1.0, Math.PI); // odometry frame, facing back down x
  private static final Pose TAG_TO_GOAL = new Pose(0.5, 0.0, Math.PI); // half a metre out, facing the tag

  /** The tag as a robot at {@code robot} would see it, off by (ex, ey). */
  private static Pose sighting(Pose robot, double ex, double ey) {
    Pose seen = TagFilter.minus(TAG, robot);
    return new Pose(seen.xMeters() + ex, seen.yMeters() + ey, seen.headingRadians());
  }

  @Test
  void locksAfterThreeSightingsThatAgree() {
    TagFilter f = new TagFilter();
    Pose robot = new Pose(1.0, 0.8, 0.1);
    assertFalse(f.hasLock());
    assertTrue(f.fieldGoal(TAG_TO_GOAL, robot, robot).isEmpty());
    assertEquals(Double.POSITIVE_INFINITY, f.secondsSinceSeen(0.0));
    for (int i = 0; i < 3; i++) {
      assertFalse(f.hasLock());
      assertTrue(f.add(sighting(robot, 0, 0), robot, 0.02 * i));
    }
    assertTrue(f.hasLock());
    assertEquals(TAG.xMeters(), f.tagPose().orElseThrow().xMeters(), 1e-9);
    assertEquals(0.06, f.secondsSinceSeen(0.10), 1e-9);
    f.reset();
    assertFalse(f.hasLock());
  }

  /** Noisy sightings from a robot driving in settle on the tag: far better than any one of them. */
  @Test
  void noisySightingsSettleOnTheTag() {
    TagFilter f = new TagFilter();
    Random r = new Random(7);
    for (int i = 0; i < 100; i++) {
      Pose robot = new Pose(0.5 + 0.015 * i, 0.9, 0.05); // 2.5 m out, closing to 1 m
      double d = TAG.xMeters() - robot.xMeters();
      double sigma = 0.005 + 0.01 * d * d;
      f.add(sighting(robot, r.nextGaussian() * sigma, r.nextGaussian() * sigma), robot, 0.02 * i);
    }
    Pose est = f.tagPose().orElseThrow();
    assertEquals(TAG.xMeters(), est.xMeters(), 0.01);
    assertEquals(TAG.yMeters(), est.yMeters(), 0.01);
  }

  @Test
  void oneStraySightingIsIgnoredAndThreeInARowAreANewTag() {
    TagFilter f = new TagFilter();
    Pose robot = new Pose(1.0, 1.0, 0.0);
    for (int i = 0; i < 5; i++) {
      f.add(sighting(robot, 0, 0), robot, 0.02 * i);
    }
    assertFalse(f.add(sighting(robot, 0.6, 0), robot, 0.10), "a stray sighting is skipped");
    assertEquals(TAG.xMeters(), f.tagPose().orElseThrow().xMeters(), 1e-9);
    assertTrue(f.add(sighting(robot, 0, 0), robot, 0.12));
    assertFalse(f.add(sighting(robot, 0.6, 0), robot, 0.14));
    assertFalse(f.add(sighting(robot, 0.6, 0), robot, 0.16));
    assertTrue(f.add(sighting(robot, 0.6, 0), robot, 0.18), "the third in a row is believed");
    assertEquals(TAG.xMeters() + 0.6, f.tagPose().orElseThrow().xMeters(), 1e-9);
    assertFalse(f.hasLock(), "and the lock starts again");
    assertFalse(f.add(new Pose(4.0, 0.0, Math.PI), robot, 0.20), "too far away to trust");
  }

  /**
   * The goal is measured from the tag, and placed on the field pose: when vision moves the field pose, the goal
   * moves with it, so the drive the robot still has to make is the same.
   */
  @Test
  void theGoalIsMeasuredFromTheTagAndAVisionCorrectionCancels() {
    TagFilter f = new TagFilter();
    Pose odometry = new Pose(1.0, 0.8, 0.2);
    for (int i = 0; i < 3; i++) {
      f.add(sighting(odometry, 0, 0), odometry, 0.02 * i);
    }
    // the field pose agrees with odometry: the goal is half a metre out of the tag's face, facing it
    Pose goal = f.fieldGoal(TAG_TO_GOAL, odometry, odometry).orElseThrow();
    assertEquals(2.5, goal.xMeters(), 1e-9);
    assertEquals(1.0, goal.yMeters(), 1e-9);
    assertEquals(0.0, goal.headingRadians(), 1e-9);
    // vision says the robot is somewhere else on the field: the same drive, from there
    Pose field = new Pose(5.0, -2.0, 1.3);
    Pose moved = f.fieldGoal(TAG_TO_GOAL, odometry, field).orElseThrow();
    Pose before = TagFilter.minus(goal, odometry);
    Pose after = TagFilter.minus(moved, field);
    assertEquals(before.xMeters(), after.xMeters(), 1e-9);
    assertEquals(before.yMeters(), after.yMeters(), 1e-9);
    assertEquals(before.headingRadians(), after.headingRadians(), 1e-9);
    // and odometry carries the estimate once the tag is out of view: the robot drove on, the goal stayed put
    Pose later = new Pose(2.0, 0.9, 0.0);
    Pose still = f.fieldGoal(TAG_TO_GOAL, later, later).orElseThrow();
    assertEquals(2.5, still.xMeters(), 1e-9);
    assertEquals(1.0, still.yMeters(), 1e-9);
  }
}
