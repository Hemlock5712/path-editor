package frc.team5712.path.core;

import java.util.List;

/**
 * The robot's planned heading as a function of distance along the path, with its first two
 * derivatives so the velocity profile can budget for the spin.
 *
 * <p>Layers, bottom to top:
 *
 * <ol>
 *   <li><b>Base.</b> With no rotation targets, {@link HeadingMode#HOLD} (the default) holds the
 *       direction the path starts out in; {@link HeadingMode#FACE_TRAVEL} faces the direction of travel
 *       (θ = tangent angle, dθ/ds = κ, d²θ/ds² = dκ/ds). With targets, the heading holds the first
 *       target before it, the last after it, and between two targets turns the shortest way with the
 *       minimum-jerk blend {@code 10τ³ − 15τ⁴ + 6τ⁵} over the whole stretch, so dθ/ds and d²θ/ds²
 *       are continuous and zero at each target.
 *   <li><b>Face-point zones,</b> in order. Inside a zone the heading is pulled to the direction from
 *       the path point to the zone's point, eased in and out with the same blend.
 * </ol>
 */
public final class HeadingProfile {
  private final PathGeometry geometry;
  // Rotation targets sorted by s, headings unwrapped so each step is the shortest turn
  private final double[] targetS;
  private final double[] targetRadians;
  // Face-point zones in authored order
  private final double[] zoneStartS;
  private final double[] zoneEndS;
  private final double[] zoneBlendMeters;
  private final double[] zonePointX;
  private final double[] zonePointY;

  private HeadingProfile(
      PathGeometry geometry,
      double[] targetS,
      double[] targetRadians,
      double[] zoneStartS,
      double[] zoneEndS,
      double[] zoneBlendMeters,
      double[] zonePointX,
      double[] zonePointY) {
    this.geometry = geometry;
    this.targetS = targetS;
    this.targetRadians = targetRadians;
    this.zoneStartS = zoneStartS;
    this.zoneEndS = zoneEndS;
    this.zoneBlendMeters = zoneBlendMeters;
    this.zonePointX = zonePointX;
    this.zonePointY = zonePointY;
  }

  /** The heading profile as before heading modes: face travel when there are no targets. */
  public static HeadingProfile build(
      PathGeometry geometry, List<RotationTarget> targets, List<FacePointZone> zones) {
    return build(geometry, targets, zones, HeadingMode.FACE_TRAVEL);
  }

  public static HeadingProfile build(
      PathGeometry geometry, List<RotationTarget> targets, List<FacePointZone> zones, HeadingMode mode) {
    if (mode == HeadingMode.HOLD && targets.isEmpty()) {
      // nothing set: hold the direction the path starts out in
      GeometrySample start = geometry.sample(0.0);
      targets = List.of(RotationTarget.atMeters(0.0, Angles.atan2(start.tangentY(), start.tangentX())));
    }
    // Resolve to metres and insertion-sort by s (a handful of targets; stable, so ties keep the
    // authored order; and no lambdas, whose first use costs tens of milliseconds at boot)
    int nt = targets.size();
    double[] ts = new double[nt];
    double[] heading = new double[nt];
    for (int j = 0; j < nt; j++) {
      RotationTarget t = targets.get(j);
      double s = t.byWaypoint() ? geometry.sAtWaypoint(t.waypoint()) : clamp(t.meters(), geometry);
      int at = j;
      while (at > 0 && ts[at - 1] > s) {
        ts[at] = ts[at - 1];
        heading[at] = heading[at - 1];
        at--;
      }
      ts[at] = s;
      heading[at] = t.headingRadians();
    }
    double[] tr = new double[nt];
    for (int j = 0; j < nt; j++) {
      tr[j] = j == 0 ? heading[0] : tr[j - 1] + Angles.wrap(heading[j] - tr[j - 1]);
    }

    int nz = zones.size();
    double[] za = new double[nz];
    double[] zb = new double[nz];
    double[] zblend = new double[nz];
    double[] zx = new double[nz];
    double[] zy = new double[nz];
    for (int j = 0; j < nz; j++) {
      FacePointZone z = zones.get(j);
      za[j] = geometry.sAtWaypoint(z.fromWaypoint());
      zb[j] = geometry.sAtWaypoint(z.toWaypoint());
      zblend[j] = Math.min(z.blendMeters(), 0.5 * (zb[j] - za[j]));
      zx[j] = z.point().x();
      zy[j] = z.point().y();
    }
    return new HeadingProfile(geometry, ts, tr, za, zb, zblend, zx, zy);
  }

  private static double clamp(double s, PathGeometry g) {
    return s < 0.0 ? 0.0 : (s > g.lengthMeters() ? g.lengthMeters() : s);
  }

  /** The heading at {@code s}. */
  public HeadingSample sample(double s) {
    return sample(geometry.sample(s));
  }

  /** The heading at a geometry sample already in hand (saves a lookup). */
  public HeadingSample sample(GeometrySample g) {
    double s = g.s();
    double theta;
    double dTheta;
    double ddTheta;

    if (targetS.length == 0) {
      theta = Angles.atan2(g.tangentY(), g.tangentX());
      dTheta = g.curvature();
      ddTheta = g.dCurvatureDs();
    } else if (s <= targetS[0]) {
      theta = targetRadians[0];
      dTheta = 0.0;
      ddTheta = 0.0;
    } else if (s >= targetS[targetS.length - 1]) {
      theta = targetRadians[targetS.length - 1];
      dTheta = 0.0;
      ddTheta = 0.0;
    } else {
      int j = 0;
      while (targetS[j + 1] <= s) {
        j++;
      }
      double length = targetS[j + 1] - targetS[j];
      double turn = targetRadians[j + 1] - targetRadians[j];
      double tau = (s - targetS[j]) / length;
      theta = targetRadians[j] + turn * Angles.smoothstep5(tau);
      dTheta = turn * Angles.smoothstep5Slope(tau) / length;
      ddTheta = turn * Angles.smoothstep5Bend(tau) / (length * length);
    }

    for (int j = 0; j < zoneStartS.length; j++) {
      double a = zoneStartS[j];
      double b = zoneEndS[j];
      if (s < a || s > b) {
        continue;
      }
      double dx = zonePointX[j] - g.xMeters();
      double dy = zonePointY[j] - g.yMeters();
      double distSq = dx * dx + dy * dy;
      if (distSq < 1e-9) {
        continue; // on top of the point: no direction to face
      }
      // Weight w(s): 0 → 1 over the first blend, 1 → 0 over the last
      double blend = zoneBlendMeters[j];
      double w = 1.0;
      double dw = 0.0;
      double ddw = 0.0;
      if (blend > 0.0) {
        if (s < a + blend) {
          double tau = (s - a) / blend;
          w = Angles.smoothstep5(tau);
          dw = Angles.smoothstep5Slope(tau) / blend;
          ddw = Angles.smoothstep5Bend(tau) / (blend * blend);
        } else if (s > b - blend) {
          double tau = (b - s) / blend;
          w = Angles.smoothstep5(tau);
          dw = -Angles.smoothstep5Slope(tau) / blend;
          ddw = Angles.smoothstep5Bend(tau) / (blend * blend);
        }
      }
      // Facing direction and its derivatives along the path. With d = point − p(s), d' = −T and
      // T' = κN: θf' = (T × d)/|d|², θf'' = (2(T × d)(T·d) − κ(T·d)|d|²)/|d|⁴.
      double face = Angles.atan2(dy, dx);
      double cross = g.tangentX() * dy - g.tangentY() * dx;
      double along = g.tangentX() * dx + g.tangentY() * dy;
      double dFace = cross / distSq;
      double ddFace = (2.0 * cross * along - g.curvature() * along * distSq) / (distSq * distSq);

      double gap = Angles.wrap(face - theta);
      double newTheta = theta + w * gap;
      double newDTheta = dTheta + dw * gap + w * (dFace - dTheta);
      double newDdTheta = ddTheta + ddw * gap + 2.0 * dw * (dFace - dTheta) + w * (ddFace - ddTheta);
      theta = newTheta;
      dTheta = newDTheta;
      ddTheta = newDdTheta;
    }
    return new HeadingSample(Angles.wrap(theta), dTheta, ddTheta);
  }
}
