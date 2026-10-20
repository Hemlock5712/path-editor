package frc.team5712.path.core;

/**
 * The planned robot heading at one distance along the path.
 *
 * @param radians heading, wrapped to (-π, π]
 * @param radiansPerMeter dθ/ds: the planned spin rate is {@code ω = v · dθ/ds}
 * @param radiansPerMeterSquared d²θ/ds²: the planned spin-up is {@code α = a·dθ/ds + v²·d²θ/ds²}
 */
public record HeadingSample(double radians, double radiansPerMeter, double radiansPerMeterSquared) {}
