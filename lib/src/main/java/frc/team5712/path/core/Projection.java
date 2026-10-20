package frc.team5712.path.core;

/**
 * The closest point on the path to a query point.
 *
 * @param s metres along the path
 * @param xMeters closest point, x
 * @param yMeters closest point, y
 * @param distanceMeters distance from the query point to the closest point
 * @param crossTrackMeters signed distance: positive when the query point is left of the path
 */
public record Projection(
    double s, double xMeters, double yMeters, double distanceMeters, double crossTrackMeters) {}
