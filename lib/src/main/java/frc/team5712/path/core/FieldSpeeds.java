package frc.team5712.path.core;

/** A field-relative velocity: m/s along x and y, and spin in rad/s (counter-clockwise positive). */
public record FieldSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadPerSec) {}
