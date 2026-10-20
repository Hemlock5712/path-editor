package frc.team5712.path.core;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Reads and writes a {@link RobotConfig} as JSON. Every key is optional; a missing one keeps the
 * {@link RobotConfig#DEFAULT} value.
 *
 * <pre>
 * { "massKg": 60, "momentOfInertiaKgM2": 6, "moduleRadiusMeters": 0.41012,
 *   "wheelRadiusMeters": 0.0508, "driveGearRatio": 6.75, "driveMotorCount": 4,
 *   "statorCurrentLimitAmps": 120, "frictionCoefficient": 0.7,
 *   "maxWheelMetersPerSecond": 4.5, "maxAngularRadPerSec": 9.4,
 *   "motor": { "nominalVolts": 12, "stallTorqueNewtonMeters": 9.362, "stallCurrentAmps": 476.1,
 *              "freeSpeedRadPerSec": 605.8 } }
 * </pre>
 */
public final class RobotFile {
  private RobotFile() {}

  public static RobotConfig read(String json) {
    return fromJson(PathFile.asMap(Json.parse(json), "robot"));
  }

  public static RobotConfig fromJson(Map<String, Object> m) {
    RobotConfig d = RobotConfig.DEFAULT;
    Motor motor = d.driveMotor();
    Object mo = m.get("motor");
    if (mo != null) {
      Map<String, Object> mm = PathFile.asMap(mo, "motor");
      motor =
          new Motor(
              PathFile.number(mm, "nominalVolts", motor.nominalVolts()),
              PathFile.number(mm, "stallTorqueNewtonMeters", motor.stallTorqueNewtonMeters()),
              PathFile.number(mm, "stallCurrentAmps", motor.stallCurrentAmps()),
              PathFile.number(mm, "freeSpeedRadPerSec", motor.freeSpeedRadPerSec()));
    }
    return new RobotConfig(
        PathFile.number(m, "massKg", d.massKg()),
        PathFile.number(m, "momentOfInertiaKgM2", d.momentOfInertiaKgM2()),
        PathFile.number(m, "moduleRadiusMeters", d.moduleRadiusMeters()),
        PathFile.number(m, "wheelRadiusMeters", d.wheelRadiusMeters()),
        PathFile.number(m, "driveGearRatio", d.driveGearRatio()),
        motor,
        (int) PathFile.number(m, "driveMotorCount", d.driveMotorCount()),
        PathFile.number(m, "statorCurrentLimitAmps", d.statorCurrentLimitAmps()),
        PathFile.number(m, "frictionCoefficient", d.frictionCoefficient()),
        PathFile.number(m, "maxWheelMetersPerSecond", d.maxWheelMetersPerSecond()),
        PathFile.number(m, "maxAngularRadPerSec", d.maxAngularRadPerSec()));
  }

  /** The JSON tree; unset optional limits are left out. */
  public static Map<String, Object> toJson(RobotConfig r) {
    Map<String, Object> m = new LinkedHashMap<>();
    m.put("massKg", r.massKg());
    m.put("momentOfInertiaKgM2", r.momentOfInertiaKgM2());
    m.put("moduleRadiusMeters", r.moduleRadiusMeters());
    m.put("wheelRadiusMeters", r.wheelRadiusMeters());
    m.put("driveGearRatio", r.driveGearRatio());
    m.put("driveMotorCount", r.driveMotorCount());
    m.put("statorCurrentLimitAmps", r.statorCurrentLimitAmps());
    m.put("frictionCoefficient", r.frictionCoefficient());
    if (Double.isFinite(r.maxWheelMetersPerSecond())) {
      m.put("maxWheelMetersPerSecond", r.maxWheelMetersPerSecond());
    }
    if (Double.isFinite(r.maxAngularRadPerSec())) {
      m.put("maxAngularRadPerSec", r.maxAngularRadPerSec());
    }
    Map<String, Object> motor = new LinkedHashMap<>();
    motor.put("nominalVolts", r.driveMotor().nominalVolts());
    motor.put("stallTorqueNewtonMeters", r.driveMotor().stallTorqueNewtonMeters());
    motor.put("stallCurrentAmps", r.driveMotor().stallCurrentAmps());
    motor.put("freeSpeedRadPerSec", r.driveMotor().freeSpeedRadPerSec());
    m.put("motor", motor);
    return m;
  }
}
