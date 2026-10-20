"""Ours v2: the Java PathFollower (lib/, frc.team5712.path.follow) driven over a JSON-lines pipe.

Each worker process starts one BenchServer (`java -cp lib/build/libs/pathcore.jar
frc.team5712.path.tools.BenchServer`) on first use and keeps it for every run it does. Build the jar
first: `lib/gradlew -p lib jar`. The protocol is in lib/README.md.

The neutral course becomes our path file:
- waypoints at the course's points; the first and last get the chord to their neighbour as the direction
  of travel (the same end rule as the neutral Catmull-Rom curve), interior tangents are automatic;
- every heading in the course (waypoint headings and fractional heading targets) becomes a rotation target
  at the same waypoint-relative position; between targets the heading follows the minimum-jerk blend;
- the shared limits: 4 m/s and 6 m/s^2 for the path, 540 deg/s and 720 deg/s^2 for the follower;
- the robot file is the bench plant's numbers (mass, inertia, module radius, gearing, Kraken curve, 120 A),
  with the planning grip mu as an option (as shipped: 0.7, the library default).

reset() sends `load` (the Java side builds the path: the robot-init work, timed here as path prep) and
then `reset` (the start, which does no path work).
"""

import atexit
import json
import math
import os
import subprocess

import numpy as np

from common import BASE_R, RP, Limits
from followers.base import Follower

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
JAR = os.path.join(ROOT, "lib", "build", "libs", "pathcore.jar")
MAIN = "frc.team5712.path.tools.BenchServer"

_server = None


class _Server:
    def __init__(self):
        if not os.path.exists(JAR):
            raise RuntimeError(f"{JAR} is missing: run `lib/gradlew -p lib jar` first")
        java = os.path.join(os.environ["JAVA_HOME"], "bin", "java") if os.environ.get("JAVA_HOME") else "java"
        self.p = subprocess.Popen([java, "-cp", JAR, MAIN], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  text=True, encoding="utf-8", bufsize=1)
        atexit.register(self.close)

    def call(self, **msg):
        self.p.stdin.write(json.dumps(msg) + "\n")
        self.p.stdin.flush()
        line = self.p.stdout.readline()
        if not line:
            raise RuntimeError("BenchServer exited")
        reply = json.loads(line)
        if "error" in reply:
            raise RuntimeError("BenchServer: " + reply["error"])
        return reply

    def close(self):
        if self.p.poll() is None:
            try:
                self.p.stdin.write('{"op":"quit"}\n')
                self.p.stdin.flush()
                self.p.wait(timeout=5)
            except Exception:
                self.p.kill()


def server():
    """This process's BenchServer, started on first use."""
    global _server
    if _server is None or _server.p.poll() is not None:
        _server = _Server()
    return _server


def robot_json(mu):
    """The bench plant (alignsim RobotParams) as our robot file."""
    stall_current = 12.0 / RP.motor_r
    return {
        "massKg": RP.mass,
        "momentOfInertiaKgM2": RP.inertia,
        "moduleRadiusMeters": BASE_R,
        "wheelRadiusMeters": RP.wheel_radius,
        "driveGearRatio": RP.drive_gear,
        "driveMotorCount": 4,
        "statorCurrentLimitAmps": RP.stator_limit,
        "frictionCoefficient": mu,
        "maxAngularRadPerSec": Limits().max_omega,
        "motor": {"nominalVolts": 12.0, "stallTorqueNewtonMeters": RP.motor_kt * stall_current,
                  "stallCurrentAmps": stall_current, "freeSpeedRadPerSec": RP.motor_kv * 12.0},
    }


def to_ours(course):
    """The neutral course as our path file's JSON tree."""
    w = course.waypoints
    n = len(w) - 1
    wps = []
    for i, p in enumerate(w):
        m = {"x": p.x, "y": p.y}
        if i == 0 or i == n:
            a, b = (w[0], w[1]) if i == 0 else (w[n - 1], w[n])
            m["tangentDegrees"] = math.degrees(math.atan2(b.y - a.y, b.x - a.x))
        wps.append(m)
    targets = [{"waypoint": pos, "degrees": math.degrees(h)} for pos, h in course.headings()]
    lim = course.limits
    return {"schema": 1, "waypoints": wps, "rotationTargets": targets, "facePointZones": [],
            "constraintZones": [], "maxMetersPerSecond": lim.max_speed,
            "maxMetersPerSecondSquared": lim.max_accel, "startMetersPerSecond": 0.0, "endMetersPerSecond": 0.0}


class OursV2(Follower):
    """Options: `mu` (planning grip, default 0.7), plus any FollowerConfig field by its Java name
    (crossTrackKp, headingKp, latencySeconds, terminalRadiusMeters, limiterFrictionCoefficient, ...)."""

    def __init__(self, mu=0.7, limits: Limits = Limits(), **follower_options):
        self.mu = mu
        self.lim = limits
        self.options = {"maxOmegaRadPerSec": limits.max_omega, "maxAlphaRadPerSecSquared": limits.max_alpha,
                        "brakeMetersPerSecondSquared": limits.max_accel}
        self.options.update(follower_options)

    @staticmethod
    def adapt(course):
        path = to_ours(course)
        srv = server()
        srv.call(op="load", path=path, robot=robot_json(0.7))
        g = srv.call(op="geometry", step=0.02)
        return {"path": path, "geometry": np.column_stack([g["x"], g["y"]])}

    @staticmethod
    def geometry(native):
        return native["geometry"]

    def reset(self, native, start):
        self.srv = server()
        r = self.srv.call(op="load", path=native["path"], robot=robot_json(self.mu), options=self.options)
        self.build_ms = r["buildMs"]
        self.srv.call(op="reset", t=start.t, pose=[float(v) for v in start.pose],
                      vel=[float(v) for v in start.vel])
        self.finished = False

    def step(self, t, pose, vel):
        r = self.srv.call(op="step", t=float(t), pose=[float(v) for v in pose], vel=[float(v) for v in vel])
        self.finished = r["done"]
        return tuple(r["cmd"])

    def done(self):
        return self.finished
