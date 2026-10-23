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
- the robot file is the bench plant's numbers (mass, inertia, module radius, gearing, Kraken curve, stator limit),
  with the grip mu as an option (OURS_MU: the tread's 1.5).

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
# PATHCORE_JAR runs the bench on another build, e.g. a saved copy while the library is being changed
JAR = os.environ.get("PATHCORE_JAR") or os.path.join(ROOT, "lib", "build", "libs", "pathcore.jar")
MAIN = "frc.team5712.path.tools.BenchServer"
LOG_DIR = os.path.join(ROOT, "bench", "out")  # BenchServer stderr, one file per worker (gitignored)

_server = None

# The robot file's frictionCoefficient: the tread's mu. The library plans and limits at 0.9 x min(mu g, g tip / h)
# (lib/README.md, Traction); before that change it planned at this mu directly (shipped 0.7).
OURS_MU = 1.5
# The stopping law's deceleration: the shared limit, but never above the library's default (6 m/s^2, the team's
# DriveToPose DECEL; braking harder costs tag accuracy, MU15.md). Raising a course's acceleration cap (the
# full-traction block) leaves it at 6.
BRAKE = 6.0


class _Server:
    def __init__(self):
        if not os.path.exists(JAR):
            raise RuntimeError(f"{JAR} is missing: run `lib/gradlew -p lib jar` first")
        java = os.path.join(os.environ["JAVA_HOME"], "bin", "java") if os.environ.get("JAVA_HOME") else "java"
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log = open(os.path.join(LOG_DIR, f"benchserver-{os.getpid()}.log"), "a")
        # a small footprint: one of these runs per bench worker, next to its Python process
        self.p = subprocess.Popen([java, "-Xmx96m", "-Xss512k", "-XX:+UseSerialGC", "-XX:ReservedCodeCacheSize=32m",
                                   "-XX:MaxMetaspaceSize=48m", "-XX:CICompilerCount=2", "-cp", JAR, MAIN],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.log,
                                  text=True, encoding="utf-8", bufsize=1)
        atexit.register(self.close)

    def call(self, **msg):
        self.p.stdin.write(json.dumps(msg) + "\n")
        self.p.stdin.flush()
        line = self.p.stdout.readline()
        if not line:
            raise RuntimeError("BenchServer exited")
        try:
            reply = json.loads(line)
        except ValueError:  # a JVM that failed to start (out of memory) prints its error here
            self.p.kill()
            raise RuntimeError("BenchServer printed a non-JSON line: " + line[:200].strip())
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
    """This process's BenchServer, started on first use (and again if it has died)."""
    global _server
    if _server is None or _server.p.poll() is not None:
        _server = _Server()
    return _server


def call_fresh(**msg):
    """A call that may start a run: if the server has died, start a new one and try once more."""
    try:
        return server().call(**msg)
    except (RuntimeError, OSError) as e:
        if "BenchServer:" in str(e):  # an error reply, not a dead process
            raise
        global _server
        _server = None
        return server().call(**msg)


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
        "centerOfMassHeightMeters": RP.cg_height,  # the library's tip limit (tip distance: the square base's)
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
        if p.tangent_deg is not None:
            m["tangentDegrees"] = p.tangent_deg
        elif i == 0 or i == n:
            a, b = (w[0], w[1]) if i == 0 else (w[n - 1], w[n])
            m["tangentDegrees"] = math.degrees(math.atan2(b.y - a.y, b.x - a.x))
        wps.append(m)
    targets = [{"waypoint": pos, "degrees": math.degrees(h)} for pos, h in course.headings()]
    lim = course.limits
    return {"schema": 1, "waypoints": wps, "rotationTargets": targets, "facePointZones": [],
            "constraintZones": [], "maxMetersPerSecond": lim.max_speed,
            "maxMetersPerSecondSquared": lim.max_accel, "startMetersPerSecond": course.plan_start_speed,
            "endMetersPerSecond": course.end_speed}


class OursV2(Follower):
    """Options: `mu` (the robot file's grip, default OURS_MU), `robot` (extra robot file fields), plus any FollowerConfig field by its Java name
    (crossTrackKp, headingKp, latencySeconds, terminalRadiusMeters, limiterFrictionCoefficient, ...)."""

    robot_overrides = {}  # robot file fields that differ from the bench plant (set for the heavy_robot course)

    def __init__(self, mu=OURS_MU, limits: Limits = Limits(), robot=None, plan_speed=None, plan_accel=None,
                 **follower_options):
        """plan_speed / plan_accel: the path's own speed and acceleration caps in place of the course's (the
        robot's real limits instead of the shared ones)."""
        self.plan_speed, self.plan_accel = plan_speed, plan_accel
        self.mu = mu
        if robot:  # robot file fields beyond the plant's (e.g. the traction rule)
            self.robot_overrides = dict(robot)
        self.lim = limits
        self.options = {"maxOmegaRadPerSec": limits.max_omega, "maxAlphaRadPerSecSquared": limits.max_alpha,
                        "brakeMetersPerSecondSquared": min(limits.max_accel, BRAKE)}
        self.options.update(follower_options)
        # adaptive grip (on by default) needs the slip detector fed every odometry tick (see slipsense.py)
        self.wants_sensing = bool(self.options.get("adaptiveGrip", 1))

    @staticmethod
    def adapt(course):
        path = to_ours(course)
        call_fresh(op="load", path=path, robot=robot_json(OURS_MU))
        g = server().call(op="geometry", step=0.02)
        return {"path": path, "geometry": np.column_stack([g["x"], g["y"]])}

    @staticmethod
    def geometry(native):
        return native["geometry"]

    def reset(self, native, start):
        # a path that starts after t = 0 is the next one of a chain: it keeps the run's slip-sensing session
        src = {"trajectory": native["trajectory"]} if "trajectory" in native else {"path": native["path"]}
        if "path" in src and (self.plan_speed or self.plan_accel):
            src["path"] = dict(src["path"])
            # raised, never lowered: a course faster than this keeps its own cap
            if self.plan_speed:
                src["path"]["maxMetersPerSecond"] = max(self.plan_speed, src["path"]["maxMetersPerSecond"])
            if self.plan_accel:
                src["path"]["maxMetersPerSecondSquared"] = max(self.plan_accel, src["path"]["maxMetersPerSecondSquared"])
        r = call_fresh(op="load", **src, robot=dict(robot_json(self.mu), **self.robot_overrides),
                       options=self.options, keepSensing=start.t > 0)
        self.srv = server()
        self.build_ms = r["buildMs"]
        extra = {} if start.handed is None else {"handed": [float(v) for v in start.handed]}
        self.srv.call(op="reset", t=start.t, pose=[float(v) for v in start.pose],
                      vel=[float(v) for v in start.vel], **extra)
        self.finished = False

    def step(self, t, pose, vel):
        r = self.srv.call(op="step", t=float(t), pose=[float(v) for v in pose], vel=[float(v) for v in vel])
        self.finished = r["done"]
        return tuple(r["cmd"])

    def done(self):
        return self.finished


class OursOnChoreo(OursV2):
    """Ours v2's follower on a Choreo trajectory (the course's .traj): Path.fromTrajectory builds our path from
    the samples (spline through their positions, Choreo's heading and speed as tables), and the same
    PathFollower follows it by position with our stopping law at the end."""

    @staticmethod
    def adapt(course):
        from followers.choreo import load_traj

        s = load_traj(course.name)["samples"]
        traj = {k: [float(r[i]) for r in s] for i, k in enumerate(("t", "x", "y", "heading", "vx", "vy"))}
        call_fresh(op="load", trajectory=traj, robot=robot_json(OURS_MU))
        g = server().call(op="geometry", step=0.02)
        return {"trajectory": traj, "geometry": np.column_stack([g["x"], g["y"]])}
