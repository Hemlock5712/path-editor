"""Write the Choreo project for the bench courses and, with --cli, generate the trajectories headless.

    python bench/choreo/make_project.py                                  # write bench.chor + *.traj params
    python bench/choreo/make_project.py --cli path/to/choreo-cli.exe     # ... and generate them

Choreo v2027.0.0-alpha-3 (github.com/SleipnirGroup/Choreo, tag v2027.0.0-alpha-3, 3a9e15f). The robot
config matches the alignsim plant (bench/alignsim/params.py RobotParams):

| Choreo field | value | from |
|---|---|---|
| frontLeft / backLeft | (+0.29, +0.29) / (-0.29, +0.29) m | module_xy 0.29 |
| mass | 60 kg | mass |
| inertia | 6 kg m^2 | inertia |
| gearing | 6.75 | drive_gear |
| radius | 0.0508 m | wheel_radius (2 in) |
| vmax | 607.37 rad/s (motor) | Kraken X60 FOC free speed, motor_kv 50.6 rad/s/V x 12 V = 5800 rpm |
| tmax | 2.328 N m (motor) | motor_kt 0.0194 N m/A x stator_limit 120 A |
| cof | 1.15 | mu_static (same value PathPlanner gets) |
| bumper front / side / back | 0.425 m | the 0.85 m x 0.85 m robot with bumpers |

Constraints on every trajectory: StopPoint at the first and last waypoint, MaxVelocity 4.0 m/s,
MaxAcceleration 6 m/s^2 and MaxAngularVelocity 540 deg/s over the whole path (Choreo has no angular
acceleration constraint; that limit comes from the motors and grip only). Waypoints are the course
waypoints; a waypoint with a course heading has fixHeading, the others are translation-only (fixHeading
false, heading guess from the neutral heading profile). targetDt 0.05 s, interval counts guessed by Choreo.
The moving-start course has no initial-velocity constraint in Choreo, so its trajectory starts at rest.
"""

import argparse
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from courses import CHOREO_TRAJ as TRAJ_FOR_COURSE  # noqa: E402
from courses import COURSES, NeutralPath  # noqa: E402

PROJECT = "bench"

MOTOR_FREE = 50.6 * 12.0  # rad/s
MOTOR_TMAX = 0.0194 * 120.0  # N m


def e(val, unit=""):
    return {"exp": f"{val!r} {unit}".strip(), "val": val}


def project():
    return {
        "name": PROJECT, "version": 2, "type": "Swerve",
        "variables": {"expressions": {}, "poses": {}},
        "config": {
            "frontLeft": {"x": e(0.29, "m"), "y": e(0.29, "m")},
            "backLeft": {"x": e(-0.29, "m"), "y": e(0.29, "m")},
            "mass": e(60.0, "kg"), "inertia": e(6.0, "kg m ^ 2"), "gearing": e(6.75), "radius": e(0.0508, "m"),
            "vmax": e(MOTOR_FREE, "rad / s"), "tmax": e(MOTOR_TMAX, "N * m"), "cof": e(1.15),
            "bumper": {"front": e(0.425, "m"), "side": e(0.425, "m"), "back": e(0.425, "m")},
            "differentialTrackWidth": e(0.58, "m"),
        },
        "generationFeatures": [],
        "codegen": {"root": None, "genVars": False, "genTrajData": False, "useChoreoLib": False},
    }


def trajectory(name):
    course = COURSES[name]
    lim = course.limits
    npath = NeutralPath(course)
    wps = []
    for i, w in enumerate(course.waypoints):
        h = math.radians(w.heading_deg) if w.heading_deg is not None else npath.heading_at_pos(float(i))
        wps.append({"x": e(w.x, "m"), "y": e(w.y, "m"), "heading": e(h, "rad"), "intervals": 40, "split": False,
                    "fixTranslation": True, "fixHeading": w.heading_deg is not None, "overrideIntervals": False})

    def con(frm, to, typ, props):
        return {"from": frm, "to": to, "data": {"type": typ, "props": props}, "enabled": True}

    cons = [con("first", None, "StopPoint", {}), con("last", None, "StopPoint", {}),
            con("first", "last", "MaxVelocity", {"max": e(lim.max_speed, "m / s")}),
            con("first", "last", "MaxAcceleration", {"max": e(lim.max_accel, "m / s ^ 2")}),
            con("first", "last", "MaxAngularVelocity", {"max": e(lim.max_omega, "rad / s")})]
    return {"name": name, "version": 3, "snapshot": None,
            "params": {"waypoints": wps, "constraints": cons, "targetDt": e(0.05, "s")},
            "trajectory": {"config": None, "sampleType": None, "waypoints": [], "samples": [], "splits": []},
            "events": []}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cli", help="path to choreo-cli(.exe) from the official release")
    a = ap.parse_args()
    with open(os.path.join(HERE, PROJECT + ".chor"), "w") as fh:
        json.dump(project(), fh, indent=1)
    for name in sorted(set(TRAJ_FOR_COURSE.values())):
        with open(os.path.join(HERE, name + ".traj"), "w") as fh:
            json.dump(trajectory(name), fh, indent=1)
    if a.cli:
        subprocess.run([a.cli, "--chor", os.path.join(HERE, PROJECT + ".chor"), "--all-trajectory", "--generate"],
                       check=True)
        for name in sorted(set(TRAJ_FOR_COURSE.values())):
            t = json.load(open(os.path.join(HERE, name + ".traj")))["trajectory"]
            print(f"{name}: {len(t['samples'])} samples, {t['samples'][-1]['t']:.3f} s" if t["samples"]
                  else f"{name}: NOT generated")


if __name__ == "__main__":
    main()
