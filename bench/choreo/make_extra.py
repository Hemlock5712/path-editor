"""Choreo trajectories for the harder courses (bench/courses_extra.py), in their own projects so the original
ones are not regenerated:

    python bench/choreo/make_extra.py                               # write extra/ and heavy/ (params only)
    python bench/choreo/make_extra.py --cli path/to/choreo-cli.exe  # ... and generate them

extra/ has the bench robot (make_project.py's table) and one trajectory per course, made the same way as
the originals (StopPoint at both ends, the course's limits, course headings fixed), except:
- intake_drive_through: no StopPoint at the end, MaxVelocity 2 m/s there instead (the robot drives through
  the piece; minimum time makes it arrive at that speed);
- three_piece_chain: one trajectory through every leg's waypoints, headings fixed where the legs set them
  (a ChoreoLib auto follows one trajectory, split at the pickups, rather than three that each stop);
- fast_long: MaxVelocity 5 m/s, the course's cap (the motors limit it to the 4.57 m/s free speed).
heavy/ is the same robot at 68 kg, 6.8 kg m^2 and a 96 A current limit (tmax 0.0194 x 96 N m), with the
S-curve as heavy_robot. fulltraction/ is the original courses with MaxAcceleration at the full-traction
block's 11.2 m/s^2 (courses_extra.FULL_TRACTION).
"""

import argparse
import copy
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import courses  # noqa: E402,F401  (first: it adds the extra courses to COURSES)
from courses_extra import EXTRA_COURSES, EXTRAS, FULL_TRACTION  # noqa: E402
from make_project import e, project, trajectory  # noqa: E402


def extra_trajectory(name):
    t = trajectory(name)
    course = courses.COURSES[name]
    if course.end_speed > 0:
        cons = t["params"]["constraints"]
        last_stop = [c for c in cons if c["from"] == "last" and c["data"]["type"] == "StopPoint"]
        cons.remove(last_stop[0])
        cons.append({"from": "last", "to": None, "enabled": True,
                     "data": {"type": "MaxVelocity", "props": {"max": e(course.end_speed, "m / s")}}})
    return t


def heavy_project():
    p = copy.deepcopy(project())
    r = EXTRAS["heavy_robot"]["robot"]
    p["config"]["mass"] = e(r["mass"], "kg")
    p["config"]["inertia"] = e(r["inertia"], "kg m ^ 2")
    p["config"]["tmax"] = e(0.0194 * r["stator_limit"], "N * m")
    return p


def write(folder, proj, names):
    d = os.path.join(HERE, folder)
    os.makedirs(d, exist_ok=True)
    proj = dict(proj, name=folder)
    with open(os.path.join(d, folder + ".chor"), "w") as fh:
        json.dump(proj, fh, indent=1)
    for n in names:
        with open(os.path.join(d, n + ".traj"), "w") as fh:
            json.dump(extra_trajectory(n), fh, indent=1)
    return os.path.join(d, folder + ".chor")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cli", help="path to choreo-cli(.exe) from the official release")
    ap.add_argument("--only", choices=["extra", "heavy", "fulltraction"], help="write and generate one project")
    a = ap.parse_args()
    projects = {"extra": (project(), [n for n in EXTRA_COURSES if n != "heavy_robot"]),
                "heavy": (heavy_project(), ["heavy_robot"]),
                "fulltraction": (project(), sorted({courses.CHOREO_TRAJ[c].split("/")[1] for c in FULL_TRACTION}))}
    for folder, (proj, names) in projects.items():
        if a.only and folder != a.only:
            continue
        chor = write(folder, proj, names)
        if not a.cli:
            continue
        t0 = time.time()
        subprocess.run([a.cli, "--chor", chor, "--all-trajectory", "--generate"], check=True)
        print(f"{folder}: generated in {time.time() - t0:.0f} s", flush=True)
        for n in names:
            t = json.load(open(os.path.join(HERE, folder, n + ".traj")))["trajectory"]
            print(f"  {n}: {len(t['samples'])} samples, {t['samples'][-1]['t']:.3f} s" if t["samples"]
                  else f"  {n}: NOT generated", flush=True)


if __name__ == "__main__":
    main()
