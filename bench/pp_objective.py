"""PathPlanner's control points as an objective for the editor's waypoint optimiser, for row (d) of
`run.py --pp-compare`: PathPlanner's best case, with the same freedoms the optimiser has on our paths and
scored by PathPlanner's own generator.

The optimiser (editor/app/src/compute/optimiser.ts) works in normalised coordinates, three per free waypoint:
(u, v), clipped to the unit disc and scaled by the corridor (0.5 m by default), move the waypoint; s scales its
tangent by 2^clamp(s, -1, 1). The first and last waypoints of every path stay put. Here the same vector moves
PathPlanner's control points:
  - 2027 (Bezier): each free anchor moves with both its handles (as dragging an anchor in PathPlanner does),
    and both handles' lengths scale by 2^s, directions kept (as the optimiser keeps our tangent directions);
  - path2: the free waypoint moves; it has no handles, so s is ignored (two of three coordinates count).
Rotation targets keep their waypoint-relative positions. A chain's legs are optimised together: the free
waypoints of every leg, scored by the sum of the legs' planned times. A candidate scores only if it plans.

The score is the planned time as in ppcompare.planned_seconds: PathPlanner 2027's trajectory generator (as
shipped: wheel COF 1.15) or path2's GUI simulation (as shipped).

JSON lines on stdin/stdout, for an optimiser that takes an external objective:
    {"op": "describe"}         -> {"dims", "free", "corridorMeters", "beforeSeconds"}
    {"op": "score", "x": [..]} -> {"seconds": t} (null when the candidate doesn't plan)
    {"op": "native", "x": [..]} -> the candidate's PathPlanner path(s)
    python bench/pp_objective.py <target> pp2027|path2 [--corridor 0.5]
"""

import argparse
import json
import math
import sys

import numpy as np

import ppcompare
from courses import to_bezier
from followers.pathplanner2 import to_path2


class Objective:
    def __init__(self, target, tool, corridor=0.5):
        if tool not in ("pp2027", "path2"):
            raise ValueError(tool)
        self.legs = ppcompare.targets()[target]
        self.tool, self.corridor = tool, corridor
        if tool == "pp2027":
            self.base = [to_bezier(c) for c in self.legs]
            self.free = [(k, i) for k, n in enumerate(self.base) for i in range(1, len(n["waypoints"]) - 1)]
        else:
            self.base = to_path2(self.legs)
            self.free = [(k, i) for k, wps in enumerate(self.base["paths"]) for i in range(1, len(wps) - 1)]
        self.dims = 3 * len(self.free)
        self.before = self.seconds(self.base)

    def candidate(self, x):
        x = np.asarray(x, float)
        if self.tool == "pp2027":
            out = [json.loads(json.dumps(n)) for n in self.base]
            for j, (k, i) in enumerate(self.free):
                u, v = _disc(x[3 * j], x[3 * j + 1])
                dx, dy = self.corridor * u, self.corridor * v
                scale = 2.0 ** max(-1.0, min(1.0, x[3 * j + 2]))
                w = out[k]["waypoints"][i]
                ax, ay = w["anchor"]
                for h in ("prev", "next"):
                    hx, hy = w[h]
                    w[h] = (ax + dx + (hx - ax) * scale, ay + dy + (hy - ay) * scale)
                w["anchor"] = (ax + dx, ay + dy)
            return out
        out = json.loads(json.dumps(self.base))
        for j, (k, i) in enumerate(self.free):
            u, v = _disc(x[3 * j], x[3 * j + 1])
            ax, ay = out["paths"][k][i]["xy"]
            out["paths"][k][i]["xy"] = (ax + self.corridor * u, ay + self.corridor * v)
        return out

    def seconds(self, native):
        try:
            if self.tool == "pp2027":
                t = ppcompare.planned_seconds("pp2027_hold", self.legs, native)
            else:
                t = ppcompare.planned_seconds("pp_path2", self.legs, native)
        except (ValueError, ZeroDivisionError, IndexError):
            return None
        return t if t is not None and math.isfinite(t) else None

    def score(self, x):
        return self.seconds(self.candidate(x))

    def describe(self):
        return {"dims": self.dims, "free": self.free, "corridorMeters": self.corridor, "beforeSeconds": self.before}


def _disc(u, v):
    L = math.hypot(u, v)
    return (u / L, v / L) if L > 1.0 else (u, v)


def serve(obj, inp=sys.stdin, out=sys.stdout):
    for line in inp:
        if not line.strip():
            continue
        msg = json.loads(line)
        op = msg.get("op")
        if op == "describe":
            reply = obj.describe()
        elif op == "score":
            reply = {"seconds": obj.score(msg["x"])}
        elif op == "native":
            reply = {"native": obj.candidate(msg["x"])}
        elif op == "quit":
            break
        else:
            reply = {"error": f"unknown op {op!r}"}
        out.write(json.dumps(reply) + "\n")
        out.flush()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("target")
    ap.add_argument("tool", choices=["pp2027", "path2"])
    ap.add_argument("--corridor", type=float, default=0.5)
    a = ap.parse_args()
    serve(Objective(a.target, a.tool, a.corridor))
