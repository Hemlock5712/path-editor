"""Row (d) of `run.py --pp-compare`: the editor's waypoint optimiser run on PathPlanner's own control points,
scored by PathPlanner's own generator. PathPlanner's best case, with the freedoms and the budget the
optimiser has on our paths.

The search is a line-for-line Python port of editor/app/src/compute/optimiser.ts (the Optimiser class behind
tools/optimise.mjs and the editor's Optimise button): Nelder-Mead with dimension-adaptive coefficients on
normalised coordinates, restarted around the best point with a halved step when the simplex collapses or
stalls, the start simplex's signs and sizes from a seeded mulberry32, and a budget of evaluations (500 per
second of budget). Only the objective differs.

The coordinates are the optimiser's: three per free waypoint. (u, v), clipped to the unit disc and scaled by
the corridor (0.5 m), move it; s scales its tangent by 2^clamp(s, -1, 1), rounded to 0.01. As in the
optimiser, the first and last waypoints of every path file and any waypoint with a set heading stay put, and
positions are rounded to 1e-5 m (what the files hold). On PathPlanner's paths:
  - 2027 (Bezier): a free anchor moves with both handles (as dragging an anchor in PathPlanner does), and
    both handles' lengths scale by the tangent scale, directions kept (as the optimiser keeps our tangent
    directions). Rotation targets keep their waypoint-relative positions.
  - path2: the free waypoint moves; it has no handles, so s changes nothing.
Each path file is optimised on its own, as the optimiser does ours: a chain's legs (2027) or paths (path2)
one at a time, each with the full budget. A candidate counts if it plans: the bench has no obstacles to keep
clear of, and neither PathPlanner has the heading checks ours has.

The score is the planned time as in ppcompare.planned_seconds: PathPlanner 2027's trajectory generator (as
shipped, wheel COF 1.15) or path2's GUI simulation (as shipped).
"""

import json
import math

import ppcompare
from courses import to_bezier
from followers.pathplanner2 import plan_time, to_path2

EVALUATIONS_PER_SECOND = 500  # optimiser.ts
DEFAULT_BUDGET_SECONDS = 3


def evaluations_for(budget_seconds):
    return max(1, _round(budget_seconds * EVALUATIONS_PER_SECOND))


def _round(x):
    """JavaScript's Math.round (halves go up)."""
    return math.floor(x + 0.5)


def _round5(x):
    return _round(x * 1e5) / 1e5


def mulberry32(seed):
    """optimiser.ts random(): the seeded generator, bit for bit."""
    a = seed & 0xFFFFFFFF

    def imul(x, y):
        return (x * y) & 0xFFFFFFFF

    def r():
        nonlocal a
        a = (a + 0x6D2B79F5) & 0xFFFFFFFF
        t = imul(a ^ (a >> 15), 1 | a)
        t = ((t + imul(t ^ (t >> 7), 61 | t)) & 0xFFFFFFFF) ^ t
        return ((t ^ (t >> 14)) & 0xFFFFFFFF) / 4294967296

    return r


# ============================================================================================ objectives
class PathFile:
    """One PathPlanner path file as the optimiser's objective. tool: 'pp2027' (base = a to_bezier dict, leg =
    its Course) or 'path2' (base = one path2 path: a list of waypoint dicts; end = its end tolerance)."""

    def __init__(self, tool, base, leg=None, end=None, corridor=0.5):
        self.tool, self.base, self.leg, self.end, self.corridor = tool, base, leg, end, corridor
        if tool == "pp2027":
            wps = leg.waypoints
            self.free = [i for i in range(1, len(wps) - 1) if wps[i].heading_deg is None]
        else:
            self.free = [i for i in range(1, len(base) - 1) if base[i]["rot"] is None]
        self.dims = 3 * len(self.free)
        self.before = self.seconds(base)

    def candidate(self, x):
        out = json.loads(json.dumps(self.base))
        for j, i in enumerate(self.free):
            u, v = x[3 * j], x[3 * j + 1]
            L = math.hypot(u, v)
            if L > 1:
                u, v = u / L, v / L
            dx, dy = self.corridor * u, self.corridor * v
            if self.tool == "pp2027":
                scale = _round(2.0 ** max(-1.0, min(1.0, x[3 * j + 2])) * 100) / 100
                w = out["waypoints"][i]
                ax, ay = w["anchor"]
                nx, ny = _round5(ax + dx), _round5(ay + dy)
                for h in ("prev", "next"):
                    hx, hy = w[h]
                    w[h] = (_round5(nx + (hx - ax) * scale), _round5(ny + (hy - ay) * scale))
                w["anchor"] = (nx, ny)
            else:
                ax, ay = out[i]["xy"]
                out[i]["xy"] = (_round5(ax + dx), _round5(ay + dy))
        return out

    def seconds(self, native):
        try:
            if self.tool == "pp2027":
                t = ppcompare.planned_seconds("pp2027_hold", [self.leg], [native])
            else:
                t = plan_time({"paths": [native], **self.end})
        except (ValueError, ZeroDivisionError, IndexError, OverflowError):
            return None
        return t if t is not None and math.isfinite(t) else None


class Optimiser:
    """optimiser.ts Optimiser: the same search, any PathFile objective."""

    def __init__(self, obj: PathFile, seed=1, max_evaluations=None):
        self.obj = obj
        self.max_evaluations = max_evaluations or evaluations_for(DEFAULT_BUDGET_SECONDS)
        self.dims = obj.dims
        self.rand = mulberry32(seed)
        self.evaluations = 0
        self.before = obj.before
        self.best_x = [0.0] * self.dims
        self.best_time = self.before
        self.best = obj.base
        self.simplex, self.values = [], []
        self.step = 0.5
        self.stalls = 0
        self.converged = False
        if self.dims > 0:
            self.restart()

    @property
    def is_converged(self):
        return self.converged or self.dims == 0

    def evaluate(self, x):
        self.evaluations += 1
        native = self.obj.candidate(x)
        t = self.obj.seconds(native)
        if t is None:
            return math.inf
        if t < self.best_time - 1e-9:
            self.best_time, self.best_x, self.best = t, list(x), native
        return t

    def restart(self):
        x0 = self.best_x
        self.simplex = [list(x0)]
        self.values = [self.evaluate(x0)]
        for i in range(self.dims):
            x = list(x0)
            x[i] = x[i] + (-1 if self.rand() < 0.5 else 1) * self.step * (0.75 + 0.5 * self.rand())
            self.simplex.append(x)
            self.values.append(self.evaluate(x))

    def iterate(self):
        if self.is_converged:
            return
        n = self.dims
        order = sorted(range(len(self.values)), key=lambda i: self.values[i])
        self.simplex = [self.simplex[i] for i in order]
        self.values = [self.values[i] for i in order]
        size = max(max(abs(v - self.simplex[0][i]) for i, v in enumerate(x)) for x in self.simplex[1:])
        if size < 2e-3 or self.stalls > 20 * n:
            self.step /= 2
            self.stalls = 0
            if self.step < 0.01:
                self.converged = True
                return
            self.restart()
            return
        alpha, gamma, rho, sigma = 1.0, 1 + 2 / n, 0.75 - 1 / (2 * n), 1 - 1 / n
        centroid = [0.0] * n
        for i in range(n):
            for d in range(n):
                centroid[d] += self.simplex[i][d] / n
        worst = self.simplex[n]

        def along(t):
            return [c + t * (c - worst[d]) for d, c in enumerate(centroid)]

        best_before = self.values[0]
        xr = along(alpha)
        fr = self.evaluate(xr)
        if fr < self.values[0]:
            xe = along(gamma)
            fe = self.evaluate(xe)
            self.simplex[n], self.values[n] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < self.values[n - 1]:
            self.simplex[n], self.values[n] = xr, fr
        else:
            outside = fr < self.values[n]
            xc = along(rho if outside else -rho)
            fc = self.evaluate(xc)
            if fc < min(fr, self.values[n]):
                self.simplex[n], self.values[n] = xc, fc
            else:
                for i in range(1, n + 1):
                    self.simplex[i] = [self.simplex[0][d] + sigma * (v - self.simplex[0][d])
                                       for d, v in enumerate(self.simplex[i])]
                    self.values[i] = self.evaluate(self.simplex[i])
        self.stalls = 0 if min(self.values) < best_before - 1e-6 else self.stalls + 1

    def run(self):
        while not self.is_converged:
            if self.evaluations >= self.max_evaluations:
                return "evaluations"
            self.iterate()
        return "converged"


# ============================================================================================ a target
def optimise_target(target, tool, budget_seconds=DEFAULT_BUDGET_SECONDS, seed=1, corridor=0.5):
    """PathPlanner's paths for a target after the optimiser, file by file: (natives, report). natives is what
    the follower takes (a list of to_bezier dicts for 2027, a to_path2 dict for path2)."""
    legs = ppcompare.targets()[target]
    evals = evaluations_for(budget_seconds)
    report = []
    if tool == "pp2027":
        natives = []
        for leg in legs:
            o = Optimiser(PathFile("pp2027", to_bezier(leg), leg=leg, corridor=corridor), seed, evals)
            stopped = o.run()
            natives.append(o.best)
            report.append({"before": o.before, "after": o.best_time, "evaluations": o.evaluations,
                           "stopped": stopped, "free": o.obj.free})
        return natives, report
    native = to_path2(legs)
    end = {"end_distance": native["end_distance"], "end_angle": native["end_angle"]}
    paths = []
    for p in native["paths"]:
        o = Optimiser(PathFile("path2", p, end=end, corridor=corridor), seed, evals)
        stopped = o.run()
        paths.append(o.best)
        report.append({"before": o.before, "after": o.best_time, "evaluations": o.evaluations,
                       "stopped": stopped, "free": o.obj.free})
    return dict(native, paths=paths), report
