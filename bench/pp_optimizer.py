"""PathPlanner's own path optimiser, ported: lib/util/path_optimizer.dart (mjansen4857/pathplanner, branch 2027,
7fbd798; the file is unchanged since 64e43f803a, 2024-10-03), with Waypoint.setHeading / setPrevControlLength /
setNextControlLength from lib/path/waypoint.dart.

A genetic algorithm over the path's Bezier handles. It never moves an anchor:
- a population of 50, 100 generations;
- each generation is sorted by fitness, the fittest 10 % carry over, and the other 90 % are children of two
  parents drawn uniformly from the fittest 50 %;
- a child takes each waypoint from parent 1 (probability 0.3), parent 2 (0.3) or parent 1 mutated (0.4);
- a mutation turns the waypoint's heading (the direction of its handles) by a uniform -5..+5 deg, and changes
  each handle's length by a uniform -0.1..+0.1 m (at least Waypoint.minControlLength, 0.25 m). Every
  waypoint mutates, the start and end included; the first population is the path with every waypoint mutated;
- fitness is the PathPlannerTrajectory total time (here ppcompare.planned_seconds: the bench's port of the
  generator, as shipped, wheel COF = the plant's static mu);
- if every anchor keeps the robot (radius = half its diagonal) inside the field, a candidate whose path points
  would take it outside scores infinity;
- the result is the best individual seen at the start of any generation (the GUI reports the same). The GUI
  then asks whether to accept it; here it is kept only if it plans faster than the original (the first
  population is all mutated, so the best can be slower).
Rotation targets keep their waypoint-relative positions, as in the GUI.

The Dart uses an unseeded Random; here one seeded generator (Python's) draws the same distributions in the same
order of use, so a run is repeatable. The GUI's field is the image's size in PathPlanner's corner-origin frame;
the bench's 2026 routes (centre origin) are shifted by half the field, and the bench courses, which aren't laid
out on a field, get a large empty one (as tools/optimise.mjs gets for them in ppcompare.optimise_ours).
"""

import json
import math
import random

POPULATION = 50
GENERATIONS = 100
MIN_CONTROL_LENGTH = 0.25  # Waypoint.minControlLength
ROBOT_SIZE = (0.85, 0.85)  # m, the bench robot with bumpers


def _heading(w):
    """Waypoint.heading: towards the next handle, else away from the previous one."""
    ax, ay = w["anchor"]
    if w["next"] is not None:
        return math.atan2(w["next"][1] - ay, w["next"][0] - ax)
    return math.atan2(ay - w["prev"][1], ax - w["prev"][0])


def _length(w, h):
    return math.dist(w[h], w["anchor"])


def mutate(w, rand):
    """_Individual.mutate on one waypoint dict {prev, anchor, next} -> a new dict."""
    m = dict(w)
    ax, ay = m["anchor"]
    theta = math.radians((rand.random() - 0.5) * 10.0)
    h = _heading(m) + theta
    # setHeading: both handles at their lengths along the new heading
    if m["prev"] is not None:
        L = _length(m, "prev")
        m["prev"] = (ax - L * math.cos(h), ay - L * math.sin(h))
    if m["next"] is not None:
        L = _length(m, "next")
        m["next"] = (ax + L * math.cos(h), ay + L * math.sin(h))
    for k in ("prev", "next"):  # setPrevControlLength / setNextControlLength, in that order
        if m[k] is None:
            continue
        L = _length(m, k) + (rand.random() - 0.5) * 0.2
        L = max(L if math.isfinite(L) else MIN_CONTROL_LENGTH, MIN_CONTROL_LENGTH)
        a = math.atan2(m[k][1] - ay, m[k][0] - ax)
        m[k] = (ax + L * math.cos(a), ay + L * math.sin(a))
    return m


class _Individual:
    def __init__(self, native, ctx):
        self.native = native
        self.fitness = ctx.fitness(native)


class _Context:
    def __init__(self, leg, base, field_size, offset):
        from followers.pathplanner import PathPlannerPath

        self.leg, self.base, self.PathPlannerPath = leg, base, PathPlannerPath
        r = math.hypot(ROBOT_SIZE[0] / 2, ROBOT_SIZE[1] / 2)
        self.lo = (r, r)
        self.hi = (field_size[0] - r, field_size[1] - r)
        self.offset = offset
        self.prevent_exit = all(self._inside(w["anchor"]) for w in base["waypoints"])
        self.evaluations = 0

    def _inside(self, p):
        x, y = p[0] + self.offset[0], p[1] + self.offset[1]
        return self.lo[0] <= x <= self.hi[0] and self.lo[1] <= y <= self.hi[1]

    def fitness(self, native):
        import ppcompare

        self.evaluations += 1
        if self.prevent_exit:
            if not all(self._inside(p.position) for p in self.PathPlannerPath(native).points):
                return math.inf
        try:
            t = ppcompare.planned_seconds("pp2027_hold", [self.leg], [native])
        except (ValueError, ZeroDivisionError, IndexError, OverflowError):
            return math.inf
        return t if t is not None and math.isfinite(t) else math.inf


def _with(native, waypoints):
    out = dict(native)
    out["waypoints"] = waypoints
    return out


def optimise_leg(leg, base, field_size, offset, seed=1):
    """PathOptimizer._optimizePathWaypoints on one path (a to_bezier dict) -> (best native, report)."""
    rand = random.Random(seed)
    ctx = _Context(leg, base, field_size, offset)
    before = ctx.fitness(base)
    pop = [_Individual(_with(base, [mutate(w, rand) for w in base["waypoints"]]), ctx) for _ in range(POPULATION)]
    best = None
    for gen in range(1, GENERATIONS + 1):
        pop.sort(key=lambda i: i.fitness)  # Python sorts stably; Dart makes no promise, which matters only on ties
        nxt = pop[:int(POPULATION * 0.1)]
        for _ in range(int(POPULATION * 0.9)):
            p1 = pop[rand.randrange(int(POPULATION * 0.5))]
            p2 = pop[rand.randrange(int(POPULATION * 0.5))]
            child = []
            for i, w in enumerate(p1.native["waypoints"]):
                prob = rand.random()
                if prob < 0.3:
                    child.append(w)
                elif prob < 0.6:
                    child.append(p2.native["waypoints"][i])
                else:
                    child.append(mutate(w, rand))
            nxt.append(_Individual(_with(p1.native, child), ctx))
        if best is None or pop[0].fitness < best[1]:
            best = (pop[0].native, pop[0].fitness, gen)
        pop = nxt
    native, after, gen = best
    found = after
    if not after < before:  # the GUI shows the result and asks; a user would keep the original
        native, after = base, before
    native = json.loads(json.dumps(native))  # tuples -> lists, as the stored files hold them
    return native, {"before": before, "after": after, "found": found, "evaluations": ctx.evaluations,
                    "best_generation": gen, "field_check": ctx.prevent_exit}


def optimise_target(target, seed=1):
    """Every leg of a ppcompare target through PathPlanner's optimiser -> (natives, report)."""
    import ppcompare
    from courses import to_bezier

    legs = ppcompare.targets()[target]
    on_field = target in ppcompare.ROUTES
    field = (16.54, 8.07) if on_field else (60.0, 40.0)
    offset = (field[0] / 2, field[1] / 2)
    natives, report = [], []
    for leg in legs:
        n, r = optimise_leg(leg, to_bezier(leg), field, offset, seed)
        natives.append(n)
        report.append(r)
    return natives, report
