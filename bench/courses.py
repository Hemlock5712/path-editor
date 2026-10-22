"""Bench courses as data, the neutral authored path, and adapters to each method's native path.

One neutral representation per course:
- waypoints the path passes through (x, y, optional heading),
- heading targets at waypoint-relative positions (1.5 = halfway between waypoints 1 and 2),
- the start state (pose = first waypoint, plus a field velocity), the shared limits,
- optional disturbances (a lateral shove, reduced grip).

The neutral *curve* is the Catmull-Rom spline through the waypoints, with the same tangent rule as the
old path-editor `SplinePath` (interior: half the chord between neighbours; ends: the chord to the one
neighbour). Adapters turn the course into what each method authors:
- old path-editor: the same control points (it builds that Catmull-Rom itself),
- PathPlanner: cubic Bezier waypoints whose handles reproduce the same cubics exactly
  (handle = anchor +- tangent / 3), so both smooth methods follow identical geometry,
- BLine: a polyline through the waypoints with a handoff radius at every interior point,
- DriveToPose chain: one goal pose per waypoint.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from common import MU_STATIC as RP_MU
from common import Limits, wrap

# ============================================================================================ data


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    heading_deg: float | None = None  # None = no heading set here
    tangent_deg: float | None = None  # direction of travel here; None = the Catmull-Rom rule


@dataclass(frozen=True)
class HeadingTarget:
    pos: float  # waypoint-relative position
    heading_deg: float


@dataclass(frozen=True)
class Impulse:
    """A lateral shove: fires when the TRUE progress along the neutral path first passes `at_fraction`
    of its length. Force is perpendicular to the path there, positive = to the left of travel."""

    at_fraction: float
    force_n: float
    duration: float


@dataclass(frozen=True)
class Course:
    name: str
    waypoints: tuple
    heading_targets: tuple = ()
    start_velocity: tuple = (0.0, 0.0, 0.0)  # field (vx, vy, w)
    limits: Limits = field(default_factory=Limits)
    impulse: Impulse | None = None
    grip: tuple | None = None  # (mu static, mu kinetic); None = the plant default
    grip_drop: tuple | None = None  # (path fraction, mu static, mu kinetic): the floor changes mid-run
    description: str = ""
    # one leg of a chain of paths (ppcompare.py): the speed the plan starts at and hands over at, m/s
    plan_start_speed: float = 0.0
    end_speed: float = 0.0

    @property
    def start_pose(self):
        w = self.waypoints[0]
        return (w.x, w.y, math.radians(w.heading_deg))

    @property
    def end_pose(self):
        w = self.waypoints[-1]
        return (w.x, w.y, math.radians(w.heading_deg))

    def headings(self):
        """Sorted (pos, heading rad) list: start, waypoint headings and heading targets, end."""
        out = []
        for i, w in enumerate(self.waypoints):
            if w.heading_deg is not None:
                out.append((float(i), math.radians(w.heading_deg)))
        for h in self.heading_targets:
            out.append((h.pos, math.radians(h.heading_deg)))
        out.sort(key=lambda e: e[0])
        return out


# ============================================================================================ neutral curve


def catmull_rom_tangents(pts, dirs=None):
    """SplinePath's tangents: per segment i, (m0, m1). dirs[i] (radians, or None) sets the direction at
    point i, keeping the rule's length (so a chain of paths can join without a kink)."""
    n = len(pts)

    def at(i, m):
        if dirs is None or dirs[i] is None:
            return m
        L = float(np.hypot(*m))
        return np.array((L * math.cos(dirs[i]), L * math.sin(dirs[i])))

    tans = []
    for i in range(n - 1):
        p0, p1 = np.asarray(pts[i], float), np.asarray(pts[i + 1], float)
        m0 = p1 - p0 if i == 0 else 0.5 * (np.asarray(pts[i + 1]) - np.asarray(pts[i - 1]))
        m1 = p1 - p0 if i == n - 2 else 0.5 * (np.asarray(pts[i + 2]) - np.asarray(pts[i]))
        tans.append((at(i, m0), at(i + 1, m1)))
    return tans


def tangent_dirs(course):
    """Each waypoint's set direction of travel (radians) or None."""
    return [None if w.tangent_deg is None else math.radians(w.tangent_deg) for w in course.waypoints]


def hermite(p0, p1, m0, m1, t):
    t = np.asarray(t)[:, None]
    t2, t3 = t * t, t * t * t
    return (2 * t3 - 3 * t2 + 1) * p0 + (t3 - 2 * t2 + t) * m0 + (-2 * t3 + 3 * t2) * p1 + (t3 - t2) * m1


class NeutralPath:
    """The authored reference curve, densely sampled (arc length from 1-2 mm chords)."""

    PER_SEGMENT = 2000

    def __init__(self, course: Course):
        self.course = course
        pts = [(w.x, w.y) for w in course.waypoints]
        tans = catmull_rom_tangents(pts, tangent_dirs(course))
        chunks, pos = [], []
        for i, (m0, m1) in enumerate(tans):
            t = np.linspace(0.0, 1.0, self.PER_SEGMENT + 1)
            if i > 0:
                t = t[1:]
            chunks.append(hermite(np.asarray(pts[i], float), np.asarray(pts[i + 1], float), m0, m1, t))
            pos.append(i + t)
        self.points = np.vstack(chunks)
        self.pos = np.concatenate(pos)  # waypoint-relative position of each sample
        d = np.hypot(*np.diff(self.points, axis=0).T)
        self.s = np.concatenate([[0.0], np.cumsum(d)])
        self.length = float(self.s[-1])
        self._tree = None

    def s_at_pos(self, pos):
        return float(np.interp(pos, self.pos, self.s))

    def point_at_s(self, s):
        return (float(np.interp(s, self.s, self.points[:, 0])), float(np.interp(s, self.s, self.points[:, 1])))

    def tangent_at_s(self, s):
        i = int(np.clip(np.searchsorted(self.s, s), 1, len(self.s) - 1))
        d = self.points[i] - self.points[i - 1]
        return d / np.hypot(*d)

    def heading_at_pos(self, pos):
        """Linear in waypoint-relative position, shortest way round (the old editor's interpolateAlongPath)."""
        hs = self.course.headings()
        if pos <= hs[0][0]:
            return hs[0][1]
        for (p0, h0), (p1, h1) in zip(hs, hs[1:]):
            if p0 <= pos <= p1:
                u = 0.0 if p1 - p0 < 1e-9 else (pos - p0) / (p1 - p0)
                return wrap(h0 + u * wrap(h1 - h0))
        return hs[-1][1]

    def project(self, xy):
        """Closest sample: (s, distance) for an (N, 2) array of points."""
        if self._tree is None:
            from scipy.spatial import cKDTree

            self._tree = cKDTree(self.points)
        d, i = self._tree.query(np.asarray(xy))
        return self.s[i], d


def polyline_distance(poly, xy):
    """Distance from each point in xy (N, 2) to a polyline (M, 2), exact per segment."""
    poly = np.asarray(poly, float)
    xy = np.asarray(xy, float)
    a, b = poly[:-1], poly[1:]
    ab = b - a
    L2 = np.maximum((ab * ab).sum(1), 1e-18)
    out = np.empty(len(xy))
    for k0 in range(0, len(xy), 512):
        p = xy[k0:k0 + 512, None, :]
        t = np.clip(((p - a) * ab).sum(2) / L2, 0.0, 1.0)
        c = a + t[..., None] * ab
        out[k0:k0 + 512] = np.sqrt(((p - c) ** 2).sum(2)).min(1)
    return out


# ============================================================================================ adapters


def to_catmull(course: Course):
    """Old path-editor: control points + heading waypoints (fractional control-point index, rad)."""
    return {"points": [(w.x, w.y) for w in course.waypoints], "headings": course.headings()}


def to_bezier(course: Course):
    """PathPlanner Waypoints (prevControl, anchor, nextControl) that reproduce the Catmull-Rom cubics, plus
    rotation targets at waypoint-relative positions and the goal end rotation."""
    pts = [np.array((w.x, w.y), float) for w in course.waypoints]
    tans = catmull_rom_tangents(pts, tangent_dirs(course))
    wps = []
    for i, p in enumerate(pts):
        prev_c = None if i == 0 else tuple(p - tans[i - 1][1] / 3.0)
        next_c = None if i == len(pts) - 1 else tuple(p + tans[i][0] / 3.0)
        wps.append({"prev": prev_c, "anchor": tuple(p), "next": next_c})
    hs = course.headings()
    n = len(pts) - 1
    # interior rotation targets (the start heading is the robot's, the end one is the GoalEndState)
    targets = [(pos, h) for pos, h in hs if 0.0 < pos < n]
    return {"waypoints": wps, "rotation_targets": targets, "goal_rotation": course.end_pose[2],
            "ideal_start_rotation": course.start_pose[2], "goal_velocity": course.end_speed,
            "ideal_start_velocity": course.plan_start_speed}


BLINE_RADIUS = 0.30  # m: the bench's chosen handoff radius (see RESULTS.md)
BLINE_DEFAULT_RADIUS = 0.45  # m: BLine-Web projectConfig default_intermediate_handoff_radius_meters


def to_bline(course: Course, handoff_radius=BLINE_RADIUS):
    """BLine path elements, as `Path.getPathElementsWithConstraintsNoWaypoints` expands them.

    Each element is a dict: {'kind': 'T', 'xy', 'radius'} or {'kind': 'R', 'rot', 't_ratio', 'profiled'}.
    The first waypoint is the authored start (Waypoint: translation, then its rotation at t_ratio 0);
    later waypoints with a heading are Waypoints (rotation at t_ratio 1 of the incoming segment, then
    the translation); waypoints without a heading are bare TranslationTargets. Heading targets between
    waypoints become RotationTargets at t_ratio = the fractional part of their position.
    """
    wps = course.waypoints
    extra = sorted(course.heading_targets, key=lambda h: h.pos)
    els = []
    for i, w in enumerate(wps):
        if i == 0:
            els.append({"kind": "T", "xy": (w.x, w.y), "radius": handoff_radius})
            els.append({"kind": "R", "rot": math.radians(w.heading_deg), "t_ratio": 0.0, "profiled": True})
            continue
        # rotation targets that sit on the segment (i-1, i)
        for h in extra:
            if i - 1 < h.pos < i or (h.pos == i and w.heading_deg is None):
                els.append({"kind": "R", "rot": math.radians(h.heading_deg), "t_ratio": h.pos - (i - 1),
                            "profiled": True})
        if w.heading_deg is not None:
            els.append({"kind": "R", "rot": math.radians(w.heading_deg), "t_ratio": 1.0, "profiled": True})
        els.append({"kind": "T", "xy": (w.x, w.y), "radius": handoff_radius})
    return {"elements": els, "radius": handoff_radius}


def bline_corner_caps(elements, max_speed, max_accel, velocity_safety=1.0, accel_safety=1.0):
    """BLine-Web's corner seeding (core/constraints/autoVelocityConstraints.ts: createAutoVelocitySolveSetup,
    buildCorners, seedCapsFromCorners, segmentCapsFromSolvedCaps, roundConstraintValue).

    For each interior translation target with turn angle theta and handoff distance d, the effective radius is
    max(d, d / tan(theta / 2)) and the lateral cap is sqrt(a * r * f), clamped to [max(0.05, 0.05 vmax), vmax],
    with f = 0.85 for paths of at most 7 targets and 1.6 otherwise. The cap goes on the target before and after
    the corner (a cap on target k limits the approach to k). Target 1 gets min(0.5 vmax, vmax). Caps are
    floored to 0.01 m/s. BLine-Web uses these only as the starting point of its simulation-based solver (and
    skips the seeding on paths with rotation keyframes); here they are the final caps."""
    vmax, amax = max_speed * velocity_safety, max_accel * accel_safety
    idx = [i for i, e in enumerate(elements) if e["kind"] == "T"]
    pts = [elements[i]["xy"] for i in idx]
    caps = {k: vmax for k in range(2, len(pts) + 1)}
    factor = 0.85 if len(pts) <= 7 else 1.6
    min_cap = max(0.05, vmax * 0.05)
    for a in range(1, len(pts) - 1):
        (x0, y0), (x1, y1), (x2, y2) = pts[a - 1], pts[a], pts[a + 1]
        l0, l1 = math.hypot(x1 - x0, y1 - y0), math.hypot(x2 - x1, y2 - y1)
        if l0 <= 1e-9 or l1 <= 1e-9:
            continue
        dot = max(-1.0, min(1.0, ((x1 - x0) * (x2 - x1) + (y1 - y0) * (y2 - y1)) / (l0 * l1)))
        turn = math.acos(dot)
        if turn < 1e-4:
            continue
        d = elements[idx[a]]["radius"]
        tan_half = math.tan(turn / 2)
        if d <= 1e-9 or not math.isfinite(tan_half) or tan_half <= 1e-9:
            continue
        r = max(d, d / tan_half, 1e-4)
        cap = max(min_cap, min(vmax, math.sqrt(max(0.0, amax * r * factor))))
        ordinal = a + 1  # 1-based ordinal of the corner target
        caps[ordinal] = min(caps[ordinal], cap)
        caps[ordinal + 1] = min(caps[ordinal + 1], cap)

    def rnd(v):
        return max(0.01, math.floor(v * 100) / 100)

    caps[1] = min(max_speed * 0.5, vmax)
    out = [dict(e) for e in elements]
    for k, i in enumerate(idx, start=1):
        out[i]["vmax"] = rnd(min(vmax, caps[k]))
    return out


def _rdp(points, tol):
    """BLine-Web simplifyPolylineRdp (iterative instead of recursive; same kept set)."""
    def seg_dist(p, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        den = dx * dx + dy * dy
        if den <= 1e-12:
            return math.dist(p, a)
        t = min(1.0, max(0.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den))
        return math.dist(p, (a[0] + dx * t, a[1] + dy * t))

    keep = {0, len(points) - 1}
    stack = [(0, len(points) - 1)]
    while stack:
        i0, i1 = stack.pop()
        if i1 <= i0 + 1:
            continue
        far, fd = -1, -1.0
        for k in range(i0 + 1, i1):
            d = seg_dist(points[k], points[i0], points[i1])
            if d > fd:
                far, fd = k, d
        if far == -1 or fd <= tol:
            continue
        keep.add(far)
        stack += [(i0, far), (far, i1)]
    return [points[k] for k in sorted(keep)]


def _min_spacing(points, spacing, min_sep=0.015):
    """BLine-Web enforceMinimumSpacing."""
    if len(points) <= 2:
        return list(points)
    kept = [points[0]]
    for k in range(1, len(points) - 1):
        if math.dist(points[k], kept[-1]) >= spacing and math.dist(points[k], points[k + 1]) >= min_sep:
            kept.append(points[k])
    if math.dist(kept[-1], points[-1]) < min_sep:
        kept[-1] = points[-1]
    else:
        kept.append(points[-1])
    return kept


def bline_curve_targets(samples, tol=0.18, spacing=0.35, max_targets=18, snap=0.18):
    """BLine-Web core/pathProfile/curveProfile.ts createCurveTranslationTargets for one stretch between two
    anchors: dedupe at 1.5 cm, RDP at 0.18 m (x1.3 per round, up to 12 rounds, until <= 18 targets),
    minimum spacing 0.35 m, then drop end points within 0.18 m of either anchor."""
    pts = []
    for p in samples:
        if not pts or math.dist(pts[-1], p) >= 0.015:
            pts.append(tuple(p))
    best = pts
    for _ in range(13):
        best = _min_spacing(_rdp(pts, tol), spacing)
        if len(best) <= max_targets:
            break
        tol *= 1.3
    i0, i1 = 0, len(best) - 1
    if math.dist(best[i0], samples[0]) <= snap:
        i0 += 1
    if i1 >= i0 and math.dist(best[i1], samples[-1]) <= snap:
        i1 -= 1
    return best[i0:i1 + 1]


def to_bline_curve(course: Course, handoff_radius=BLINE_DEFAULT_RADIUS):
    """BLine with each stretch between waypoints drawn as a curve and turned into targets the way BLine-Web
    does (bline_curve_targets on the neutral curve); every target gets the editor's default 0.45 m handoff
    radius. Waypoints keep their headings (rotation at t_ratio 1 of the last sub-segment before them)."""
    if course.heading_targets:
        raise ValueError("to_bline_curve: fractional heading targets are not mapped")
    npath = NeutralPath(course)
    els = []
    for i, w in enumerate(course.waypoints):
        if i == 0:
            els.append({"kind": "T", "xy": (w.x, w.y), "radius": handoff_radius})
            els.append({"kind": "R", "rot": math.radians(w.heading_deg), "t_ratio": 0.0, "profiled": True})
            continue
        seg = npath.points[(npath.pos >= i - 1) & (npath.pos <= i)][::5]
        for p in bline_curve_targets([tuple(map(float, q)) for q in seg]):
            els.append({"kind": "T", "xy": p, "radius": handoff_radius})
        if w.heading_deg is not None:
            els.append({"kind": "R", "rot": math.radians(w.heading_deg), "t_ratio": 1.0, "profiled": True})
        els.append({"kind": "T", "xy": (w.x, w.y), "radius": handoff_radius})
    return {"elements": els, "radius": handoff_radius}


def to_goal_chain(course: Course):
    """DriveToPose chain: one goal pose per waypoint after the start; heading = the waypoint's, else the
    neutral heading profile there."""
    npath = NeutralPath(course)
    goals = []
    for i, w in enumerate(course.waypoints[1:], start=1):
        h = math.radians(w.heading_deg) if w.heading_deg is not None else npath.heading_at_pos(float(i))
        goals.append((w.x, w.y, h))
    return {"start": course.start_pose, "goals": goals}


# ============================================================================================ courses

def _hairpin():
    # 2 m out, a half circle around (2, 1) sampled every 30 deg (radius 1 m; the Catmull-Rom joins
    # tighten it to 0.75 m at the ends of the arc), 2 m back
    pts = [(0.0, 0.0), (1.0, 0.0)]
    for a in (-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0):
        r = math.radians(a)
        pts.append((2.0 + math.cos(r), 1.0 + math.sin(r)))
    pts += [(1.0, 2.0), (0.0, 2.0)]
    wps = [Waypoint(*p) for p in pts]
    wps[0] = Waypoint(0.0, 0.0, 0.0)
    wps[-1] = Waypoint(0.0, 2.0, 0.0)
    return tuple(wps)


# The shove keeps its size against the grip: 800 N on the old mu 1.15 plant, scaled with mu (at mu 1.5, 800 N is
# under the 883 N static grip of a 60 kg robot and would not slide it at all)
SHOVE_N = round(800.0 * RP_MU / 1.15)

S_CURVE = (Waypoint(0.0, 0.0, 0.0), Waypoint(1.25, 0.0), Waypoint(2.5, 0.8), Waypoint(3.75, 1.6),
           Waypoint(5.0, 1.6, 0.0))

COURSES = {
    "straight_3m": Course(
        "straight_3m", (Waypoint(0.0, 0.0, 0.0), Waypoint(3.0, 0.0, 0.0)),
        description="3 m straight, heading held at 0"),
    "s_curve": Course(
        "s_curve", S_CURVE,
        description="S-curve (lane change): 5 m ahead, 1.6 m to the left, min radius 1.1 m, heading held"),
    "hairpin": Course(
        "hairpin", _hairpin(),
        description="2 m out, 180 deg turn on a 1 m radius (0.75 m at the joins), 2 m back; heading held"),
    "slalom": Course(
        "slalom",
        (Waypoint(0.0, 0.0, 0.0), Waypoint(1.75, 0.4), Waypoint(3.5, -0.4), Waypoint(5.25, 0.4),
         Waypoint(7.0, 0.0, 0.0)),
        description="through 3 gate centres at x = 1.75 / 3.5 / 5.25 m, y = +0.4 / -0.4 / +0.4 m"),
    "long_auto": Course(
        "long_auto",
        (Waypoint(0.0, 0.0, 0.0), Waypoint(3.0, 0.5), Waypoint(5.5, 2.0, 90.0), Waypoint(7.5, 3.2),
         Waypoint(9.0, 2.2), Waypoint(9.3, 0.3, 180.0)),
        description="auto-style route, heading 0 -> 90 deg at waypoint 2 -> 180 deg at the end"),
    "moving_start": Course(
        "moving_start",
        (Waypoint(0.0, 0.0, 0.0), Waypoint(1.5, 0.4), Waypoint(2.6, 1.4), Waypoint(3.0, 3.0, 0.0)),
        start_velocity=(0.0, 2.0, 0.0),
        description="starts moving 2 m/s sideways (+y), path heads +x then curves left"),
    "hit_mid_path": Course(
        "hit_mid_path", S_CURVE, impulse=Impulse(at_fraction=0.5, force_n=SHOVE_N, duration=0.3),
        description="the S-curve with a 0.3 s lateral shove (to the left of travel) at half distance"),
    "low_grip_s_curve": Course(
        "low_grip_s_curve", S_CURVE, grip=(0.91, 0.79),
        description="the S-curve on mu 0.91 static / 0.79 kinetic (61 % of normal grip)"),
    "grip_drop_s_curve": Course(
        "grip_drop_s_curve", S_CURVE, grip_drop=(0.5, 0.91, 0.79),
        description="the S-curve; halfway along, the floor drops from mu 1.5 / 1.3 to 0.91 / 0.79 (a worn carpet seam)"),
}

QUICK_COURSES = ["straight_3m", "s_curve", "moving_start"]

# course -> Choreo trajectory (bench/choreo/*.traj); courses that only change the plant reuse the S-curve's
CHOREO_TRAJ = {c: c for c in ("straight_3m", "s_curve", "hairpin", "slalom", "long_auto", "moving_start")}
CHOREO_TRAJ.update({"hit_mid_path": "s_curve", "low_grip_s_curve": "s_curve", "grip_drop_s_curve": "s_curve"})


def t_end(course: Course):
    """Simulated time per run: generous for slow tails."""
    return 4.0 + NeutralPath(course).length / 1.2


# the harder courses (courses_extra.py; what they need beyond a path is in scenarios.py). Their Choreo
# trajectories are in bench/choreo/extra, and heavy/ (a project with the heavy robot), from choreo/make_extra.py
from courses_extra import EXTRA_COURSES  # noqa: E402

COURSES.update(EXTRA_COURSES)
CHOREO_TRAJ.update({c: "extra/" + c for c in EXTRA_COURSES if c != "heavy_robot"})
CHOREO_TRAJ["heavy_robot"] = "heavy/heavy_robot"

# the full-traction block's copies of the original nine (run.py runs them apart from the main grid)
from courses_extra import FULL_TRACTION  # noqa: E402

CHOREO_TRAJ.update({c: "fulltraction/ft_" + CHOREO_TRAJ[c[3:]] for c in FULL_TRACTION})
COURSES.update(FULL_TRACTION)
MAIN_COURSES = [c for c in COURSES if c not in FULL_TRACTION]
