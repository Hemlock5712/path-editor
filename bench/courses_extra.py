"""Harder courses (phase 7 bench): data only. The behaviour each needs (a moving goal, a piece update, a chain of
paths, a heavier robot, a wall) is in scenarios.py; courses.py adds these to COURSES at its end.

Each entry of EXTRAS says what is special about a course:
- `look`: one line for the editor's Bench page ("what to look at");
- `legs`: a chain of paths (followers.chain.Chain runs one path per leg, rolling handoffs where end_speed > 0);
- `goal_shift`: (reveal distance m, dx m, dy m, dtheta deg): the true goal is off the map pose by this much, and
  is revealed when the measured distance to the map goal drops below the reveal distance (a tag lock);
- `piece`: (reveal distance m, lateral offset m): a game piece at the path's end (driven through at the end
  speed); the detection moves it sideways by the offset (sign alternating with the seed) when the measured
  distance to it drops below the reveal distance;
- `walls`: field walls (('y', 0.575) = the line y = 0.575 with the field below it; ('x', 6.575) = x = 6.575 with
  the field to its left), for the closest-approach metric;
- `robot`: plant overrides (mass kg, inertia kg m^2, stator current limit A); methods with a robot model are
  given the same numbers;
"""

import math

from common import Limits
from courses import S_CURVE, Course, Waypoint


def W(x, y, h=None, t=None):
    return Waypoint(x, y, h, t)


# ---------------------------------------------------------------------------------------------- figure eight
def _figure_eight():
    # x = 2 sin t, y = 0.9 sin 2t (a lemniscate); t from -1.2 to pi + 1.2: from the lower left through the
    # origin (heading up and right), round the whole right lobe, back through the origin (up and left) and
    # out to the upper left. One crossing, passed 5.8 m of path apart
    n = 18
    pts = []
    for k in range(n + 1):
        t = -1.2 + k * (math.pi + 2.4) / n
        pts.append((round(2.0 * math.sin(t), 4), round(0.9 * math.sin(2 * t), 4)))
    wps = [W(*p) for p in pts]
    wps[0] = W(*pts[0], 0.0)
    wps[-1] = W(*pts[-1], 0.0)
    return tuple(wps)


# ---------------------------------------------------------------------------------------------- wall hug
WALL_Y = 0.575  # the top wall: 0.575 m from the path = 15 cm from the 0.85 m robot's bumper
WALL_X = 6.575  # the right wall, the same distance from the second leg


def _wall_hug():
    pts = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0), (5.0, 0.0)]
    for a in (60.0, 30.0, 0.0):  # a 1 m-radius quarter turn round (5, -1), down the right wall
        r = math.radians(a)
        pts.append((5.0 + math.cos(r), -1.0 + math.sin(r)))
    pts += [(6.0, -2.0), (6.0, -3.0)]
    wps = [W(*p) for p in pts]
    wps[0] = W(0.0, 0.0, 0.0)
    # the turn's ends get their direction of travel set, so the curve doesn't bulge 1 cm toward the walls there
    wps[3] = W(5.0, 0.0, None, 0.0)
    wps[6] = W(6.0, -1.0, None, -90.0)
    wps[-1] = W(6.0, -3.0, 0.0)
    return tuple(wps)


# ---------------------------------------------------------------------------------------------- three pieces
# core-builder's neutral_three_piece (bench/ppcompare.py on v2-ppcompare, 8c5b6fc..d5c9273): the 2026 field,
# centre origin, blue alliance; scoring poses face the hub's centre
HUB = (-3.645, 0.0)
SHOT_C = (-5.70, 2.20)
ROLL = 2.0


def aim(x, y):
    return math.degrees(math.atan2(HUB[1] - y, HUB[0] - x))


THREE_PIECE_LEGS = (
    Course("trench_out", (W(-4.70, -3.45, 0.0, 0.0), W(-3.645, -3.45), W(-2.40, -3.20), W(-1.20, -2.20, 90.0, 90.0)),
           end_speed=ROLL),
    Course("sweep", (W(-1.20, -2.20, 90.0, 90.0), W(-1.20, -0.80), W(-1.20, 0.80), W(-1.20, 2.20, 90.0, 90.0)),
           plan_start_speed=ROLL, end_speed=ROLL),
    Course("trench_back", (W(-1.20, 2.20, 90.0, 90.0), W(-2.40, 3.20), W(-3.645, 3.45), W(-4.90, 3.30),
                           W(*SHOT_C, aim(*SHOT_C))), plan_start_speed=ROLL),
)


def _joined(legs):
    wps = list(legs[0].waypoints)
    for leg in legs[1:]:
        wps += list(leg.waypoints[1:])
    return tuple(wps)


EXTRA_COURSES = {
    "spin_while_driving": Course(
        "spin_while_driving",
        (W(0.0, 0.0, 0.0), W(1.25, 0.0, 90.0), W(2.5, 0.0, 180.0), W(3.75, 0.0, 270.0), W(5.0, 0.0, 360.0)),
        description="5 m straight while the robot turns a full 360 deg (a heading target at every waypoint)"),
    "figure_eight": Course(
        "figure_eight", _figure_eight(),
        description="a self-crossing loop (a lemniscate, 4 m x 1.8 m) through the origin twice; heading held"),
    "wall_hug": Course(
        "wall_hug", _wall_hug(),
        description="along a wall with the bumpers 15 cm off it, then a 90 deg turn to run 15 cm off the next wall"),
    "tag_align_moving_goal": Course(
        "tag_align_moving_goal", (W(0.0, 0.0, 0.0), W(2.5, 0.5), W(5.0, 0.0, 0.0)),
        description="5 m to a scoring pose; the true goal is 8 cm sideways and 3 deg off the map pose, revealed 1.5 m out"),
    "intake_drive_through": Course(
        "intake_drive_through", (W(0.0, 0.0, 0.0), W(2.0, 0.4), W(4.0, 0.6, 0.0)), end_speed=ROLL,
        description="drive through a game piece at 2 m/s; its detected position moves 10 cm sideways 2 m out"),
    "three_piece_chain": Course(
        "three_piece_chain", _joined(THREE_PIECE_LEGS),
        description="three paths with rolling handoffs at 2 m/s (core-builder's neutral_three_piece, 2026 field)"),
    "fast_long": Course(
        "fast_long", (W(0.0, 0.0, 0.0), W(2.0, 0.5), W(4.0, 0.7), W(6.0, 0.5), W(8.0, 0.0, 0.0)),
        limits=Limits(max_speed=5.0),
        description="an 8 m gentle curve with a 5 m/s cap (above the robot's 4.57 m/s free speed)"),
    "heavy_robot": Course(
        "heavy_robot", S_CURVE,
        description="the S-curve with a 68 kg robot (MOI 6.8) and a 96 A current limit (20 % lower)"),
}

EXTRAS = {
    "spin_while_driving": {
        "look": "the full turn on the way: who keeps to the line while spinning, and who slows or slips to do it"},
    "figure_eight": {
        "look": "the crossing: does progress stay on the right lap, or jump to the other one"},
    "wall_hug": {
        "walls": (("y", WALL_Y), ("x", WALL_X)),
        "look": "the closest the bumpers get to the walls (15 cm to spare): who runs wide at the turn"},
    "tag_align_moving_goal": {
        "goal_shift": (1.5, 0.0, 0.08, 3.0),
        "look": "time to settle on the TRUE goal after the tag moves it 8 cm and 3 deg, and the error left there"},
    "intake_drive_through": {
        "piece": (2.0, 0.10),
        "look": "how close each passes the piece after it moves 10 cm, and whether it is still going at 2 m/s"},
    "three_piece_chain": {
        "legs": THREE_PIECE_LEGS,
        "look": "the whole auto's time with two rolling handoffs, and whether the hand-overs stay smooth"},
    "fast_long": {
        "look": "who can use the top speed: time on a long gentle curve with the cap above what the robot can do"},
    "heavy_robot": {
        "robot": {"mass": 68.0, "inertia": 6.8, "stator_limit": 96.0},
        "look": "a heavier, weaker robot: who plans with the right robot and still settles cleanly"},
}
