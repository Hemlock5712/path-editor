"""The port of PathPlanner's own optimiser (pp_optimizer.py)."""

import math
import random

import pp_optimizer
from courses import COURSES, to_bezier


def test_a_mutation_turns_the_handles_together_and_keeps_the_anchor():
    w = to_bezier(COURSES["s_curve"])["waypoints"][2]
    rand = random.Random(3)
    for _ in range(200):
        m = pp_optimizer.mutate(w, rand)
        assert m["anchor"] == w["anchor"]
        a = math.atan2(m["next"][1] - m["anchor"][1], m["next"][0] - m["anchor"][0])
        b = math.atan2(m["anchor"][1] - m["prev"][1], m["anchor"][0] - m["prev"][0])
        assert abs(math.remainder(a - b, 2 * math.pi)) < 1e-9  # still one heading through the anchor
        turn = math.degrees(math.remainder(a - pp_optimizer._heading(w), 2 * math.pi))
        assert abs(turn) <= 5.0 + 1e-9
        for h in ("prev", "next"):
            d = pp_optimizer._length(m, h) - pp_optimizer._length(w, h)
            assert pp_optimizer._length(m, h) >= pp_optimizer.MIN_CONTROL_LENGTH - 1e-12
            assert d <= 0.1 + 1e-9


def test_the_ga_never_moves_an_anchor_is_repeatable_and_never_worse(monkeypatch):
    monkeypatch.setattr(pp_optimizer, "GENERATIONS", 8)
    leg = COURSES["s_curve"]
    base = to_bezier(leg)
    a, ra = pp_optimizer.optimise_leg(leg, base, (60.0, 40.0), (30.0, 20.0), seed=5)
    b, _ = pp_optimizer.optimise_leg(leg, base, (60.0, 40.0), (30.0, 20.0), seed=5)
    assert a == b
    assert [tuple(w["anchor"]) for w in a["waypoints"]] == [tuple(w["anchor"]) for w in base["waypoints"]]
    assert ra["after"] <= ra["before"]  # a slower result is not accepted
    assert ra["evaluations"] == 1 + 50 + 8 * 45


def test_the_field_check_rejects_paths_that_leave_the_field():
    leg = COURSES["s_curve"]
    ctx = pp_optimizer._Context(leg, to_bezier(leg), (5.0, 2.0), (0.0, 0.0))
    assert not ctx.prevent_exit  # the anchors already put the robot outside a 5 x 2 m field: no check
    ctx = pp_optimizer._Context(leg, to_bezier(leg), (60.0, 40.0), (30.0, 20.0))
    assert ctx.prevent_exit and math.isfinite(ctx.fitness(to_bezier(leg)))
