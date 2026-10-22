"""chain_plan.joint_legs: each join takes one speed and one direction, from the joint profile."""

from chain_plan import joint_legs
from courses_extra import THREE_PIECE_LEGS


def test_joins_share_one_speed_and_direction_and_the_ends_stay_at_rest():
    legs, at = joint_legs(THREE_PIECE_LEGS)
    assert legs[0].plan_start_speed == 0.0 and legs[-1].end_speed == 0.0
    for a, b in zip(legs, legs[1:]):
        assert a.end_speed == b.plan_start_speed > 2.0
        assert a.waypoints[-1].tangent_deg == b.waypoints[0].tangent_deg
        assert (a.waypoints[-1].x, a.waypoints[-1].y) == (b.waypoints[0].x, b.waypoints[0].y)


def test_speeds_only_keeps_the_authored_directions():
    legs, at = joint_legs(THREE_PIECE_LEGS, free_joins=False)
    for a, b in zip(THREE_PIECE_LEGS, legs):
        assert [w.tangent_deg for w in a.waypoints[1:-1]] == [w.tangent_deg for w in b.waypoints[1:-1]]
    assert all(abs(d - 90.0) < 0.5 for d, _ in at.values())
