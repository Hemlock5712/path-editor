"""Every follower reaches the end of the straight course with a perfect pose."""

import math

import pytest

from sim import run_one

CONVERGING = {  # method key, options -> final true error bound (cm)
    ("pathplanner", None): 5.0,  # time-sampled, stops when the trajectory time runs out
    ("pathplanner_hold", None): 2.0,
    ("drive_to_pose_chain", None): 2.0,
    ("bline_readme", (("kp", 2.0), ("tol", 0.01))): 2.0,
}


@pytest.mark.parametrize("key,opts", list(CONVERGING))
def test_reaches_end_of_straight(key, opts):
    r = run_one(("straight_3m", key, dict(opts) if opts else None, 0, "truth"))
    assert not math.isnan(r["t_5cm"])
    assert r["err_end_cm"] < CONVERGING[(key, opts)]
    assert r["ct_max_cm"] < 1.0


@pytest.mark.parametrize("key", ["bline_readme", "bline_readme_r45", "bline_curve", "pathplanner"])
def test_as_shipped_variants_pass_the_end(key):
    # BLine with the README gains overshoots the end (P 5 on distance cannot brake at 6 m/s^2) but must
    # still get there and declare itself done.
    r = run_one(("straight_3m", key, None, 0, "truth"))
    assert not math.isnan(r["t_5cm"]) and not math.isnan(r["t_done"])
