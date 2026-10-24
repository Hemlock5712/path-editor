"""Method registry. Each entry: key -> (module, class name, default options). Options are plain dicts so
jobs pickle cleanly for multiprocessing (spawn on Windows)."""

import importlib

from courses import BLINE_DEFAULT_RADIUS, BLINE_RADIUS

METHODS = {
    "old_followpath": ("followers.old_followpath", "OldFollowPath", {}),
    "bline_readme": ("followers.bline", "BLine", {"radius": BLINE_RADIUS}),
    "bline_readme_r45": ("followers.bline", "BLine", {"radius": BLINE_DEFAULT_RADIUS}),
    "bline_curve": ("followers.bline", "BLine", {"curve": True}),
    "pathplanner": ("followers.pathplanner", "PathPlanner", {}),
    "pathplanner_hold": ("followers.pathplanner", "PathPlanner", {"hold": True}),
    "drive_to_pose_chain": ("followers.drive_to_pose_chain", "DriveToPoseChain", {}),
    "choreo": ("followers.choreo", "ChoreoLib", {}),
    "choreo_hybrid": ("followers.choreo", "ChoreoHybrid", {}),
    "bline_caps": ("followers.bline", "BLine", {"radius": BLINE_DEFAULT_RADIUS, "caps": True}),
    "ours_v2": ("followers.ours_v2", "OursV2", {}),  # as shipped: the Java FollowerConfig defaults
    "pp_path2": ("followers.pathplanner2", "PathPlanner2", {}),  # new-path branch (pre-release), as shipped
}

LABELS = {
    "old_followpath": "Old path-editor FollowPath",
    "bline_readme": "BLine (README gains, r 0.30)",
    "bline_readme_r45": "BLine (README gains, r 0.45 default)",
    "bline_curve": "BLine (README gains, curve targets, r 0.45)",
    "pathplanner": "PathPlanner (kP 5)",
    "pathplanner_hold": "PathPlanner (kP 5) + hold",
    "drive_to_pose_chain": "DriveToPose chain",
    "choreo": "Choreo + ChoreoLib follower (as shipped)",
    "choreo_hybrid": "Choreo geometry + DriveToPose finish (hybrid prototype)",
    "bline_caps": "BLine (README gains, r 0.45 + BLine-Web corner caps)",
    "ours_v2": "Ours v2 (as shipped)",
    "pp_path2": "PathPlanner path2 (new-path, pre-release), as shipped",
}


def build(key, opts=None):
    """-> (class, kwargs). opts overrides the registry defaults."""
    mod, cls, default = METHODS[key]
    kw = dict(default or {})
    kw.update(opts or {})
    return getattr(importlib.import_module(mod), cls), kw
