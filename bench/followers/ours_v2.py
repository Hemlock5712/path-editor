"""Slot for our v2 follower (PLAN.md phase 3). Not built yet; run.py skips it."""

from followers.base import Follower


class OursV2(Follower):
    def __init__(self, **_):
        raise NotImplementedError("ours v2 is phase 3 (see PLAN.md)")
