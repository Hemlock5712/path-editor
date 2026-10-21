"""A chain of paths run back to back, as a command group would: one follower per leg, the next leg's command
starting the loop after the last one ends (WPILib's SequentialCommandGroup initialises the next command when
one finishes and runs it from the next loop). Legs that hand over at speed have `end_speed` > 0 in the
course; each method authors that its own way (our endMetersPerSecond, PathPlanner's GoalEndState).
"""

import numpy as np

from followers import build
from followers.base import Follower, Start


class Chain(Follower):
    def __init__(self, inner, **inner_options):
        cls, kw = build(inner, inner_options)
        self.cls = cls
        self.f = cls(**kw)
        self.end_output = self.f.end_output

    def adapt(self, legs):
        return [self.f.adapt(leg) for leg in legs]

    def geometry(self, native):
        return np.vstack([np.asarray(self.cls.geometry(n)) for n in native])

    def reset(self, native, start):
        self.legs = native
        self.k = 0
        self.switch = False
        self.finished = False
        self.f.reset(native[0], start)
        self.build_ms = getattr(self.f, "build_ms", None)

    def step(self, t, pose, vel):
        if self.switch:
            self.k += 1
            self.switch = False
            self.f.reset(self.legs[self.k], Start(t, pose, vel))
        cmd = self.f.step(t, pose, vel)
        if self.f.done():
            if self.k == len(self.legs) - 1:
                self.finished = True
            else:
                self.switch = True
        return cmd

    def done(self):
        return self.finished
