"""Weight-transfer sanity check: slip against commanded acceleration on a straight, with and without a centre of
mass height. `python bench/wt_check.py`

The plant gets the bench's request pattern: a ramp of field speed requests at `a` m/s^2, updated every 20 ms (the
50 Hz loop) and applied at 250 Hz, up to 4 m/s, held, then a ramp back down to rest at the same rate. Reported: wheel
slip summed over the four wheels (ms), the largest ratio of a wheel's static force demand to its grip
(mu_static x its momentary normal force), and where in the run the slip happened.
"""

import math
import os
import sys
from dataclasses import replace

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from alignsim.plant import SwervePlant  # noqa: E402
from common import RP  # noqa: E402

DT = 0.002


def run(a, cg, vmax=4.0, hold=0.5, plant_rp=None):
    rp = replace(plant_rp or RP, cg_height=cg, wheel_radius_error=0.0)
    p = SwervePlant(rp, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), None)
    t, v_req = 0.0, 0.0
    t_up = vmax / a
    T = t_up + hold + t_up + 0.5
    slip = {"up": 0.0, "hold": 0.0, "down": 0.0, "rest": 0.0}
    worst = 0.0
    next_req = 0.0
    while t < T:
        if t >= next_req - 1e-9:  # the 50 Hz request, applied by the 250 Hz thread (the plant holds it)
            next_req += 0.02
            tr = next_req  # the request is the speed the plan wants by the next loop
            if tr <= t_up:
                v_req = a * tr
            elif tr <= t_up + hold:
                v_req = vmax
            else:
                v_req = max(0.0, vmax - a * (tr - t_up - hold))
            p.set_chassis_robot(v_req, 0.0, 0.0)
        p.step(DT)
        t += DT
        phase = "up" if t <= t_up else "hold" if t <= t_up + hold else "down" if t <= 2 * t_up + hold else "rest"
        slip[phase] += sum(1 for s in p.slipping if s) * DT
        for i in range(4):
            if p.normals[i] > 0:
                # the motor force this wheel would put down without slipping, against its grip
                worst = max(worst, abs(_motor_force(p, i)) / (rp.mu_static * p.normals[i]))
    return sum(slip.values()) * 1000, worst, {k: round(v * 1000) for k, v in slip.items()}


def _motor_force(p, i):
    """The drive force the motor puts out this step (the same sums as SwervePlant.step), for the ratio."""
    rp = p.rp
    sw = p.wheel[i]
    sp = p.sp_int[i]
    volts = p.kv_volts * sp + rp.drive_kp * (sp - sw)
    if abs(sp) > 1e-3:
        volts += math.copysign(rp.drive_ks, sp)
    volts = max(-12.0, min(12.0, volts))
    rotor = sw / rp.wheel_radius * rp.drive_gear
    amps = max(-rp.stator_limit, min(rp.stator_limit, (volts - rotor / rp.motor_kv) / rp.motor_r))
    return rp.motor_kt * amps * rp.drive_gear / rp.wheel_radius


if __name__ == "__main__":
    print("| accel m/s^2 | CG m | slip ms (4 wheels) | up / hold / down / at rest ms | worst force / grip |")
    print("|---|---|---|---|---|")
    for a in (4, 6, 8, 10, 12):
        for cg in (0.0, 0.2):
            s, w, ph = run(a, cg)
            print(f"| {a} | {cg:g} | {s:.0f} | {ph['up']} / {ph['hold']} / {ph['down']} / {ph['rest']} | {w:.2f} |")
