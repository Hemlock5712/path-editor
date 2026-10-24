# alignsim: the bench's simulated robot

The parts of the alignsim simulator (copied from Precision-Videos `record/v2-race/sim`) that the bench still uses.

| Module | What it models |
| --- | --- |
| `params.py` | Every number: the robot (mass, inertia, modules, Kraken X60 FOC, stator limit, tread) and loop timing |
| `plant.py` | Swerve physics at 500 Hz, described below |
| `odometry.py` | WPILib swerve odometry (least squares + gyro) |
| `geom.py` | Pose maths |

**`plant.py`** models each module's steering lag. The drive motor is a Kraken X60 FOC torque-speed
curve with a stator current limit and an onboard velocity loop. Wheel contact is stick/slip: a brush
tire model up to the friction circle, then kinetic sliding, with the load on each wheel shifting with
the acceleration (centre of mass 0.2 m up) and a tipping limit. So hard commands really spin wheels, and
**the encoders over-read when that happens**. It also has a gyro with noise, drift and scale error.

`bench/sim.py` runs the loop timing on top of it: physics at 500 Hz, a 250 Hz odometry thread that applies the
newest request, and the 50 Hz robot loop.
