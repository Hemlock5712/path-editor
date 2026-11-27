---
title: Glossary
description: The words these pages use, each in one or two sentences.
---

Each entry links to the page that explains it in full.

## The robot and the field

| Word | Meaning |
| --- | --- |
| **Pose** | A position on the field and the way the robot faces: x and y in meters, and a heading. |
| **Heading** | The direction the robot faces. On a swerve robot it is separate from the direction it travels. [The heading](/concepts/heading/) |
| **Field velocity** | How fast the robot is moving along the field's x and y, plus its turn rate. Also called field-relative. [What your drivetrain provides](/start/drivetrain/) |
| **Field frame** | Where the field's origin is and which way its axes point. The library uses the center of the field with +X toward the blue wall. [The field frame](/concepts/field-frame/) |
| **Blue frame** | The frame every path is drawn and stored in. On red the library flips it. [Alliances](/guides/alliances/) |
| **Real field frame** | The frame the robot's own pose is in, where things physically are. It is not flipped on red. |
| **Wheel odometry** | The robot's pose from wheels and gyro only. It never jumps, and it drifts slowly. [Tag alignment](/guides/tag-alignment/) |
| **Bumpers, footprint** | The robot's outline. A footprint is the bumpers plus a mechanism's reach past them. [Footprints](/concepts/footprints/) |

## Paths

| Word | Meaning |
| --- | --- |
| **Path** | A curve, a heading for every point on it, and a speed for every point on it. |
| **Waypoint** | A point you clicked. The path passes through it. [The curve](/concepts/the-curve/) |
| **Tangent** | The direction the path is going at a point. |
| **Curvature** | How sharply the path is turning, in 1/m, written κ. A circle of radius 2 m has curvature 0.5. |
| **s** | Distance along the path from its start, in meters. Nearly everything in the library is indexed by it. |
| **Quintic spline** | The kind of curve used between waypoints: a polynomial of degree five, which leaves enough freedom to match curvature at each end. |
| **Rotation target** | A heading the robot should have reached at a point on the path. |
| **Face-point zone** | A stretch of path where the robot keeps facing a spot on the field. |
| **Constraint zone** | A stretch of path with its own, lower, speed or acceleration limit. [The speed plan](/concepts/speed-plan/) |
| **Event** | A named mark on a path that runs a command when the robot passes it. [Events](/guides/events/) |
| **Named point** | A spot marked in the editor that robot code can drive to by name. [Drive to a pose](/guides/drive-to-pose/) |
| **Rolling end** | A path that ends still moving and hands its speed to the next. [Chaining paths](/guides/chaining-paths/) |

## Planning

| Word | Meaning |
| --- | --- |
| **Speed plan, velocity profile** | The fastest speed at each point of a path that breaks no limit. [The speed plan](/concepts/speed-plan/) |
| **Forward pass, backward pass** | The two sweeps that build the speed plan: how fast can the robot get here, and can it still slow down in time. |
| **Traction limit** | The most acceleration the tires can give before slipping, or the robot before tipping, whichever is lower. Written A. [Traction](/concepts/traction/) |
| **Traction share** | The fraction of the traction limit that paths plan to. 0.9 by default, which leaves a margin. |
| **Grip, μ** | The friction coefficient between the tread and the carpet. |
| **On-the-fly path** | A path planned when a command starts, from wherever the robot is. [On-the-fly paths](/concepts/on-the-fly/) |
| **Grown obstacle** | An obstacle made larger by the robot's reach, so the robot's center only has to stay outside it. [The pathfinder](/concepts/pathfinder/) |
| **Visibility graph** | The grown obstacles' corners, joined wherever one can see another in a straight line. The shortest route runs along it. |
| **A\*** | The search that finds the shortest route through the graph. |
| **Margin** | The extra clearance kept between the robot and an obstacle. |
| **Snap** | Moving a start or goal that is inside a grown obstacle to the nearest free point. |
| **Re-plan** | Planning again from where the robot is, after a shove or when the goal has moved. |

## Following

| Word | Meaning |
| --- | --- |
| **Follower** | The object that turns a path and the robot's measured pose into a velocity command, each loop. [The follower](/concepts/the-follower/) |
| **Loop** | One pass of the robot program, 20 ms by default. |
| **Progress** | Where the robot is along the path, as a value of s. |
| **Cross-track error** | How far the robot is off the path, sideways. |
| **Latency** | How late the drivetrain answers a command. The follower reads the plan this far ahead. |
| **Feedback** | A correction made from an error that has already happened. The P gains are feedback. |
| **Feed-forward** | A command made from what the plan is about to ask for, sent before any error appears. [Feed-forward](/concepts/feed-forward/) |
| **P gain, kP** | How strongly an error is corrected: output per unit of error. |
| **Stopping law** | The rule that caps speed by what the robot can still stop from. [The stopping law](/concepts/stopping-law/) |
| **Ease-in** | The last, gentle part of the stopping law, where speed is proportional to the distance left. |
| **Limiter** | The last stage of every command, which holds it to what the wheels can do this loop. [The output limiter](/concepts/output-limiter/) |
| **Tolerance** | How close to the goal counts as finished. 2 cm and 1° by default. |
| **Settled** | Inside the tolerance, slower than 5 cm/s, for 0.1 s. |
| **Moving goal** | An end of a path tied to a pose that robot code supplies each loop. [Moving goals](/guides/moving-goals/) |
| **Lock** | A filter has seen enough agreeing sightings to trust its estimate. [The tag filter](/concepts/tag-filter/) |

## Driver assists

| Word | Meaning |
| --- | --- |
| **Rail** | A line on the field the robot is held to while the driver drives along it. [Rails](/guides/rails/) |
| **Pull-in** | How fast a rail pulls the robot onto its line, as a share of the driver's speed along it. |
| **Intake assist** | A sideways nudge that lines the intake up on the piece the driver is heading for. [Intake assist](/guides/intake-assist/) |
| **Eagerness** | How much sooner than "just in time" the intake assist lines up. |

## WPILib and the project

| Word | Meaning |
| --- | --- |
| **Vendordep** | A library a robot project installs by putting one JSON file in its `vendordeps` folder. [Install](/start/install/) |
| **Commands v3** | WPILib 2027's command framework. Every command in the library is one. |
| **Mechanism** | Commands v3's name for a piece of the robot that one command at a time may control. The drivetrain is one. |
| **Fork** | Starting a command as a child of the running one. Event commands are forked. |
| **Op mode** | A class that runs for one mode of a match, such as an autonomous routine. |
| **Deploy folder** | `src/main/deploy`, copied to the robot with the code. The library reads `deploy/paths`. [Project files](/start/project-files/) |
| **Robot init** | The time the robot program starts, before it is enabled. All path planning happens here. |
| **Bench** | The team's Python simulator that races followers against each other. Many numbers on these pages come from it. |
