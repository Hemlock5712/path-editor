# Path Editor v2

A rewrite in progress of team 5712's path editor and follower. The old version is on `master`.
See [PLAN.md](PLAN.md) for the design and the order of work.

| Folder | What's in it |
| --- | --- |
| `bench/` | Python sim bench (`alignsim`) that races our follower against PathPlanner, Choreo, BLine and CTRE |
| `lib/` | Robot library (Java, WPILib 2027), the reference version of the path maths |
| `editor/` | Web editor (React + TypeScript, `editor/app`), with a TS port of the path maths (`editor/core`); see [editor/README.md](editor/README.md) |
| `fixtures/` | Shared test vectors that the Java and TS maths must both reproduce |

Bench requirements: Python 3.11+, `numpy`, `opencv-python-headless`, and `matplotlib` for plots.
