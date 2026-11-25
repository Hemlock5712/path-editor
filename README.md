# Myelin

Myelin is team 5712's path editor and follower. The old version is on `master`.
See [lib/README.md](lib/README.md) for how it works, [RESEARCH.md](RESEARCH.md) for how the follower compares with
control practice outside FRC and what to try next, and [PLAN.md](PLAN.md) for why it was built this way.

| Folder | What's in it |
| --- | --- |
| `bench/` | Python sim bench (`alignsim`) that races our follower against PathPlanner, Choreo, BLine and CTRE |
| `lib/` | Robot library (Java, WPILib 2027), the reference version of the path maths |
| `editor/` | Web editor (React + TypeScript, `editor/app`), with a TS port of the path maths (`editor/core`); see [editor/README.md](editor/README.md) |
| `fixtures/` | Shared test vectors that the Java and TS maths must both reproduce |

Bench requirements: Python 3.11+, `numpy`, `opencv-python-headless`, and `matplotlib` for plots.

## End-to-end tests

One command from the repo root runs the whole chain: the editor authors every test path (Playwright), the Java
suite loads the files with `PathLibrary.loadAll()` and runs them with the real follower through the WPILib adapters
in simulation, and the results go to [e2e/RESULTS.md](e2e/RESULTS.md).

```
node tools/e2e.mjs                    # or: npm --prefix editor/app run e2e:full
node tools/e2e.mjs --skip-author      # the Java half only, on the last authored files (e2e/out)
```

It needs Node 22, the WPILib 2027 install (for the adapters) and Microsoft Edge (Playwright drives it).

