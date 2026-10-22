"""Re-baseline every table at a new plant, in steps that each fit a ten-minute foreground command, and diff the
result against the tables committed before it:

    python bench/rebaseline.py start     # snapshot the committed RESULTS.md, clear old checkpoints
    python bench/rebaseline.py next      # run the next step (repeat until it says "all done"; a killed step resumes)
    python bench/rebaseline.py status
    python bench/rebaseline.py diff      # (also the last step) out/rebaseline/diff.md

Steps, in order:
- main: `run.py --both --procs 4 --resume` (sweeps, the main grid with the harder courses, pathfinding; it
  checkpoints after each sweep and pose grid, so a killed run carries on);
- ppcompare: the optimisers, then one target's runs per chunk, then ppcompare.json and its RESULTS.md block;
- musweep: the grip sweep's optimiser runs, then its runs in chunks, then musweep.json and its block;
- losses: bench/losses.py with a perfect pose and with wheel odometry, into out/rebaseline/losses_*.md;
- diff: every table row of RESULTS.md against the snapshot.

The diff also flags ours' wheel slip with a perfect pose, course by course, against the snapshot's: above
SLIP_FLAG_MS and twice what it was means planning at the new grip slides (and a cornering margin is needed).
"""

import json
import multiprocessing as mp
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
ROOT = os.path.dirname(HERE)
OUT = os.path.join(HERE, "out")
DIR = os.path.join(OUT, "rebaseline")
STATE = os.path.join(DIR, "state.json")
MD = os.path.join(HERE, "RESULTS.md")
SEEDS = list(range(10))
BUDGET_S = 480  # stop starting new chunks after this long in one call
SLIP_FLAG_MS = 300  # ours' mean slip (perfect pose) above this and above twice the snapshot's is flagged


def state():
    return json.load(open(STATE)) if os.path.exists(STATE) else None


def save(st):
    json.dump(st, open(STATE, "w"))


def start():
    os.makedirs(DIR, exist_ok=True)
    before = subprocess.run(["git", "show", "HEAD:bench/RESULTS.md"], cwd=ROOT, capture_output=True, text=True,
                            encoding="utf-8", check=True).stdout
    open(os.path.join(DIR, "before.md"), "w", encoding="utf-8").write(before)
    rev = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
    for f in ("results.partial.json",):
        if os.path.exists(os.path.join(OUT, f)):
            os.remove(os.path.join(OUT, f))
    save({"before_rev": rev, "started": time.strftime("%Y-%m-%d %H:%M"), "done": [], "rows": {}})
    print("snapshot of", rev, "->", os.path.join(DIR, "before.md"))


# ============================================================================================ chunked runs
def _chunks(st, name, jobs, job_fn):
    """Runs `jobs` through `job_fn` in chunks until done or out of budget; rows are kept in the state."""
    rows = st["rows"].setdefault(name, [])
    t0 = time.time()
    with mp.get_context("spawn").Pool(4) as pool:
        while len(rows) < len(jobs) and time.time() - t0 < BUDGET_S:
            chunk = jobs[len(rows):len(rows) + 40]
            rows += pool.map(job_fn, chunk, chunksize=1)
            save(st)
            print(f"{name}: {len(rows)}/{len(jobs)}", flush=True)
    return len(rows) == len(jobs)


def _strip(r):
    return {k: v for k, v in r.items() if k != "trace"}


def _ppc_job(a):
    import ppcompare

    return _strip(ppcompare._job(a))


def step_ppcompare(st):
    import ppcompare
    from run import agg

    names = list(ppcompare.targets())
    methods = ppcompare.RUN_METHODS + ppcompare.OPTIMISED
    if "ppc_opt" not in st:
        with mp.get_context("spawn").Pool(4) as pool:
            st["ppc_opt"] = ppcompare.prepare_optimised(pool, names)
        save(st)
        print("ppcompare: optimised")
        return False
    jobs = [(n, m, s, p) for p in ("truth", "odom") for n in names for m in methods for s in SEEDS]
    if not _chunks(st, "ppc", jobs, _ppc_job):
        return False
    by = {}
    for (n, m, _, p), r in zip(jobs, st["rows"]["ppc"]):
        by.setdefault(p, {}).setdefault(n, {}).setdefault(m, []).append(r)
    out = {"meta": {"seeds": SEEDS, "poses": ["truth", "odom"], "targets": names, "methods": methods, "pending": {},
                    "t_sim": {n: ppcompare.sim_seconds(ppcompare.targets()[n]) for n in names}, "wall_s": 0.0,
                    "path2_tuned": ppcompare.path2_tuned()},
           "runs": {p: {n: {m: agg(v) for m, v in ms.items()} for n, ms in ns.items()} for p, ns in by.items()},
           "optimiser": st["ppc_opt"]}
    with open(os.path.join(OUT, "ppcompare.json"), "w") as fh:
        json.dump(out, fh)
    ppcompare.write_md(out)
    st["rows"].pop("ppc")
    return True


def step_musweep(st):
    import musweep
    import ppcompare as P
    from run import agg

    names = list(P.targets())
    from courses import COURSES

    courses = [c for c in COURSES if c in names]
    ks = musweep.keys()
    if "mu_opt" not in st:
        with mp.get_context("spawn").Pool(4) as pool:
            st["mu_opt"] = P.prepare_optimised(pool, names, tools=[f"ours@{m:g}" for m in P.MU_SWEEP[1:]],
                                               path=P.MU_FILE)
        save(st)
        print("musweep: optimised")
        return False
    jobs = [(n, k, s, p) for p in ("truth", "odom") for n in names for k in ks for s in SEEDS]
    jobs += [(n, k, s, "slipodom") for n in courses for k in ks + musweep.PP_REF for s in SEEDS]
    if not _chunks(st, "mu", jobs, _ppc_job):
        return False
    runs = {}
    for (n, k, _, p), r in zip(jobs, st["rows"]["mu"]):
        runs.setdefault(p, {}).setdefault(n, {}).setdefault(k, []).append(r)
    out = {"meta": {"seeds": SEEDS, "targets": names, "courses": courses, "mu": list(P.MU_SWEEP), "keys": ks,
                    "wall_s": 0.0},
           "runs": {p: {n: {k: agg(v) for k, v in kk.items()} for n, kk in ns.items()} for p, ns in runs.items()},
           "optimiser": st["mu_opt"]}
    with open(os.path.join(OUT, "musweep.json"), "w") as fh:
        json.dump(out, fh)
    musweep.write_md(out, json.load(open(os.path.join(OUT, "ppcompare.json"))))
    st["rows"].pop("mu")
    return True


def step_main(st):
    r = subprocess.run([sys.executable, os.path.join(HERE, "run.py"), "--both", "--procs", "4", "--resume"], cwd=ROOT)
    return r.returncode == 0


def step_losses(st):
    for pose in ("truth", "odom"):
        f = os.path.join(DIR, f"losses_{pose}.md")
        if os.path.exists(f):
            continue
        r = subprocess.run([sys.executable, os.path.join(HERE, "losses.py"), "--pose", pose], cwd=ROOT,
                           capture_output=True, text=True, encoding="utf-8")
        if r.returncode != 0:
            print(r.stderr[-2000:])
            return False
        open(f, "w", encoding="utf-8").write(r.stdout)
        return pose == "odom"
    return True


# ============================================================================================ diff
def tables(text):
    """{(section, sub-section, table title line, first cell): [cells]} for every markdown table row."""
    out, h2, h3, title = {}, "", "", ""
    for line in text.splitlines():
        if line.startswith("## "):
            h2, h3, title = line[3:].strip(), "", ""
        elif line.startswith("### "):
            h3, title = line[4:].strip(), ""
        elif line.startswith("**") and not line.startswith("| "):
            title = re.sub(r"\*\*(.+?)\*\*.*", r"\1", line)
        elif line.startswith("| ") and not line.startswith("|---"):
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            if cells[0] in ("method", "course", "row", "target", "variant", "route"):
                continue
            out[(h2, h3, title, cells[0])] = cells[1:]
    return out


def diff():
    before = tables(open(os.path.join(DIR, "before.md"), encoding="utf-8").read())
    after = tables(open(MD, encoding="utf-8").read())
    changed = [k for k in after if k in before and after[k] != before[k]]
    L = ["# Re-baseline diff", "", f"Before: RESULTS.md at {state()['before_rev']}; after: the working copy.", "",
         f"Rows in both: {sum(1 for k in after if k in before)}; changed: {len(changed)}; new: "
         f"{sum(1 for k in after if k not in before)}; gone: {sum(1 for k in before if k not in after)}.", ""]
    L += ["## Ours' wheel slip with a perfect pose (the new grip's cornering margin)", "",
          "| course | before ms | after ms | flag |", "|---|---|---|---|"]
    res = json.load(open(os.path.join(OUT, "results.json")))
    import courses  # noqa: F401  (first: it adds the extra courses)
    from courses_extra import EXTRA_COURSES

    for c, ms in res["runs"]["truth"].items():
        a = ms["ours_v2"]["wheel_slip_ms_mean"]
        k = next((k for k in before if k[1].startswith("Perfect pose (the control law") and k[2] == c
                  and k[3] == "Ours v2 (as shipped)"), None)
        b = before[k][-2] if k else "-"
        try:
            bv = float(b)
        except ValueError:
            bv = 0.0
        flag = "SLIPS MORE" if a > max(SLIP_FLAG_MS, 2 * bv) else ""
        L.append(f"| {c}{' (harder)' if c in EXTRA_COURSES else ''} | {b} | {a:.0f} | {flag} |")
    L += ["", "## Changed rows", ""]
    for k in changed:
        L.append(f"- {' / '.join(x for x in k[:3] if x)}: **{k[3]}**")
        L.append(f"  - before: {' | '.join(before[k])}")
        L.append(f"  - after:  {' | '.join(after[k])}")
    f = os.path.join(DIR, "diff.md")
    open(f, "w", encoding="utf-8").write("\n".join(L) + "\n")
    print("wrote", f, f"({len(changed)} changed rows)")


STEPS = [("main", step_main), ("ppcompare", step_ppcompare), ("musweep", step_musweep), ("losses", step_losses)]


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    if cmd == "start":
        return start()
    st = state()
    if st is None:
        raise SystemExit("run `python bench/rebaseline.py start` first")
    if cmd == "diff":
        return diff()
    todo = [n for n, _ in STEPS if n not in st["done"]]
    if cmd == "status":
        print("done:", st["done"], "to do:", todo, "pending rows:", {k: len(v) for k, v in st["rows"].items()})
        return
    if not todo:
        diff()
        print("all done")
        return
    name = todo[0]
    if dict(STEPS)[name](st):
        st["done"].append(name)
        save(st)
        print(name, "done")
        if len(todo) == 1:
            diff()
            print("all done")


if __name__ == "__main__":
    main()
