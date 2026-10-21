"""The editor's bench snapshot: small, still the same tables, and the committed one parses."""

import json
import os

import run


def _export():
    trace = [[i * 0.04, i * 0.0123456, -i * 0.001, 0.5] for i in range(13)]
    return {
        "schema": 1,
        "source": "out/results.json",
        "meta": {"seeds": [0, 1], "courses": ["c"], "poses": ["odom"], "tracePose": "odom", "traceSeeds": [0, 1]},
        "metrics": [],
        "methods": [{"key": "ours_v2", "label": "Ours v2"}],
        "courses": [{"key": "c", "description": "a course"}],
        "runs": {"odom": {"c": {"ours_v2": {"n": 2, "t_done": {"mean": 2.123456, "worst": None, "nan": 0}}}}},
        "geometry": {"c": {"ours_v2": [[0.00012, 1.23456]]}},
        "traces": {"c": {"0": {"ours_v2": trace}, "1": {"ours_v2": trace}}},
    }


def test_lite_keeps_the_tables_and_thins_the_traces():
    lite = run.lite_export(_export())
    assert lite["runs"]["odom"]["c"]["ours_v2"]["t_done"] == {"mean": 2.123, "worst": None, "nan": 0}
    assert lite["geometry"]["c"]["ours_v2"] == [[0.0, 1.235]]
    assert lite["meta"]["traceSeeds"] == [0]
    assert list(lite["traces"]["c"]) == ["0"]
    tr = lite["traces"]["c"]["0"]["ours_v2"]
    # every 5th sample and the last: 0, 5, 10, 12
    assert [round(p[0] / 0.04) for p in tr] == [0, 5, 10, 12]
    assert tr[1] == [0.2, 0.062, -0.005, 0.5]


def test_the_committed_snapshot_is_small_and_parses():
    path = run.LITE_EXPORT
    assert os.path.getsize(path) < 2 * 1024 * 1024
    with open(path) as fh:
        d = json.load(fh)
    assert d["schema"] == run.EDITOR_SCHEMA
    assert len(d["meta"]["traceSeeds"]) == 1
    assert set(d["traces"]) == set(d["meta"]["courses"])
