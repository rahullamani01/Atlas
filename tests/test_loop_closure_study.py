import importlib.util
from pathlib import Path
from types import SimpleNamespace

spec = importlib.util.spec_from_file_location(
    "loop_closure_study", Path(__file__).resolve().parents[1] / "scripts/loop_closure_study.py"
)
loop_closure_study = importlib.util.module_from_spec(spec)
spec.loader.exec_module(loop_closure_study)


def test_study_pairs_seeds_and_saves_configuration(monkeypatch, tmp_path):
    calls = []

    def fake_run(scenario, seed):
        calls.append((seed, scenario.loop_closure))
        return SimpleNamespace(metrics={"slam_rms": 1.0, "slam_map_error": 10})

    monkeypatch.setattr(loop_closure_study, "run_slam", fake_run)
    data = loop_closure_study.run_study([2, 4])
    assert calls == [(2, False), (2, True), (4, False), (4, True)]
    assert data["scenario"]["name"] == "loop"
    assert data["seeds"] == [2, 4]
    assert len(data["results"]) == 4
    output = tmp_path / "study.png"
    loop_closure_study.plot_study(data, output)
    assert output.exists()
