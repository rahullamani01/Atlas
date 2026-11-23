import importlib.util
from pathlib import Path
from types import SimpleNamespace

spec = importlib.util.spec_from_file_location(
    "lidar_noise_study", Path(__file__).resolve().parents[1] / "scripts/lidar_noise_study.py"
)
study = importlib.util.module_from_spec(spec)
spec.loader.exec_module(study)


def test_noise_study_pairs_seeds_and_plots(monkeypatch, tmp_path):
    calls = []

    def fake_run(scenario, seed):
        calls.append((seed, scenario.lidar_sigma))
        return SimpleNamespace(metrics={"filter_rms": scenario.lidar_sigma})

    monkeypatch.setattr(study, "run_localization", fake_run)
    data = study.run_study([1, 2])
    assert calls == [(seed, sigma) for seed in [1, 2] for sigma in [0.02, 0.1, 0.3]]
    assert data["scenario"]["name"] == "localization"
    output = tmp_path / "noise.png"
    study.plot_study(data, output)
    assert output.exists()
