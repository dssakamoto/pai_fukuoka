"""scripts/run_eval.py のテスト (TASK-0026)。"""

import csv
import importlib.util
import json

import pytest

from so101_vlm.robot.model_paths import REPO_ROOT

SCRIPT_PATH = REPO_ROOT / "scripts" / "run_eval.py"


@pytest.fixture(scope="module")
def script():
    spec = importlib.util.spec_from_file_location("run_eval_script", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_help_exits_zero(script, capsys):
    with pytest.raises(SystemExit) as exc:
        script.main(["--help"])
    assert exc.value.code == 0
    out = capsys.readouterr().out
    assert "--provider" in out and "--trials" in out and "--min-success-rate" in out


@pytest.mark.render
def test_small_run_writes_csv_and_summary(script, tmp_path, capsys):
    code = script.main(["--provider", "oracle", "--trials", "2", "--seed", "2", "--out-dir", str(tmp_path), "--run-id", "r"])
    assert code == 0
    out = capsys.readouterr().out
    assert "[1/2]" in out and "[2/2]" in out and "成功率" in out
    with open(tmp_path / "r" / "trials.csv", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    assert [r["seed"] for r in rows] == ["2", "3"] and [r["colour"] for r in rows] == ["red", "blue"]
    summary = json.loads((tmp_path / "r" / "summary.json").read_text(encoding="utf-8"))
    assert summary["n_trials"] == 2


def test_run_id_is_not_overwritten(script, tmp_path, capsys):
    (tmp_path / "r").mkdir()
    (tmp_path / "r" / "trials.csv").write_text("previous", encoding="utf-8")
    code = script.main(["--trials", "1", "--out-dir", str(tmp_path), "--run-id", "r"])
    assert code == 2 and (tmp_path / "r" / "trials.csv").read_text(encoding="utf-8") == "previous"
    assert "既にあります" in capsys.readouterr().err


@pytest.mark.parametrize("run_id", ["../x", "a/b", "a\\b", ".."])
def test_run_id_cannot_escape_the_output_dir(script, tmp_path, run_id):
    assert script.main(["--trials", "1", "--out-dir", str(tmp_path / "out"), "--run-id", run_id]) == 2
    assert not (tmp_path / "out").exists()


def test_missing_api_key_exits_two_and_leaves_nothing(script, tmp_path, monkeypatch, capsys):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    code = script.main(["--provider", "anthropic", "--trials", "1", "--out-dir", str(tmp_path), "--run-id", "k"])
    assert code == 2 and "ANTHROPIC_API_KEY" in capsys.readouterr().err
    assert not (tmp_path / "k").exists()


@pytest.mark.parametrize("argv", [["--trials", "0"], ["--colours", "purple"], ["--instruction-template", "none"], ["--provider", "mystery"]])
def test_bad_arguments_exit_two_and_leave_nothing(script, tmp_path, argv):
    code = script.main([*argv, "--out-dir", str(tmp_path), "--run-id", "b"])
    assert code == 2 and not (tmp_path / "b").exists()


@pytest.mark.render
def test_min_success_rate_gates_the_exit_code(script, tmp_path):
    assert script.main(["--trials", "1", "--seed", "2", "--out-dir", str(tmp_path), "--run-id", "ok", "--min-success-rate", "0.5"]) == 0
    assert script.main(["--trials", "1", "--seed", "2", "--out-dir", str(tmp_path), "--run-id", "ng", "--min-success-rate", "1.1"]) == 1


def test_existing_record_dir_is_refused_before_anything_runs(script, tmp_path, capsys):
    record = tmp_path / "ds"
    record.mkdir()
    code = script.main(["--trials", "1", "--out-dir", str(tmp_path / "out"), "--run-id", "r", "--record-dir", str(record)])
    assert code == 2 and "既にあります" in capsys.readouterr().err
    assert not (tmp_path / "out").exists()


def test_record_without_lerobot_is_an_actionable_error_and_leaves_nothing(script, tmp_path, monkeypatch, capsys):
    import so101_vlm.record.lerobot_writer as writer_module

    def missing():
        raise ImportError("lerobot が入っていません。`uv sync --group record` で入れてください")

    monkeypatch.setattr(writer_module, "load_lerobot", missing)
    code = script.main(["--trials", "1", "--out-dir", str(tmp_path / "out"), "--run-id", "r", "--record-dir", str(tmp_path / "ds")])
    assert code == 2 and "uv sync --group record" in capsys.readouterr().err
    assert not (tmp_path / "ds").exists() and not (tmp_path / "out" / "r").exists()
