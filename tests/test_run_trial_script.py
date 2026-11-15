"""scripts/run_trial.py のテスト (AC-02)。"""

import importlib.util

import pytest

from so101_vlm.robot.model_paths import REPO_ROOT
from so101_vlm.vlm.schema import PlanError

SCRIPT_PATH = REPO_ROOT / "scripts" / "run_trial.py"


@pytest.fixture
def script():
    spec = importlib.util.spec_from_file_location("run_trial_script", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_help_exits_zero(script, capsys):
    with pytest.raises(SystemExit) as exc:
        script.main(["--help"])
    assert exc.value.code == 0
    out = capsys.readouterr().out
    assert "--instruction" in out and "AC-02" in out and "purple" in out


@pytest.mark.parametrize("argv", [["--colour", "purple"], ["--seed", "x"]])
def test_bad_arguments_exit_two_from_argparse(script, argv):
    with pytest.raises(SystemExit) as exc:
        script.main(argv)
    assert exc.value.code == 2


@pytest.mark.parametrize("argv", [["--provider", "mystery"], ["--instruction", "  "], ["--seed", "-1"]])
def test_bad_values_exit_two(script, capsys, argv):
    assert script.main(argv) == 2
    assert "誤り" in capsys.readouterr().err


def test_missing_key_exits_two_and_names_the_variable(script, capsys, monkeypatch):
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    assert script.main(["--provider", "openai"]) == 2
    assert "OPENAI_API_KEY" in capsys.readouterr().err


@pytest.mark.render
def test_normal_trial_prints_the_stages_and_succeeds(script, capsys):
    code = script.main(["--provider", "oracle", "--seed", "2", "--colour", "red"])
    out = capsys.readouterr().out
    assert code == 0 and "判定: 成功" in out and "ロボットを動かしました" in out and "検出誤差" in out
    assert [l.split(":")[0] for l in out.splitlines()[:5]] == ["指示", "VLM", "検出", "推定位置", "実行"]


@pytest.mark.render
def test_nonexistent_colour_does_not_move_the_robot_and_shows_the_reason(script, capsys, monkeypatch):
    # AC-02: VLM が「紫のブロックは見つからない」と答える (PlanError) -> 動かさず、理由を表示、終了コード 1
    class NoPurple:
        provider, model = "fake", "fake-1"

        def plan(self, image, instruction):
            raise PlanError("pick がありません: there is no purple block in the image")

    monkeypatch.setattr(script, "default_client_factory_for", lambda: (lambda name: (lambda seed, colour: NoPurple())))
    code = script.main(["--provider", "openai", "--instruction", "put the purple block into the box"])
    out = capsys.readouterr().out
    assert code == 1
    assert "put the purple block into the box" in out and "動かしていません" in out
    assert "there is no purple block in the image" in out and "vlm_failure" in out
    assert "ロボットを動かしました" not in out
