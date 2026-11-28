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


# =============================================================== --box-xy (SGT-014)
@pytest.mark.parametrize("argv", [["--box-xy", "0.9", "0.9"], ["--box-xy", "0.0", "0.0"], ["--box-xy", "nan", "0.15"]])
def test_box_xy_outside_the_region_is_rejected_before_anything_runs(script, capsys, argv):
    assert script.main(["--provider", "oracle", *argv]) == 2
    err = capsys.readouterr().err
    assert "引数・設定の誤り" in err and ("収まらず" in err or "数値" in err)


def test_box_xy_needs_two_numbers(script):
    with pytest.raises(SystemExit) as exc:
        script.main(["--box-xy", "0.2"])
    assert exc.value.code == 2


def test_a_paid_run_with_box_xy_is_refused_without_confirmation(script, capsys, monkeypatch):
    monkeypatch.setattr("sys.stdin.isatty", lambda: False, raising=False)
    monkeypatch.setenv("OPENAI_API_KEY", "dummy-key")
    code = script.main(["--provider", "openai", "--box-xy", "0.20", "0.15"])  # 確認 (--yes) が無い非対話の実行
    captured = capsys.readouterr()
    assert code == 2 and "--yes" in captured.err and "有料" in captured.out  # 目安は表示し、API は呼ばない


@pytest.mark.render
def test_oracle_trial_with_box_xy_follows_the_moved_box(script, capsys):
    code = script.main(["--provider", "oracle", "--seed", "2", "--colour", "red", "--box-xy", "0.20", "0.15"])
    out = capsys.readouterr().out
    assert code in (0, 1) and "判定" in out  # 動かした箱で最後まで通る (成功・失敗は seed 次第)
