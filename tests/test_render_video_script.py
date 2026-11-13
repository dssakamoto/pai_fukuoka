"""scripts/render_video.py のテスト (TASK-0028)。"""

import importlib.util

import imageio.v3 as iio
import pytest

from so101_vlm.robot.model_paths import REPO_ROOT

SCRIPT_PATH = REPO_ROOT / "scripts" / "render_video.py"


@pytest.fixture(scope="module")
def script():
    spec = importlib.util.spec_from_file_location("render_video_script", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_help_exits_zero(script, capsys):
    with pytest.raises(SystemExit) as exc:
        script.main(["--help"])
    assert exc.value.code == 0
    out = capsys.readouterr().out
    assert "--speed" in out and "倍速" in out and "--trials" in out


@pytest.mark.render
def test_small_run_writes_a_playable_video_and_reports_the_length(script, tmp_path, capsys):
    out = tmp_path / "v" / "demo.mp4"
    code = script.main(["--trials", "1", "--seed", "2", "--speed", "6", "--lang", "en", "--out", str(out)])
    assert code == 0
    text = capsys.readouterr()
    assert "[1/1]" in text.out and "6 倍速" in text.out and "動画:" in text.out
    assert "1〜3 分" in text.err  # 1 試行だけなので、長さが条件から外れる警告が出る
    frames = iio.imread(out)
    assert frames.shape[1:] == (480, 640, 3) and frames.shape[0] > 100


def test_existing_output_is_not_overwritten_without_the_flag(script, tmp_path, capsys):
    out = tmp_path / "demo.mp4"
    out.write_bytes(b"previous")
    assert script.main(["--trials", "1", "--out", str(out)]) == 2
    assert out.read_bytes() == b"previous" and "--overwrite" in capsys.readouterr().err


@pytest.mark.parametrize(
    "argv",
    [["--speed", "7"], ["--speed", "0"], ["--trials", "0"], ["--colours", "purple"], ["--provider", "mystery"], ["--font", "no/such.ttf"]],
)
def test_bad_arguments_exit_two_and_leave_no_file(script, tmp_path, argv):
    out = tmp_path / "bad.mp4"
    assert script.main([*argv, "--out", str(out)]) == 2
    assert not out.exists()


def test_missing_api_key_exits_two_and_leaves_no_file(script, tmp_path, monkeypatch, capsys):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    out = tmp_path / "k.mp4"
    assert script.main(["--provider", "anthropic", "--trials", "1", "--out", str(out)]) == 2
    assert "ANTHROPIC_API_KEY" in capsys.readouterr().err and not out.exists()
