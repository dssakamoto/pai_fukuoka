"""scripts/run_fixed_pick.py (main(argv)) のテスト (TASK-0013, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0013/pick-testcases.md 8 節 (TC-CLI-*)
scripts/ はパッケージではないため importlib で読み込み、main(argv) を直接呼ぶ
(tests/perception/test_generate_calibration.py と同じ方式)。出力先は必ず tmp_path の --out-dir を渡す。
実行時間の予算: < 14 s。動画なし 4 回 (計 約 3 s)、動画あり 1 回 (約 10 s)。
動画のテストは TC-CLI-08 の 1 件だけ (640x480 のまま。描画は 1 フレーム約 41 ms で
画像サイズにほとんど依らないため縮小しても 10.3 s -> 7.9 s にしかならない)。他は全て --no-video。
pytest のマーカーは登録されていないので slow マーカーは付けない。
信頼性: 🔵 要件・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない仮定
"""

import dataclasses
import importlib.util
import json
import subprocess
import sys

import pytest

from so101_vlm.robot.model_paths import REPO_ROOT

SCRIPT_PATH = REPO_ROOT / "scripts" / "run_fixed_pick.py"
SUMMARY_NAME = "fixed_pick_summary.json"
FLAG_KEYS = {"clipped", "rate_limited", "rejected", "stopped"}


@pytest.fixture(scope="module")
def script():
    spec = importlib.util.spec_from_file_location("run_fixed_pick", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def summary(out_dir):
    return json.loads((out_dir / SUMMARY_NAME).read_text(encoding="utf-8"))


def trial_lines(text):
    """標準出力のうち試行の行 ('seed=' を含む行)。"""
    return [line for line in text.splitlines() if "seed=" in line]


# =============================================================== 正常系
@pytest.mark.parametrize("headless", [False, True], ids=["default", "headless"])
def test_single_trial_without_video(script, tmp_path, capsys, headless):
    """TC-CLI-01 / 04 1 試行・動画なし (--headless は受け付けるだけ) 🟡/🔴"""
    argv = ["--seed", "0", "--colour", "red", "--no-video", "--out-dir", str(tmp_path)]
    code = script.main([*argv, "--headless"] if headless else argv)
    out = capsys.readouterr().out
    assert code == 0  # 【確認内容】: 成功は終了コード 0
    lines = trial_lines(out)
    assert len(lines) == 1 and "seed=0" in lines[0] and "red" in lines[0] and "成功" in lines[0]
    assert "成功 1/1" in out and "計画失敗 0" in out  # 【確認内容】: 集計行 (T6)
    rows = summary(tmp_path)
    assert len(rows) == 1
    row = rows[0]
    assert row["seed"] == 0 and row["colour"] == "red"
    assert row["success"] is True and row["failure"] is None and row["planning_error"] is None
    assert set(row["flag_counts"]) == FLAG_KEYS and not any(row["flag_counts"].values())
    assert not list(tmp_path.glob("*.mp4")) and not list(tmp_path.glob("*_carry.png"))  # 【確認内容】: 動画なし


def test_n_seeds_runs_consecutive_seeds(script, tmp_path, capsys):
    """TC-CLI-02 --n-seeds 🟡"""
    code = script.main(["--seed", "0", "--n-seeds", "2", "--no-video", "--out-dir", str(tmp_path)])
    out = capsys.readouterr().out
    assert code == 0
    lines = trial_lines(out)
    assert len(lines) == 2 and "seed=0" in lines[0] and "seed=1" in lines[1]
    assert [r["seed"] for r in summary(tmp_path)] == [0, 1]
    assert "成功 2/2" in out


def test_colour_all_with_planning_failure_keeps_exit_code_zero(script, tmp_path, capsys):
    """TC-CLI-03 --colour all と計画失敗 (終了コード 0。青 seed 16 が self_collision) 🔵"""
    code = script.main(["--seed", "16", "--colour", "all", "--no-video", "--out-dir", str(tmp_path)])
    out = capsys.readouterr().out
    assert code == 0  # 【確認内容】: 計画失敗だけでは 1 にしない (FR-06)
    lines = trial_lines(out)
    assert len(lines) == 3
    assert ["red" in lines[0], "blue" in lines[1], "green" in lines[2]] == [True] * 3  # 【確認内容】: 赤・青・緑の順
    assert "self_collision" in lines[1]
    assert "計画失敗 1" in out and "成功 2/2" in out  # 【確認内容】: 計画可能な 2 件が成功
    blue = summary(tmp_path)[1]
    assert blue["planning_error"] == "self_collision" and blue["failure"] == "planning_failure"


def test_out_dir_is_created_and_nothing_written_outside(script, tmp_path):
    """TC-CLI-06 --out-dir のフォルダを作る (外には書かない) 🟡"""
    target = tmp_path / "a" / "b"
    assert script.main(["--seed", "0", "--no-video", "--out-dir", str(target)]) == 0
    assert (target / SUMMARY_NAME).is_file()
    assert sorted(p.name for p in tmp_path.iterdir()) == ["a"]  # 【確認内容】: tmp 直下は a だけ


def test_execution_failure_gives_exit_code_one(script, tmp_path, capsys, monkeypatch):
    """TC-CLI-05 実行の失敗は終了コード 1 🟡"""
    # 【遅延 import】: 未実装の間も、他のテストが「スクリプトがない」理由で失敗するようにする
    from so101_vlm.eval.failure import FailureKind
    from so101_vlm.eval.judge import JudgeReason, JudgeResult

    real = script.run_fixed_pick

    def failing(*args, **kwargs):
        out = real(*args, **kwargs)
        bad = JudgeResult(False, (JudgeReason.OUTSIDE_INNER,), (0.1, 0.0), 0.0199, 0.0)
        return dataclasses.replace(out, judge=bad, failure=FailureKind.NOT_IN_BOX)

    monkeypatch.setattr(script, "run_fixed_pick", failing)
    code = script.main(["--seed", "0", "--no-video", "--out-dir", str(tmp_path)])
    assert code == 1  # 【確認内容】: 実行の失敗が 1 件でもあれば 1
    assert "not_in_box" in capsys.readouterr().out


def test_default_artifact_names(script, tmp_path):
    """TC-CLI-07 動画・PNG の既定のファイル名 (描画しない) 🟡"""
    assert script.default_video_path(tmp_path, "red", 0) == tmp_path / "fixed_pick_red_seed0.mp4"
    assert script.default_carry_png_path(tmp_path, "red", 0) == tmp_path / "fixed_pick_red_seed0_carry.png"


# =============================================================== 動画 (1 件のみ、約 10 s)
@pytest.fixture(scope="module")
def video_run(script, tmp_path_factory):
    """【テスト前準備】: 赤 seed 0 を 640x480 の動画つきで 1 回だけ実行する (約 10 s)。"""
    base = tmp_path_factory.mktemp("video")
    video, out_dir = base / "v" / "red0.mp4", base / "out"
    argv = ["--seed", "0", "--colour", "red", "--headless", "--video", str(video), "--out-dir", str(out_dir)]
    return script.main(argv), video, out_dir


def test_video_png_and_summary_are_saved(video_run):
    """TC-CLI-08 動画を保存できる (1 件のみ) 🔵 (完了条件 2。PNG の目視は verify-complete で行う)"""
    import imageio.v2 as imageio

    code, video, out_dir = video_run
    assert code == 0
    assert video.is_file() and video.stat().st_size > 0  # 【確認内容】: 実測 265 KB
    reader = imageio.get_reader(str(video))
    try:
        assert reader.get_next_data().shape == (480, 640, 3)  # 【確認内容】: 640x480
        assert 140 <= reader.count_frames() <= 200  # 【確認内容】: 実測 161 = floor(323 / 2)
        assert reader.get_meta_data()["fps"] == 25
    finally:
        reader.close()
    png = out_dir / "fixed_pick_red_seed0_carry.png"
    assert png.is_file() and imageio.imread(str(png)).shape == (480, 640, 3)  # 【確認内容】: 運搬中の 1 フレーム
    assert summary(out_dir)[0]["success"] is True


# =============================================================== 異常系
@pytest.mark.parametrize(
    "argv",
    [
        ["--colour", "purple"],
        ["--seed", "-1"],
        ["--seed", "abc"],
        ["--n-seeds", "0"],
        ["--video", "x.mp4", "--no-video"],
        ["--video", "x.mp4", "--n-seeds", "2"],
    ],
    ids=["colour", "seed-negative", "seed-text", "n-seeds-zero", "video-and-no-video", "video-multi"],
)
def test_bad_arguments_exit_with_code_two(script, tmp_path, argv):
    """TC-CLI-E01 引数の誤り → 終了コード 2 (argparse) 🔵/🟡"""
    with pytest.raises(SystemExit) as exc:
        script.main([*argv, "--out-dir", str(tmp_path)])
    assert exc.value.code == 2


def test_help_works_as_a_subprocess():
    """TC-CLI-E02 --help はサブプロセスで動く (約 1 s) 🟡"""
    proc = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), "--help"], capture_output=True, text=True, timeout=60
    )
    assert proc.returncode == 0
    assert "--seed" in proc.stdout and "--out-dir" in proc.stdout
