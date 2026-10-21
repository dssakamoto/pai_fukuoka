"""scripts/generate_calibration.py (main(argv)) のテスト (TASK-0009, Red フェーズ)。

テストケース定義: docs/implements/so101-vlm-pickplace/TASK-0009/calibration-testcases.md (G-01..G-12, G-E01, G-E02)
scripts/ はパッケージではないため importlib で読み込み、main(argv) を直接呼ぶ (サブプロセスは G-12 のみ)。
リポジトリの configs/calibration.yaml は書き換えない (--out を必ず渡す。G-04 は読むだけ)。
終了コード: 0 一致/成功、1 --check 不一致、2 読み込み不可・real の保護・親ディレクトリなし。
信頼性: 🔵 要件・設計・実測に基づく / 🟡 妥当な推測 / 🔴 要件にない決定
"""

import dataclasses
import importlib.util
import re
import subprocess
import sys

import pytest

from so101_vlm.perception.calibration import (
    DEFAULT_CALIBRATION_PATH,
    calibrations_close,
    load_calibration,
    save_calibration,
)
from so101_vlm.robot.model_paths import REPO_ROOT

SCRIPT_PATH = REPO_ROOT / "scripts" / "generate_calibration.py"


@pytest.fixture(scope="module")
def script():
    spec = importlib.util.spec_from_file_location("generate_calibration", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def generated_bytes(script, tmp_path_factory):
    """main() で生成したファイルのバイト列 (基準)。"""
    p = tmp_path_factory.mktemp("gen") / "base.yaml"
    assert script.main(["--out", str(p)]) == 0
    return p.read_bytes()


def _with_changes(src, dst, *, fx_delta=0.0, source=None):
    cal = load_calibration(src)
    cam = dataclasses.replace(cal.camera, fx=cal.camera.fx + fx_delta)
    save_calibration(dataclasses.replace(cal, camera=cam, source=source or cal.source), dst)
    return dst


@pytest.fixture
def real_file(script, tmp_path):
    base = tmp_path / "base.yaml"
    assert script.main(["--out", str(base)]) == 0
    return _with_changes(base, tmp_path / "real.yaml", source="real")


def test_generate_writes_file_matching_committed(script, tmp_path):
    """G-01: --out に書いた内容はコミット済みのファイルと一致する (値とバイト)。🟡"""
    p = tmp_path / "c.yaml"
    assert script.main(["--out", str(p)]) == 0
    assert p.is_file()
    committed = load_calibration()
    if committed.source == "real":
        pytest.skip("コミット済みが real のため比べない")
    assert calibrations_close(load_calibration(p), committed, atol=1e-9)
    assert p.read_bytes() == DEFAULT_CALIBRATION_PATH.read_bytes()  # 🔴 R-02 と同じ理由


def test_generate_output_is_sim_with_sim_comment(script, tmp_path):
    """G-02: 生成したファイルは source: sim と sim のコメント。🔴"""
    p = tmp_path / "c.yaml"
    script.main(["--out", str(p)])
    assert load_calibration(p).source == "sim"
    assert p.read_text(encoding="utf-8").splitlines()[0] == "# 自動生成ファイル (source: sim)。手で編集しないこと。"


def test_check_matching_file_returns_0_and_does_not_touch_it(script, tmp_path):
    """G-03: --check が一致なら 0、ファイルは変わらない。🔴"""
    p = tmp_path / "c.yaml"
    script.main(["--out", str(p)])
    before, mtime = p.read_bytes(), p.stat().st_mtime_ns
    assert script.main(["--out", str(p), "--check"]) == 0
    assert p.read_bytes() == before and p.stat().st_mtime_ns == mtime


def test_check_default_path_returns_0_without_modifying_committed(script):
    """G-04: --check を既定パス (コミット済み) に対して実行。🟡"""
    if load_calibration().source == "real":
        pytest.skip("コミット済みが real のため skip")
    before = DEFAULT_CALIBRATION_PATH.read_bytes()
    assert script.main(["--check"]) == 0
    assert DEFAULT_CALIBRATION_PATH.read_bytes() == before


def test_check_mismatch_returns_1_and_reports_difference(script, tmp_path, capsys):
    """G-05: --check が不一致なら 1、差の最大値を標準エラーに出す。🔴"""
    base = tmp_path / "base.yaml"
    script.main(["--out", str(base)])
    p = _with_changes(base, tmp_path / "c.yaml", fx_delta=1e-6)
    before = p.read_bytes()
    capsys.readouterr()
    assert script.main(["--out", str(p), "--check"]) == 1
    err = capsys.readouterr().err
    assert err and re.search(r"[0-9.]+e-0?[67]", err), err  # 【確認内容】: 差 1e-06 付近 (書式は実装任せ)
    assert p.read_bytes() == before


@pytest.mark.parametrize("kind", ["missing", "version2", "empty"])
def test_check_unreadable_file_returns_2(script, tmp_path, capsys, kind):
    """G-06: --check でファイルがない・読めないなら 2 (例外・SystemExit を出さない)。🔴"""
    p = tmp_path / "c.yaml"
    if kind == "version2":
        p.write_text("version: 2\n", encoding="utf-8")
    elif kind == "empty":
        p.write_text("", encoding="utf-8")
    capsys.readouterr()
    assert script.main(["--out", str(p), "--check"]) == 2
    assert capsys.readouterr().err  # 【確認内容】: パスまたは理由を標準エラーへ


def test_generate_refuses_to_overwrite_real_file(script, real_file, capsys):
    """G-07: 既存が source: real なら上書きを拒否して 2 (--force の案内を出す)。🔴"""
    before = real_file.read_bytes()
    capsys.readouterr()
    assert script.main(["--out", str(real_file)]) == 2
    assert real_file.read_bytes() == before
    assert "--force" in capsys.readouterr().err


def test_force_overwrites_real_file(script, real_file, generated_bytes):
    """G-08: --force なら real を上書きする。🔴"""
    assert script.main(["--out", str(real_file), "--force"]) == 0
    assert load_calibration(real_file).source == "sim"
    assert real_file.read_bytes() == generated_bytes


def test_generate_overwrites_existing_sim_file(script, tmp_path, generated_bytes):
    """G-09: 既存が sim なら --force なしで上書きする。🔴"""
    base = tmp_path / "base.yaml"
    script.main(["--out", str(base)])
    p = _with_changes(base, tmp_path / "c.yaml", fx_delta=1.0)
    assert script.main(["--out", str(p)]) == 0
    assert p.read_bytes() == generated_bytes


def test_check_treats_real_file_as_mismatch(script, real_file):
    """G-10: --check は real のファイルを不一致 (1) とする。🔴 (O-T2: 要件 2.10 / 2.11 の文言どおり)"""
    before = real_file.read_bytes()
    assert script.main(["--out", str(real_file), "--check"]) == 1
    assert real_file.read_bytes() == before


def test_generate_is_deterministic(script, tmp_path):
    """G-11: 2 回の生成でバイト一致。🟡"""
    a, b = tmp_path / "a.yaml", tmp_path / "b.yaml"
    script.main(["--out", str(a)])
    script.main(["--out", str(b)])
    assert a.read_bytes() == b.read_bytes()


def test_script_runs_as_subprocess_from_other_cwd(tmp_path, generated_bytes):
    """G-12: サブプロセスで実行できる (cwd がリポジトリ外でも)。生成と --check の 2 回だけ。🟡"""
    p = tmp_path / "c.yaml"
    cmd = [sys.executable, str(SCRIPT_PATH), "--out", str(p)]
    gen = subprocess.run(cmd, capture_output=True, timeout=60, cwd=tmp_path)
    assert gen.returncode == 0, gen.stderr.decode()
    chk = subprocess.run([*cmd, "--check"], capture_output=True, timeout=60, cwd=tmp_path)
    assert chk.returncode == 0, chk.stderr.decode()
    assert p.read_bytes() == generated_bytes


def test_unknown_option_exits_with_argparse_code_2(script):
    """G-E01: 未知のオプションは argparse の SystemExit(2)。🔴"""
    with pytest.raises(SystemExit) as exc:
        script.main(["--bogus"])
    assert exc.value.code == 2


def test_generate_with_missing_parent_returns_2(script, tmp_path):
    """G-E02: --out の親ディレクトリがない (生成モード) は 2。ディレクトリは作らない。🔴"""
    assert script.main(["--out", str(tmp_path / "missing" / "c.yaml")]) == 2
    assert not (tmp_path / "missing").exists()
