"""scripts/compare_providers.py のテスト (TASK-0029)。キーも実装もない環境でも、「未評価」の表が出ることを確かめる。"""

import importlib.util
import json

import pytest

from so101_vlm.robot.model_paths import REPO_ROOT

SCRIPT_PATH = REPO_ROOT / "scripts" / "compare_providers.py"
KEYS = ("ANTHROPIC_API_KEY", "OPENAI_API_KEY", "GEMINI_API_KEY")


@pytest.fixture(scope="module")
def script():
    spec = importlib.util.spec_from_file_location("compare_providers_script", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def no_keys(monkeypatch):
    for key in KEYS:
        monkeypatch.delenv(key, raising=False)


def test_help_exits_zero(script, capsys):
    with pytest.raises(SystemExit) as exc:
        script.main(["--help"])
    assert exc.value.code == 0
    out = capsys.readouterr().out
    assert "--providers" in out and "未評価" in out and "ollama" in out


def test_default_providers_exclude_gemma(script):
    assert script.DEFAULT_PROVIDERS.split(",") == ["anthropic", "openai", "gemini"]  # ollama は明示したときだけ


def test_without_keys_every_provider_is_not_evaluated_and_the_table_says_so(script, tmp_path, no_keys, capsys):
    notes = tmp_path / "notes" / "cmp.md"
    code = script.main(["--no-oracle", "--trials", "2", "--out-dir", str(tmp_path), "--run-id", "r", "--notes", str(notes)])
    assert code == 0
    out = capsys.readouterr().out
    for key in KEYS:
        assert key in out  # 理由に、設定すべき環境変数名が出る
    text = (tmp_path / "r" / "comparison.md").read_text(encoding="utf-8")
    assert text.count("未評価") >= 3 and "満たしたプロバイダはありません" in text
    assert notes.read_text(encoding="utf-8") == text
    data = json.loads((tmp_path / "r" / "comparison.json").read_text(encoding="utf-8"))
    assert [p["status"] for p in data["providers"]] == ["not_evaluated"] * 3
    assert not (tmp_path / "r" / "anthropic").exists()


def test_require_target_fails_when_nobody_was_evaluated(script, tmp_path, no_keys):
    args = ["--no-oracle", "--trials", "1", "--out-dir", str(tmp_path)]
    assert script.main([*args, "--run-id", "a"]) == 0
    assert script.main([*args, "--run-id", "b", "--require-target"]) == 1


@pytest.mark.parametrize(
    "argv",
    [["--providers", "mystery"], ["--target", "0"], ["--target", "1.5"], ["--trials", "0"], ["--run-id", "../x"], ["--colours", "purple"]],
)
def test_bad_arguments_exit_two_and_write_nothing(script, tmp_path, no_keys, argv):
    assert script.main([*argv, "--no-oracle", "--out-dir", str(tmp_path / "out")]) == 2
    assert not (tmp_path / "out").exists()


def test_existing_run_is_not_overwritten(script, tmp_path, no_keys):
    args = ["--no-oracle", "--trials", "1", "--out-dir", str(tmp_path), "--run-id", "r"]
    assert script.main(args) == 0
    before = (tmp_path / "r" / "comparison.md").read_bytes()
    assert script.main(args) == 2 and (tmp_path / "r" / "comparison.md").read_bytes() == before


@pytest.mark.render
def test_oracle_reference_row_is_evaluated_next_to_unevaluated_providers(script, tmp_path, no_keys):
    code = script.main(["--providers", "anthropic", "--trials", "2", "--seed", "2", "--out-dir", str(tmp_path), "--run-id", "o"])
    assert code == 0
    data = json.loads((tmp_path / "o" / "comparison.json").read_text(encoding="utf-8"))
    by_name = {p["provider"]: p for p in data["providers"]}
    assert by_name["oracle"]["status"] == "evaluated" and by_name["oracle"]["reference"] is True
    assert by_name["oracle"]["mean_pick_error_mm"] is not None and by_name["anthropic"]["status"] == "not_evaluated"
    assert (tmp_path / "o" / "oracle" / "trials.csv").exists()
    assert "(参考)" in (tmp_path / "o" / "comparison.md").read_text(encoding="utf-8")
