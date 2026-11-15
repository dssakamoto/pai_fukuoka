"""README が実際のリポジトリとずれていないか (TASK-0031, AC-09)。

再現手順の README は、書かれたコマンドが実在しないと意味がない。相対リンク、スクリプト、設定ファイル、
`uv run python scripts/...` のオプションが、実在することを確かめる (内容の正しさは、新しい clone で手順を実行して確かめた)。
"""

import re

import pytest

from so101_vlm.robot.model_paths import REPO_ROOT

README = (REPO_ROOT / "README.md").read_text(encoding="utf-8")
_FLAG = re.compile(r"(?<![\w-])(--[a-z][a-z0-9-]*)")


def _commands(text: str) -> list[tuple[str, str]]:
    """README の `uv run python scripts/X.py ...` の行から (スクリプト, 引数の文字列) を取り出す。"""
    return [(m.group(1), m.group(2)) for m in re.finditer(r"uv run (?:python|mjpython) (scripts/\w+\.py)([^\n`]*)", text)]


def _flags(args: str) -> list[str]:
    return _FLAG.findall(args.split("#")[0])


def test_relative_links_point_to_existing_files():
    links = re.findall(r"\]\(([^)\s]+)\)", README)
    relative = [l.split("#")[0] for l in links if not l.startswith(("http://", "https://", "#", "mailto:")) and l.split("#")[0]]
    assert relative, "リンクが 1 つもない"
    missing = [l for l in relative if not (REPO_ROOT / l).exists()]
    assert missing == []


def test_every_mentioned_script_and_config_exists():
    paths = set(re.findall(r"`((?:scripts|configs|docs|assets|src|tests)/[\w./-]+?)`", README))
    # 配列・ディレクトリ名 (末尾が / や、説明用の <...>) は除く
    files = {p for p in paths if "<" not in p and not p.endswith("/") and "." in p.rsplit("/", 1)[-1]}
    missing = sorted(p for p in files if not (REPO_ROOT / p).exists() and not p.startswith(("results/", "media/")))
    assert missing == []


def test_every_command_in_the_readme_uses_flags_that_the_script_defines():
    commands = _commands(README)
    assert len(commands) >= 12
    problems = []
    for script, args in commands:
        path = REPO_ROOT / script
        if not path.exists():
            problems.append(f"{script}: ファイルがない")
            continue
        source = path.read_text(encoding="utf-8")
        problems += [f"{script}: {flag} が定義されていない" for flag in _flags(args) if f'"{flag}"' not in source]
    assert problems == []


def test_the_quickstart_runs_the_scripts_in_the_documented_order():
    quick = README.split("## クイックスタート", 1)[1].split("\n## ", 1)[0]
    order = [script for script, _ in _commands(quick)]
    assert order == [
        "scripts/capture_scene.py", "scripts/run_fixed_pick.py", "scripts/run_trial.py", "scripts/run_eval.py",
        "scripts/render_video.py",
    ]
    assert "uv sync" in quick and "uv run pytest -m" in quick


def test_the_readme_covers_what_ac09_and_task0031_require():
    for needed in ("## 動作環境", "## クイックスタート", "## 設定ファイル", "## トラブルシューティング", "## ライセンス・出典"):
        assert needed in README
    assert "Apache-2.0" in README and "assets/so101/LICENSE" in README and "SOURCE.md" in README
    assert ".env.example" in README and "API キー" in README


# --- 検査の道具そのものの確認 (ずれを本当に検出できるか) ------------------------------------------------------
def test_the_checker_finds_unknown_flags_in_a_wrong_readme():
    text = "```bash\nuv run python scripts/run_eval.py --trials 20 --no-such-flag 1\n```"
    commands = _commands(text)
    assert commands == [("scripts/run_eval.py", " --trials 20 --no-such-flag 1")]
    source = (REPO_ROOT / "scripts/run_eval.py").read_text(encoding="utf-8")
    unknown = [f for f in _flags(commands[0][1]) if f'"{f}"' not in source]
    assert unknown == ["--no-such-flag"]


@pytest.mark.parametrize("args,expected", [(" --a 1 --b-c  # --not-a-flag", ["--a", "--b-c"]), (" x -y", [])])
def test_flag_extraction(args, expected):
    assert _flags(args) == expected
