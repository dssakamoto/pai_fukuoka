"""CI 設定の整合性のテスト (TASK-0030)。実際の実行は GitHub Actions 上でしか確かめられないので、設定の取り違えだけを守る。"""

import re
import tomllib

import yaml

from so101_vlm.robot.model_paths import REPO_ROOT

WORKFLOW = REPO_ROOT / ".github" / "workflows" / "ci.yml"
PYPROJECT = REPO_ROOT / "pyproject.toml"


def _workflow():
    doc = yaml.safe_load(WORKFLOW.read_text(encoding="utf-8"))
    doc["triggers"] = doc.pop("on", doc.pop(True, None))  # PyYAML は `on:` を True と読む
    return doc


def _pyproject():
    return tomllib.loads(PYPROJECT.read_text(encoding="utf-8"))


def _run_steps():
    return [s["run"] for s in _workflow()["jobs"]["test"]["steps"] if "run" in s]


def test_ci_runs_on_push_to_main_and_on_pull_requests():
    triggers = _workflow()["triggers"]
    assert triggers["push"]["branches"] == ["main"] and "pull_request" in triggers


def test_ci_installs_from_the_lock_file_and_without_optional_groups():
    runs = _run_steps()
    assert any(r.strip() == "uv sync --frozen" for r in runs)
    assert not any("--group" in r or "--all-groups" in r or "--extra" in r for r in runs)  # torch / dora を入れない


def test_ci_excludes_rendering_and_real_api_tests_and_measures_coverage():
    pytest_runs = [r for r in _run_steps() if "pytest" in r]
    assert len(pytest_runs) == 1
    assert 'not render' in pytest_runs[0] and 'not api' in pytest_runs[0] and "--cov" in pytest_runs[0]


def test_actions_are_pinned_to_exact_versions():
    for step in _workflow()["jobs"]["test"]["steps"]:
        if "uses" in step:
            assert re.search(r"@v\d+\.\d+\.\d+$", step["uses"]), f"完全なバージョンで固定してください: {step['uses']}"


def test_workflow_has_minimal_permissions_and_a_timeout():
    wf = _workflow()
    assert wf["permissions"] == {"contents": "read"}
    assert wf["jobs"]["test"]["timeout-minutes"] <= 60


def test_coverage_threshold_is_at_least_80_percent():
    cfg = _pyproject()["tool"]["coverage"]
    assert cfg["report"]["fail_under"] >= 80  # AC-07
    assert cfg["run"]["source"] == ["so101_vlm"]


def test_pytest_markers_are_registered_and_strict():
    ini = _pyproject()["tool"]["pytest"]["ini_options"]
    names = {m.split(":")[0] for m in ini["markers"]}
    assert {"render", "api"} <= names and "--strict-markers" in ini["addopts"]


def test_python_version_matches_the_ci_interpreter():
    # CI は .python-version から Python を選ぶ。pyproject の requires-python と食い違わないこと
    version = (REPO_ROOT / ".python-version").read_text(encoding="utf-8").strip()
    assert version.startswith("3.12") and _pyproject()["project"]["requires-python"].startswith(">=3.12")
