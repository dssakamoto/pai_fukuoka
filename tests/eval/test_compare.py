"""eval.compare のテスト (TASK-0029, Red)。プロバイダ比較の実行・判定・表は、偽のクライアントと偽の試行で確かめる (API・描画なし)。"""

import json
from types import SimpleNamespace

import pytest

from so101_vlm.eval.compare import (
    ProviderUnavailableError,
    ProviderResult,
    compare_providers,
    evaluate_provider,
    format_markdown,
    write_report,
)
from so101_vlm.eval.failure import FAILURE_GROUP, FailureKind
from so101_vlm.eval.trial import TrialOutcome
from so101_vlm.vlm import MissingApiKeyError

ARGS = dict(n_trials=4, base_seed=10, colours=("red", "blue"), instruction_template="put the {colour} block into the box")


class _Client:
    def __init__(self, provider="fake", model="fake-1"):
        self.provider, self.model = provider, model


def _factory_for(*, missing=(), unimplemented=(), unavailable=()):
    def factory_for(name):
        if name in unavailable:
            raise ProviderUnavailableError("ollama に接続できません")
        if name in missing:
            raise MissingApiKeyError(f"{name} の API キーがありません。環境変数 {name.upper()}_API_KEY を設定してください")
        if name in unimplemented:
            raise NotImplementedError(f"プロバイダ {name!r} の実装はまだ登録されていません")
        return lambda seed, colour: _Client(name, f"{name}-model")

    return factory_for


def _outcome(seed, colour, instruction, *, ok=True, failure=None, error_mm=6.0):
    # TrialOutcome.success は run が成功のときだけ真 (run_trial の結果と同じ形にする)
    run = SimpleNamespace(success=True, planning_error=None, plan_time_s=0.1, other_block_moved=False) if ok else None
    return TrialOutcome(
        seed=seed, colour=colour, instruction=instruction, plan=None,
        pick_estimate=None if error_mm is None else (0.2, 0.0, 0.015), place_estimate=None, pick_error_mm=error_mm,
        run=run, failure=None if ok else (failure or FailureKind.NOT_IN_BOX), error=None, vlm_time_s=0.5,
    )  # fmt: skip


def _trial_fn(log=None, *, fail_seeds=(), error_mm=6.0, boom_at=None):
    def trial(seed, colour, instruction, vlm, **kw):
        if log is not None:
            log.append((vlm.provider, seed, colour, instruction))
        if boom_at is not None and seed == boom_at:
            raise RuntimeError("503 service unavailable")
        if seed in fail_seeds:
            return _outcome(seed, colour, instruction, ok=False, failure=FailureKind.PLANNING_FAILURE, error_mm=error_mm)
        return _outcome(seed, colour, instruction, error_mm=error_mm)

    return trial


# --- 1 プロバイダの評価 ------------------------------------------------------------------------
def test_evaluated_provider_has_summary_and_error_statistics():
    r = evaluate_provider("gemini", client_factory_for=_factory_for(), trial_fn=_trial_fn(fail_seeds=(11,)), **ARGS)
    assert r.status == "evaluated" and r.reason is None and (r.provider, r.model) == ("gemini", "gemini-model")
    assert r.summary.n_trials == 4 and r.summary.n_success == 3 and r.summary.success_rate == 0.75
    assert r.mean_pick_error_mm == 6.0 and r.median_pick_error_mm == 6.0
    assert r.mean_vlm_time_s == pytest.approx(0.5) and len(r.records) == 4
    assert r.meets_target(0.7) is True and r.meets_target(0.8) is False


def test_median_differs_from_mean_for_skewed_errors():
    errors = iter([2.0, 4.0, 30.0, 4.0])

    def trial(seed, colour, instruction, vlm, **kw):
        return _outcome(seed, colour, instruction, error_mm=next(errors))

    r = evaluate_provider("gemini", client_factory_for=_factory_for(), trial_fn=trial, **ARGS)
    assert r.mean_pick_error_mm == pytest.approx(10.0) and r.median_pick_error_mm == pytest.approx(4.0)


def test_trials_without_an_estimate_do_not_enter_the_error_statistics():
    errors = iter([None, 8.0, None, 4.0])

    def trial(seed, colour, instruction, vlm, **kw):
        return _outcome(seed, colour, instruction, ok=False, failure=FailureKind.VLM_FAILURE, error_mm=next(errors))

    r = evaluate_provider("gemini", client_factory_for=_factory_for(), trial_fn=trial, **ARGS)
    assert r.mean_pick_error_mm == pytest.approx(6.0) and r.median_pick_error_mm == pytest.approx(6.0)
    none = evaluate_provider("gemini", client_factory_for=_factory_for(), trial_fn=_trial_fn(error_mm=None), **ARGS)
    assert none.mean_pick_error_mm is None and none.median_pick_error_mm is None


def test_target_boundary_is_inclusive():
    r = evaluate_provider("gemini", client_factory_for=_factory_for(), trial_fn=_trial_fn(fail_seeds=(11, 12)), **ARGS)
    assert r.summary.success_rate == 0.5 and r.meets_target(0.5) is True and r.meets_target(0.51) is False


def test_missing_key_is_not_evaluated_and_names_the_variable():
    r = evaluate_provider("openai", client_factory_for=_factory_for(missing=("openai",)), trial_fn=_trial_fn(), **ARGS)
    assert r.status == "not_evaluated" and "OPENAI_API_KEY" in r.reason
    assert r.summary is None and r.records == () and r.meets_target(0.0) is False


def test_unimplemented_provider_is_not_evaluated():
    r = evaluate_provider("ollama", client_factory_for=_factory_for(unimplemented=("ollama",)), trial_fn=_trial_fn(), **ARGS)
    assert r.status == "not_evaluated" and "実装" in r.reason


def test_unavailable_provider_is_not_evaluated_not_zero_percent():
    # Ollama が未起動・モデル未取得のとき、成功率 0% ではなく「未評価」にする (AC-04)
    r = evaluate_provider("ollama", client_factory_for=_factory_for(unavailable=("ollama",)), trial_fn=_trial_fn(), **ARGS)
    assert r.status == "not_evaluated" and "利用できません" in r.reason and r.summary is None


def test_unexpected_error_aborts_only_that_provider_and_keeps_partial_records():
    r = evaluate_provider("gemini", client_factory_for=_factory_for(), trial_fn=_trial_fn(boom_at=12), **ARGS)
    assert r.status == "aborted" and "503" in r.reason and "RuntimeError" in r.reason
    assert [x.seed for x in r.records] == [10, 11] and r.summary.n_trials == 2
    assert r.meets_target(0.7) is False  # 中断した評価は、基準を満たしたとは言わない


def test_keyboard_interrupt_is_not_swallowed():
    def trial(*a, **k):
        raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        evaluate_provider("gemini", client_factory_for=_factory_for(), trial_fn=trial, **ARGS)


# --- 複数プロバイダ: 同一条件 -------------------------------------------------------------------------
def test_every_provider_gets_exactly_the_same_trials():
    log = []
    results = compare_providers(["anthropic", "openai", "gemini"], client_factory_for=_factory_for(), trial_fn=_trial_fn(log), **ARGS)
    per_provider = {p: [(s, c, i) for q, s, c, i in log if q == p] for p in ("anthropic", "openai", "gemini")}
    assert per_provider["anthropic"] == per_provider["openai"] == per_provider["gemini"]
    assert [s for s, _, _ in per_provider["gemini"]] == [10, 11, 12, 13]
    assert [c for _, c, _ in per_provider["gemini"]] == ["red", "blue", "red", "blue"]
    assert [r.provider for r in results] == ["anthropic", "openai", "gemini"]


def test_unavailable_providers_do_not_stop_the_others():
    results = compare_providers(
        ["anthropic", "openai", "gemini", "ollama"],
        client_factory_for=_factory_for(missing=("anthropic", "openai"), unimplemented=("ollama",)),
        trial_fn=_trial_fn(), **ARGS,
    )  # fmt: skip
    assert [r.status for r in results] == ["not_evaluated", "not_evaluated", "evaluated", "not_evaluated"]


def test_progress_callback_gets_provider_and_counts():
    seen = []
    compare_providers(["gemini"], client_factory_for=_factory_for(), trial_fn=_trial_fn(), on_record=lambda p, d, n, r: seen.append((p, d, n)), **ARGS)
    assert seen == [("gemini", 1, 4), ("gemini", 2, 4), ("gemini", 3, 4), ("gemini", 4, 4)]


def test_duplicate_or_empty_provider_lists_are_refused():
    with pytest.raises(ValueError):
        compare_providers([], client_factory_for=_factory_for(), trial_fn=_trial_fn(), **ARGS)
    with pytest.raises(ValueError):
        compare_providers(["gemini", "gemini"], client_factory_for=_factory_for(), trial_fn=_trial_fn(), **ARGS)


# --- 表 ----------------------------------------------------------------------------------------------
def _mixed():
    return compare_providers(
        ["anthropic", "openai", "gemini"],
        client_factory_for=_factory_for(missing=("anthropic",)),
        trial_fn=_trial_fn(fail_seeds=(11,)), **ARGS,
    )  # fmt: skip


def test_markdown_table_has_a_row_per_provider_and_marks_unevaluated():
    text = format_markdown(_mixed(), target=0.7, **ARGS)
    lines = text.splitlines()
    header = next(l for l in lines if l.startswith("| プロバイダ"))
    for col in ("モデル", "状態", "成功率", "検出誤差", "[mm]", "VLM 失敗", "IK 失敗", "把持失敗", "配置失敗"):
        assert col in header
    rows = {l.split("|")[1].strip(): l for l in lines if l.startswith("| ") and not l.startswith("| プロバイダ") and not l.startswith("| ---")}
    assert set(rows) == {"anthropic", "openai", "gemini"}
    assert "未評価" in rows["anthropic"] and "-" in rows["anthropic"]
    assert "評価済み" in rows["openai"] and "75.0% (3/4)" in rows["openai"] and "6.0" in rows["openai"]


def test_markdown_states_conditions_reasons_and_the_ac01_verdict():
    text = format_markdown(_mixed(), target=0.7, **ARGS)
    assert "10〜13" in text and "red" in text and "blue" in text and "put the {colour} block into the box" in text
    assert "ANTHROPIC_API_KEY" in text  # 未評価の理由
    assert "70%" in text and "openai" in text.split("AC-01")[1] and "gemini" in text.split("AC-01")[1]


def test_verdict_says_none_when_nobody_meets_the_target():
    results = compare_providers(["anthropic"], client_factory_for=_factory_for(missing=("anthropic",)), trial_fn=_trial_fn(), **ARGS)
    text = format_markdown(results, target=0.7, **ARGS)
    assert "満たしたプロバイダはありません" in text


def test_reference_row_is_labelled_and_excluded_from_the_verdict():
    ref = evaluate_provider("oracle", client_factory_for=_factory_for(), trial_fn=_trial_fn(), reference=True, **ARGS)
    assert ref.reference is True and ref.meets_target(0.7) is True
    text = format_markdown([ref], target=0.7, **ARGS)
    assert "参考" in text and "満たしたプロバイダはありません" in text


def test_table_cells_escape_pipes_and_newlines():
    r = ProviderResult(
        provider="x|y", model="m", status="not_evaluated", reason="line1\nline2 | pipe", records=(), summary=None,
        mean_pick_error_mm=None, median_pick_error_mm=None, mean_vlm_time_s=None, reference=False,
    )
    text = format_markdown([r], target=0.7, **ARGS)
    row = next(l for l in text.splitlines() if l.startswith("| x"))
    assert row.count("|") == next(l for l in text.splitlines() if l.startswith("| プロバイダ")).count("|")
    assert "\nline2" not in text.split("未評価の理由")[1].splitlines()[0]


# --- 保存 -----------------------------------------------------------------------------------------------
def test_write_report_saves_markdown_json_and_per_provider_files(tmp_path):
    results = _mixed()
    paths = write_report(tmp_path / "run", results, target=0.7, **ARGS)
    assert paths["markdown"].read_text(encoding="utf-8").startswith("# ")
    data = json.loads(paths["json"].read_text(encoding="utf-8"))
    assert [p["provider"] for p in data["providers"]] == ["anthropic", "openai", "gemini"]
    assert data["conditions"]["base_seed"] == 10 and data["target"] == 0.7
    assert data["providers"][0]["status"] == "not_evaluated" and data["providers"][1]["success_rate"] == 0.75
    assert (tmp_path / "run" / "openai" / "trials.csv").exists() and (tmp_path / "run" / "gemini" / "summary.json").exists()
    assert not (tmp_path / "run" / "anthropic").exists()  # 未評価には、記録がない


def test_failure_groups_are_counted_per_provider():
    results = compare_providers(["gemini"], client_factory_for=_factory_for(), trial_fn=_trial_fn(fail_seeds=(10, 11)), **ARGS)
    s = results[0].summary
    assert s.by_group == {FAILURE_GROUP[FailureKind.PLANNING_FAILURE].value: 2}


# --- トークン使用量 (評価のコストを実測する) ---------------------------------------------------------------
class _MeteredClient(_Client):
    usage = {"calls": 4, "input_tokens": 2800, "output_tokens": 500}


def _metered_factory_for(name):
    shared = _MeteredClient(name, f"{name}-model")
    return lambda seed, colour: shared


def test_usage_is_taken_from_the_client_and_missing_usage_is_none():
    r = evaluate_provider("gemini", client_factory_for=_metered_factory_for, trial_fn=_trial_fn(), **ARGS)
    assert r.usage == {"calls": 4, "input_tokens": 2800, "output_tokens": 500}
    assert evaluate_provider("gemini", client_factory_for=_factory_for(), trial_fn=_trial_fn(), **ARGS).usage is None
    assert evaluate_provider("x", client_factory_for=_factory_for(missing=("x",)), trial_fn=_trial_fn(), **ARGS).usage is None


def test_usage_appears_in_the_table_and_json(tmp_path):
    results = compare_providers(["gemini", "openai"], client_factory_for=lambda n: _metered_factory_for(n) if n == "gemini" else _factory_for()(n), trial_fn=_trial_fn(), **ARGS)
    text = format_markdown(results, target=0.7, **ARGS)
    header = next(l for l in text.splitlines() if l.startswith("| プロバイダ"))
    assert "トークン" in header
    rows = {l.split("|")[1].strip(): l for l in text.splitlines() if l.startswith("| gemini") or l.startswith("| openai")}
    assert "2800 / 500" in rows["gemini"] and "| - |" in rows["openai"]
    assert rows["gemini"].count("|") == header.count("|")
    paths = write_report(tmp_path / "r", results, target=0.7, **ARGS)
    data = json.loads(paths["json"].read_text(encoding="utf-8"))
    assert data["providers"][0]["usage"] == {"calls": 4, "input_tokens": 2800, "output_tokens": 500} and data["providers"][1]["usage"] is None
