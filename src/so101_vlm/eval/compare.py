"""プロバイダ別の評価と比較表 (TASK-0029)。

🔵 AC-01 / AC-04 / AC-05: 同じシード・同じ色の並び・同じ指示で各プロバイダを評価し、成功率と検出誤差 (mm) を表にする。
API キーのないプロバイダ・クライアントが未実装のプロバイダ・使えないプロバイダ (Ollama が未起動など) は「未評価」(評価しない)、評価の途中で想定外の例外が出たプロバイダは
「中断」(途中までの記録は残す) にして、他のプロバイダの評価は続ける。KeyboardInterrupt は握りつぶさない。
oracle (真の配置から作る偽 VLM) は「参考」の行で、AC-01 の判定には含めない。
検出誤差 = VLM の出力から算出した 3 次元位置 (ブロック中心の推定) と、sim の真のブロック中心の距離 [mm]。
推定が出なかった試行 (VLM 失敗など) は、平均・中央値に入れない。
"""

import json
import statistics
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

from so101_vlm.eval.runner import (
    DEFAULT_COLOURS,
    DEFAULT_INSTRUCTION,
    Summary,
    TrialRecord,
    oracle_client_factory,
    run_trials,
    summarize,
    validate_trial_plan,
    write_csv,
    write_summary,
)
from so101_vlm.eval.trial import run_trial
from so101_vlm.perception.calibration import Calibration, load_calibration
from so101_vlm.vlm import MissingApiKeyError, create_client
from so101_vlm.vlm.config import load_vlm_config

class ProviderUnavailableError(RuntimeError):
    """【機能概要】: プロバイダが今は使えない (ローカルのサーバが未起動、モデル未取得など)。評価は「未評価」にする 🔵"""


ClientFactoryFor = Callable[[str], Callable[[int, str], object]]
EVALUATED, NOT_EVALUATED, ABORTED = "evaluated", "not_evaluated", "aborted"
_STATUS_LABEL = {EVALUATED: "評価済み", NOT_EVALUATED: "未評価", ABORTED: "中断"}
_GROUPS = (("vlm", "VLM 失敗"), ("ik", "IK 失敗"), ("grasp", "把持失敗"), ("place", "配置失敗"))
DEFAULT_TARGET = 0.7  # AC-01 (暫定目標。実測後に見直す)


@dataclass(frozen=True)
class ProviderResult:
    """【機能概要】: 1 プロバイダの評価結果。status は evaluated / not_evaluated / aborted 🔵

    reason: 未評価・中断の理由、records / summary: 評価した試行 (未評価は空 / None、中断は途中まで)、
    mean_/median_pick_error_mm: 推定のある試行だけの検出誤差、reference: 参考の行 (AC-01 の判定に含めない)、
    usage: API の呼び出し回数と入力・出力のトークン数の累計 (クライアントが記録を持つときだけ。キャッシュから返した分は含まない)。
    """

    provider: str
    model: str | None
    status: str
    reason: str | None
    records: tuple[TrialRecord, ...]
    summary: Summary | None
    mean_pick_error_mm: float | None
    median_pick_error_mm: float | None
    mean_vlm_time_s: float | None
    reference: bool
    usage: dict[str, int] | None = None

    def meets_target(self, target: float) -> bool:
        """【機能概要】: 評価済みで、成功率が target 以上 (ちょうどを含む)。未評価・中断は常に False 🔵"""
        return (
            self.status == EVALUATED and self.summary is not None
            and self.summary.success_rate is not None and self.summary.success_rate >= target
        )  # fmt: skip


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def _result(name, status, reason, records, model, reference, usage=None) -> ProviderResult:
    errors = [r.pick_error_mm for r in records if r.pick_error_mm is not None]
    return ProviderResult(
        provider=name, model=records[0].model if records else model, status=status, reason=reason,
        records=tuple(records), summary=summarize(records) if records else None,
        mean_pick_error_mm=_mean(errors), median_pick_error_mm=statistics.median(errors) if errors else None,
        mean_vlm_time_s=_mean([r.vlm_time_s for r in records if r.vlm_time_s is not None]), reference=reference,
        usage=usage,
    )  # fmt: skip


def default_client_factory_for(calibration: Calibration | None = None) -> ClientFactoryFor:
    """【機能概要】: プロバイダ名から、試行ごとの VlmClient を返すファクトリを作る関数を返す 🔵

    oracle は試行ごとに真値から作る。それ以外は create_client を 1 回だけ呼んで使い回す (キャッシュ・再試行の設定を共有)。
    キー未設定は MissingApiKeyError、実装未登録は NotImplementedError、未知のプロバイダは ValueError。
    """
    cal = load_calibration() if calibration is None else calibration
    config = load_vlm_config()

    def factory_for(name: str) -> Callable[[int, str], object]:
        if name == "oracle":
            return oracle_client_factory(cal)
        if name == "ollama":  # キーがない代わりに、起動とモデルの取得を確かめる。使えなければ「未評価」(0% と誤解させない)
            from so101_vlm.vlm.ollama_client import unavailable_reason

            reason = unavailable_reason(config.providers["ollama"])
            if reason is not None:
                raise ProviderUnavailableError(reason)
        client = create_client(config, name)
        return lambda seed, colour: client

    return factory_for


# 🔵 Intent: 評価を始める前に、キーと実装の有無を分かる形で判定する (未評価は例外ではなく結果として返す、AC-04)。
def evaluate_provider(
    name: str,
    *,
    n_trials: int,
    base_seed: int,
    client_factory_for: ClientFactoryFor,
    colours: Sequence[str] = DEFAULT_COLOURS,
    instruction_template: str = DEFAULT_INSTRUCTION,
    trial_fn: Callable[..., object] = run_trial,
    reference: bool = False,
    model_hint: str | None = None,
    on_record: Callable[[str, int, int, TrialRecord], None] | None = None,
    **trial_kwargs: object,
) -> ProviderResult:
    """【機能概要】: 1 プロバイダを評価する。未評価・中断も結果として返す (例外にしない) 🔵

    不正な試行列の指定は ValueError (評価の前に出す)。未知のプロバイダ名 (client_factory_for が出す ValueError) も ValueError。
    """
    n_trials, base_seed, colours = validate_trial_plan(n_trials, base_seed, colours, instruction_template)
    try:
        client_for = client_factory_for(name)
    except MissingApiKeyError as exc:
        return _result(name, NOT_EVALUATED, str(exc), [], model_hint, reference)
    except NotImplementedError as exc:
        return _result(name, NOT_EVALUATED, f"クライアント未実装: {exc}", [], model_hint, reference)
    except ProviderUnavailableError as exc:
        return _result(name, NOT_EVALUATED, f"利用できません: {exc}", [], model_hint, reference)
    records: list[TrialRecord] = []

    def collect(done: int, total: int, record: TrialRecord) -> None:
        records.append(record)
        if on_record is not None:
            on_record(name, done, total, record)

    def metered_usage() -> dict[str, int] | None:
        recorded = getattr(client_for(base_seed, colours[0]), "usage", None)  # 共有のクライアントなら、累計が読める
        return dict(recorded) if recorded else None

    try:
        run_trials(
            n_trials, base_seed, client_for, colours=colours, instruction_template=instruction_template,
            trial_fn=trial_fn, on_record=collect, **trial_kwargs,
        )  # type: ignore[arg-type]
    except Exception as exc:  # noqa: BLE001  KeyboardInterrupt は BaseException なので通る
        return _result(name, ABORTED, f"{type(exc).__name__}: {exc}", records, model_hint, reference, metered_usage())
    return _result(name, EVALUATED, None, records, model_hint, reference, metered_usage())


def compare_providers(
    providers: Sequence[str],
    *,
    client_factory_for: ClientFactoryFor,
    reference: Sequence[str] = ("oracle",),
    model_hints: dict[str, str] | None = None,
    on_record: Callable[[str, int, int, TrialRecord], None] | None = None,
    **kwargs: object,
) -> list[ProviderResult]:
    """【機能概要】: providers を順に、全く同じ試行 (シード・色・指示) で評価する。空や重複は ValueError 🔵

    reference に含まれる名前 (既定 oracle) は「参考」の行にする。kwargs は evaluate_provider に渡す。
    """
    names = list(providers)
    if not names or len(set(names)) != len(names):
        raise ValueError(f"providers は重複のない 1 つ以上の名前で指定してください: {names!r}")
    hints = model_hints or {}
    return [
        evaluate_provider(
            n, client_factory_for=client_factory_for, reference=n in reference, model_hint=hints.get(n),
            on_record=on_record, **kwargs,
        )  # type: ignore[arg-type]
        for n in names
    ]


# --------------------------------------------------------------------------- 表
def _cell(text: object) -> str:
    """表のセル用: 改行は空白、縦棒は全角にして、列がずれないようにする。"""
    return " ".join(str(text).split()).replace("|", "｜")


def _num(value: float | None, digits: int = 1) -> str:
    return "-" if value is None else f"{value:.{digits}f}"


def _row(r: ProviderResult) -> list[str]:
    name = f"{r.provider} (参考)" if r.reference else r.provider
    cells = [name, r.model or "-", _STATUS_LABEL[r.status]]
    if r.status == EVALUATED and r.summary is not None and r.summary.success_rate is not None:
        s = r.summary
        cells.append(f"{100 * s.success_rate:.1f}% ({s.n_success}/{s.n_trials})")
        cells += [_num(r.mean_pick_error_mm), _num(r.median_pick_error_mm)]
        cells += [str(s.by_group.get(key, 0)) for key, _ in _GROUPS]
    else:
        cells += ["-"] * (3 + len(_GROUPS))
    cells.append(_num(r.mean_vlm_time_s, 2) if r.status == EVALUATED else "-")
    cells.append(f"{r.usage['input_tokens']} / {r.usage['output_tokens']}" if r.usage else "-")
    return [_cell(c) for c in cells]


def _seed_range(base_seed: int, n_trials: int) -> str:
    return str(base_seed) if n_trials == 1 else f"{base_seed}〜{base_seed + n_trials - 1}"


def format_markdown(
    results: Sequence[ProviderResult],
    *,
    target: float,
    n_trials: int,
    base_seed: int,
    colours: Sequence[str],
    instruction_template: str,
) -> str:
    """【機能概要】: 条件・比較表・未評価の理由・AC-01 の判定・検出誤差の定義を、Markdown の文字列にする 🔵"""
    header = ["プロバイダ", "モデル", "状態", "成功率", "検出誤差 平均 [mm]", "検出誤差 中央値 [mm]",
              *(label for _, label in _GROUPS), "VLM 時間 平均 [s]", "トークン 入力 / 出力"]  # fmt: skip
    lines = [
        "# プロバイダ別の評価と比較表", "",
        "## 条件", "",
        f"- 試行数: {n_trials} (プロバイダごと)",
        f"- シード: {_seed_range(base_seed, n_trials)} (試行 i は {base_seed} + i)。全プロバイダで同じ",
        f"- 対象の色: {', '.join(colours)} (この順に回す)",
        f"- 指示: `{instruction_template}` ({{colour}} を色名に置き換える)",
        f"- 成功率の基準: {100 * target:.0f}% 以上 (暫定目標)", "",
        "## 比較表", "",
        "| " + " | ".join(header) + " |",
        "| " + " | ".join("---" for _ in header) + " |",
        *("| " + " | ".join(_row(r)) + " |" for r in results), "",
    ]
    unavailable = [r for r in results if r.status != EVALUATED]
    if unavailable:
        lines += ["## 未評価の理由と中断の理由", ""]
        lines += [f"- {_cell(r.provider)} ({_STATUS_LABEL[r.status]}): {_cell(r.reason)}" for r in unavailable]
        lines.append("")
    met = [r.provider for r in results if r.meets_target(target) and not r.reference]
    lines += ["## AC-01 の判定", ""]
    if met:
        lines.append(f"成功率 {100 * target:.0f}% 以上を満たしたプロバイダ: {', '.join(met)}")
    else:
        lines.append(f"成功率 {100 * target:.0f}% 以上を満たしたプロバイダはありません (参考の行は判定に含めない)")
    lines += [
        "", "## 注記", "",
        "- 検出誤差 = VLM の出力から算出した 3 次元位置 (ブロック中心の推定) と、シミュレーション上の真のブロック中心の距離 [mm] (AC-05)。"
        "推定が出なかった試行 (VLM 失敗など) は平均・中央値に入れない。",
        "- 検出誤差には、枠が完全でも出る深度方式の偏り (約 6〜9 mm。斜めのカメラで枠の中心画素の深度が側面に当たる) を含む。",
        "- 参考 = oracle: 真の配置から作った枠 (実際の VLM ではない)。パイプラインの上限を見るための行。",
        "- トークン = API を呼んだ試行の入力・出力トークンの累計 (キャッシュから返した分は含まない)。料金 = 入力 x 入力単価 + 出力 x 出力単価 (単価は各社の料金表)。",
        "- 未評価 = API キーがない、またはクライアント未実装。評価していない (成功率 0% ではない)。中断 = 評価の途中で想定外の例外。",
        "",
    ]
    return "\n".join(lines)


def _safe_name(name: str) -> str:
    if not name or "/" in name or "\\" in name or name in {".", ".."}:
        raise ValueError(f"プロバイダ名にパス区切りは使えません: {name!r}")
    return name


def write_report(
    out_dir: str | Path,
    results: Sequence[ProviderResult],
    *,
    target: float,
    n_trials: int,
    base_seed: int,
    colours: Sequence[str],
    instruction_template: str,
) -> dict[str, Path]:
    """【機能概要】: comparison.md / comparison.json と、記録のあるプロバイダごとの <名前>/trials.csv・summary.json を書く 🔵"""
    out = Path(out_dir)
    names = [_safe_name(r.provider) for r in results]
    out.mkdir(parents=True, exist_ok=True)
    markdown = out / "comparison.md"
    markdown.write_text(
        format_markdown(results, target=target, n_trials=n_trials, base_seed=base_seed, colours=colours,
                        instruction_template=instruction_template),
        encoding="utf-8",
    )  # fmt: skip
    providers = []
    for name, r in zip(names, results, strict=True):
        s = r.summary
        providers.append({
            "provider": name, "model": r.model, "status": r.status, "reason": r.reason, "reference": r.reference,
            "n_trials": None if s is None else s.n_trials, "n_success": None if s is None else s.n_success,
            "success_rate": None if s is None else s.success_rate,
            "mean_pick_error_mm": r.mean_pick_error_mm, "median_pick_error_mm": r.median_pick_error_mm,
            "mean_vlm_time_s": r.mean_vlm_time_s, "usage": r.usage, "by_failure": {} if s is None else s.by_failure,
            "by_group": {} if s is None else s.by_group, "meets_target": r.meets_target(target),
        })  # fmt: skip
        if r.records:
            write_csv(out / name / "trials.csv", r.records)
            write_summary(out / name / "summary.json", s)  # type: ignore[arg-type]
    document = {
        "conditions": {
            "n_trials": n_trials, "base_seed": base_seed, "colours": list(colours),
            "instruction_template": instruction_template,
        },
        "target": target, "providers": providers,
    }
    js = out / "comparison.json"
    js.write_text(json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
    return {"markdown": markdown, "json": js}
