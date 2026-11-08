"""複数試行の自動実行と集計 (TASK-0026)。

🔵 FR-12 / design.md 4.8: シードと指示を変えた試行を繰り返し、試行ごとの記録 (trials.csv) と成功率・失敗の分類を出す。
試行 i のシードは base_seed + i (configs/eval.yaml の規約)、対象の色は colours を順に回す。
集計・CSV は純粋ロジック。1 試行は run_trial (TASK-0024)。想定外の例外 (認証エラーなど) は握りつぶさず、そこで止める
(途中までの記録は on_record で受け取れる / TrialCsv が 1 試行ごとに書き出す)。
"""

import csv
import json
import time
from collections import Counter
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from types import TracebackType

from so101_vlm.eval.eval_config import load_eval_config
from so101_vlm.eval.failure import FAILURE_GROUP, FailureKind
from so101_vlm.eval.fixed_pick import COLOUR_TO_BODY
from so101_vlm.eval.layout import sample_layout
from so101_vlm.eval.oracle_vlm import OracleVlm
from so101_vlm.eval.trial import TrialOutcome, run_trial
from so101_vlm.eval.validation import require_int
from so101_vlm.perception.calibration import Calibration, load_calibration

DEFAULT_COLOURS = tuple(COLOUR_TO_BODY)
DEFAULT_INSTRUCTION = "put the {colour} block into the box"
_COLOUR_SLOT = "{colour}"


@dataclass(frozen=True)
class TrialRecord:
    """【機能概要】: 1 試行の記録 (CSV の 1 行)。フラットな値だけを持つ。🔵

    design.md 4.8: シード・指示・プロバイダ・モデル・検出結果 (pick_label / pick_bbox / place_bbox)・成否・失敗の種類・
    所要時間・検出誤差 (pick_error_mm = 推定と真のブロック中心の距離 [mm])。bbox は "x1 y1 x2 y2"、pick_xyz は "x y z" [m]。
    値が無いものは None (CSV では空)。total_time_s は 1 試行の実時間 (VLM 呼び出しを含む)。
    """

    trial: int
    seed: int
    colour: str
    instruction: str
    provider: str
    model: str
    success: bool
    failure: str | None
    failure_group: str | None
    error: str | None
    planning_error: str | None
    pick_error_mm: float | None
    pick_label: str
    pick_xyz: str
    pick_bbox: str
    place_bbox: str
    vlm_time_s: float | None
    plan_time_s: float | None
    total_time_s: float
    other_block_moved: bool


CSV_COLUMNS = tuple(f.name for f in fields(TrialRecord))


def _join(values: Sequence[object]) -> str:
    return " ".join(str(v) for v in values)


def record_from_outcome(
    trial: int, outcome: TrialOutcome, *, provider: str, model: str, total_time_s: float
) -> TrialRecord:
    """【機能概要】: TrialOutcome を CSV の 1 行 (TrialRecord) にする 🔵"""
    plan, run = outcome.plan, outcome.run
    failure = None if outcome.failure is None else outcome.failure.value
    return TrialRecord(
        trial=trial, seed=outcome.seed, colour=outcome.colour, instruction=outcome.instruction,
        provider=provider, model=model, success=outcome.success, failure=failure,
        failure_group=None if outcome.failure is None else FAILURE_GROUP[outcome.failure].value,
        error=outcome.error, planning_error=None if run is None else run.planning_error,
        pick_error_mm=outcome.pick_error_mm,
        pick_label="" if plan is None else plan.pick.label,
        pick_xyz="" if outcome.pick_estimate is None else _join(round(v, 5) for v in outcome.pick_estimate),
        pick_bbox="" if plan is None else _join(plan.pick.bbox_px),
        place_bbox="" if plan is None else _join(plan.place.bbox_px),
        vlm_time_s=outcome.vlm_time_s, plan_time_s=None if run is None else run.plan_time_s,
        total_time_s=total_time_s, other_block_moved=False if run is None else run.other_block_moved,
    )  # fmt: skip


def oracle_client_factory(calibration: Calibration | None = None) -> Callable[[int, str], OracleVlm]:
    """【機能概要】: 試行ごとに、その配置の真値から枠を作る偽 VLM (API キー不要) を返すファクトリ 🟡"""
    cal = load_calibration() if calibration is None else calibration
    place = load_eval_config().placement

    def client_for(seed: int, colour: str) -> OracleVlm:
        layout = sample_layout(seed, place.n_blocks, place.area, place.min_gap, place.max_attempts)
        return OracleVlm(layout, colour, cal)

    return client_for


def run_trials(
    n_trials: int,
    base_seed: int,
    client_for: Callable[[int, str], object],
    *,
    colours: Sequence[str] = DEFAULT_COLOURS,
    instruction_template: str = DEFAULT_INSTRUCTION,
    trial_fn: Callable[..., TrialOutcome] = run_trial,
    on_record: Callable[[int, int, TrialRecord], None] | None = None,
    **trial_kwargs: object,
) -> list[TrialRecord]:
    """【機能概要】: n_trials 回、run_trial を回して記録を返す 🔵

    試行 i: seed = base_seed + i、対象の色 = colours[i % len(colours)]、指示 = テンプレートの {colour} を色名に置換 (他の波括弧は触らない)。
    client_for(seed, colour) がその試行の VlmClient を返す (プロバイダ・モデルは client の属性から記録)。
    on_record(何件目, 全件数, 記録) は 1 試行ごとに呼ぶ。trial_kwargs は run_trial にそのまま渡す。
    不正な引数は ValueError。run_trial の想定外の例外はそのまま上に出す。
    """
    n_trials = require_int("n_trials", n_trials, 1)
    base_seed = require_int("base_seed", base_seed, 0)
    colours = tuple(colours)
    if not colours or any(c not in COLOUR_TO_BODY for c in colours):
        raise ValueError(f"colours は {DEFAULT_COLOURS} から 1 つ以上で指定してください: {colours!r}")
    if _COLOUR_SLOT not in instruction_template:
        raise ValueError(f"instruction_template には {_COLOUR_SLOT} が必要です: {instruction_template!r}")
    records: list[TrialRecord] = []
    for i in range(n_trials):
        seed, colour = base_seed + i, colours[i % len(colours)]
        instruction = instruction_template.replace(_COLOUR_SLOT, colour)
        client = client_for(seed, colour)
        started = time.perf_counter()
        outcome = trial_fn(seed, colour, instruction, client, **trial_kwargs)
        record = record_from_outcome(
            i, outcome, provider=str(getattr(client, "provider", "unknown")),
            model=str(getattr(client, "model", "unknown")), total_time_s=time.perf_counter() - started,
        )
        records.append(record)
        if on_record is not None:
            on_record(i + 1, n_trials, record)
    return records


# --------------------------------------------------------------------------- 集計
@dataclass(frozen=True)
class Summary:
    """【機能概要】: 成功率と失敗の分類。success_rate / mean_* は、対象がなければ None 🔵

    by_failure: 失敗の種類 (FailureKind の値) -> 件数、by_group: design.md 4.8 の 4 分類 (vlm / ik / grasp / place) -> 件数。
    mean_pick_error_mm は、推定のある試行だけの平均。
    """

    n_trials: int
    n_success: int
    success_rate: float | None
    by_failure: dict[str, int]
    by_group: dict[str, int]
    mean_pick_error_mm: float | None
    mean_total_time_s: float | None


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def summarize(records: Sequence[TrialRecord]) -> Summary:
    """【機能概要】: 記録から成功率・失敗の種類別/グループ別の件数・平均を求める 🔵"""
    n = len(records)
    ok = sum(1 for r in records if r.success)
    return Summary(
        n_trials=n, n_success=ok, success_rate=ok / n if n else None,
        by_failure=dict(Counter(r.failure for r in records if r.failure)),
        by_group=dict(Counter(r.failure_group for r in records if r.failure_group)),
        mean_pick_error_mm=_mean([r.pick_error_mm for r in records if r.pick_error_mm is not None]),
        mean_total_time_s=_mean([r.total_time_s for r in records]),
    )  # fmt: skip


def _fmt(value: float | None, unit: str) -> str:
    return "-" if value is None else f"{value:.1f} {unit}"


def format_summary(s: Summary) -> str:
    """【機能概要】: 成功率と失敗の種類の表を、端末に出す文字列にする 🔵"""
    rate = "-" if s.success_rate is None else f"{100 * s.success_rate:.1f}%"
    lines = [f"成功率: {rate} ({s.n_success}/{s.n_trials})"]
    if s.by_failure:
        lines.append("失敗の種類:")
        for kind, count in sorted(s.by_failure.items(), key=lambda kv: (-kv[1], kv[0])):
            group = FAILURE_GROUP[FailureKind(kind)].value
            lines.append(f"  {kind:<20} {count:>3}  ({group})")
        lines.append("分類別: " + ", ".join(f"{g}={c}" for g, c in sorted(s.by_group.items())))
    lines.append(f"検出誤差 (平均): {_fmt(s.mean_pick_error_mm, 'mm')}   1 試行の時間 (平均): {_fmt(s.mean_total_time_s, 's')}")
    return "\n".join(lines)


def write_summary(path: str | Path, summary: Summary) -> Path:
    """【機能概要】: 集計を JSON で保存する (TASK-0029 の比較表の元)。親ディレクトリは作る 🔵"""
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(asdict(summary), ensure_ascii=False, indent=2), encoding="utf-8")
    return out


# --------------------------------------------------------------------------- CSV
def _cell(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


class TrialCsv:
    """【機能概要】: trials.csv を 1 試行ごとに追記して flush する。途中で止まっても、そこまでの記録が残る 🔵

    with で使う。ヘッダ行は開いた時点で書く。UTF-8、改行・カンマ・引用符を含む値は csv モジュールが引用する。
    """

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._file = None
        self._writer = None

    def __enter__(self) -> "TrialCsv":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.path, "w", encoding="utf-8", newline="")  # noqa: SIM115
        self._writer = csv.writer(self._file, lineterminator="\n")
        self._writer.writerow(CSV_COLUMNS)
        self._file.flush()
        return self

    def append(self, record: TrialRecord) -> None:
        """【機能概要】: 1 行書いて flush する。with の外では RuntimeError 🔵"""
        if self._writer is None or self._file is None:
            raise RuntimeError("TrialCsv は with の中で使ってください")
        self._writer.writerow([_cell(getattr(record, name)) for name in CSV_COLUMNS])
        self._file.flush()

    def __exit__(self, exc_type: type | None, exc: BaseException | None, tb: TracebackType | None) -> None:
        if self._file is not None:
            self._file.close()
        self._file = self._writer = None


def write_csv(path: str | Path, records: Sequence[TrialRecord]) -> Path:
    """【機能概要】: 記録を trials.csv にまとめて書く (列は CSV_COLUMNS)。記録が 0 件でもヘッダは書く 🔵"""
    with TrialCsv(path) as out:
        for record in records:
            out.append(record)
    return Path(path)
