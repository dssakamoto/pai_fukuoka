"""1 試行の結果を、人が読む行にする (AC-02)。

🔵 AC-02: 存在しない色を指定したとき、動作せず理由が表示される。VLM・知覚・計画の失敗は、ロボットを動かす前に拒否する
(FR-03 / FR-06)。ここでは、その事実 (動かしていない) と理由 (VLM の言葉、IK のどの検査か) を、段階の順
(指示 -> 検出 -> 推定位置 -> 実行 -> 判定) に並べて表示する。純粋ロジック (sim なし)。
"""

from so101_vlm.eval.failure import FailureKind
from so101_vlm.eval.trial import TrialOutcome

# 🟡 失敗の種類ごとの説明 (人が読む)。FailureKind の全種類を網羅する (テストで確認)
FAILURE_EXPLANATION: dict[FailureKind, str] = {
    FailureKind.VLM_FAILURE: "VLM の応答が使えない (対象が見つからない・形式が不正・呼び出しの失敗)",
    FailureKind.PERCEPTION_FAILURE: "検出枠から 3 次元位置を求められない (深度が取れないなど)",
    FailureKind.PLANNING_FAILURE: "軌道を作れない (IK・作業範囲・障害物との干渉の検査で拒否)",
    FailureKind.GRASP_SLIP: "ブロックを持ち上げられない、または運搬中に落とした",
    FailureKind.DROP: "箱の上で放す前にブロックを落とした",
    FailureKind.COLLISION_OTHER: "他のブロックに当たって動かした",
    FailureKind.KNOCKED_OVER: "ブロックが倒れた",
    FailureKind.NOT_IN_BOX: "ブロックが箱の中に入らなかった",
}


def _executed_reason(run: object, kind: FailureKind) -> str:
    """実行後の失敗の理由: 判定の失敗条件と、そのときの数値 (箱中心からのずれ・高さ・速さ、持ち上げ量)。"""
    judge, trace = getattr(run, "judge", None), getattr(run, "trace", None)
    if judge is None:
        return "実行後の判定が失敗しました"
    conditions = ", ".join(r.value for r in judge.reasons) or "(判定の条件は満たしたが、動作の記録から分類)"
    dx, dy = judge.offset_xy
    parts = [
        f"判定の失敗条件: {conditions}",
        f"箱中心からのずれ ({dx * 1000:.0f}, {dy * 1000:.0f}) mm",
        f"ブロック中心の高さ {judge.height * 1000:.0f} mm",
        f"速さ {judge.speed * 100:.1f} cm/s",
    ]
    if trace is not None and kind is FailureKind.GRASP_SLIP:
        parts.append(f"持ち上げ {trace.lift_end_rise * 1000:.0f} mm")
    return "; ".join(parts)


def _one_line(text: object) -> str:
    return " ".join(str(text).split())


# 🔵 Intent: 失敗の理由は、種類ごとの出どころ (VLM の言葉 / IK の検査名 / 実行後の判定) から、必ず 1 行で出す。
def format_trial_report(outcome: TrialOutcome, *, provider: str, model: str) -> list[str]:
    """【機能概要】: 1 試行の結果を 指示・VLM・検出・推定位置・実行・判定・理由 の行にする。outcome が不正なら ValueError 🔵"""
    if not isinstance(outcome, TrialOutcome):
        raise ValueError(f"outcome は TrialOutcome で指定してください: {type(outcome).__name__}")
    lines = [f"指示: {outcome.instruction}", f"VLM: {provider} / {model}"]
    plan = outcome.plan
    if plan is None:
        lines.append("検出: なし")
    else:
        lines.append(
            f"検出: pick = {plan.pick.label} {plan.pick.bbox_px} / place = {plan.place.label} {plan.place.bbox_px}"
        )
    if outcome.pick_estimate is not None:
        x, y, z = outcome.pick_estimate
        error = "" if outcome.pick_error_mm is None else f"  検出誤差 {outcome.pick_error_mm:.1f} mm (真のブロック中心との距離)"
        lines.append(f"推定位置: pick = ({x:.3f}, {y:.3f}, {z:.3f}) m{error}")
    run = outcome.run
    moved = run is not None and run.execution is not None
    lines.append("実行: ロボットを動かしました" if moved else "実行: ロボットは動かしていません (動作の前に拒否)")
    if outcome.failure is None:
        lines.append("判定: 成功")
        return lines
    lines.append(f"判定: 失敗 ({outcome.failure.value}): {FAILURE_EXPLANATION[outcome.failure]}")
    if outcome.failure is FailureKind.PLANNING_FAILURE and run is not None:
        reason = f"{run.planning_error} (経由点: {run.planning_waypoint})"
    elif moved:
        reason = _executed_reason(run, outcome.failure)
    else:
        reason = outcome.error or ""
    lines.append(f"理由: {_one_line(reason)}".rstrip())
    return lines
