# 把持の調整記録 (TASK-0014)

## 結論

TASK-0013 時点の物理パラメータのまま、固定位置の pick & place は安定している。**パラメータの変更は不要**だった。

## 計測 (2026-11-06)

```bash
uv run python scripts/run_fixed_pick.py --colour all --n-seeds 10 --seed 0 --no-video
```

- 計画可能な配置での成功: **29/29 (100%)**(赤・青・緑 x seed 0〜9)
- 計画失敗: 1 件(seed=5 green, x=0.161 y=-0.023, `not_converged` / pre_grasp)。FR-06 どおり動作前の拒否で、把持の問題ではない。作業範囲の手前端(x が小さい側)で IK が収束しない配置。TASK-0024 以降で配置の範囲を見直す候補。
- 完了条件「10 回中 8 回以上」を満たす。

## 変更したパラメータ

なし(`assets/scene_pickplace.xml` は未変更)。

## 環境メモ

Windows で実行して確認。mujoco の読み込みには VC++ 再頒布可能パッケージ (x64) が必要。
