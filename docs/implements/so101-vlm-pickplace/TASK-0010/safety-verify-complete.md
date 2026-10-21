# 安全フィルタ(TASK-0010) TDD開発完了記録

## 確認すべきドキュメント

- `docs/tasks/so101-vlm-pickplace/TASK-0010.md`
- `docs/implements/so101-vlm-pickplace/TASK-0010/safety-requirements.md`
- `docs/implements/so101-vlm-pickplace/TASK-0010/safety-testcases.md`

## 最終結果 (2026-10-21)

- **判定**: OK(完全実装済み、スコープ外失敗なし)
- **TC 網羅**: safety-testcases.md の 124 TC-ID すべてが tests/safety のテストにある(124/124、余分な ID なし)
- **テスト**: `uv run pytest -q` = 724 passed(約 7 s)、`tests/safety` = 255 passed(約 1 s)、遅いテストなし(最大 0.16 s)
- **カバレッジ**: `uv run pytest --cov -q` 全体 100%(safety/ の 4 モジュールも 100%)
- **要件網羅率**: 100% / 実装率 100% / 成功率 100%

## 完了条件

| 完了条件 | 結果 | 根拠 |
|---|---|---|
| 4 つの機能の境界ケースのテストが通る | 達成 | clip_to_limits / limit_step / is_in_workspace / check_state_fresh の境界テスト(tests/safety/test_limits.py、TC-CL/TC-LS/TC-WS/TC-ST)。timeout ちょうどは fresh、超過は stale |
| 速度上限の倍率を設定で変えられる | 達成 | `SafetyConfig.speed_scale`(0 < s <= 1)。`dataclasses.replace(config, speed_scale=0.3)` で 30% を検証(TC-SC、TC-AS)。configs/motion.yaml は 1.0 |
| NFR-02 | 達成 | 最終クリップで常に ctrlrange 内。閉ループでも範囲内 |
| NFR-07(2)(3)(4) | 達成 | 速度 30%、範囲外目標の拒否(rejected、q_current 保持)、状態途絶で stopped(q_current 保持) |

## 独立スポットチェック(throwaway: scratchpad/safety_verify/)

1. 実部品の合成: pickplace モデルで 20 個の到達可能なタスク格子点を solve_ik(全て解けた)し、q=0 から IK 解への関節空間の線形軌道(1 tick の最大関節比 0.99)を previous-command 基準で apply_safety に通した。
   - scale 1.0 / 0.3 とも clipped / rate_limited / rejected / stopped は全 tick で False、出力は入力と一致(不一致 0)。最大比 0.9898 / 0.9899。
   - 1.5 倍速にすると、scale 1.0 は 819 tick、scale 0.3 は 2714 tick(計 3533 tick)の全てで rate_limited が True。`np.abs(cmd-prev)` を m = vmax*scale*dt で割った比の最大は 1.0 で、m を超えなかった。
2. SimRobot 閉ループ(2 s、遠い目標、フィルタ経由):
   - scale 0.3: 関節の平均速度の最大 0.301 rad/s(vmax*scale = 0.30)、周期ごとのピーク 0.387 rad/s(サーボ動特性による超過、報告のみ)、q は ctrlrange 内(下限側の余裕 0.17 rad)。
   - scale 1.0: 平均速度の最大 1.002 rad/s(上限 1.00)、ピーク 1.402 rad/s、q は ctrlrange 内。
3. 作業範囲: SCENE_SPEC から再生成した 172 点すべてが範囲内(最小 (0.15,-0.10,0.015)、最大 (0.35,0.23,0.14))。範囲外の目標は rejected=True、stopped=False、q_command == q_current。状態途絶は stopped=True、reason=stale_state、q_command == q_current。単調時刻で now-last = timeout ちょうどは fresh、超過は stale。時刻が逆行した場合(last > now)は stale 扱い。
4. 既定値: `load_safety_config()` が configs/motion.yaml を読み、vmax=1.0 x6、control_dt=0.02、speed_scale=1.0、state_timeout=0.2、workspace lower=(0.12,-0.13,0.00)、upper=(0.38,0.26,0.17) と決定値に一致。safety: 内の未知キーは ValueError(「未知のキー: bogus (safety)」)。

## 未確認の決定事項(ユーザー未確認、既定値で進行)

| ID | 内容 | 採用した既定値 | 信頼性 |
|---|---|---|---|
| D1 | 安全用の作業範囲を配置用 work_area と別に持ち、箱まで広げる | x 0.12〜0.38 / y -0.13〜0.26 / z 0.00〜0.17 | 🔴 |
| D2 | vmax(STS3215 の実機値が資料にない) | 全関節 1.0 rad/s(実機初回は x0.3) | 🔴 |
| D3 | state_timeout | 0.2 s(10 周期)。TASK-0035 で調整 | 🔴 |
| D4 | 実機の speed_scale の置き場所 | configs/real.yaml(TASK-0035 で決定)。motion.yaml は 1.0 | 🟡 |
| D5 | 速度制限の方式 | 一様スケーリング(方向を保つ) | 🟡 |
| D6 | 床面 z=0 の目標を許すか | 許す(z >= 0.00 を含む)。机との接触は TASK-0012/0013 で確認 | 🟡 |
| D7 | gripper も同じ vmax | 同じ値 | 🟡 |

## 注意点

- 速度制限の実測では、サーボ動特性により 1 周期のピークが vmax*scale を超える(scale 0.3 で 0.387、scale 1.0 で 1.402 rad/s)。平均速度は上限に収まる。実機の確認は TASK-0035。
- q_current が可動域外の場合、最終クリップを優先するため変化量が m を超えうる(仕様どおり)。
- 問題なし。スコープ外のテスト失敗なし。次は TASK-0012(把持・運搬の軌道生成)。
