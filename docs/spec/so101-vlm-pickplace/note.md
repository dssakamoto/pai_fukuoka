# so101-vlm-pickplace 開発コンテキストノート

**作成日**: 2026-10-18

## 技術スタック

- 言語: Python(3.12 に固定する想定。MuJoCo・LeRobot の対応版を実装時に確認)
- パッケージ管理: uv
- シミュレータ: MuJoCo。SO-101 モデルは公式 `TheRobotStudio/SO-ARM100` の `Simulation/SO101`(Apache-2.0)
- ノード間通信: dora-rs(講義第6回)。Mac で動かなければ multiprocessing
- ロボット学習・実機ドライバ: LeRobot(講義第5回)
- VLM: Anthropic / OpenAI / Gemini(API)、Gemma(Ollama でローカル)
- v2: SmolVLA(LeRobot)。微調整は GPU 環境(Colab など、未決)
- テスト: pytest、pytest-cov(カバレッジ 80%)、GitHub Actions
- ROS 2: 使わない(任意。レポートで移植方針のみ)

## 開発ルール(ユーザーのグローバルルールより)

- TDD: テストを先に書く(RED → GREEN → REFACTOR)。カバレッジ 80% 以上
- 不変性: データは frozen な dataclass とし、配列は書き換えずに新しく返す
- 小さいファイル(200〜400 行目安、最大 800 行)、小さい関数(50 行未満)
- エラーは握りつぶさず、ユーザーが対処できるメッセージにする
- 秘密情報(API キー)をコードに書かない。環境変数で渡す
- コミットは conventional commits(feat, fix, ...)

## 環境(ヒアリング・確認済み)

- 開発機: macOS(Apple Silicon)、メモリ 16GB、GPU 学習なし
- 実機: SO-101 のフォロワーとリーダー。**この Mac に接続**(udev は使えない)
- 作業時間: 1 日 1〜2 時間。工数は AI 支援前提の本人の作業時間で見積もる

## 関連文書

- 要件定義書: docs/requirements.md
- 設計文書: docs/design.md
- 旧タスク一覧: docs/tasks.md
- 図解版: docs/要件定義書.html

## 注意事項

- 外部ライブラリ(LeRobot、dora-rs、各 VLM の SDK)の API は、実装前に公式ドキュメントで確認する
- 実機カメラが深度を取れるかは未確認。RGB のみなら、作業台の高さを使う方式にする(TASK-0036)
- 提出期限は未確認。工数(合計 84h)は、1 日 1〜2 時間の作業では 36 日に収まらない(overview.md 参照)
