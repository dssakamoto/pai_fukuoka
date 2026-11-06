# VLM プロバイダの仕様確認 (TASK-0016)

確認日: 2026-11-06。各社の公式ドキュメントを読んで記録した。モデル名・API は変わりやすいので、実装時と評価前に再確認する。
🔵 = ドキュメントに明記 / 🟡 = ドキュメントから推測 / 🔴 = 未確認(実測が必要)

## 一覧

| | Anthropic | OpenAI | Gemini | Gemma (Ollama) |
|---|---|---|---|---|
| 使うモデル | `claude-haiku-5-5`(安価)。精度を見るなら `claude-sonnet-5-5` / `claude-opus-5-5` | `gpt-6-luna`(最安)。精度を見るなら `gpt-6.1-terra` | `gemini-3.8-flash` | `gemma4:e4b`(6.6〜9.5GB) |
| SDK | `anthropic` | `openai` | `google-genai` | `ollama`、または `openai`(`base_url=http://localhost:11434/v1/`) |
| 画像入力 | base64 / url / file_id。`image/jpeg,png,gif,webp` | base64 data URI(`input_image`)/ url / file_id | inline base64(20MB まで)/ File API | `images=[...]`(SDK はパス・bytes)。OpenAI 互換は base64 のみ |
| 構造化出力 | `output_config.format={"type":"json_schema",...}` または `messages.parse(output_format=Pydantic)` | `responses.parse(text_format=Pydantic)` / `text.format` に json_schema + strict | `response_format` に JSON schema(Interactions API) | `format=<JSON schema>` |
| 座標 | **絶対ピクセル** `[x1,y1,x2,y2]` | 規定なし(空間認識は苦手と明記) | **0〜1000 正規化 `[ymin,xmin,ymax,xmax]`**(y が先) | Gemma 4 は Gemini と同じ 0〜1000 `[y,x,y,x]` |

## Anthropic 🔵

- 出典: platform.claude.com/docs/en/build-with-claude/{vision, vision-coordinates, structured-outputs}, about-claude/models/overview
- モデル ID: `claude-fable-5-1`, `claude-opus-5-5`, `claude-sonnet-5-5`, `claude-haiku-5-5`。全モデルが画像入力に対応。
- 座標: 「絶対ピクセルが最も良い」。**Claude が処理した(リサイズ後の)画像のピクセル**で返る。原点は左上。
  - 長辺 1568px・視覚トークン 1568 を超えると縮小される(高解像度 tier は 2576px)。
  - 対策: 送る前に自分でリサイズして、座標が 1 対 1 になるようにする。640x480 は縮小されない。
- 注意: `output_config.format` では `minimum` / `maximum` / `minLength` などのスキーマ項目が 400 になる。`stop_reason: "refusal"` と `max_tokens` 切れでは、スキーマに従わないことがある。
- 座標の精度はあくまで近似。小さいものは精度が落ちるので、可視化(TASK-0023)で確認する。

## OpenAI 🟡

- 出典: developers.openai.com/api/docs/{models, guides/images-vision, guides/structured-outputs}(platform.openai.com は 301 で移転)
- モデル ID: `gpt-6-astra`(旗艦)、`gpt-6.1-sol`、`gpt-6.1-terra`、`gpt-6-luna`(最安)。最新モデルはすべて画像入力に対応。
- 画像: `client.responses.create(input=[{"role":"user","content":[{"type":"input_text",...},{"type":"input_image","image_url":"data:image/png;base64,..."}]}])`
- 構造化出力: `client.responses.parse(..., text_format=PydanticModel)`。strict のスキーマは、全オブジェクトに `additionalProperties: false`、全フィールドを `required` にする必要がある。🟡(要約から)
- 座標: ドキュメントに bbox の規約がない。「精密な空間位置の特定は苦手」「座標が重要なときは `detail: "original"` を使い、画像を制限内にリサイズして座標を元に戻す」とある。
  - 🔴 返ってくる座標がどの空間かは実測が必要。プロンプトで「画像の絶対ピクセル、左上原点」と明示し、可視化で確認する。
  - 640x480 を `detail: "original"` で送り、リサイズを避ける。
- 🔴 Chat Completions の画像形式は未確認。Responses API を使う前提にする。

## Gemini 🔵

- 出典: aistudio.google.com/docs/{image-understanding, structured-output, models}(ai.google.dev は 301 で移転)
- モデル ID: `gemini-3.8-flash`(ドキュメントの例はすべてこれ)、`gemini-3.6-flash`、`gemini-3.5-flash-lite`、`gemini-3.1-flash-lite`。lite の画像入力は明記なし。
- 座標: `[ymin, xmin, ymax, xmax]`、**0〜1000 に正規化**。「元画像のサイズで戻す」必要がある。
  - `x_px = x/1000*W`, `y_px = y/1000*H`(W, H は元画像)。
- SDK: 現在のドキュメントは `client.interactions.create(model=..., input=[{"type":"text",...},{"type":"image","data":<base64>,"mime_type":...}], response_format={"type":"text","mime_type":"application/json","schema":...})`。出力は `interaction.output_text`。
  - 🔴 旧来の `generate_content` が今も使えるかは未確認。実装時に SDK のバージョンで確かめる。
- 注意: `minLength` / `maxLength` などは黙って無視される。出力は必ず検証する。

## Gemma (Ollama) 🟡

- 出典: ollama.com/library/{gemma3,gemma3n,gemma4}, docs.ollama.com/capabilities/{vision,structured-outputs}, ai.google.dev/gemma/docs/{core, capabilities/vision/image}
- 画像対応のタグとダウンロードサイズ(16GB に収まるもの):
  - `gemma4:e2b` 4.6〜7.5GB / `gemma4:e4b` 6.6〜9.5GB / `gemma4:12b` 7.7〜8.0GB
  - `gemma3:4b` 3.4GB / `gemma3:12b` 8.2GB / `gemma3n:e4b` 9.8GB
  - 収まらない: `gemma4:26b`(16GB 以上)、`gemma4:31b`、`gemma3:27b`(17GB)
- 重みの読み込みメモリ(Gemma 4 モデルカード、Q4_0、20% のオーバーヘッド込み、KV キャッシュは別): E2B 2.9GB / E4B 4.5GB / 12B 6.7GB。
- **選定: `gemma4:e4b`**。bbox を公式にサポートするのは Gemma 4(Gemma 3 のモデルカードに記載なし)。メモリは 16GB に余裕がある。`num_ctx` を小さく抑えて KV キャッシュを節約する。
- 座標: Gemma 4 は 0〜1000 の正規化 `[y_min, x_min, y_max, x_max]`(Gemini と同じ。順序は公式ヘルパーのコードから推測 🟡)。
- 呼び出し: `ollama.chat(model="gemma4:e4b", messages=[{"role":"user","content":...,"images":[path]}], format=<schema>)`。OpenAI 互換は `response_format` 対応だが、json_schema の形は Ollama のドキュメントに例がない 🔴。
- ⚠ **このPC(開発機)では動作確認しない方針**(ユーザ判断。メモリ不足で止まる懸念)。TASK-0022 は実機の別PC、またはスキップ。AC-04 により、動かせないプロバイダはレポートで「未評価」と書く。
- 🔴 Ollama 側の最低バージョンは gemma4 のページに記載なし。gemma3 は 0.6 以上。

## 実装への影響

| 項目 | 方針 |
|---|---|
| 座標の変換 | 各クライアント内で `bbox_px`(元画像の絶対ピクセル `x1,y1,x2,y2`)に変換する。Anthropic と OpenAI は絶対ピクセル(送った画像のサイズ基準)、Gemini と Gemma は 0〜1000 の `[y,x,y,x]`。 |
| 画像サイズ | シミュレーションの画像は 640x480。各社の縮小条件に当たらないので、リサイズ後の座標の補正は不要。実機で大きい画像を使うときに再検討する。 |
| 構造化出力 | 4 社とも使える。それでも `schema.py`(TASK-0017)で必ず検証する。スキーマには `minimum` などを書かず、範囲は検証側で見る。 |
| 共通プロンプト | 座標形式を、プロバイダごとに明示する。 |
| 実測が必要 (🔴) | OpenAI の座標空間、Gemini SDK の呼び出し形式、Gemma の json_schema 形式。 |

## 1 試行の通し (TASK-0024) の実測

偽 VLM (`OracleVlm`: 真の配置から作った枠) で `run_trial` を赤・seed 0〜9 で実行 (2026-11-06):

- 成功 9/10。失敗は seed 1 の `obstacle_clearance` (置き場所の推定が x で 6.6 mm ずれ、経路が他のブロックに近づく)。
- pick の推定誤差は 6.2〜9.3 mm。xy は +5 mm 前後、z は 2〜3 mm 低い方向に偏る。
  原因: 斜めのカメラでは、枠の中心画素の深度が上面ではなく手前の側面に当たり、「表面の点 - 半辺」の補正が合わない。
- 改善案 (未実施): 枠の中心画素の光線と、ブロック中心の高さの水平面との交点を使う (深度に依らない)。
  実 VLM の枠はもっとゆるいので、AC-05 の検出誤差の評価前に再検討する。
