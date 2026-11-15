"""1 試行を、指示つきで実行し、各段階と失敗の理由を表示する (AC-02)。

  # 実 VLM で。存在しない色を指示すると、ロボットは動かず、理由が表示される (AC-02)
  uv run python scripts/run_trial.py --provider openai --instruction "put the purple block into the box"
  # 普通の試行 (API キー不要の偽 VLM = 真の配置から作る枠)
  uv run python scripts/run_trial.py --provider oracle --seed 2 --colour red --instruction "put the red block into the box"

表示: 指示 -> VLM -> 検出 (枠) -> 推定位置と検出誤差 -> 実行 (動かしたか) -> 判定 (失敗なら種類と理由)。
--colour は、成功を判定する対象 (シミュレーション上の真値) で、VLM には渡さない。指示が存在しない色のときは、何でもよい。
VLM の応答はキャッシュされる (configs/vlm.yaml)。API キーは .env か環境変数から読む。
終了コード: 0 = 成功、1 = 試行は行ったが失敗 (VLM が対象を見つけられない場合を含む)、2 = 引数・設定の誤り (API キー未設定を含む)。
"""

import argparse
import sys

from so101_vlm.env import load_dotenv
from so101_vlm.eval.compare import default_client_factory_for
from so101_vlm.eval.fixed_pick import COLOUR_TO_BODY
from so101_vlm.eval.trial import run_trial
from so101_vlm.eval.trial_report import format_trial_report
from so101_vlm.vlm import MissingApiKeyError


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--provider", default="oracle", help="oracle (偽 VLM) または configs/vlm.yaml のプロバイダ")
    parser.add_argument("--seed", type=int, default=0, help="ブロックの配置のシード")
    parser.add_argument("--colour", default="red", choices=list(COLOUR_TO_BODY), help="成功を判定する対象 (真値)")
    parser.add_argument("--instruction", default="put the red block into the box", help="VLM への指示 (自然言語)")
    args = parser.parse_args(argv)
    load_dotenv()  # API キーを .env から (既存の環境変数は上書きしない。値は表示しない)

    try:
        client = default_client_factory_for()(args.provider)(args.seed, args.colour)
        outcome = run_trial(args.seed, args.colour, args.instruction, client)
    except MissingApiKeyError as exc:
        print(f"API キーがありません: {exc}", file=sys.stderr)
        return 2
    except (ValueError, NotImplementedError, FileNotFoundError) as exc:
        print(f"引数・設定の誤り: {exc}", file=sys.stderr)
        return 2

    provider = str(getattr(client, "provider", args.provider))
    print("\n".join(format_trial_report(outcome, provider=provider, model=str(getattr(client, "model", "-")))))
    usage = getattr(client, "usage", None)
    if usage and usage["calls"]:
        print(f"使用トークン: 入力 {usage['input_tokens']} / 出力 {usage['output_tokens']} ({usage['calls']} 回の呼び出し)")
    return 0 if outcome.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
