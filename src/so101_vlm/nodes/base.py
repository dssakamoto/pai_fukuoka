"""ノードの基底クラス (TASK-0025)。

🔵 design.md 2 章: ノードは薄い入出力の殻にして、処理はライブラリを呼ぶ。dora に依存しない純粋なクラスにして、
ランタイム (dora_runtime.py の dora、local_runner.py のプロセス内再生) が emit / wait を差し込む。
  emit(name, payload, meta): 出力を送る。wait(predicate): 条件が満たされるまで入力を処理して待つ
  (motion が robot_state の返事を待つのに使う。ローカルランナーでは、返事は送信の中で届いているので何もしない)。
"""

from collections.abc import Callable

from so101_vlm.nodes.node_config import NodeConfig

Emit = Callable[[str, object, dict | None], None]
Wait = Callable[[Callable[[], bool]], None]


class StopRequested(Exception):
    """【機能概要】: 待っている間に stop が来た (または入力が途絶えた)。ノードは静かに終わる 🔵"""


class NodeBase:
    """【機能概要】: 入力を受けて処理し、emit で出力する。finished が True になったらランタイムはループを抜ける 🔵

    サブクラスは handle(name, payload, meta) を実装する。stop 入力は共通で finished にする
    (evaluator が結果を出したら stop を全ノードに送る。配線が循環していても全員が確実に終わる)。
    """

    def __init__(self, config: NodeConfig) -> None:
        self.config = config
        self.finished = False
        self._emit: Emit | None = None
        self._wait: Wait | None = None

    def bind(self, emit: Emit, wait: Wait) -> None:
        """【機能概要】: ランタイムが出力と待ちの手段を差し込む 🔵"""
        self._emit, self._wait = emit, wait

    def emit(self, name: str, payload: object, meta: dict | None = None) -> None:
        """【機能概要】: 出力 name を送る。bind 前は RuntimeError 🔵"""
        if self._emit is None:
            raise RuntimeError("bind() の前に emit は使えません")
        self._emit(name, payload, meta)

    def wait_until(self, predicate: Callable[[], bool]) -> None:
        """【機能概要】: predicate が True になるまで入力を処理して待つ。stop が来たら StopRequested 🔵"""
        if self._wait is None:
            raise RuntimeError("bind() の前に wait は使えません")
        self._wait(predicate)

    def start(self) -> None:
        """【機能概要】: 入力を待たずに始める処理 (入力のない起点ノード用)。既定は何もしない 🔵"""

    def on_input(self, name: str, payload: object, meta: dict) -> None:
        """【機能概要】: 入力 1 件を処理する。stop は共通で終了 🔵"""
        if name == "stop":
            self.finished = True
            return
        self.handle(name, payload, meta)

    def handle(self, name: str, payload: object, meta: dict) -> None:
        """【機能概要】: stop 以外の入力の処理 (サブクラスで実装) 🔵"""
        raise NotImplementedError
