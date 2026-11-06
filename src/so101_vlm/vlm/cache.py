"""VLM 応答のキャッシュ (TASK-0018)。

🔵 design.md 4.3: キーは (画像のハッシュ、指示、プロバイダ、モデル)。再現実験で API を呼ばない。
保存するのは VLM の生の応答テキスト。ファイル名はキー (安全な文字だけ) + ".txt"。
"""

import hashlib
import re
from pathlib import Path

import numpy as np

_SAFE_KEY = re.compile(r"^[A-Za-z0-9_-]{1,128}$")


# 🔵 Intent: 画像は shape と dtype も混ぜる (同じバイト列で縦横だけ違う画像を区別する)。
def make_key(image: np.ndarray, instruction: str, provider: str, model: str, tag: str) -> str:
    """【機能概要】: 画像・指示・プロバイダ・モデル・プロンプトの識別子から 64 桁の hex キーを作る 🔵"""
    h = hashlib.sha256()
    arr = np.ascontiguousarray(image)
    for part in (str(arr.shape), str(arr.dtype), instruction, provider, model, tag):
        h.update(part.encode("utf-8"))
        h.update(b"\x00")
    h.update(arr.tobytes())
    return h.hexdigest()


class VlmCache:
    """【機能概要】: ディレクトリに 1 キー 1 ファイルで保存する単純なキャッシュ 🔵

    enabled=False なら何も読み書きしない (ディレクトリも作らない)。
    危険なキー (パス区切りなど) は、読み書きともに無視する。
    """

    def __init__(self, directory: str | Path, enabled: bool = True) -> None:
        self._dir = Path(directory)
        self._enabled = enabled

    def _path(self, key: str) -> Path | None:
        if not self._enabled or not isinstance(key, str) or not _SAFE_KEY.match(key):
            return None
        return self._dir / f"{key}.txt"

    def get(self, key: str) -> str | None:
        """【機能概要】: 保存済みテキスト。なし・読めない場合は None 🔵"""
        path = self._path(key)
        if path is None:
            return None
        try:
            return path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            return None

    def put(self, key: str, text: str) -> None:
        """【機能概要】: テキストを保存する。途中書きを避けるため一時ファイル経由で置き換える 🟡"""
        path = self._path(key)
        if path is None:
            return
        self._dir.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        tmp.write_text(text, encoding="utf-8")
        tmp.replace(path)
