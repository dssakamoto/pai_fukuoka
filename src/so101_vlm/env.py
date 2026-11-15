"""`.env` からの API キーの読み込み (TASK-0019〜0021 の付随)。

🔵 README / .env.example: API キーは .env に書く (.env は git 管理外)。python-dotenv を足さずに、必要な範囲だけ読む。
決まり: 既存の環境変数は上書きしない (シェルで設定したものが優先)。空の値は無いものとして扱う (キー未設定のエラーを出すため)。
読み込んだ変数の「名前」だけを返し、値は返さない・表示しない。
"""

import os
import re
from collections.abc import MutableMapping
from pathlib import Path

from so101_vlm.robot.model_paths import REPO_ROOT

ENV_FILE_VAR = "SO101_ENV_FILE"
_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_TRAILING_COMMENT = re.compile(r"\s+#.*$")


def default_env_path() -> Path:
    """【機能概要】: 読む .env のパス。環境変数 SO101_ENV_FILE があればそれ、なければリポジトリ直下の .env 🔵"""
    custom = os.environ.get(ENV_FILE_VAR)
    return Path(custom) if custom else REPO_ROOT / ".env"


def _parse_value(raw: str) -> str:
    raw = raw.strip()
    if raw[:1] in ("'", '"'):
        end = raw.find(raw[0], 1)
        if end != -1:
            return raw[1:end]
    return _TRAILING_COMMENT.sub("", raw).strip()


# 🔵 Intent: 値は返さず、読み込んだ名前だけを返す (ログに秘密を出さない)。
def load_dotenv(path: str | Path | None = None, environ: MutableMapping[str, str] | None = None) -> list[str]:
    """【機能概要】: .env の KEY=VALUE を environ (既定 os.environ) に読み込み、読み込んだ名前の一覧を返す 🔵

    ファイルが無ければ何もしない。コメント・空行・不正な行は無視。`export KEY=...`、引用符つきの値、BOM・CRLF に対応。
    既存の (空でない) 環境変数は上書きしない。値が空の行は読み込まない。environ が辞書でなければ ValueError。
    """
    target = os.environ if environ is None else environ
    if not isinstance(target, MutableMapping):
        raise ValueError(f"environ は辞書で指定してください: {type(target).__name__}")
    file = default_env_path() if path is None else Path(path)
    try:
        text = file.read_text(encoding="utf-8-sig")
    except FileNotFoundError:
        return []
    loaded: list[str] = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        name, _, raw = line.partition("=")
        name = name.strip().removeprefix("export").strip() if name.strip().startswith("export ") else name.strip()
        value = _parse_value(raw)
        if not _NAME.match(name) or not value:
            continue
        if not str(target.get(name, "")).strip():
            target[name] = value
            loaded.append(name)
    return loaded
