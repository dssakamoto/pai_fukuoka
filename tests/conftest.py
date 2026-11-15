"""全テスト共通の fixture。"""

import pytest

from so101_vlm.env import ENV_FILE_VAR


@pytest.fixture(autouse=True)
def _ignore_the_developers_dotenv(monkeypatch, tmp_path_factory):
    """スクリプトが読む .env を、存在しないファイルに向ける。開発者の本物の .env (API キー) が、
    「キーがない」前提のテストを壊したり、実 API を呼んだりしないようにする。.env を読むテスト自身は、自分で上書きする。"""
    monkeypatch.setenv(ENV_FILE_VAR, str(tmp_path_factory.getbasetemp() / "no-such.env"))
