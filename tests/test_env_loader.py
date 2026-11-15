"""so101_vlm.env.load_dotenv のテスト (TASK-0019〜0021 の付随)。API キーを .env から読む。値は表示しない・上書きしない。"""

import pytest

from so101_vlm.env import ENV_FILE_VAR, load_dotenv


def _write(tmp_path, text, name=".env"):
    p = tmp_path / name
    p.write_text(text, encoding="utf-8")
    return p


def test_loads_plain_quoted_and_exported_values(tmp_path):
    p = _write(tmp_path, 'A_KEY=plain\nB_KEY="double quoted"\nC_KEY=\'single quoted\'\nexport D_KEY=exported\n  E_KEY =  spaced  \n')
    env = {}
    assert load_dotenv(p, env) == ["A_KEY", "B_KEY", "C_KEY", "D_KEY", "E_KEY"]
    assert env == {"A_KEY": "plain", "B_KEY": "double quoted", "C_KEY": "single quoted", "D_KEY": "exported", "E_KEY": "spaced"}


def test_comments_blank_lines_and_malformed_lines_are_ignored(tmp_path):
    p = _write(tmp_path, "# comment\n\nNOEQUALS\n=novalue\n1BAD=x\nBAD-NAME=x\nGOOD=1 # trailing comment\nQUOTED=\"a # not a comment\"\n")
    env = {}
    assert load_dotenv(p, env) == ["GOOD", "QUOTED"]
    assert env == {"GOOD": "1", "QUOTED": "a # not a comment"}


def test_empty_values_stay_unset_so_the_missing_key_error_fires(tmp_path):
    p = _write(tmp_path, "ANTHROPIC_API_KEY=\nOPENAI_API_KEY=   \n")
    env = {}
    assert load_dotenv(p, env) == [] and env == {}


def test_existing_environment_wins_but_a_blank_one_is_filled(tmp_path):
    p = _write(tmp_path, "KEEP=from_file\nBLANK=from_file\nNEW=from_file\n")
    env = {"KEEP": "from_shell", "BLANK": "  "}
    assert load_dotenv(p, env) == ["BLANK", "NEW"]
    assert env == {"KEEP": "from_shell", "BLANK": "from_file", "NEW": "from_file"}


def test_missing_file_is_not_an_error(tmp_path):
    assert load_dotenv(tmp_path / "nope.env", {}) == []


def test_utf8_bom_and_crlf_are_handled(tmp_path):
    p = tmp_path / ".env"
    p.write_bytes("﻿A_KEY=1\r\nB_KEY=2\r\n".encode("utf-8"))
    env = {}
    assert load_dotenv(p, env) == ["A_KEY", "B_KEY"] and env == {"A_KEY": "1", "B_KEY": "2"}


def test_values_are_never_returned_or_printed(tmp_path, capsys):
    p = _write(tmp_path, "SECRET_KEY=super-secret-value\n")
    names = load_dotenv(p, {})
    assert names == ["SECRET_KEY"] and "super-secret-value" not in repr(names)
    assert capsys.readouterr() == ("", "") or "super-secret-value" not in "".join(capsys.readouterr())


def test_env_file_variable_selects_the_file(tmp_path, monkeypatch):
    p = _write(tmp_path, "FROM_ALT=1\n", name="alt.env")
    monkeypatch.setenv(ENV_FILE_VAR, str(p))
    env = {}
    assert load_dotenv(environ=env) == ["FROM_ALT"] and env == {"FROM_ALT": "1"}


def test_default_file_is_the_repo_dotenv(monkeypatch, tmp_path):
    monkeypatch.delenv(ENV_FILE_VAR, raising=False)
    from so101_vlm.env import default_env_path
    from so101_vlm.robot.model_paths import REPO_ROOT

    assert default_env_path() == REPO_ROOT / ".env"


@pytest.mark.parametrize("bad", [None, 5])
def test_bad_environ_is_value_error(tmp_path, bad):
    with pytest.raises(ValueError):
        load_dotenv(tmp_path / "x.env", bad if bad is not None else "not a mapping")
